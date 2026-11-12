#!/usr/bin/env python3
"""장기판 네 모서리로 카메라(RealSense D455) ↔ 로봇(E0509) 좌표계를 맞추는 보정 도구.

카메라와 장기판은 고정되어 있다고 가정한다 (eye-to-hand). 장기판 기준 네 점을
좌상단 → 우상단 → 우하단 → 좌하단 순서로 다음을 반복한다.

  1. 카메라 화면에서 점을 클릭 → SPACE: 픽셀 위치 기록
  2. 키보드/버튼 조그(누르고 있는 동안 이동)로 그리퍼 팁을 그 점에 맞춤
     → SPACE: 로봇 base 좌표계 팁 위치 기록

네 쌍(픽셀 ↔ base 3D 점)이 모이면 PnP로 카메라 자세 T_base_camera (p_base = R·p_cam + t)를
구해 config/calibration.yaml에 저장한다. 판(무늬 없는 나무)에서는 D455F depth가 잘 안 잡혀서
보정은 depth를 쓰지 않고, depth는 잡힐 때만 교차검증에 쓴다. 조그는 어느 단계에서나 쓸 수 있고,
--jog-only로 카메라·점 찍기 없이 로봇 패널만 띄울 수도 있다.

키 조작 (창에 포커스가 있을 때):
    마우스 클릭      점 선택 (보정 후에는 클릭한 점의 base 좌표 표시)
    방향키           선택 점 1픽셀 이동
    W / S            조그 base X+ / X-   (누르고 있는 동안 이동, 떼면 정지)
    A / D            조그 base Y+ / Y-
    R / F            조그 base Z+ (위) / Z- (아래)
    1 ~ 6            조그 속도 1 / 3 / 10 / 25 / 50 / 75 %
    H                홈 자세로 이동 (확인 후)
    G                그리퍼 위치 650 (팁 오프셋 기준)
    SPACE / ENTER    현재 단계 확정 (카메라 점 측정 또는 로봇 자세 기록)
    U                한 단계 되돌리기
    ESC              모든 이동 정지 + 종료 확인
"""

from __future__ import annotations

import argparse
import shutil
import signal
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

import cv2
import numpy as np
import yaml
from PIL import Image, ImageTk

import calib_math as cm
from robot_ui import (GRIPPER_TIP_OFFSET_MM, GRIPPER_TIP_POS_CMD, JOG_SPEEDS, JogApp, KeyTracker, ManualRobot,
                      Robot, RobotPanel)

HERE = Path(__file__).resolve().parent
# D400 visual preset. 판(무늬 없는 나무)에서는 high_accuracy가 구멍이 가장 많았다 (유효 21% vs default 38%).
DEPTH_PRESETS = {"default": 1, "hand": 2, "high_accuracy": 3, "high_density": 4, "medium_density": 5}
CORNERS = [
    ("top_left", "좌상단", "TOP-LEFT"),
    ("top_right", "우상단", "TOP-RIGHT"),
    ("bottom_right", "우하단", "BOTTOM-RIGHT"),
    ("bottom_left", "좌하단", "BOTTOM-LEFT"),
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="장기판 4점 카메라-로봇 보정")
    p.add_argument("--output", type=Path, default=HERE / "config" / "calibration.yaml")
    p.add_argument("--robot-ns", default="dsr01", help="Doosan 드라이버 네임스페이스")
    p.add_argument("--jog-only", action="store_true", help="카메라·점 찍기 없이 로봇 패널(조그/그리퍼/홈)만 띄운다")
    p.add_argument("--tcp-offset", type=float, nargs=3, default=list(GRIPPER_TIP_OFFSET_MM),
                   metavar=("X", "Y", "Z"),
                   help="posx(컨트롤러 TCP) 기준 그리퍼 팁 오프셋 [mm, TCP 좌표계]. 기본은 플랜지→팁 "
                        f"고정 {GRIPPER_TIP_OFFSET_MM[2]:.0f} mm. 컨트롤러 TCP는 J6 플랜지 원점이어야 한다")
    p.add_argument("--gripper-cmd", action="store_true",
                   help=f"시작할 때 그리퍼에 위치 명령 {GRIPPER_TIP_POS_CMD}을 보낸다 (원래 222mm 보정 팁을 사용한 측정 조건)")
    p.add_argument("--manual-robot", action="store_true",
                   help="ROS 없이 펜던트 좌표(x y z A B C)를 직접 입력 (조그/그리퍼/홈 비활성)")
    p.add_argument("--jog-speed", type=int, default=2, choices=range(1, len(JOG_SPEEDS) + 1),
                   help=f"시작 조그 속도 단계 {JOG_SPEEDS} %% 중 번호 (기본 2)")
    p.add_argument("--home-vel", type=float, default=30.0, help="홈 이동 관절 속도 [deg/s]")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--serial", default=None, help="여러 카메라 연결 시 시리얼")
    p.add_argument("--avg-frames", type=int, default=30, help="카메라 점 확정 시 평균낼 프레임 수")
    p.add_argument("--patch", type=int, default=3, help="depth 중앙값을 낼 픽셀 반경 (3 → 7x7)")
    p.add_argument("--depth-preset", default="default", choices=list(DEPTH_PRESETS),
                   help="depth 프리셋 (depth는 교차검증·표시용, 보정 자체는 픽셀만 씀)")
    p.add_argument("--warn-rms-mm", type=float, default=5.0, help="이 값을 넘으면 경고")
    args = p.parse_args()
    if not np.allclose(args.tcp_offset, GRIPPER_TIP_OFFSET_MM, rtol=0., atol=1e-9):
        p.error('이 프로젝트는 J6 플랜지 기준 보정용 --tcp-offset 0 0 222를 사용합니다. 집기 명령용 160mm와 구분합니다.')
    return args


# --------------------------------------------------------------------------- 카메라

class Camera:
    """color에 정렬된 depth를 백그라운드 스레드로 계속 받아 최근 N프레임을 보관한다."""

    def __init__(self, args):
        import pyrealsense2 as rs

        self.rs = rs
        self.args = args
        self.serial = args.serial
        self.preset_name = args.depth_preset
        self.pipeline = None
        self._start()  # 첫 연결 실패는 바로 예외로 알린다
        self._lock = threading.Lock()
        self._color = None
        self._depths: deque[np.ndarray] = deque(maxlen=max(args.avg_frames, 5))
        self._stop = threading.Event()
        self.error: str | None = None
        self.last_frame = 0.0
        self.reconnects = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _start(self):
        """파이프라인을 연다. 재연결 때도 처음 연결한 같은 카메라(시리얼)만 연다."""
        rs, a = self.rs, self.args
        pipeline = rs.pipeline()
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
        cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
        profile = pipeline.start(cfg)
        dev = profile.get_device()
        ds = dev.first_depth_sensor()
        if ds.supports(rs.option.visual_preset):
            try:
                ds.set_option(rs.option.visual_preset, DEPTH_PRESETS[a.depth_preset])
            except RuntimeError:
                pass
        self.depth_scale = ds.get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.info = {
            "model": dev.get_info(rs.camera_info.name),
            "serial": dev.get_info(rs.camera_info.serial_number),
            "firmware": dev.get_info(rs.camera_info.firmware_version),
        }
        self.serial = self.info["serial"]
        self.pipeline = pipeline

    def _stop_pipeline(self):
        p, self.pipeline = self.pipeline, None
        if p is None:
            return
        # 죽은 장치에서 stop()이 오래 걸릴 수 있어 별도 스레드로 시도만 한다
        t = threading.Thread(target=lambda: self._safe_stop(p), daemon=True)
        t.start()
        t.join(timeout=2.0)

    @staticmethod
    def _safe_stop(p):
        try:
            p.stop()
        except RuntimeError:
            pass

    @property
    def age(self) -> float:
        """마지막 프레임 이후 경과 시간 [s]. 카메라가 끊기면 커진다."""
        return time.time() - self.last_frame

    @property
    def lost(self) -> bool:
        return self.age > 0.5

    def _run(self):
        lost_since = None
        warmup = 15  # 자동 노출 안정화
        while not self._stop.is_set():
            try:
                if self.pipeline is None:
                    self._start()
                    warmup = 15
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError as e:
                now = time.time()
                if lost_since is None:
                    lost_since = now
                    print(f"[카메라] 프레임이 끊겼다: {e} — 재연결을 시도한다 (USB 케이블/포트 확인)", flush=True)
                    with self._lock:
                        self._depths.clear()  # 끊기기 전 depth를 섞지 않는다
                if now - lost_since > 1.5:  # 장치가 다시 잡히면 파이프라인을 새로 연다
                    self._stop_pipeline()
                    self._stop.wait(1.0)
                continue
            except Exception as e:  # noqa: BLE001 — 메인 루프에 알리기 위해
                self.error = str(e)
                return
            if warmup:
                warmup -= 1
                continue
            frames = self.align.process(frames)
            d, c = frames.get_depth_frame(), frames.get_color_frame()
            if not d or not c:
                continue
            color = np.asanyarray(c.get_data()).copy()
            depth = np.asanyarray(d.get_data()).copy()
            with self._lock:
                self._color = color
                self._depths.append(depth)
            self.last_frame = time.time()
            if lost_since is not None:
                self.reconnects += 1
                print(f"[카메라] 다시 연결됐다 ({time.time() - lost_since:.1f}초 끊김)", flush=True)
                lost_since = None

    def latest_color(self):
        with self._lock:
            return None if self._color is None else self._color.copy()

    def buffered(self) -> int:
        with self._lock:
            return len(self._depths)

    def measure(self, u: int, v: int, radius: int, n_frames: int | None = None):
        """픽셀 (u, v) 주변 depth를 여러 프레임에서 모아 카메라 좌표 3D 점(m)을 구한다.

        반환: (xyz ndarray 또는 None, depth 표준편차 mm, 유효 프레임 수)
        """
        with self._lock:
            depths = list(self._depths)
        if n_frames:
            depths = depths[-n_frames:]
        if not depths:
            return None, 0.0, 0
        h, w = depths[0].shape
        y0, y1 = max(v - radius, 0), min(v + radius + 1, h)
        x0, x1 = max(u - radius, 0), min(u + radius + 1, w)
        z, z_std, n = cm.robust_depth([d[y0:y1, x0:x1] for d in depths])
        if n == 0:
            return None, 0.0, 0
        z_m = z * self.depth_scale
        xyz = self.rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], z_m)
        return np.array(xyz, dtype=float), z_std * self.depth_scale * 1000.0, n

    def norm(self, u: float, v: float) -> np.ndarray:
        """픽셀 → 왜곡을 제거한 정규화 좌표 (x/z, y/z). RealSense 왜곡 모델을 그대로 쓴다."""
        x, y, z = self.rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], 1.0)
        return np.array([x / z, y / z])

    def intrinsics_dict(self):
        i = self.intr
        return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
                "distortion_model": str(i.model).split(".")[-1], "coeffs": [float(c) for c in i.coeffs]}

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._stop_pipeline()


# --------------------------------------------------------------------------- 보정 계산/저장

def r3(v, nd=6):
    return [round(float(x), nd) for x in v]


def solve(points, fx: float):
    """PnP: 픽셀(정규화 좌표) ↔ 그리퍼 팁 base 좌표로 카메라 자세를 구한다. depth는 교차 검증에만 쓴다."""
    norm = np.array([p["norm"] for p in points])
    base = np.array([p["tip"] for p in points])
    r_cb, t_cb = cm.solve_pnp(norm, base)
    r, t = cm.invert(r_cb, t_cb)  # T_base_camera
    reproj_px = np.linalg.norm(cm.project_norm(r_cb, t_cb, base) - norm, axis=1) * fx
    n, d = cm.fit_plane(base)
    err_mm = []
    for p, b in zip(norm, base):
        hit = cm.ray_plane(t, r @ np.array([p[0], p[1], 1.0]), n, d)
        err_mm.append(np.inf if hit is None else np.linalg.norm(hit - b) * 1000)
    err_mm = np.array(err_mm)
    flat_mm = np.abs(base @ n + d) * 1000  # 네 점이 한 평면에서 벗어난 정도
    depth_err = None
    if all(p["cam"] is not None for p in points):
        cam = np.array([p["cam"] for p in points])
        depth_err = np.linalg.norm(cm.transform(r, t, cam) - base, axis=1) * 1000
    return {"R": r, "t": t, "err_mm": err_mm, "rms_mm": float(np.sqrt(np.mean(err_mm ** 2))),
            "reproj_px": reproj_px, "plane": (n, d), "flat_mm": flat_mm, "depth_err_mm": depth_err, "base": base}


def report_lines(res, names) -> list[str]:
    out = ["=== 보정 결과 (PnP) ==="]
    for i, n in enumerate(names):
        out.append(f"  {n:13s} 판 위 잔차 {res['err_mm'][i]:6.2f} mm  재투영 {res['reproj_px'][i]:5.2f} px")
    out.append(f"  RMS {res['rms_mm']:.2f} mm, 최대 {res['err_mm'].max():.2f} mm "
               f"(재투영 최대 {res['reproj_px'].max():.2f} px)")
    out.append(f"  네 팁 점의 평면 이탈 최대 {res['flat_mm'].max():.1f} mm (크면 팁 높이가 점마다 다르다)")
    t = res["t"]
    out.append(f"  카메라 위치 base ({t[0] * 1000:.0f}, {t[1] * 1000:.0f}, {t[2] * 1000:.0f}) mm")
    if res["depth_err_mm"] is not None:
        out.append("  depth 교차검증 (카메라 depth→base vs 팁, mm): "
                   + " ".join(f"{e:.1f}" for e in res["depth_err_mm"]))
    else:
        out.append("  depth 교차검증: 일부 점에 depth가 없어 생략 (PnP 결과에는 영향 없음)")
    return out


def static_tf_cmd(t, q):
    return ("ros2 run tf2_ros static_transform_publisher "
            f"--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} "
            f"--qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} "
            "--frame-id base_link --child-frame-id camera_color_optical_frame")


def save_yaml(path: Path, res, points, cam: Camera, robot_info: dict):
    r, t = res["R"], res["t"]
    q = cm.matrix_to_quat_xyzw(r)
    n, d = res["plane"]
    data = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "method": "PnP (IPPE + LM) of 4 board corners: image pixel ↔ gripper tip base position",
        "T_base_camera": {
            "parent_frame": "base_link",
            "child_frame": "camera_color_optical_frame",
            "translation": r3(t),
            "rotation_xyzw": r3(q, 8),
            "matrix": [r3(row, 8) for row in cm.to_homogeneous(r, t)],
        },
        "residual": {
            "board_rms_mm": round(res["rms_mm"], 3),
            "board_max_mm": round(float(res["err_mm"].max()), 3),
            "reproj_max_px": round(float(res["reproj_px"].max()), 3),
        },
        # 판 윗면(팁이 닿은 면) 평면, base 좌표계: normal · p + d = 0
        "board_plane_base": {"normal": r3(n, 8), "d": round(float(d), 6)},
        "camera": {**cam.info, "color_intrinsics": cam.intrinsics_dict(), "depth_preset": cam.preset_name},
        "robot": robot_info,
        "corners": [
            {
                "name": p["name"],
                "pixel": [int(p["pixel"][0]), int(p["pixel"][1])],
                "robot_posx": r3(p["posx"], 4),
                "base_xyz": r3(p["tip"]),
                "error_mm": round(float(e), 3),
                "reproj_px": round(float(rp), 3),
                "camera_depth_xyz": None if p["cam"] is None else r3(p["cam"]),
                "gripper_stroke": p.get("gripper_stroke"),
            }
            for p, e, rp in zip(points, res["err_mm"], res["reproj_px"])
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        shutil.copy2(path, path.with_suffix(".yaml.bak"))
    header = (
        "# 장기판 4점 카메라-로봇 보정 결과 (calibrate_board.py가 생성)\n"
        "# 단위: m (robot_posx만 mm/deg). p_base = R * p_cam + t\n"
        "# 픽셀 → base: 정규화 광선을 T_base_camera로 돌려 board_plane_base와 교차시킨다 (depth 불필요)\n"
        f"# static TF: {static_tf_cmd(t, q)}\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, default_flow_style=None, width=120)
    return static_tf_cmd(t, q)


# --------------------------------------------------------------------------- 화면

def put(img, text, org, color=(255, 255, 255), scale=0.6, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_zoom(img, center, half=20, scale=8):
    """center 주변을 확대해 위쪽 모서리(center 반대편)에 붙인다 (정밀 클릭용)."""
    if center is None:
        return
    h, w = img.shape[:2]
    u, v = center
    pad = cv2.copyMakeBorder(img, half, half, half, half, cv2.BORDER_CONSTANT)
    crop = pad[v:v + 2 * half + 1, u:u + 2 * half + 1]
    zoom = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    zh, zw = zoom.shape[:2]
    c = half * scale + scale // 2
    cv2.line(zoom, (c, 0), (c, zh), (0, 255, 255), 1)
    cv2.line(zoom, (0, c), (zw, c), (0, 255, 255), 1)
    if zh + 10 < h and zw + 10 < w:
        x0 = 10 if u > w // 2 else w - zw - 10
        img[10:10 + zh, x0:x0 + zw] = zoom
        cv2.rectangle(img, (x0 - 1, 9), (x0 + zw, 10 + zh), (255, 255, 255), 1)


# --------------------------------------------------------------------------- 보정 UI (Tk)

class App:
    """왼쪽: 카메라 영상 + 단계 안내/로그, 오른쪽: 로봇 패널(조그/그리퍼/홈)."""

    def __init__(self, args, cam: Camera, robot, robot_info: dict):
        self.args, self.cam, self.robot, self.robot_info = args, cam, robot, robot_info
        self.root = tk.Tk()
        self.root.title("장기판 카메라-로봇 보정")
        if isinstance(robot, ManualRobot):
            robot.parent = self.root

        self.points: list[dict] = []
        self.result = None
        self.saved = False
        self.idx, self.phase = 0, "cam"
        self.sel = self.mouse = self.verify = None
        self.busy = False
        self.running = True
        self.warned_gripper_unknown = False
        self._photo = None

        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.scale = min(1.0, (sw - 470) / args.width, (sh - 330) / args.height)
        self.disp = (int(args.width * self.scale), int(args.height * self.scale))
        self._build()

        self.keys = KeyTracker(self.root, self.on_press, self.on_release)
        self.root.bind("<FocusOut>", lambda e: self.root.after(50, self._check_focus))
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        signal.signal(signal.SIGINT, lambda *_: self.root.after(0, self.quit))
        self.refresh_step()
        self.root.after(15, self.tick)

    # ------------------------------------------------------------------ 화면 구성
    def _build(self):
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(main, width=self.disp[0], height=self.disp[1], bg="black",
                                highlightthickness=0, cursor="crosshair")
        self.canvas.grid(row=0, column=0, sticky="nw")
        self.img_item = self.canvas.create_image(0, 0, anchor="nw")
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Leave>", lambda e: setattr(self, "mouse", None))
        self.canvas.bind("<Button-1>", self.on_click)

        bottom = tk.Frame(main, padx=8, pady=6)
        bottom.grid(row=1, column=0, sticky="nsew")
        main.rowconfigure(1, weight=1)
        left = tk.Frame(bottom, width=420)
        left.pack(side="left", fill="y")
        wl = 420
        self.lbl_step = tk.Label(left, font=("TkDefaultFont", 14, "bold"), wraplength=wl, justify="left", anchor="w")
        self.lbl_step.pack(fill="x")
        self.lbl_hint = tk.Label(left, wraplength=wl, justify="left", anchor="w", fg="#333")
        self.lbl_hint.pack(fill="x", pady=(2, 4))
        fc = tk.Frame(left)
        fc.pack(fill="x")
        self.lbl_corners = [tk.Label(fc, anchor="w", font=("TkFixedFont", 10)) for _ in CORNERS]
        for i, lbl in enumerate(self.lbl_corners):
            lbl.grid(row=i // 2, column=i % 2, sticky="w", padx=(0, 16))
        self.lbl_sel = tk.Label(left, anchor="w", justify="left", font=("TkFixedFont", 10))
        self.lbl_sel.pack(fill="x", pady=4)
        fb = tk.Frame(left)
        fb.pack(fill="x")
        tk.Button(fb, text="확정 (SPACE)", takefocus=0, command=self.confirm).pack(side="left", expand=True, fill="x")
        tk.Button(fb, text="되돌리기 (U)", takefocus=0, command=self.undo).pack(side="left", expand=True, fill="x")
        tk.Button(fb, text="종료 (ESC)", takefocus=0, command=self.ask_quit).pack(side="left", expand=True, fill="x")

        fl = tk.Frame(bottom)
        fl.pack(side="left", fill="both", expand=True, padx=(10, 0))
        self.txt = tk.Text(fl, height=10, font=("TkFixedFont", 9), state="disabled", takefocus=0, wrap="word")
        sb = tk.Scrollbar(fl, command=self.txt.yview, takefocus=0)
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self.panel = RobotPanel(main, self.root, self.robot, self.args.tcp_offset, self.log,
                                self.args.jog_speed, self.args.home_vel, self.robot_info.get("tcp_name"))
        self.panel.grid(row=0, column=1, rowspan=2, sticky="n")

    def log(self, msg: str):
        print(msg, flush=True)
        self.txt.configure(state="normal")
        self.txt.insert("end", msg + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def refresh_step(self):
        i = self.idx
        if self.phase == "cam":
            ko = CORNERS[i][1]
            self.lbl_step.config(text=f"[{i + 1}/4 {ko}] 카메라 점 찍기", fg="#1565c0")
            self.lbl_hint.config(text=f"화면에서 {ko} 점을 정확히 클릭하고 SPACE (확대창 참고, 방향키로 1픽셀). "
                                      "점이 팔에 가리지 않게 한다. 조그는 언제든 쓸 수 있다.")
        elif self.phase == "robot":
            ko = CORNERS[i][1]
            hint = (f"조그로 팁을 {ko} 점에 맞춘다 (가까워지면 속도 1). 그리퍼는 {GRIPPER_TIP_POS_CMD}. "
                    "닿기 직전에 SPACE.")
            pred = self._predict_last()
            if pred is not None:
                hint += f"\n3점 추정 위치 ≈ ({pred[0]:.1f}, {pred[1]:.1f}, {pred[2]:.1f}) mm"
            self.lbl_step.config(text=f"[{i + 1}/4 {ko}] 그리퍼 팁 맞추기", fg="#e65100")
            self.lbl_hint.config(text=hint)
        else:
            r = self.result
            self.lbl_step.config(text=f"완료 — RMS {r['rms_mm']:.2f} mm, 최대 {r['err_mm'].max():.2f} mm (저장됨)",
                                 fg="#2e7d32")
            self.lbl_hint.config(text="화면을 클릭하면 그 점의 base 좌표(판 평면 위)가 나온다. 팁을 그 점에 대고 "
                                      "SPACE를 누르면 오차를 보여준다. 판 안쪽 점으로 확인한다. U: 마지막 점 다시.")
        for k, lbl in enumerate(self.lbl_corners):
            mark = "  "
            if k < len(self.points):
                mark = "✔ " if self.points[k]["tip"] is not None else "◐ "
            elif k == i and self.phase != "done":
                mark = "▶ "
            lbl.config(text=f"{mark}{k + 1}. {CORNERS[k][1]}")

    def _predict_last(self):
        """앞 3점으로 4번째 점 위치를 아핀 근사 (이동 참고용, 수 mm 오차)."""
        if len(self.points) != 4 or self.points[3]["tip"] is not None:
            return None
        f = cm.affine_2d(np.array([p["norm"] for p in self.points[:3]]),
                         np.array([p["tip"] for p in self.points[:3]]))
        return f(self.points[3]["norm"]) * 1000

    def pixel_to_base(self, u, v):
        """보정 결과로 픽셀 → 판 평면 위 base 점 (m). depth를 쓰지 않는다."""
        r, t = self.result["R"], self.result["t"]
        x, y = self.cam.norm(u, v)
        return cm.ray_plane(t, r @ np.array([x, y, 1.0]), *self.result["plane"])

    # ------------------------------------------------------------------ 입력
    def _img_xy(self, e):
        u = min(max(int(e.x / self.scale), 0), self.args.width - 1)
        v = min(max(int(e.y / self.scale), 0), self.args.height - 1)
        return u, v

    def on_motion(self, e):
        self.mouse = self._img_xy(e)

    def on_click(self, e):
        self.root.focus_set()  # 그리퍼 입력창 포커스 해제
        if self.busy:
            return
        if self.phase == "cam":
            self.sel = self._img_xy(e)
        elif self.phase == "done":
            self.verify = self._img_xy(e)

    def on_press(self, k):
        try:
            in_entry = isinstance(self.root.focus_get(), tk.Entry)
        except (KeyError, tk.TclError):
            in_entry = False
        if in_entry:  # 그리퍼 값 입력 중: ESC만 받아 입력창을 빠져나간다
            if k == "escape":
                self.root.focus_set()
            return
        if k in ("up", "down", "left", "right"):
            self.nudge(k)
        elif self.busy and k in ("w", "s", "a", "d", "r", "f"):
            return  # 카메라 측정 중에는 팔을 움직이지 않는다
        elif self.panel.key_press(k):
            return
        elif k in ("space", "return", "kp_enter"):
            self.confirm()
        elif k == "u":
            self.undo()
        elif k == "escape":
            self.ask_quit()

    def on_release(self, k):
        self.panel.key_release(k)

    def _check_focus(self):
        try:
            focused = self.root.focus_get()
        except (KeyError, tk.TclError):
            focused = None
        if focused is None:  # 창 밖으로 포커스가 나가면 뗌 이벤트를 못 받으므로 정지
            self.stop_all()

    def stop_all(self):
        self.keys.reset()
        self.panel.stop_all()

    def nudge(self, k):
        if self.phase != "cam" or not self.sel or self.busy:
            return
        du, dv = {"up": (0, -1), "down": (0, 1), "left": (-1, 0), "right": (1, 0)}[k]
        self.sel = (min(max(self.sel[0] + du, 0), self.args.width - 1),
                    min(max(self.sel[1] + dv, 0), self.args.height - 1))

    # ------------------------------------------------------------------ 단계 진행
    def confirm(self):
        if self.busy:
            return
        if self.phase == "cam":
            if not self.sel:
                self.log("먼저 화면에서 점을 클릭한다.")
                return
            if self.cam.lost:
                self.log("카메라가 끊겨 있다. 화면이 다시 움직이면 점을 다시 확인하고 찍는다.")
                return
            if self.panel.moving:
                self.stop_all()
            self.busy = True
            self.lbl_step.config(text="카메라 점 측정 중…")
            # 클릭 이후의 새 프레임으로 버퍼를 채운 뒤 측정
            self.root.after(int(1000 * self.args.avg_frames / self.args.fps), self._finish_cam)
        elif self.phase == "robot":
            ok = self.panel.gripper_ok()
            if ok is False:
                s = self.robot.gripper_stroke
                if not messagebox.askyesno("그리퍼", f"그리퍼 명령값이 {s}이다. 팁 오프셋은 {GRIPPER_TIP_POS_CMD} "
                                           "기준이라 위치가 틀릴 수 있다. 그래도 기록할까?", parent=self.root):
                    return
            elif ok is None and not self.warned_gripper_unknown and self.robot.can_jog:
                self.warned_gripper_unknown = True
                self.log(f"※ 그리퍼 개도를 확인할 수 없다. {GRIPPER_TIP_POS_CMD}인지 확인한다 (G 키로 설정).")
            self.busy = True
            if self.panel.moving:
                self.stop_all()
                self.root.after(400, self._record_robot)  # 정지·안정 대기
            else:
                self._record_robot()
        else:
            self._verify_tip()

    def _finish_cam(self):
        self.busy = False
        if self.cam.lost:  # 측정 대기 중에 끊기면 멈춘 프레임으로 기록하지 않는다
            self.log("측정 중에 카메라가 끊겼다. 다시 연결되면 다시 찍는다.")
            self.refresh_step()
            return
        u, v = self.sel
        # 보정에는 픽셀만 쓴다. depth는 잡히면 교차검증용으로 같이 남긴다.
        xyz, zstd, n = self.cam.measure(u, v, self.args.patch)
        if xyz is not None and n < max(5, self.cam.buffered() // 2):
            xyz = None
        self.points.append({"name": CORNERS[self.idx][0], "pixel": (u, v), "norm": self.cam.norm(u, v),
                            "cam": xyz, "posx": None, "tip": None})
        dtxt = f"depth ({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:.4f}) m" if xyz is not None else "depth 없음(괜찮음)"
        self.log(f"[{self.idx + 1}/4 {CORNERS[self.idx][1]}] 픽셀 ({u},{v}) 기록, {dtxt}")
        self.phase, self.sel = "robot", None
        self.refresh_step()

    def _record_robot(self):
        self.busy = False
        try:
            posx = self.robot.posx()
        except RuntimeError as e:
            self.log(f"[오류] 로봇 자세 읽기 실패: {e}")
            return
        tip = cm.tip_position_m(posx, self.args.tcp_offset)
        p = self.points[self.idx]
        p["posx"], p["tip"] = posx, tip
        p["gripper_stroke"] = self.robot.gripper_stroke
        self.log(f"[{self.idx + 1}/4 {CORNERS[self.idx][1]}] 팁 base ({tip[0] * 1000:.1f}, {tip[1] * 1000:.1f}, "
                 f"{tip[2] * 1000:.1f}) mm  posx {np.round(posx, 2).tolist()}")
        self.idx += 1
        if self.idx < 4:
            self.phase = "cam"
        else:
            self._compute()
        self.refresh_step()

    def _compute(self):
        try:
            self.result = solve(self.points, self.cam.intr.fx)
        except ValueError as e:
            self.log(f"[오류] 계산 실패: {e}. 마지막 점을 다시 기록한다.")
            self.idx = 3
            self.points[3]["posx"] = self.points[3]["tip"] = None
            return
        for line in report_lines(self.result, [c[0] for c in CORNERS]):
            self.log(line)
        if self.result["rms_mm"] > self.args.warn_rms_mm:
            self.log(f"※ RMS가 {self.args.warn_rms_mm} mm를 넘는다. 잔차가 큰 점을 U로 되돌려 다시 찍는 것을 권장.")
        tf_cmd = save_yaml(self.args.output, self.result, self.points, self.cam, self.robot_info)
        self.saved = True
        self.log(f"[저장] {self.args.output}")
        self.log(f"static TF: {tf_cmd}")
        self.phase = "done"

    def _verify_tip(self):
        if not self.verify:
            self.log("먼저 확인할 점을 클릭한다 (팁을 대기 전에).")
            return
        b = self.pixel_to_base(*self.verify)
        if b is None:
            self.log("클릭한 점이 판 평면과 만나지 않는다.")
            return
        if self.panel.moving:
            self.stop_all()
        try:
            tip = cm.tip_position_m(self.robot.posx(), self.args.tcp_offset)
        except RuntimeError as e:
            self.log(f"[오류] {e}")
            return
        d = (tip - b) * 1000
        self.log(f"검증: 카메라→base ({b[0] * 1000:.1f}, {b[1] * 1000:.1f}, {b[2] * 1000:.1f}) / 팁 "
                 f"({tip[0] * 1000:.1f}, {tip[1] * 1000:.1f}, {tip[2] * 1000:.1f}) → 차이 "
                 f"({d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f}) |{np.linalg.norm(d):.1f}| mm")

    def undo(self):
        if self.busy:
            return
        if self.phase == "robot":
            self.points.pop()
            self.phase = "cam"
            self.log("↶ 카메라 점을 다시 찍는다.")
        elif self.phase == "cam" and self.idx > 0:
            self.idx -= 1
            self.points[-1]["posx"] = self.points[-1]["tip"] = None
            self.phase = "robot"
            self.log("↶ 직전 로봇 자세를 다시 기록한다.")
        elif self.phase == "done":
            self.idx = 3
            self.points[3]["posx"] = self.points[3]["tip"] = None
            self.phase, self.verify, self.result = "robot", None, None
            self.log("↶ 마지막 로봇 자세를 다시 기록한다 (저장 파일은 다시 계산하면 덮어씀).")
        else:
            return
        self.refresh_step()

    # ------------------------------------------------------------------ 루프
    def tick(self):
        if not self.running:
            return
        self.panel.tick()
        if self.cam.error:
            self.log(f"[오류] 카메라: {self.cam.error}")
            self.quit()
            return
        img = self.cam.latest_color()
        if img is not None:
            self._draw(img)
            if self.scale != 1.0:
                img = cv2.resize(img, self.disp, interpolation=cv2.INTER_AREA)
            self._photo = ImageTk.PhotoImage(Image.fromarray(img[:, :, ::-1]))
            self.canvas.itemconfigure(self.img_item, image=self._photo)
        self._update_sel_label()
        self.root.after(15, self.tick)

    def _draw(self, img):
        if self.cam.lost:
            h, w = img.shape[:2]
            img[:] = (img * 0.4).astype(np.uint8)
            cv2.rectangle(img, (0, h // 2 - 40), (w, h // 2 + 40), (0, 0, 160), -1)
            put(img, f"CAMERA LOST {self.cam.age:.0f}s - reconnecting (check USB cable)", (30, h // 2 + 12),
                (255, 255, 255), 1.0, 2)
        for i, p in enumerate(self.points):
            col = (0, 220, 0) if p["tip"] is not None else (0, 165, 255)
            cv2.circle(img, p["pixel"], 7, col, 2)
            put(img, f"{i + 1} {CORNERS[i][2]}", (p["pixel"][0] + 9, p["pixel"][1] - 9), col, 0.6)
        if len(self.points) >= 2:
            cv2.polylines(img, [np.array([p["pixel"] for p in self.points], np.int32)], len(self.points) == 4,
                          (0, 220, 0), 1)
        if self.sel and self.phase == "cam":
            cv2.drawMarker(img, self.sel, (0, 255, 255), cv2.MARKER_CROSS, 28, 1)
        if self.verify and self.phase == "done":
            cv2.drawMarker(img, self.verify, (255, 0, 255), cv2.MARKER_CROSS, 28, 1)
        draw_zoom(img, self.mouse or self.sel)

    def _update_sel_label(self):
        txt = ""
        if self.phase == "cam" and self.sel:
            xyz, _, _ = self.cam.measure(*self.sel, self.args.patch, n_frames=5)
            txt = f"선택 ({self.sel[0]},{self.sel[1]})  " + (
                f"depth {xyz[2] * 1000:.0f} mm" if xyz is not None else "depth 없음 (보정엔 무관)")
        elif self.phase == "done" and self.verify:
            b = self.pixel_to_base(*self.verify)
            if b is not None:
                b = b * 1000
                txt = f"클릭 ({self.verify[0]},{self.verify[1]}) → base ({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}) mm"
            else:
                txt = "클릭한 점이 판 평면 밖"
        self.lbl_sel.config(text=txt)

    def ask_quit(self):
        self.stop_all()
        msg = "보정을 종료할까?" if self.saved else "아직 저장하지 않았다. 종료할까?"
        if messagebox.askyesno("종료", msg, parent=self.root):
            self.quit()

    def quit(self):
        if not self.running:
            return
        self.running = False
        self.keys.reset()
        self.panel.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# --------------------------------------------------------------------------- 메인

def connect_robot(args):
    robot = ManualRobot() if args.manual_robot else Robot(args.robot_ns)
    print(f"[로봇] {robot.posx_srv} 대기 중…")
    if not robot.wait(10.0):
        robot.close()
        ws = HERE.parents[2]
        sys.exit(f"[오류] {robot.posx_srv} 서비스가 없다. 다른 터미널에서 로봇 드라이버를 먼저 띄운다\n"
                 "  (<로봇 IP>는 실제 주소로 바꾼다):\n"
                 f"  source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash\n"
                 "  ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<로봇 IP> "
                 "port:=12345 name:=dsr01\n  (ROS 없이 하려면 --manual-robot)")
    if robot.can_jog and not robot.jog_ready():
        robot.close()
        sys.exit("[오류] motion/jog_multi 서비스가 없다.")
    tcp_name = robot.tcp_name()
    print(f"[로봇] 현재 TCP: {tcp_name or '(알 수 없음)'}, 팁 오프셋 {args.tcp_offset} mm")
    if not args.manual_robot and tcp_name != '':
        robot.close()
        sys.exit('[오류] 컨트롤러 TCP가 기본 J6 플랜지 원점인지 확인할 수 없습니다. '
                 '별도의 그리퍼 TCP를 사용하지 않습니다.')
    print('  기준: 컨트롤러 TCP=J6 플랜지 원점, 보정용 팁 연장=로컬 +Z 222mm. 집기용 160mm는 별도입니다.')
    if args.gripper_cmd and robot.can_jog:
        time.sleep(0.5)  # 그리퍼 노드와 토픽 연결 대기
        robot.gripper_position(GRIPPER_TIP_POS_CMD)
        print(f"[그리퍼] 위치 명령 {GRIPPER_TIP_POS_CMD} 전송")
    return robot, tcp_name


def main():
    args = parse_args()

    if args.jog_only:
        if args.manual_robot:
            sys.exit("[오류] --jog-only는 ROS 로봇 연결이 필요하다 (--manual-robot과 함께 쓸 수 없다).")
        robot, tcp_name = connect_robot(args)
        try:
            app = JogApp(robot, args.tcp_offset, args.jog_speed, args.home_vel, tcp_name)
            signal.signal(signal.SIGINT, lambda *_: app.root.after(0, app.quit))
            app.run()
        finally:
            robot.stop_sync()
            robot.close()
        return

    print("[카메라] RealSense 연결 중…")
    try:
        cam = Camera(args)
    except RuntimeError as e:
        sys.exit(f"[오류] RealSense 시작 실패: {e}\n  다른 프로그램(realsense-viewer, cam_visualizer, ROS 드라이버)이 쓰는지 확인")
    print(f"[카메라] {cam.info['model']} S/N {cam.info['serial']}, {args.width}x{args.height}@{args.fps}")

    robot = None
    try:
        robot, tcp_name = connect_robot(args)
        robot_info = {"namespace": args.robot_ns, "posx_source": robot.posx_srv, "ref": "DR_BASE",
                      "tcp_name": tcp_name, "tcp_offset_mm": r3(args.tcp_offset, 3),
                      "gripper_position_cmd": GRIPPER_TIP_POS_CMD}
        print("키 조작" + __doc__.split("키 조작", 1)[1])
        deadline = time.time() + 15
        while cam.buffered() < 5 and cam.error is None and time.time() < deadline:
            time.sleep(0.05)
        app = App(args, cam, robot, robot_info)
        app.run()
        if not app.saved:
            print("보정을 끝내지 않고 종료했다. 파일은 바뀌지 않았다.")
        elif app.result is None:
            print(f"되돌린 뒤 다시 계산하지 않고 종료했다. {args.output}에는 직전 결과가 남아 있다.")
    finally:
        if robot is not None:
            robot.stop_sync()  # 어떤 경로로 끝나든 조그 정지
            robot.close()
        cam.close()


if __name__ == "__main__":
    main()
