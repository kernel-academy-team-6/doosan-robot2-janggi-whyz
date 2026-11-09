#!/usr/bin/env python3
"""RealSense 컬러/깊이 미리보기, 원본 RGB-D + MP4 녹화, RGB-D 사진 촬영."""
from __future__ import annotations

import argparse
import csv
import json
import queue
import re
import shutil
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk, PngImagePlugin

ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT / "src/janggi/calibration/config/calibration.yaml"


def now():
    return datetime.now().astimezone().isoformat()


def new_folder(root, kind):
    path = Path(root) / f"{kind}_{datetime.now().astimezone():%Y%m%d_%H%M%S_%f}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def copy_calibration(folder):
    if CALIBRATION.is_file():
        shutil.copy2(CALIBRATION, folder / "board_calibration.yaml")


def start_recording_stream(folder, open_camera):
    """SDK 세대별 파일 형식 차이(.db3 / .bag)를 실제 오류에 따라 처리한다."""
    path = folder / "rgbd.db3"
    try:
        open_camera(path)
    except RuntimeError as exc:
        required = re.search(r"output file must have\s+(\.bag|\.db3)\s+extension", str(exc), re.I)
        if required is None or required.group(1).lower() == path.suffix:
            raise
        path = path.with_suffix(required.group(1).lower())
        open_camera(path)
    return path


def save_snapshot(root, color, depth, metadata, frame_info):
    folder = Path(root) / "photos"
    folder.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now().astimezone():%Y%m%d_%H%M%S_%f}"
    color_path = folder / f"color_{stamp}.png"
    depth_path = folder / f"depth_{stamp}.png"
    # 별도 JSON 없이도 깊이 단위·카메라 파라미터는 PNG 텍스트에 보존한다.
    info = PngImagePlugin.PngInfo()
    info.add_text("capture_metadata", json.dumps({**metadata, "captured_at": now(), "frame": frame_info,
                  "depth_units": "uint16; meters = value * depth_scale_m"}, ensure_ascii=False))
    Image.fromarray(cv2.cvtColor(color, cv2.COLOR_BGR2RGB)).save(color_path, pnginfo=info)
    Image.fromarray(depth).save(depth_path, pnginfo=info)
    return color_path


class Recording:
    """MP4와 SDK 원본 기록의 메타데이터. 카메라 스레드에서만 사용한다."""

    def __init__(self, folder, size, fps, metadata, recording_file="rgbd.db3"):
        self.folder, self.frames = folder, 0
        self.started = time.monotonic()
        self.metadata = {**metadata, "started_at": now(), "status": "recording",
                         "mp4_fps": fps, "recording_file": recording_file,
                         "note": "RGB-D recording: native unaligned streams; MP4: acquired color frames at fixed FPS"}
        self.video = cv2.VideoWriter(str(folder / "color.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self.video.isOpened():
            self.video.release()
            raise RuntimeError("MP4 인코더를 열 수 없습니다 (OpenCV mp4v).")
        try:
            self.timestamps = (folder / "frames.csv").open("w", newline="", encoding="utf-8")
            self.csv = csv.DictWriter(self.timestamps, fieldnames=["index", "elapsed_s", "color_number",
                                      "color_timestamp_ms", "depth_number", "depth_timestamp_ms", "timestamp_domain"])
            self.csv.writeheader()
            write_json(folder / "metadata.json", self.metadata)
            copy_calibration(folder)
        except Exception:
            self.video.release()
            if hasattr(self, "timestamps"):
                self.timestamps.close()
            raise

    def append(self, color, info):
        self.video.write(color)
        self.csv.writerow({"index": self.frames, "elapsed_s": f"{time.monotonic() - self.started:.6f}", **info})
        self.frames += 1
        if self.frames % 30 == 0:
            self.timestamps.flush()

    def finish(self, error=None):
        self.video.release()
        self.timestamps.close()
        self.metadata.update(ended_at=now(), frames=self.frames,
                             elapsed_s=round(time.monotonic() - self.started, 3),
                             status="interrupted" if error else "complete", error=error)
        write_json(self.folder / "metadata.json", self.metadata)


class CameraWorker(threading.Thread):
    def __init__(self, args):
        super().__init__(daemon=True)
        self.args = args
        self.commands, self.events = queue.Queue(), queue.Queue()
        self.lock = threading.Lock()
        self.latest = None
        self.pipe = self.recording = None
        self.metadata = {}
        self.last = None
        self.running = True

    def open_camera(self, recording_path=None):
        rs, a = self.rs, self.args
        cfg = rs.config()
        if a.serial:
            cfg.enable_device(a.serial)
        cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
        cfg.enable_stream(rs.stream.depth, a.depth_width, a.depth_height, rs.format.z16, a.fps)
        if recording_path is not None:
            cfg.enable_record_to_file(str(recording_path))
        pipe = rs.pipeline()
        try:
            profile = pipe.start(cfg)
        except Exception:
            try:
                pipe.stop()
            except RuntimeError:
                pass
            raise
        self.pipe = pipe
        self.align = rs.align(rs.stream.color)
        dev = profile.get_device()
        color = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth = profile.get_stream(rs.stream.depth).as_video_stream_profile()

        def intrinsics(stream):
            i = stream.get_intrinsics()
            return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy,
                        ppx=i.ppx, ppy=i.ppy, model=str(i.model), coeffs=list(i.coeffs))

        extr = depth.get_extrinsics_to(color)
        self.metadata = dict(device=dev.get_info(rs.camera_info.name),
                             serial=dev.get_info(rs.camera_info.serial_number),
                             depth_scale_m=dev.first_depth_sensor().get_depth_scale(),
                             color_intrinsics=intrinsics(color), depth_intrinsics=intrinsics(depth),
                             depth_to_color=dict(rotation=list(extr.rotation), translation_m=list(extr.translation)),
                             fps=a.fps, aligned_depth_intrinsics="color_intrinsics",
                             preview_max_depth_m=a.max_depth)
        self.last = None

    def close_camera(self):
        if self.pipe is not None:
            pipe, self.pipe = self.pipe, None
            pipe.stop()  # SDK recorder를 닫고 원본 녹화 파일을 확정한다.

    def stop_recording(self, restart=True, error=None):
        try:
            self.close_camera()
        except Exception as exc:
            error = error or str(exc)
            raise
        finally:
            if self.recording is not None:
                recording, self.recording = self.recording, None
                recording.finish(error)
                self.events.put(("saved", str(recording.folder)))
        if restart:
            self.open_camera()

    def handle(self, command, root):
        if command == "quit":
            self.running = False
        elif command == "record":
            if self.recording is not None:
                return
            if shutil.disk_usage(root).free < 1024**3:
                self.events.put(("notice", "녹화하려면 최소 1 GB의 여유 공간이 필요합니다."))
                return
            folder = new_folder(root, "video")
            self.close_camera()
            # SDK 녹화 대상은 pipeline 시작 전에 정하므로 녹화 경계에서 스트림을 다시 연다.
            path = start_recording_stream(folder, self.open_camera)
            self.recording = Recording(folder, (self.args.width, self.args.height), self.args.fps,
                                       self.metadata, recording_file=path.name)
            self.events.put(("recording", str(folder)))
        elif command == "stop":
            if self.recording is not None:
                self.stop_recording()
        elif command == "photo":
            if self.last is None:
                self.events.put(("notice", "아직 프레임이 없습니다. 잠시 후 촬영하세요."))
                return
            color, depth, info = self.last
            folder = save_snapshot(root, color, depth, self.metadata, info)
            self.events.put(("photo", str(folder)))

    def run(self):
        error = None
        try:
            import pyrealsense2 as rs
            self.rs = rs
            self.open_camera()
            self.events.put(("ready", self.metadata["device"]))
            t0, n, fps, disk_check = time.monotonic(), 0, 0.0, 0.0
            while self.running:
                try:
                    command, root = self.commands.get_nowait()
                except queue.Empty:
                    command = None
                if command:
                    self.handle(command, root)
                    if not self.running:
                        break
                frames = self.pipe.wait_for_frames(3000)
                aligned = self.align.process(frames)
                cf, df = aligned.get_color_frame(), aligned.get_depth_frame()
                if not cf or not df:
                    continue
                color = np.asanyarray(cf.get_data()).copy()
                depth = np.asanyarray(df.get_data()).copy()
                info = dict(color_number=cf.get_frame_number(), color_timestamp_ms=cf.get_timestamp(),
                            depth_number=df.get_frame_number(), depth_timestamp_ms=df.get_timestamp(),
                            timestamp_domain=str(cf.get_frame_timestamp_domain()))
                self.last = color, depth, info
                if self.recording:
                    self.recording.append(color, info)
                n += 1
                elapsed = time.monotonic() - t0
                if elapsed >= 1.0:
                    fps, n, t0 = n / elapsed, 0, time.monotonic()
                scaled = cv2.convertScaleAbs(depth, alpha=255 * self.metadata["depth_scale_m"] / self.args.max_depth)
                visual = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                visual[depth == 0] = 0
                with self.lock:
                    self.latest = color, visual, fps
                if self.recording and time.monotonic() - disk_check > 1:
                    disk_check = time.monotonic()
                    if shutil.disk_usage(self.recording.folder).free < 1024**3:
                        self.stop_recording()
                        self.events.put(("notice", "남은 공간이 1 GB 미만이라 녹화를 종료했습니다."))
        except Exception as exc:
            error = str(exc)
            self.events.put(("error", error))
        finally:
            try:
                self.stop_recording(restart=False, error=error)
            except Exception as exc:
                self.events.put(("error", f"파일 마무리 실패: {exc}"))
            self.events.put(("closed", None))


class RecorderUI:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.worker = CameraWorker(args)
        self.closing = self.busy = self.recording = False
        self.started = 0
        self.output = tk.StringVar(value=str(args.output.expanduser().resolve()))
        self.status = tk.StringVar(value="카메라 연결 중… 다른 카메라 프로그램은 종료해 주세요.")
        self.stats = tk.StringVar()
        root.title("RealSense RGB-D 녹화 / 촬영")
        root.geometry("1280x650")
        root.minsize(800, 480)
        controls = ttk.Frame(root, padding=10)
        controls.pack(fill="x")
        ttk.Label(controls, text="저장 폴더").pack(side="left")
        self.entry = ttk.Entry(controls, textvariable=self.output)
        self.entry.pack(side="left", fill="x", expand=True, padx=8)
        self.browse = ttk.Button(controls, text="폴더 선택", command=self.choose_folder)
        self.browse.pack(side="left")
        previews = ttk.Frame(root, padding=10)
        previews.pack(fill="both", expand=True)
        previews.rowconfigure(0, weight=1)
        previews.columnconfigure((0, 1), weight=1, uniform="preview")
        self.canvases = []
        for column, title in enumerate(("컬러 · 저장되는 원본 화면", f"깊이 · 컬러에 정렬 / 0–{args.max_depth:g} m")):
            pane = ttk.LabelFrame(previews, text=title, padding=4)
            pane.grid(row=0, column=column, sticky="nsew", padx=3)
            # Canvas는 이미지 크기를 위젯의 요청 크기로 전파하지 않는다.
            # Label에 매 프레임 이미지를 넣으면 여백까지 다음 크기에 반영되어 계속 커질 수 있다.
            canvas = tk.Canvas(pane, width=1, height=1, background="#171c23",
                               highlightthickness=0, borderwidth=0)
            canvas.pack(fill="both", expand=True)
            canvas.image_item = canvas.create_image(0, 0, anchor="center")
            self.canvases.append(canvas)
        row = ttk.Frame(root, padding=10)
        row.pack(fill="x")
        self.record_button = ttk.Button(row, text="● 녹화 시작 (R)", command=self.toggle_record, state="disabled")
        self.record_button.pack(side="left", padx=4)
        self.photo_button = ttk.Button(row, text="사진 촬영 (Space)", command=lambda: self.send("photo"), state="disabled")
        self.photo_button.pack(side="left", padx=4)
        ttk.Button(row, text="종료 (Esc)", command=self.quit).pack(side="right")
        ttk.Label(row, textvariable=self.stats).pack(side="left", padx=14)
        ttk.Label(root, textvariable=self.status, padding=10, wraplength=1200).pack(fill="x")
        root.bind("<r>", self.record_key)
        root.bind("<R>", self.record_key)
        root.bind("<space>", self.photo_key)
        root.bind("<Escape>", lambda e: self.quit())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.worker.start()
        root.after(40, self.tick)

    def choose_folder(self):
        folder = filedialog.askdirectory(initialdir=self.output.get(), parent=self.root)
        if folder:
            self.output.set(folder)

    def record_key(self, event):
        if self.root.focus_get() is not self.entry:
            self.toggle_record()
            return "break"

    def photo_key(self, event):
        if self.root.focus_get() is not self.entry:
            self.send("photo")
            return "break"

    def send(self, command):
        if self.closing or not self.worker.is_alive() or self.busy:
            return
        if str(self.photo_button["state"]) == "disabled":
            return
        root = Path(self.output.get()).expanduser().resolve()
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("저장 폴더 오류", str(exc), parent=self.root)
            return
        self.worker.commands.put((command, root))
        if command in ("record", "stop", "photo"):
            self.busy = True
            self.record_button.configure(state="disabled")
            self.photo_button.configure(state="disabled")
            self.status.set("녹화 준비 중…" if command == "record" else
                            "녹화 파일 마무리 중…" if command == "stop" else "사진 저장 중…")

    def toggle_record(self):
        self.send("stop" if self.recording else "record")

    def quit(self):
        if self.closing:
            return
        self.closing = True
        self.status.set("카메라 종료 및 파일 저장 중… 창이 닫힐 때까지 기다려 주세요.")
        self.record_button.configure(state="disabled")
        self.photo_button.configure(state="disabled")
        self.worker.commands.put(("quit", None))

    def tick(self):
        while True:
            try:
                kind, value = self.worker.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("ready", "recording", "saved", "photo", "notice"):
                self.busy = False
                if not self.closing:
                    self.record_button.configure(state="normal")
                    self.photo_button.configure(state="normal")
                if kind == "recording":
                    self.recording, self.started = True, time.monotonic()
                    self.record_button.configure(text="■ 녹화 정지 (R)")
                    self.entry.configure(state="disabled")
                    self.browse.configure(state="disabled")
                elif kind == "saved":
                    self.recording = False
                    self.record_button.configure(text="● 녹화 시작 (R)")
                    self.entry.configure(state="normal")
                    self.browse.configure(state="normal")
                prefix = {"ready": "연결됨", "recording": "녹화 중", "saved": "녹화 저장 완료",
                          "photo": "사진 저장 완료", "notice": "알림"}[kind]
                if not self.closing:
                    self.status.set(f"{prefix}: {value}")
            elif kind == "error":
                self.status.set(f"오류: {value}")
                self.record_button.configure(state="disabled")
                self.photo_button.configure(state="disabled")
                if not self.closing:
                    messagebox.showerror("카메라 / 저장 오류", f"{value}\n\nUSB 3 연결과 다른 카메라 프로그램을 확인한 뒤 다시 실행하세요.", parent=self.root)
            elif kind == "closed":
                if self.closing:
                    self.root.destroy()
                    return
                self.record_button.configure(state="disabled")
                self.photo_button.configure(state="disabled")
        if self.closing and not self.worker.is_alive():
            self.root.destroy()
            return
        with self.worker.lock:
            latest = self.worker.latest
        if latest is not None:
            color, depth, fps = latest
            for canvas, frame in zip(self.canvases, (color, depth)):
                w, h = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
                image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                image.thumbnail((w, h))
                canvas.image = ImageTk.PhotoImage(image)
                canvas.coords(canvas.image_item, w / 2, h / 2)
                canvas.itemconfigure(canvas.image_item, image=canvas.image)
            elapsed = int(time.monotonic() - self.started) if self.recording else 0
            self.stats.set(f"{fps:.1f} FPS" + (f"   ● REC {elapsed // 60:02}:{elapsed % 60:02}" if self.recording else ""))
        self.root.after(40, self.tick)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "tools/captures")
    p.add_argument("--serial", help="카메라가 여러 대일 때 사용할 RealSense 시리얼")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--depth-width", type=int, default=848)
    p.add_argument("--depth-height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--max-depth", type=float, default=1.5, help="깊이 미리보기 최대 거리 [m]; 원본 깊이에는 영향 없음")
    a = p.parse_args()
    if min(a.width, a.height, a.depth_width, a.depth_height, a.fps, a.max_depth) <= 0:
        p.error("해상도, FPS, 최대 깊이는 양수여야 합니다.")
    return a


if __name__ == "__main__":
    args = parse_args()
    window = tk.Tk()
    RecorderUI(window, args)
    window.mainloop()
