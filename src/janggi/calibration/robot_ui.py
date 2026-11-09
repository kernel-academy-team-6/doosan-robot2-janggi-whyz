"""로봇 클라이언트 + Tk 로봇 패널 (조그 / 그리퍼 / 홈).

보정 도구(calibrate_board.py)와 조그 전용 모드가 같이 쓴다.
"""

from __future__ import annotations

import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog
from typing import Callable

import yaml

import calib_math as cm
from jog import JogController

HERE = Path(__file__).resolve().parent
POSES_FILE = HERE / "config" / "robot_poses.yaml"

# 조그 키 → 방향 (base 좌표계)
JOG_KEYS = {"w": "X+", "s": "X-", "a": "Y+", "d": "Y-", "r": "Z+", "f": "Z-"}
# 조그 속도 단계 [%] (키 1~6). multi_jog 속도 = 433 mm/s × 단위벡터 × % (Doosan 사양, 가감속 때문에 짧게 누르면 더 느림)
JOG_SPEEDS = [1.0, 3.0, 10.0, 25.0, 50.0, 75.0]
# X11 키 자동반복은 Release→Press 쌍을 연달아 보낸다. 이 시간 안에 Press가 다시 오면 '계속 누름'으로 본다.
KEY_RELEASE_DEBOUNCE_MS = 30
# UI가 이 시간 이상 멈추면 조그 중인 로봇을 감시 스레드가 정지시킨다.
WATCHDOG_S = 0.3

# J6 플랜지 중심 → 그리퍼 팁, 툴 z축(아래) 방향. 그리퍼 위치 명령 650 기준 실측: 6.3+2.5+4.9+8.5 cm
GRIPPER_TIP_OFFSET_MM = (0.0, 0.0, 63.0 + 25.0 + 49.0 + 85.0)  # 222 mm
GRIPPER_TIP_POS_CMD = 650
GRIPPER_OPEN, GRIPPER_CLOSE = 0, 700  # gripper_service_node의 open/close 값
GRIPPER_MIN, GRIPPER_MAX = 0, 700  # RUNBOOK: position_cmd 범위 0(열림)~700(닫힘)

# 홈 자세 기본값 [deg]: 툴이 아래를 보는 준비 자세 (J6 90°). config/robot_poses.yaml이 있으면 그 값을 쓴다.
DEFAULT_HOME_JOINTS = [0.0, 0.0, 90.0, 0.0, 90.0, 90.0]
DR_QSTOP = 1  # move_stop 모드: Quick stop (Stop Category 2)


def load_home() -> list[float]:
    try:
        data = yaml.safe_load(POSES_FILE.read_text(encoding="utf-8")) or {}
        home = [float(x) for x in data["home_joint_deg"]]
        if len(home) == 6:
            return home
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        pass
    return list(DEFAULT_HOME_JOINTS)


def save_home(joints: list[float]):
    POSES_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(POSES_FILE, "w", encoding="utf-8") as f:
        f.write("# 보정/조그 도구의 홈 자세 (robot_ui.py가 저장). 단위 deg\n")
        yaml.safe_dump({"home_joint_deg": [round(float(x), 3) for x in joints]}, f, default_flow_style=None)


# --------------------------------------------------------------------------- 로봇 클라이언트

class Robot:
    """Doosan ROS 2 드라이버 클라이언트. executor를 백그라운드에서 돌려 UI 스레드가 막히지 않게 한다.

    조그·정지·홈 이동은 응답을 기다리지 않고 보낸다 (fire-and-forget).
    """

    can_jog = True

    def __init__(self, ns: str):
        import rclpy
        from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx, GetCurrentTcp, JogMulti, MoveJoint, MoveStop
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import Int32

        self.rclpy = rclpy
        self.srv = {"posx": GetCurrentPosx, "posj": GetCurrentPosj, "tcp": GetCurrentTcp, "jog": JogMulti,
                    "movej": MoveJoint, "stop": MoveStop}
        self.Int32 = Int32
        rclpy.init()
        self.node = rclpy.create_node("janggi_robot_ui")
        p = f"/{ns.strip('/')}" if ns else ""
        self.prefix = p
        self.posx_srv = f"{p}/aux_control/get_current_posx"
        n = self.node
        self.posx_cli = n.create_client(GetCurrentPosx, self.posx_srv)
        self.posj_cli = n.create_client(GetCurrentPosj, f"{p}/aux_control/get_current_posj")
        self.tcp_cli = n.create_client(GetCurrentTcp, f"{p}/tcp/get_current_tcp")
        self.jog_cli = n.create_client(JogMulti, f"{p}/motion/jog_multi")
        self.movej_cli = n.create_client(MoveJoint, f"{p}/motion/move_joint")
        self.stop_cli = n.create_client(MoveStop, f"{p}/motion/move_stop")
        self.grip_pub = n.create_publisher(Int32, f"{p}/gripper/position_cmd", 10)
        self.gripper_stroke: int | None = None  # 그리퍼 노드가 마지막으로 받은 명령값
        n.create_subscription(Int32, f"{p}/gripper/stroke", self._on_stroke, 10)

        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._lock = threading.Lock()

    def _on_stroke(self, msg):
        self.gripper_stroke = int(msg.data)

    def wait(self, timeout: float) -> bool:
        return self.posx_cli.wait_for_service(timeout_sec=timeout)

    def jog_ready(self, timeout=2.0) -> bool:
        return self.jog_cli.wait_for_service(timeout_sec=timeout)

    def _call(self, cli, req, timeout=3.0):
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout) or fut.result() is None:
            raise RuntimeError(f"서비스 응답 없음: {cli.srv_name}")
        return fut.result()

    def posx(self) -> list[float]:
        req = self.srv["posx"].Request()
        req.ref = 0  # DR_BASE
        res = self._call(self.posx_cli, req)
        if not res.success or not res.task_pos_info:
            raise RuntimeError("get_current_posx 실패 (success=false)")
        return [float(x) for x in res.task_pos_info[0].data[:6]]

    def posj(self) -> list[float]:
        res = self._call(self.posj_cli, self.srv["posj"].Request())
        if not res.success:
            raise RuntimeError("get_current_posj 실패")
        return [float(x) for x in res.pos]

    def tcp_name(self) -> str | None:
        if not self.tcp_cli.wait_for_service(timeout_sec=1.0):
            return None
        try:
            res = self._call(self.tcp_cli, self.srv["tcp"].Request(), timeout=2.0)
            return res.info if res.success else None
        except RuntimeError:
            return None

    def jog(self, vec, speed: float):
        """base 좌표계 multi_jog. speed 0이면 정지. 응답을 기다리지 않는다."""
        req = self.srv["jog"].Request()
        req.jog_axis = [float(x) for x in vec]
        req.move_reference = 0  # MOVE_REFERENCE_BASE
        req.speed = float(speed)
        with self._lock:  # UI 스레드와 감시 스레드가 함께 부를 수 있다
            return self.jog_cli.call_async(req)

    def move_joint(self, joints, vel: float, acc: float):
        """관절 이동. 응답(이동 완료)은 반환된 future로 확인한다."""
        req = self.srv["movej"].Request()
        req.pos = [float(x) for x in joints]
        req.vel, req.acc = float(vel), float(acc)
        req.sync_type = 0  # 서비스 응답 = 이동 완료
        with self._lock:
            return self.movej_cli.call_async(req)

    def move_stop(self):
        req = self.srv["stop"].Request()
        req.stop_mode = DR_QSTOP
        with self._lock:
            return self.stop_cli.call_async(req)

    def gripper_position(self, value: int):
        self.grip_pub.publish(self.Int32(data=int(value)))

    def stop_sync(self, timeout=1.5):
        done = threading.Event()
        self.jog([0.0] * 6, 0.0).add_done_callback(lambda _f: done.set())
        done.wait(timeout)

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class ManualRobot:
    """ROS 없이 펜던트에 표시된 좌표를 입력받는다. 조그·그리퍼·홈 불가."""

    posx_srv = "manual input"
    can_jog = False
    gripper_stroke = None

    def __init__(self):
        self.parent = None  # App이 Tk root를 넣어준다

    def wait(self, timeout):
        return True

    def posx(self) -> list[float]:
        while True:
            # 팁 오프셋을 자세로 돌려야 하므로 A B C까지 받는다
            s = simpledialog.askstring("로봇 좌표 입력", "펜던트 base 좌표 'x y z A B C' (mm, deg)",
                                       parent=self.parent)
            if s is None:
                raise RuntimeError("입력 취소")
            try:
                vals = [float(x) for x in s.replace(",", " ").split()]
            except ValueError:
                vals = []
            if len(vals) == 6:
                return vals

    def tcp_name(self):
        return None

    def jog(self, vec, speed):
        pass

    def move_stop(self):
        pass

    def stop_sync(self, timeout=0):
        pass

    def close(self):
        pass


# --------------------------------------------------------------------------- 키 입력

class KeyTracker:
    """Tk 키 이벤트를 '눌림/뗌'으로 정리한다. X11 자동반복(Release+Press 쌍)은 계속 누름으로 본다.

    repeat_keys에 있는 키는 자동반복마다 on_press가 다시 불린다 (예: 픽셀 이동 방향키).
    """

    def __init__(self, root: tk.Tk, on_press: Callable[[str], None], on_release: Callable[[str], None],
                 repeat_keys=("up", "down", "left", "right")):
        self.root, self.on_press, self.on_release = root, on_press, on_release
        self.repeat_keys = set(repeat_keys)
        self.down: set[str] = set()
        self.pending: dict[str, str] = {}
        root.bind_all("<KeyPress>", self._press)
        root.bind_all("<KeyRelease>", self._release)

    def _press(self, e):
        k = e.keysym.lower()
        if k in self.repeat_keys:
            self.on_press(k)
            return
        if k in self.pending:  # 자동반복: 직전 Release를 취소
            self.root.after_cancel(self.pending.pop(k))
            return
        if k in self.down:
            return
        self.down.add(k)
        self.on_press(k)

    def _release(self, e):
        k = e.keysym.lower()
        if k in self.down and k not in self.pending:
            self.pending[k] = self.root.after(KEY_RELEASE_DEBOUNCE_MS, lambda: self._released(k))

    def _released(self, k):
        self.pending.pop(k, None)
        self.down.discard(k)
        self.on_release(k)

    def reset(self):
        for aid in self.pending.values():
            self.root.after_cancel(aid)
        self.pending.clear()
        self.down.clear()


# --------------------------------------------------------------------------- 로봇 패널

class RobotPanel(tk.Frame):
    """팁 위치 / 조그 / 그리퍼 / 홈 패널. 키: W S A D R F 조그, 1~6 속도, H 홈, G 그리퍼 650."""

    def __init__(self, master, root: tk.Tk, robot, tcp_offset, log: Callable[[str], None],
                 speed_level: int = 2, home_vel: float = 30.0, tcp_name: str | None = None):
        super().__init__(master, padx=8, pady=6)
        self.root, self.robot, self.tcp_offset, self.log = root, robot, tcp_offset, log
        self.home_vel = home_vel
        self.home = load_home()
        self.homing = None  # 진행 중인 move_joint future
        self.running = True
        self.hb = time.time()
        self.watchdog_fired = threading.Event()
        self.live_posx: list[float] | None = None
        self.live_err: str | None = None
        self.jog = JogController(self.robot.jog, JOG_SPEEDS[speed_level - 1])
        self.speed_var = tk.IntVar(value=speed_level)
        self._build(tcp_name)
        if robot.can_jog:
            threading.Thread(target=self._watchdog, daemon=True).start()
            threading.Thread(target=self._poll, daemon=True).start()

    # ------------------------------------------------------------------ 구성
    def _build(self, tcp_name):
        st = tk.NORMAL if self.robot.can_jog else tk.DISABLED
        mono = ("TkFixedFont", 10)

        fr = tk.LabelFrame(self, text="로봇", padx=6, pady=2)
        fr.pack(fill="x")
        self.lbl_tip = tk.Label(fr, anchor="w", font=mono)
        self.lbl_tip.pack(fill="x")
        o = self.tcp_offset
        tk.Label(fr, anchor="w", fg="#555",
                 text=f"TCP {tcp_name or '-'} / 팁 오프셋 ({o[0]:g}, {o[1]:g}, {o[2]:g}) mm").pack(fill="x")

        fj = tk.LabelFrame(self, text="조그 — 누르고 있는 동안 이동 (base)", padx=6, pady=4)
        fj.pack(fill="x", pady=6)
        g = tk.Frame(fj)
        g.pack()
        layout = {"X+": (0, 1), "Y+": (1, 0), "Y-": (1, 2), "X-": (2, 1), "Z+": (0, 4), "Z-": (2, 4)}
        key_of = {v: k for k, v in JOG_KEYS.items()}
        for d, (r, c) in layout.items():
            b = tk.Button(g, text=f"{d}\n({key_of[d].upper()})", width=6, height=2, takefocus=0, state=st)
            b.grid(row=r, column=c, padx=2, pady=2)
            if self.robot.can_jog:
                b.bind("<ButtonPress-1>", lambda e, d=d: self.jog_press(d))
                b.bind("<ButtonRelease-1>", lambda e, d=d: self.jog.release(d))
        tk.Button(g, text="STOP", width=6, height=2, bg="#c62828", fg="white", activebackground="#e53935",
                  takefocus=0, command=self.stop_all).grid(row=1, column=1, padx=2, pady=2)
        tk.Frame(g, width=16).grid(row=0, column=3)
        fs = tk.Frame(fj)
        fs.pack(pady=(4, 0))
        for i, s in enumerate(JOG_SPEEDS, start=1):
            tk.Radiobutton(fs, text=f"{i}:{s:g}%", value=i, variable=self.speed_var, takefocus=0, state=st,
                           fg="#c62828" if s >= 50 else "black",
                           command=lambda: self.set_speed(self.speed_var.get())).grid(
                row=(i - 1) // 3, column=(i - 1) % 3, sticky="w")
        self.lbl_jog = tk.Label(fj, anchor="w", font=mono)
        self.lbl_jog.pack(fill="x")

        fg = tk.LabelFrame(self, text="그리퍼", padx=6, pady=4)
        fg.pack(fill="x")
        row = tk.Frame(fg)
        row.pack(fill="x")
        tk.Button(row, text=f"열기 ({GRIPPER_OPEN})", takefocus=0, state=st,
                  command=lambda: self.gripper(GRIPPER_OPEN)).pack(side="left", expand=True, fill="x")
        tk.Button(row, text=f"팁 기준 {GRIPPER_TIP_POS_CMD} (G)", takefocus=0, state=st,
                  command=lambda: self.gripper(GRIPPER_TIP_POS_CMD)).pack(side="left", expand=True, fill="x")
        tk.Button(row, text=f"닫기 ({GRIPPER_CLOSE})", takefocus=0, state=st,
                  command=lambda: self.gripper(GRIPPER_CLOSE)).pack(side="left", expand=True, fill="x")
        row2 = tk.Frame(fg)
        row2.pack(fill="x", pady=(4, 0))
        tk.Label(row2, text=f"위치 ({GRIPPER_MIN}~{GRIPPER_MAX})").pack(side="left")
        self.grip_entry = tk.Entry(row2, width=6, state=st)
        self.grip_entry.insert(0, str(GRIPPER_TIP_POS_CMD))
        self.grip_entry.pack(side="left", padx=4)
        self.grip_entry.bind("<Return>", lambda e: self._grip_from_entry())
        tk.Button(row2, text="적용", takefocus=0, state=st, command=self._grip_from_entry).pack(side="left")
        self.lbl_grip = tk.Label(fg, anchor="w", font=mono)
        self.lbl_grip.pack(fill="x")

        fh = tk.LabelFrame(self, text="홈 자세", padx=6, pady=4)
        fh.pack(fill="x", pady=6)
        self.lbl_home = tk.Label(fh, anchor="w", font=mono)
        self.lbl_home.pack(fill="x")
        row3 = tk.Frame(fh)
        row3.pack(fill="x")
        tk.Button(row3, text="홈으로 이동 (H)", takefocus=0, state=st, command=self.go_home).pack(
            side="left", expand=True, fill="x")
        tk.Button(row3, text="현재 자세를 홈으로 저장", takefocus=0, state=st, command=self.set_home).pack(
            side="left", expand=True, fill="x")
        self._show_home()

    def _show_home(self):
        self.lbl_home.config(text="J " + " ".join(f"{x:.1f}" for x in self.home) + f"  ({self.home_vel:g}°/s)")

    # ------------------------------------------------------------------ 키
    def key_press(self, k: str) -> bool:
        """처리한 키면 True."""
        if k in JOG_KEYS:
            self.jog_press(JOG_KEYS[k])
        elif k in ("1", "2", "3", "4", "5", "6"):
            self.set_speed(int(k))
        elif k == "h":
            self.go_home()
        elif k == "g":
            self.gripper(GRIPPER_TIP_POS_CMD)
        else:
            return False
        return True

    def key_release(self, k: str):
        if k in JOG_KEYS:
            self.jog.release(JOG_KEYS[k])

    # ------------------------------------------------------------------ 동작
    @property
    def moving(self) -> bool:
        return self.jog.moving or self.homing is not None

    def jog_press(self, d: str):
        if not self.robot.can_jog:
            return
        if self.homing is not None:  # 홈 이동 중에 조그 키를 누르면 홈 이동을 멈춘다
            self.stop_all()
            self.log("홈 이동을 멈췄다.")
            return
        self.jog.press(d)

    def set_speed(self, level: int):
        self.speed_var.set(level)
        self.jog.set_speed(JOG_SPEEDS[level - 1])

    def stop_all(self):
        """조그 정지 + 진행 중인 홈 이동 정지."""
        self.jog.stop(force=True)
        if self.homing is not None:
            self.robot.move_stop()
            self.homing = None

    def gripper(self, value: int):
        if not self.robot.can_jog:
            return
        value = int(min(max(value, GRIPPER_MIN), GRIPPER_MAX))
        self.robot.gripper_position(value)
        self.grip_entry.delete(0, "end")
        self.grip_entry.insert(0, str(value))
        self.log(f"그리퍼 위치 명령 {value}" + ("" if value == GRIPPER_TIP_POS_CMD else
                                          f" (팁 오프셋은 {GRIPPER_TIP_POS_CMD} 기준)"))

    def _grip_from_entry(self):
        try:
            self.gripper(int(float(self.grip_entry.get())))
        except ValueError:
            self.log("그리퍼 위치는 숫자로 입력한다.")
        self.root.focus_set()  # 입력창에 포커스가 남으면 조그 키가 글자로 들어간다

    def go_home(self):
        if not self.robot.can_jog or self.homing is not None:
            return
        self.jog.stop(force=True)
        if not messagebox.askyesno("홈 이동", "홈 자세로 이동한다. 경로에 장애물이 없는지 확인했나?\n"
                                   + " ".join(f"{x:.1f}" for x in self.home), parent=self.root):
            return
        self.log("홈으로 이동 중… (멈추려면 STOP / 조그 키 / ESC)")
        fut = self.robot.move_joint(self.home, self.home_vel, self.home_vel * 2)
        self.homing = fut

        def done(f):
            if self.homing is f:
                self.homing = None
                ok = f.result() is not None and f.result().success
                self.root.after(0, lambda: self.log("홈 도착." if ok else "홈 이동 실패/중단."))
        fut.add_done_callback(done)

    def set_home(self):
        try:
            j = self.robot.posj()
        except RuntimeError as e:
            self.log(f"[오류] {e}")
            return
        txt = " ".join(f"{x:.1f}" for x in j)
        if not messagebox.askyesno("홈 저장", f"현재 관절값을 홈으로 저장할까?\n{txt}", parent=self.root):
            return
        save_home(j)
        self.home = j
        self._show_home()
        self.log(f"홈 저장: {txt} → {POSES_FILE}")

    # ------------------------------------------------------------------ 주기 갱신
    def tick(self):
        """UI 루프에서 매번 부른다 (감시용 heartbeat + 라벨 갱신)."""
        self.hb = time.time()
        if self.watchdog_fired.is_set():
            self.watchdog_fired.clear()
            self.stop_all()
            self.log("[감시] UI가 멈춰 조그를 정지했다. 키를 다시 누른다.")
        if not self.robot.can_jog:
            self.lbl_tip.config(text="수동 입력 모드 (조그 없음)")
            return
        if self.live_posx is not None:
            t = cm.tip_position_m(self.live_posx, self.tcp_offset) * 1000
            self.lbl_tip.config(text=f"팁 base ({t[0]:7.1f}, {t[1]:7.1f}, {t[2]:7.1f}) mm", fg="black")
        else:
            self.lbl_tip.config(text=f"팁 위치 읽는 중… {self.live_err or ''}", fg="#c62828")
        if self.homing is not None:
            self.lbl_jog.config(text="● 홈 이동 중", fg="#e65100")
        elif self.jog.moving:
            self.lbl_jog.config(text=f"● 이동 중 {' '.join(sorted(self.jog.held))}  {self.jog.speed:g}%", fg="#e65100")
        else:
            self.lbl_jog.config(text=f"■ 정지  (속도 {self.jog.speed:g}%)", fg="#2e7d32")
        s = self.robot.gripper_stroke
        if s is None:
            self.lbl_grip.config(text="현재 명령값: 모름 (한 번 설정하면 표시)", fg="#c62828")
        elif s == GRIPPER_TIP_POS_CMD:
            self.lbl_grip.config(text=f"현재 명령값: {s} ✔ 팁 오프셋 기준", fg="#2e7d32")
        else:
            self.lbl_grip.config(text=f"현재 명령값: {s} — 팁 오프셋은 {GRIPPER_TIP_POS_CMD} 기준", fg="#e65100")

    def gripper_ok(self) -> bool | None:
        s = self.robot.gripper_stroke
        return None if s is None else s == GRIPPER_TIP_POS_CMD

    def _watchdog(self):
        """UI 스레드가 멈추면(뗌 이벤트를 못 받으면) 로봇을 직접 정지시킨다."""
        while self.running:
            if self.jog.moving and time.time() - self.hb > WATCHDOG_S and not self.watchdog_fired.is_set():
                try:
                    self.robot.jog([0.0] * 6, 0.0)
                finally:
                    self.watchdog_fired.set()
            time.sleep(0.05)

    def _poll(self):
        while self.running:
            try:
                self.live_posx, self.live_err = self.robot.posx(), None
            except Exception as e:  # noqa: BLE001
                self.live_err = str(e)
            time.sleep(0.2)

    def shutdown(self):
        self.running = False
        self.stop_all()


# --------------------------------------------------------------------------- 조그 전용 창

class JogApp:
    """카메라·점 찍기 없이 로봇만 움직이는 창."""

    def __init__(self, robot, tcp_offset, speed_level=2, home_vel=30.0, tcp_name=None):
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("로봇 조그")
        self.panel = RobotPanel(self.root, self.root, robot, tcp_offset, self.log, speed_level, home_vel, tcp_name)
        self.panel.pack(fill="x")
        self.txt = tk.Text(self.root, height=8, width=56, font=("TkFixedFont", 9), state="disabled", takefocus=0)
        self.txt.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        tk.Label(self.root, fg="#555", text="W/S X±  A/D Y±  R/F Z±  1~6 속도  H 홈  G 그리퍼 650  ESC 정지·종료"
                 ).pack(pady=(0, 6))
        self.keys = KeyTracker(self.root, self.on_press, self.panel.key_release)
        self.root.bind("<FocusOut>", lambda e: self.root.after(50, self._check_focus))
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(15, self.tick)

    def log(self, msg):
        print(msg, flush=True)
        self.txt.configure(state="normal")
        self.txt.insert("end", msg + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def on_press(self, k):
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        if k == "escape":
            self.panel.stop_all()
            if messagebox.askyesno("종료", "조그 창을 닫을까?", parent=self.root):
                self.quit()
        else:
            self.panel.key_press(k)

    def _check_focus(self):
        try:
            focused = self.root.focus_get()
        except (KeyError, tk.TclError):
            focused = None
        if focused is None:
            self.keys.reset()
            self.panel.stop_all()

    def tick(self):
        if self.panel.running:
            self.panel.tick()
            self.root.after(15, self.tick)

    def quit(self):
        self.keys.reset()
        self.panel.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
