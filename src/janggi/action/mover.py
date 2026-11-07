"""장기판 좌표로 그리퍼 팁을 옮긴다.

    from action import move
    move(0, 0)        # 좌상단 교차점 (판 표면, 높이 0)
    move(8, 9, 10)    # 우하단 교차점 위 10 mm

이동 방식 (말·판에 부딪히지 않게):
    1. 현재 팁이 이동 높이(travel_mm, 기본 30 mm)보다 낮으면 그 자리에서 수직으로 올린다
    2. 이동 높이로 목표 교차점 위까지 직선 이동한다
    3. 목표 높이(height_mm, 기본 0 = 판 표면)까지 수직으로 내린다
툴 자세는 보정 때 자세를 그대로 쓴다. 서비스 응답(= 이동 완료)까지 기다린다.
"""

from __future__ import annotations

import threading
from pathlib import Path

import numpy as np

from .board import DEFAULT_CALIBRATION, BoardGeometry, check_range

DEFAULT_HEIGHT_MM = 0.0    # 도착 높이: 판 윗면 위 팁 높이 (0 = 보정 때 팁을 댄 판 표면)
DEFAULT_TRAVEL_MM = 30.0   # 교차점 사이를 옮겨 다닐 때의 팁 높이. 말·판과 부딪히지 않게 띄운다
DEFAULT_VEL = (100.0, 30.0)   # move_line 속도 [mm/s, deg/s]
DEFAULT_ACC = (200.0, 60.0)   # move_line 가속도 [mm/s², deg/s²]


class BoardMover:
    """Doosan ROS 2 드라이버로 장기판 좌표 이동. with 문으로 쓰거나 close()를 부른다."""

    def __init__(self, ns: str = "dsr01", calibration: str | Path = DEFAULT_CALIBRATION,
                 height_mm: float = DEFAULT_HEIGHT_MM, travel_mm: float = DEFAULT_TRAVEL_MM,
                 vel=DEFAULT_VEL, acc=DEFAULT_ACC, wait_s: float = 10.0):
        import rclpy
        from dsr_msgs2.srv import GetCurrentPosx, MoveLine, MoveStop
        from rclpy.executors import SingleThreadedExecutor

        self.board = BoardGeometry.from_calibration(calibration)
        self.height_mm, self.travel_mm = float(height_mm), float(travel_mm)
        self.vel, self.acc = tuple(vel), tuple(acc)
        self._srv = {"posx": GetCurrentPosx, "movel": MoveLine, "stop": MoveStop}
        self._rclpy = rclpy
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self.node = rclpy.create_node("janggi_board_mover")
        p = f"/{ns.strip('/')}" if ns else ""
        self._posx = self.node.create_client(GetCurrentPosx, f"{p}/aux_control/get_current_posx")
        self._movel = self.node.create_client(MoveLine, f"{p}/motion/move_line")
        self._stop = self.node.create_client(MoveStop, f"{p}/motion/move_stop")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        for cli in (self._posx, self._movel):
            if not cli.wait_for_service(timeout_sec=wait_s):
                self.close()
                raise RuntimeError(f"{cli.srv_name} 서비스가 없다. 로봇 드라이버(bringup)가 떠 있는지 확인한다.")

    # ------------------------------------------------------------------ 저수준
    def _call(self, cli, req, timeout: float):
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"{cli.srv_name} 응답 없음 ({timeout:.0f}s)")
        res = fut.result()
        if res is None:
            raise RuntimeError(f"{cli.srv_name} 호출 실패")
        return res

    def current_posx(self) -> list[float]:
        req = self._srv["posx"].Request()
        req.ref = 0  # DR_BASE
        res = self._call(self._posx, req, 3.0)
        if not res.success or not res.task_pos_info:
            raise RuntimeError("get_current_posx 실패")
        return [float(v) for v in res.task_pos_info[0].data[:6]]

    def current_tip(self) -> np.ndarray:
        """현재 팁 위치 (base, mm)."""
        return self.board.tip_from_posx(self.current_posx())

    def move_line(self, posx: list[float]):
        """절대 posx로 직선 이동하고 끝날 때까지 기다린다."""
        cur = self.current_posx()
        dist = float(np.linalg.norm(np.subtract(posx[:3], cur[:3])))
        req = self._srv["movel"].Request()
        req.pos = [float(v) for v in posx]
        req.vel, req.acc = list(self.vel), list(self.acc)
        req.ref = 0   # DR_BASE
        req.mode = 0  # 절대
        req.sync_type = 0  # 응답 = 이동 완료
        timeout = 15.0 + 3.0 * dist / max(self.vel[0], 1.0)
        try:
            res = self._call(self._movel, req, timeout)
        except TimeoutError:
            self.stop()
            raise
        if not res.success:
            raise RuntimeError(f"move_line 실패 (목표 {np.round(posx, 1).tolist()}). 로봇 상태/알람을 확인한다.")

    def stop(self):
        req = self._srv["stop"].Request()
        req.stop_mode = 1  # DR_QSTOP
        self._stop.call_async(req)

    # ------------------------------------------------------------------ 장기판 이동
    def move(self, x: float, y: float, height_mm: float | None = None,
             travel_mm: float | None = None) -> np.ndarray:
        """장기판 (x, y) 교차점으로 팁을 옮긴다 (도착 높이 height_mm). 도착한 팁 위치(base, mm)를 돌려준다.

        이동 높이(travel_mm)로 올라가서 옮긴 뒤 도착 높이로 수직으로 내린다.
        """
        check_range(x, y)
        h = self.height_mm if height_mm is None else float(height_mm)
        travel = self.travel_mm if travel_mm is None else float(travel_mm)
        if h < 0 or travel < 0:
            raise ValueError("높이는 0 이상이어야 한다 (판 아래로 내려가지 않는다)")
        travel = max(travel, h)
        above = self.board.tip_target(x, y, travel)   # 목표 교차점 위 이동 높이
        target = self.board.tip_target(x, y, h)       # 최종 도착점

        # 1) 현재 팁이 이동 높이보다 낮으면 그 자리에서 수직으로 올린다
        tip = self.current_tip()
        if tip[2] < above[2] - 1.0:
            lift = tip.copy()
            lift[2] = above[2]
            self.move_line(self.board.flange_pose(lift))
        # 2) 이동 높이로 목표 위까지
        self.move_line(self.board.flange_pose(above))
        # 3) 도착 높이까지 수직으로 내린다
        if travel - h > 0.5:
            self.move_line(self.board.flange_pose(target))
        return self.current_tip()

    def close(self):
        # spin 스레드를 먼저 끝낸 뒤 노드·컨텍스트를 정리해야 종료 시 abort가 나지 않는다
        if getattr(self, "_closed", False):
            return
        self._closed = True
        self._executor.shutdown(timeout_sec=1.0)
        self._spin.join(timeout=2.0)
        self.node.destroy_node()
        if self._own_context:
            self._rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


_default: BoardMover | None = None


def move(x: float, y: float, height_mm: float | None = None, travel_mm: float | None = None) -> np.ndarray:
    """기본 BoardMover(dsr01, 기본 보정 파일)로 장기판 (x, y) 교차점으로 이동한다 (기본 높이 0)."""
    global _default
    if _default is None:
        _default = BoardMover()
    return _default.move(x, y, height_mm, travel_mm)


def shutdown():
    """move()가 만든 기본 연결을 닫는다."""
    global _default
    if _default is not None:
        _default.close()
        _default = None
