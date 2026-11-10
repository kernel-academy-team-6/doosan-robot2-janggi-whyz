"""사람 ↔ 로봇 장기 대국 파이프라인 (GUI·카메라·ROS와 분리된 진행 로직).

한 턴:
    1. 사람이 실제 판에 두고 '착수 완료'를 누른다
    2. 판 상태를 읽는다 (vision.fresh_state: 안정화된 판 + 칸별 검출 박스 중심 픽셀)
       → 게임 판과의 차이로 사람이 둔 수를 찾고 규칙을 확인해 반영한다
    3. 판 상태 → 엔진(Fairy-Stockfish) → 로봇의 수
    4. 로봇이 둔다. 잡는 수면 먼저 상대 말을 집어 판 밖(보정 사각형 바깥) 버리는 자리에 놓는다.
       말을 집을 때는 검출 박스 중심을 카메라-로봇 보정으로 base 좌표로 바꿔 move_abs로 간다
       (박스 중심이 교차점과 꼭 맞지 않으므로). 놓을 때는 교차점에 move(x, y).
    5. 관찰 자세로 돌아가고 다시 사람 차례.

vision 인터페이스: fresh_state() -> (board, centers)
    board   {(x, y): (side, kind)}
    centers {(x, y): (u, v)}  해당 칸에 할당된 검출 박스 중심 픽셀 (최근 프레임 중앙값)
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SRC = HERE.parent                          # src/janggi/  (action, engine)
WS = SRC.parents[1]
VISION = WS / "janggi"                     # janggi/      (piece_detection, realtime_ui/rules.py)
for p in (SRC, VISION, VISION / "realtime_ui"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rules import Game, detect_top_side, infer_move, other  # noqa: E402

SIDE_KO = {"blue": "초", "red": "한"}
KIND_KO = {"king": "궁", "cha": "차", "po": "포", "horse": "마", "sang": "상", "sa": "사", "soldier": "졸"}
POSES_FILE = WS / "src" / "janggi" / "calibration" / "config" / "robot_poses.yaml"
DEFAULT_OBSERVE_JOINTS = [0.0, 0.0, 90.0, 0.0, 90.0, 90.0]  # = 보정 도구 홈 기본값


def load_observe_joints() -> list[float]:
    """로봇이 카메라를 가리지 않는 관찰 자세 = 보정 도구의 홈 자세 (robot_poses.yaml, 없으면 기본값)."""
    try:
        j = [float(v) for v in yaml.safe_load(POSES_FILE.read_text(encoding="utf-8"))["home_joint_deg"]]
        if len(j) == 6:
            return j
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        pass
    return list(DEFAULT_OBSERVE_JOINTS)


def piece_name(piece) -> str:
    side, kind = piece
    return SIDE_KO[side] + ("졸" if kind == "soldier" and side == "blue" else
                            "병" if kind == "soldier" else KIND_KO[kind])


def median_centers(board, hist) -> dict:
    """프레임별 {칸: (u, v)} 기록에서 board에 있는 칸마다 박스 중심 픽셀의 중앙값."""
    out = {}
    for c in board:
        pts = [h[c] for h in hist if c in h]
        if pts:
            out[c] = tuple(float(v) for v in np.median(np.array(pts, float), axis=0))
    return out


class TurnError(Exception):
    """사람이 둔 수를 읽지 못함 등, 사람이 판을 확인하고 다시 누르면 되는 문제."""


@dataclass
class RobotGameConfig:
    human_side: str = "blue"          # 사람 진영 (먼저 둠)
    movetime_ms: int = 1000           # 엔진 생각 시간
    piece_height_mm: float = 12.0     # 말 높이 (박스 중심 = 말 윗면 중심)
    max_offset_mm: float = 18.0       # 카메라로 찾은 말이 교차점에서 이만큼 넘게 떨어지면 교차점 위치로 집는다
    observe_joints: list[float] = field(default_factory=load_observe_joints)
    observe_vel: float = 30.0         # 관찰 자세 이동 속도 [deg/s]


@dataclass
class RobotPlan:
    uci: str | None
    src: tuple | None = None
    dst: tuple | None = None
    piece: tuple | None = None
    captured: tuple | None = None
    score: str = ""
    pick_mm: np.ndarray | None = None      # 로봇 말 집을 팁 위치 (base, mm)
    pick_from: str = ""                    # 'camera' | 'grid'
    capture_mm: np.ndarray | None = None   # 잡을 상대 말 집을 팁 위치
    capture_from: str = ""

    @property
    def is_pass(self) -> bool:
        return self.uci is not None and self.src == self.dst

    def describe(self) -> str:
        if self.uci is None:
            return "둘 수 있는 수가 없다"
        if self.is_pass:
            return f"한수쉼 ({self.uci})"
        cap = f" x{piece_name(self.captured)}" if self.captured else ""
        return (f"{piece_name(self.piece)} {self.src[0]},{self.src[1]}→{self.dst[0]},{self.dst[1]}{cap}"
                f"  [{self.uci}, {self.score}]")


class RobotGame:
    """picker=None이면 로봇 없이 엔진까지만 (계획만 출력)."""

    def __init__(self, engine, cam, geometry, picker=None, cfg: RobotGameConfig | None = None,
                 log: Callable[[str], None] = print):
        self.engine, self.cam, self.geom, self.picker = engine, cam, geometry, picker
        self.cfg = cfg or RobotGameConfig()
        self.log = log
        self.game: Game | None = None
        self.robot_side = other(self.cfg.human_side)
        self.last_plan: RobotPlan | None = None

    # ------------------------------------------------------------------ 시작
    def start(self, board) -> Game:
        if not board:
            raise TurnError("인식된 말이 없다")
        top = detect_top_side(board)
        self.game = Game(dict(board), top_side=top, turn=self.cfg.human_side)
        self.log(f"새 대국: 사람 {SIDE_KO[self.cfg.human_side]}, 로봇 {SIDE_KO[self.robot_side]}, "
                 f"위쪽 {SIDE_KO[top]}, 말 {len(board)}개")
        self.go_observe()
        return self.game

    def go_observe(self):
        if self.picker is not None:
            self.picker.mover.move_joint(self.cfg.observe_joints, self.cfg.observe_vel, self.cfg.observe_vel)

    # ------------------------------------------------------------------ 1-2. 사람 수 읽기
    def apply_human_move(self, observed) -> tuple | None:
        """인식된 판을 검증 없이 그대로 게임 판으로 쓰고 로봇 차례로 넘긴다.

        인식이 완벽하지 않아 이전 판과의 차이·규칙 검사로 막지 않는다. 한 수로 읽히면 기보용으로 로그만 남긴다.
        """
        g = self.game
        if g.turn != self.cfg.human_side:
            raise TurnError("사람 차례가 아니다")
        if not observed:
            raise TurnError("인식된 말이 없다")
        mv = infer_move(g.board, observed, g.turn)
        if mv is not None:
            src, dst = mv
            cap = g.board.get(dst)
            cap_s = f" x{piece_name(cap)}" if cap else ""
            self.log(f"사람: {piece_name(g.board[src])} {src[0]},{src[1]}→{dst[0]},{dst[1]}{cap_s}")
        else:
            diff = sorted(c for c in set(g.board) | set(observed) if g.board.get(c) != observed.get(c))
            self.log(f"사람: 인식된 판 그대로 사용 (이전 판과 다른 칸 {len(diff)}개)")
        g.board = dict(observed)
        g.turn = self.robot_side
        return mv

    # ------------------------------------------------------------------ 3. 엔진
    def grasp_from_camera(self, cell, centers) -> tuple[np.ndarray, str]:
        """칸의 말을 집을 팁 위치. 검출 박스 중심 → base 좌표, 없거나 너무 멀면 교차점."""
        pc = self.picker.cfg if self.picker else None
        grasp_h = pc.grasp_height_mm if pc else 0.0
        grid = self.geom.tip_target(*cell, grasp_h)
        uv = centers.get(cell)
        if uv is None:
            self.log(f"  {cell}: 검출 박스가 없어 교차점 위치로 집는다")
            return grid, "grid"
        top = self.cam.pixel_to_base(*uv, height_mm=self.cfg.piece_height_mm)
        n = self.geom.normal()
        p = top + n * (grasp_h - self.geom.height_above(top))
        off = p - grid
        off_h = float(np.linalg.norm(off - n * (n @ off)))
        if off_h > self.cfg.max_offset_mm:
            self.log(f"  {cell}: 카메라 위치가 교차점에서 {off_h:.0f} mm 떨어져 교차점 위치로 집는다")
            return grid, "grid"
        return p, "camera"

    def plan(self, centers) -> RobotPlan:
        g = self.game
        res = self.engine.best_move(g.board, g.top_side, g.turn, self.cfg.movetime_ms)
        if res.no_move:
            return RobotPlan(None, score=res.score)
        plan = RobotPlan(res.uci, res.src, res.dst, g.board.get(res.src), g.board.get(res.dst), res.score)
        if plan.is_pass:
            return plan
        if plan.piece is None or plan.piece[0] != self.robot_side:
            raise RuntimeError(f"엔진 수 {res.uci}의 출발 칸 {res.src}에 로봇 말이 없다 (판 방향 확인)")
        plan.pick_mm, plan.pick_from = self.grasp_from_camera(res.src, centers)
        if plan.captured:
            plan.capture_mm, plan.capture_from = self.grasp_from_camera(res.dst, centers)
        return plan

    # ------------------------------------------------------------------ 4. 로봇 동작
    def execute(self, plan: RobotPlan, stop_check: Callable[[], bool] = lambda: False):
        g = self.game
        if plan.uci is None:
            return
        if plan.is_pass:
            g.pass_turn()
            self.log("로봇: 한수쉼")
            return
        if self.picker is not None:
            if plan.captured:
                self.log(f"  잡은 말 {piece_name(plan.captured)} 치우기 ({plan.capture_from})")
                self.picker.pick_abs(plan.capture_mm)
                if stop_check():
                    raise RuntimeError("정지")
                self.picker.discard()
            if stop_check():
                raise RuntimeError("정지")
            self.log(f"  {piece_name(plan.piece)} 집기 ({plan.pick_from}) → {plan.dst} 놓기")
            self.picker.pick_abs(plan.pick_mm)
            if stop_check():
                raise RuntimeError("정지")
            self.picker.place(*plan.dst)
        g.move(plan.src, plan.dst, check=False)
        self.log(f"로봇: {plan.describe()}")
        if not self.has_moves(g.turn):
            self.log(f"{SIDE_KO[g.turn]}이 둘 수 있는 수가 없다 — {SIDE_KO[other(g.turn)]} 승")

    def has_moves(self, side) -> bool:
        g = self.game
        return bool(self.engine.perft_moves(g.board, g.top_side, side))

    # ------------------------------------------------------------------ 한 턴 전체
    def robot_turn(self, centers, stop_check=lambda: False) -> RobotPlan:
        t0 = time.time()
        plan = self.plan(centers)
        self.last_plan = plan
        if plan.src is not None and not plan.is_pass:
            callback = getattr(self, 'on_move_planned', None)
            if callback is not None:
                callback(plan.src, plan.dst)
        self.log(f"엔진: {plan.describe()} ({time.time() - t0:.1f}s)")
        if plan.uci is None:
            self.log(f"로봇({SIDE_KO[self.robot_side]})이 둘 수 없다 — 사람 승")
            return plan
        self.execute(plan, stop_check)
        self.go_observe()
        return plan

    def human_done(self, vision, stop_check=lambda: False) -> RobotPlan:
        """'착수 완료' 한 번에 하는 일: 판 읽기 → 사람 수 반영 → 엔진 → 로봇 → 관찰 자세."""
        board, centers = vision.fresh_state()
        self.apply_human_move(board)
        return self.robot_turn(centers, stop_check)
