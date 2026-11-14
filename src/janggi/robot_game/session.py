"""사람 ↔ 로봇 장기 대국 파이프라인 (GUI·카메라·ROS와 분리된 진행 로직).

한 턴:
    1. 사람이 실제 판에 두고 '착수 완료'를 누른다
    2. 판 상태를 읽는다 (vision.fresh_state: 현재 보이는 분류된 판 + 칸별 검출 박스 중심 픽셀)
       → 이전 판과 달라도 현재 판으로 갱신하고 로봇 차례로 설정한다
    3. 판 상태 → 엔진(Fairy-Stockfish) → 로봇의 수
    4. 로봇이 둔다. 잡는 수면 먼저 상대 말을 집어 판 밖(보정 사각형 바깥) 버리는 자리에 놓는다.
       말을 집을 때는 검출 박스 중심을 카메라-로봇 보정으로 base 좌표로 바꿔 move_abs로 간다
       (박스 중심이 교차점과 꼭 맞지 않으므로). 놓을 때는 교차점에 move(x, y).
    5. 관찰 자세로 돌아가고 다시 사람 차례.

vision 인터페이스: fresh_state() -> (board, centers)
    board   {(x, y): (side, kind)}
    centers {(x, y): (u, v)}  해당 칸에 할당된 현재 검출 박스 중심 픽셀
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

from rules import Game, Position, detect_top_side, infer_move, other  # noqa: E402
from observations import TargetNotFound

SIDE_KO = {"blue": "초", "red": "한"}
KIND_KO = {"king": "궁", "cha": "차", "po": "포", "horse": "마", "sang": "상", "sa": "사", "soldier": "졸"}
POSES_FILE = WS / "src" / "janggi" / "calibration" / "config" / "robot_poses.yaml"
DEFAULT_OBSERVE_JOINTS = [0.0, -13.694, 134.216, 0.0, 59.485, 90.0]  # = 실물에서 저장한 홈


def load_observe_joints() -> list[float]:
    """로봇이 카메라를 가리지 않는 관찰 자세 = 보정 도구의 홈 자세 (robot_poses.yaml, 없으면 기본값)."""
    try:
        j = [float(v) for v in yaml.safe_load(POSES_FILE.read_text(encoding="utf-8"))["home_joint_deg"]]
        if len(j) == 6:
            return j
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        pass
    # Match pick_place2's configured home when no explicit observation pose exists.
    package_config = WS / 'src/janggi_pick_place2/config/pick_place.yaml'
    try:
        params = yaml.safe_load(package_config.read_text())['/janggi_pick_place2']['ros__parameters']
        joints = [float(v) for v in params['home_joint_deg']]
        if len(joints) == 6:
            return joints
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
    piece_height_mm: float = 5.0     # 말 높이 (박스 중심 = 말 윗면 중심)
    max_offset_mm: float = 18.0       # 카메라로 찾은 말이 교차점에서 이만큼 넘게 떨어지면 교차점 위치로 집는다
    observe_joints: list[float] = field(default_factory=load_observe_joints)
    observe_vel: float = 30.0         # 관찰 자세 이동 속도 [deg/s]
    command_settings: object = None  # PickPlace2 profiles also used by --no-motion preview.
    manipulation: object = None
    virtual: bool = False           # Gazebo uses a flat board surface at base Z=-20 mm.


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
        self.winner = None
        self.victory_pending = False
        self.last_execution_completed = False

    # ------------------------------------------------------------------ 시작
    def start(self, board, *, observe=True) -> Game:
        self.winner = None
        self.victory_pending = False
        self.last_execution_completed = False
        top = detect_top_side(board)
        self.game = Game(dict(board), top_side=top, turn=self.cfg.human_side)
        self.log(f"새 대국: 사람 {SIDE_KO[self.cfg.human_side]}, 로봇 {SIDE_KO[self.robot_side]}, "
                 f"위쪽 {SIDE_KO[top]}, 말 {len(board)}개")
        if observe:
            self.go_observe()
        return self.game

    def go_observe(self):
        if self.picker is not None:
            if hasattr(self.picker, 'go_observe'):
                settings = getattr(self.picker, 'settings', None)
                self.picker.go_observe(
                    self.cfg.observe_joints,
                    settings.joint_velocity_deg_s if settings else self.cfg.observe_vel,
                    settings.joint_acceleration_deg_s2 if settings else self.cfg.observe_vel)
            else:  # Legacy isolated test picker, not the realtime UI path.
                self.picker.mover.move_joint(self.cfg.observe_joints, self.cfg.observe_vel, self.cfg.observe_vel)

    # ------------------------------------------------------------------ 1-2. 사람 수 읽기
    def apply_human_move(self, observed) -> tuple | None:
        """인식된 판을 검증 없이 그대로 게임 판으로 쓰고 로봇 차례로 넘긴다.

        인식이 완벽하지 않아 이전 판과의 차이·규칙 검사로 막지 않는다. 한 수로 읽히면 기보용으로 로그만 남긴다.
        """
        g = self.game
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
        g.top_side = detect_top_side(observed, default=g.top_side)
        g.turn = self.robot_side
        return mv

    # ------------------------------------------------------------------ 3. 엔진
    def grasp_from_camera(self, cell, centers, kind=None) -> tuple[np.ndarray, str]:
        """Detected piece centre only; never substitute a nominal grid intersection."""
        pc = self.picker.cfg if self.picker else None
        settings = getattr(self.cfg,'command_settings',None)
        group = None if kind is None else 2 if kind=='king' else 0 if kind in ('sa','soldier') else 1
        grasp_h = (self.picker.grasp_height_for_kind(kind) if kind is not None
                   and hasattr(self.picker,'grasp_height_for_kind') else
                   settings.piece_grip_heights_mm[group] if settings is not None and group is not None else
                   pc.grasp_height_mm if pc else 0.0)
        def command_point(point):
            if kind is not None and hasattr(self.picker,'command_target_for_kind'):
                return self.picker.command_target_for_kind(point,kind)
            if settings is not None and group is not None:
                point=np.asarray(point,float).copy()
                point[2]=self.geom.command_surface_z(point, settings, virtual=getattr(self.cfg, 'virtual', False))+grasp_h
            return point
        uv = centers.get(cell)
        if uv is None:
            raise TargetNotFound(f"{cell}: 집을 말의 검출 중심이 없습니다. 격자점으로 대체하지 않습니다.")
        top_height = (self.picker.piece_height_for_kind(kind) if kind is not None
                      and hasattr(self.picker,'piece_height_for_kind') else
                      settings.piece_thicknesses_mm[group] if settings is not None and group is not None else
                      self.cfg.piece_height_mm)
        top = np.asarray(self.cam.pixel_to_base(*uv, height_mm=top_height),float)
        if top.shape != (3,) or not np.isfinite(top).all():
            raise RuntimeError(f"{cell}: 말 중심을 유효한 베이스 좌표로 변환하지 못했습니다.")
        # Preserve the detected centre XY exactly. Only the Z target changes.
        p = top.copy()
        n = self.geom.normal()
        p[2] += (grasp_h-self.geom.height_above(top))/n[2]

        return command_point(p), "camera"

    def _winning_king_capture(self):
        """Presentation ending: an immediately capturable king ends the game.

        Use the existing local move/own-king-safety rules, not a straight-line
        proximity heuristic. Other moves still come from Fairy-Stockfish.
        """
        g=self.game
        if g.turn != self.robot_side:
            return None
        enemy=[c for c,p in g.board.items() if p==(self.cfg.human_side,'king')]
        own=[c for c,p in g.board.items() if p==(self.robot_side,'king')]
        if len(enemy)!=1 or len(own)!=1:
            return None
        dst=enemy[0];position=Position(g.board,g.top_side)
        candidates=[src for src,p in g.board.items() if p[0]==self.robot_side and dst in position.legal_moves(src)]
        if not candidates:
            return None
        src=min(candidates,key=lambda c:(abs(c[0]-dst[0])+abs(c[1]-dst[1]),c))
        from engine import EngineResult, square_name
        return EngineResult(square_name(src,g.top_side)+square_name(dst,g.top_side),src,dst,
                            score='궁 포획 승리 수')

    def plan(self, centers) -> RobotPlan:
        g = self.game
        if not g.board:
            return RobotPlan(None, score="현재 분류된 말이 없음")
        res = self._winning_king_capture()
        if res is None:
            res = self.engine.best_move(g.board, g.top_side, g.turn, self.cfg.movetime_ms)
        if res.no_move:
            return RobotPlan(None, score=res.score)
        plan = RobotPlan(res.uci, res.src, res.dst, g.board.get(res.src), g.board.get(res.dst), res.score)
        if plan.is_pass:
            return plan
        if plan.piece is None or plan.piece[0] != self.robot_side:
            raise RuntimeError(f"엔진 수 {res.uci}의 출발 칸 {res.src}에 로봇 말이 없다 (판 방향 확인)")
        plan.pick_mm, plan.pick_from = self.grasp_from_camera(res.src, centers, plan.piece[1])
        if plan.captured:
            plan.capture_mm, plan.capture_from = self.grasp_from_camera(res.dst, centers, plan.captured[1])
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
        options=getattr(self.cfg,'manipulation',None) if hasattr(self,'cfg') else None
        coordinated=(options is None or options.adaptive_grasp_enabled or options.refresh_before_pick or options.retry_move_enabled)
        if self.picker is not None and getattr(self,'vision',None) is not None and coordinated:
            from manipulation import ManipulationCoordinator, ManipulationOptions
            completed=ManipulationCoordinator(self,self.vision,self.cfg.manipulation or ManipulationOptions(),
                                             lambda:self._check_stop(stop_check)).execute(plan)
            if completed is False:
                return False
        elif self.picker is not None:
            if hasattr(self.picker,'set_operation_yaw'):
                self.picker.set_operation_yaw(None)
            if plan.captured:
                self.log(f"  잡은 말 {piece_name(plan.captured)} 치우기 ({plan.capture_from})")
                if hasattr(self.picker, 'set_piece_kind'):
                    self.picker.set_piece_kind(plan.captured[1])
                self.picker.pick_abs(plan.capture_mm)
                if stop_check():
                    raise RuntimeError("정지")
                self.picker.discard()
            if stop_check():
                raise RuntimeError("정지")
            self.log(f"  {piece_name(plan.piece)} 집기 ({plan.pick_from}) → {plan.dst} 놓기")
            if hasattr(self.picker, 'set_piece_kind'):
                self.picker.set_piece_kind(plan.piece[1])
            self.picker.pick_abs(plan.pick_mm)
            if stop_check():
                raise RuntimeError("정지")
            self.picker.place(*plan.dst)
        g.move(plan.src, plan.dst, check=False)
        self.log(f"로봇: {plan.describe()}")
        return True

    def _finish_checkmate(self, centers, stop_check):
        """After confirmed mate, physically collect the losing king for spectators.

        Check is never enough: local checkmate and no engine legal replies must
        agree before this presentation-only cleanup is allowed.
        """
        g=self.game
        if g.turn != self.cfg.human_side or g.status() != '외통':
            return False
        if self.has_moves(self.cfg.human_side):
            return False
        kings=[c for c,p in g.board.items() if p==(self.cfg.human_side,'king')]
        if len(kings)!=1:
            return False
        cell=kings[0];self.victory_pending=True
        self.log('[승리 마무리] 외통 확인 · 상대 궁을 묘지로 옮깁니다.')
        if self.picker is not None:
            vision=getattr(self,'vision',None)
            if vision is not None and hasattr(vision,'fresh_state'):
                _,centers=vision.fresh_state()
            point,_=self.grasp_from_camera(cell,centers,'king')
            if vision is not None:
                from manipulation import ManipulationCoordinator, ManipulationOptions
                options=getattr(self.cfg,'manipulation',None) or ManipulationOptions()
                coordinator=ManipulationCoordinator(self,vision,options,lambda:self._check_stop(stop_check))
                _,cleared=coordinator.capture_only((self.cfg.human_side,'king'),point)
                self._execution_observed=True
                if not cleared:
                    self.last_execution_completed=False
                    self.log('[궁 포획 미확인] 승리 선언을 보류합니다. 현재 판에서 다시 시도하세요.')
                    return False
            else:
                self._check_stop(stop_check)
                self.picker.set_piece_kind('king')
                self.picker.pick_abs(point)
                self._check_stop(stop_check)
                self.picker.discard()
                self.go_observe()
        g.board.pop(cell,None)
        return True

    @staticmethod
    def _check_stop(stop_check):
        if stop_check():
            raise RuntimeError('로봇 정지 요청')

    def has_moves(self, side) -> bool:
        g = self.game
        return bool(self.engine.perft_moves(g.board, g.top_side, side))

    # ------------------------------------------------------------------ 한 턴 전체
    def plan_with_recovery(self, centers, stop_check):
        for attempt in range(3):
            self._check_stop(stop_check)
            try:return self.plan(centers)
            except TargetNotFound:
                vision=getattr(self,'vision',None)
                if attempt==2 or self.picker is None or vision is None:raise
                from manipulation import ManipulationCoordinator, ManipulationOptions
                coordinator=ManipulationCoordinator(self,vision,self.cfg.manipulation or ManipulationOptions(),
                                                     lambda:self._check_stop(stop_check))
                self.log(f'[계획 영상 복구 {attempt+1}/2] 홈 복귀 후 검출 중심을 다시 읽습니다.')
                coordinator.notify_retry('vision')
                coordinator.observe_home()
                _,centers=vision.fresh_state()

    def robot_turn(self, centers, stop_check=lambda: False) -> RobotPlan:
        if getattr(self,'winner',None) is not None:
            return RobotPlan(None,score='대국 종료')
        t0 = time.time()
        plan = self.plan_with_recovery(centers,stop_check)
        self.last_plan = plan
        if plan.src is not None and not plan.is_pass:
            callback = getattr(self, 'on_move_planned', None)
            if callback is not None:
                callback(plan.src, plan.dst)
        self.log(f"엔진: {plan.describe()} ({time.time() - t0:.1f}s)")
        if plan.uci is None:
            self.log(f"로봇({SIDE_KO[self.robot_side]})이 둘 수 없다 — 사람 승")
            return plan
        self._execution_observed=False
        self.last_execution_completed=False
        captured_king=bool(plan.captured and plan.captured[1]=='king')
        if captured_king:self.victory_pending=True
        if self.execute(plan, stop_check) is False:
            return plan
        self.last_execution_completed=True
        if not self._execution_observed:
            self.go_observe()
        if captured_king or self._finish_checkmate(centers,stop_check):
            self.winner=self.robot_side
            self.victory_pending=False
            ending='상대 궁 묘지 포획 완료' if self.picker is not None else '궁 포획 승리 수 계산 (로봇 동작 없음)'
            self.log(f'[대국 종료] {ending} · {SIDE_KO[self.winner]} 승리')
        return plan

    def human_done(self, vision, stop_check=lambda: False) -> RobotPlan:
        """'착수 완료' 한 번에 하는 일: 판 읽기 → 사람 수 반영 → 엔진 → 로봇 → 관찰 자세."""
        if self.winner is not None:
            return RobotPlan(None,score='대국 종료')
        board, centers = vision.fresh_state()
        self.apply_human_move(board)
        return self.robot_turn(centers, stop_check)
