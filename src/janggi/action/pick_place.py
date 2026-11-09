"""장기말 집기·놓기·버리기. BoardMover 위의 얇은 층.

    picker = PiecePicker(mover)
    picker.pick_abs(tip_xyz)      # 카메라로 찾은 말 위치(base, mm)에서 집는다 (move_abs)
    picker.place(x, y)            # 장기판 교차점에 놓는다 (move)
    picker.discard()              # 잡은 말을 판 바깥(보정 사각형 밖) 버리는 자리에 놓는다

높이는 모두 판 윗면에서 법선 방향 [mm]. 실제 로봇에서는 말 크기·그리퍼 개도에 맞춰 PickConfig를 조정한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .board import MAX_X, MAX_Y
from .mover import GRIPPER_CLOSE, BoardMover


@dataclass
class PickConfig:
    grasp_height_mm: float = 0.0      # 집을 때 팁 높이 (0 = 판 표면. 팁이 판에 닿아도 부딪히지 않는다)
    place_height_mm: float = 0.0      # 놓을 때 팁 높이 (0 = 판 표면)
    travel_mm: float = 40.0           # 옮길 때 팁 높이 (말을 든 채 다른 말 위를 지나간다)
    open_cmd: int = 350               # 집기 전 개도. 완전히 열면(0) 이웃 말에 손가락이 닿을 수 있다
    close_cmd: int = GRIPPER_CLOSE    # 집을 때 명령값 (말을 쥐면 그 자리에서 멈춘다)
    discard_x: float = -1.5           # 버리는 자리: 판 좌표 x (0 왼쪽 바깥 = 음수, 8 오른쪽 바깥 = 8보다 큼)
    discard_step: float = 1.0         # 버리는 자리 간격 [칸]
    discard_height_mm: float = 0.0    # 버릴 때 팁 높이 (판 평면 기준. 판 밖 바닥은 판보다 낮다)


class PiecePicker:
    """on_grip(cmd, tip_mm)은 그리퍼 명령 직후 불린다 (시뮬레이터가 말을 붙이고 떼는 데 쓴다)."""

    def __init__(self, mover: BoardMover, cfg: PickConfig | None = None,
                 on_grip: Callable[[int, np.ndarray], None] | None = None):
        self.mover, self.cfg, self.on_grip = mover, cfg or PickConfig(), on_grip
        self.discarded = 0

    def _grip(self, cmd: int):
        self.mover.gripper(cmd)
        if self.on_grip:
            self.on_grip(cmd, self.mover.current_tip())

    def grasp_point(self, top_center_mm, piece_height_mm: float = 12.0) -> np.ndarray:
        """말 윗면 중심(base, mm) → 집을 팁 위치 (같은 말 중심축에서 grasp_height 높이)."""
        b = self.mover.board
        p = np.asarray(top_center_mm, float)
        return p + b.normal() * (self.cfg.grasp_height_mm - b.height_above(p))

    def pick_abs(self, grasp_mm) -> np.ndarray:
        """base 좌표 grasp_mm(팁 목표)에서 말을 집고 이동 높이로 들어 올린다. 집은 팁 위치를 돌려준다."""
        c = self.cfg
        p = np.asarray(grasp_mm, float)
        b = self.mover.board
        above = p + b.normal() * (c.travel_mm - b.height_above(p))
        self.mover.move_abs(*above, travel_mm=c.travel_mm)   # 먼저 말 위로 (그리퍼를 열기 전에 다른 말과 떨어진다)
        self._grip(c.open_cmd)
        tip = self.mover.move_abs(*p, travel_mm=c.travel_mm)
        self._grip(c.close_cmd)
        self.mover.move_abs(*above, travel_mm=c.travel_mm)
        return tip

    def place(self, x: float, y: float) -> np.ndarray:
        """들고 있는 말을 장기판 (x, y) 교차점에 놓는다 (move 사용). 놓은 팁 위치를 돌려준다."""
        c = self.cfg
        tip = self.mover.move(x, y, c.place_height_mm, c.travel_mm)
        self._grip(c.open_cmd)
        self.mover.move(x, y, c.travel_mm, c.travel_mm)
        return tip

    def discard_spot(self, index: int | None = None) -> np.ndarray:
        """index번째 버리는 자리 (base, mm). 판 바깥 한 줄에 discard_step 간격, 다 차면 바깥으로 한 줄 더."""
        c = self.cfg
        i = self.discarded if index is None else index
        per_col = int(MAX_Y / c.discard_step) + 1
        col, row = divmod(i, per_col)
        out = -1.0 if c.discard_x < 0 else 1.0
        x = c.discard_x + out * col
        y = row * c.discard_step
        if 0 <= x <= MAX_X:
            raise ValueError(f"버리는 자리 x={x}가 판 안이다 (discard_x는 0보다 작거나 {MAX_X}보다 커야 한다)")
        return self.mover.board.tip_target(x, y, c.discard_height_mm, check=False)

    def discard(self) -> np.ndarray:
        """들고 있는 말을 다음 버리는 자리에 놓는다."""
        c = self.cfg
        spot = self.discard_spot()
        tip = self.mover.move_abs(*spot, travel_mm=c.travel_mm)
        self._grip(c.open_cmd)
        self.discarded += 1
        b = self.mover.board
        self.mover.move_abs(*(spot + b.normal() * (c.travel_mm - c.discard_height_mm)), travel_mm=c.travel_mm)
        return tip
