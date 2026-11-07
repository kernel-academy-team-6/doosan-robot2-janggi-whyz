"""카메라 픽셀 → 장기판 교차점 매핑과 판 상태 안정화.

장기판 좌표 (src/janggi/action과 같음):
    (0,0) 좌상단 · (8,0) 우상단 · (8,9) 우하단 · (0,9) 좌하단   x: 0~8, y: 0~9

매핑에는 보정 결과(src/janggi/calibration/config/calibration.yaml)의 네 모서리 픽셀을 쓴다.
말은 판보다 위로 솟아 있어서, 카메라 중심에서 멀수록 말 윗면이 바깥쪽으로 밀려 보인다(시차).
카메라-판 거리 Z와 말 높이 h로 검출 중심을 판 평면 위치로 되돌린 뒤 교차점에 할당한다.
"""

from __future__ import annotations

from collections import Counter, deque
from pathlib import Path

import cv2
import numpy as np
import yaml

from .detector import Detection

WS = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION = WS / "src" / "janggi" / "calibration" / "config" / "calibration.yaml"
COLS, ROWS = 9, 10
CORNER_NAMES = ("top_left", "top_right", "bottom_right", "bottom_left")
CORNER_BOARD = np.array([[0, 0], [8, 0], [8, 9], [0, 9]], dtype=np.float64)

Cell = tuple[int, int]
Piece = tuple[str, str]          # (side, kind)
Board = dict[Cell, Piece]


class BoardMapper:
    def __init__(self, corner_px, principal_px=None, camera_dist_mm: float | None = None,
                 piece_height_mm: float = 12.0):
        self.corner_px = np.asarray(corner_px, dtype=np.float64).reshape(4, 2)
        self.H = cv2.getPerspectiveTransform(self.corner_px.astype(np.float32), CORNER_BOARD.astype(np.float32))
        self.H_inv = np.linalg.inv(self.H)
        self.principal = None if principal_px is None else np.asarray(principal_px, dtype=np.float64)
        self.Z = camera_dist_mm
        self.piece_height_mm = piece_height_mm
        self._grid = np.array([[self.board_to_pixel(x, y) for x in range(COLS)] for y in range(ROWS)])

    @classmethod
    def from_calibration(cls, path: str | Path = DEFAULT_CALIBRATION, piece_height_mm: float = 12.0):
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"보정 파일이 없다: {path} (src/janggi/calibration에서 먼저 보정한다)")
        d = yaml.safe_load(path.read_text(encoding="utf-8"))
        by = {c["name"]: c for c in d["corners"]}
        px = [by[n]["pixel"] for n in CORNER_NAMES]
        intr = d["camera"]["color_intrinsics"]
        z = None
        try:  # 카메라 원점에서 판 평면까지 거리
            t = np.array(d["T_base_camera"]["translation"], dtype=float)
            n = np.array(d["board_plane_base"]["normal"], dtype=float)
            z = abs(float(n @ t + d["board_plane_base"]["d"])) * 1000.0
        except (KeyError, TypeError):
            pass
        return cls(px, (intr["ppx"], intr["ppy"]), z, piece_height_mm)

    # ------------------------------------------------------------------ 변환
    def _parallax(self, p: np.ndarray) -> np.ndarray:
        """말 윗면 픽셀 → 같은 말의 판 평면 위치 픽셀 (카메라가 판을 거의 수직으로 내려다본다고 가정)."""
        if self.principal is None or not self.Z or self.piece_height_mm <= 0:
            return p
        s = (self.Z - self.piece_height_mm) / self.Z
        return self.principal + (p - self.principal) * s

    def pixel_to_board(self, u: float, v: float, on_piece: bool = True) -> np.ndarray:
        """픽셀 → 장기판 실수 좌표 (x, y). on_piece면 말 높이 시차를 보정한다."""
        p = np.array([u, v], dtype=np.float64)
        if on_piece:
            p = self._parallax(p)
        q = self.H @ np.array([p[0], p[1], 1.0])
        return q[:2] / q[2]

    def board_to_pixel(self, x: float, y: float) -> np.ndarray:
        """장기판 좌표 → 판 평면 위 픽셀."""
        q = self.H_inv @ np.array([x, y, 1.0])
        return q[:2] / q[2]

    def grid_pixels(self) -> np.ndarray:
        """(ROWS, COLS, 2) 교차점 픽셀 (판 평면). 화면에 격자를 그릴 때 쓴다. 미리 계산해 둔 값이다."""
        return self._grid

    # ------------------------------------------------------------------ 할당
    def assign(self, dets: list[Detection], max_dist: float = 0.45) -> Board:
        """검출마다 가장 가까운 교차점을 정하고(det.board), 교차점마다 신뢰도가 가장 높은 말을 남긴다."""
        best: dict[Cell, Detection] = {}
        for d in dets:
            bx, by = self.pixel_to_board(*d.center)
            cx, cy = int(round(bx)), int(round(by))
            dist = float(np.hypot(bx - cx, by - cy))
            d.board, d.board_dist = None, dist
            if not (0 <= cx < COLS and 0 <= cy < ROWS) or dist > max_dist:
                continue
            d.board = (cx, cy)
            if (cx, cy) not in best or d.conf > best[(cx, cy)].conf:
                best[(cx, cy)] = d
        for cell, d in best.items():  # 같은 칸에서 밀린 검출은 할당 해제 (화면 표시용)
            for o in dets:
                if o is not d and o.board == cell:
                    o.board = None
        return {cell: (d.side, d.kind) for cell, d in best.items()}


class BoardStabilizer:
    """최근 N프레임에서 칸마다 가장 많이 나온 값(빈칸 포함)을 확정 상태로 쓴다. 단일 프레임 오검출을 거른다."""

    def __init__(self, window: int = 8, min_ratio: float = 0.6):
        self.window, self.min_ratio = window, min_ratio
        self.history: deque[Board] = deque(maxlen=window)

    def update(self, board: Board) -> Board:
        self.history.append(dict(board))
        return self.state()

    def state(self) -> Board:
        if not self.history:
            return {}
        n = len(self.history)
        cells = set().union(*self.history)
        out: Board = {}
        for c in cells:
            votes = Counter(h.get(c) for h in self.history)
            val, cnt = votes.most_common(1)[0]
            if val is not None and cnt >= self.min_ratio * n:
                out[c] = val
        return out

    def reset(self):
        self.history.clear()
