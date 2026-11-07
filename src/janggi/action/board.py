"""장기판 좌표 ↔ 로봇 base 좌표 변환. ROS 의존 없음 (pytest로 검증).

장기판 좌표계 (calibration과 같은 '장기판 기준' 방향):
    (0, 0) = 좌상단 교차점, (8, 0) = 우상단, (8, 9) = 우하단, (0, 9) = 좌하단
    x: 0~8 (가로 방향 교차점 9개), y: 0~9 (세로 방향 교차점 10개)

네 모서리의 base 좌표는 보정 때 그리퍼 팁을 직접 대서 잰 값(calibration.yaml의 corners[].base_xyz)이다.
판 안쪽 점은 네 모서리를 이중선형 보간한다. 카메라 변환을 거치지 않으므로 카메라 오차가 끼지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

COLS, ROWS = 9, 10  # 교차점 개수 (x 방향, y 방향)
MAX_X, MAX_Y = COLS - 1, ROWS - 1
CORNER_NAMES = ("top_left", "top_right", "bottom_right", "bottom_left")
DEFAULT_CALIBRATION = Path(__file__).resolve().parents[1] / "calibration" / "config" / "calibration.yaml"


def zyz_to_matrix(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """Doosan posx (A, B, C) ZYZ 오일러각 → 회전행렬. R = Rz(A) Ry(B) Rz(C)."""
    a, b, c = np.radians([a_deg, b_deg, c_deg])

    def rz(t):
        return np.array([[np.cos(t), -np.sin(t), 0.0], [np.sin(t), np.cos(t), 0.0], [0.0, 0.0, 1.0]])

    def ry(t):
        return np.array([[np.cos(t), 0.0, np.sin(t)], [0.0, 1.0, 0.0], [-np.sin(t), 0.0, np.cos(t)]])

    return rz(a) @ ry(b) @ rz(c)


@dataclass
class BoardGeometry:
    corners_mm: np.ndarray          # (4, 3) TL, TR, BR, BL 팁 위치, base, mm
    tool_abc: np.ndarray            # 보정 때 툴 자세 (A, B, C) deg
    tcp_offset_mm: np.ndarray       # posx(플랜지) 기준 팁 오프셋, 툴 좌표계 mm

    @classmethod
    def from_calibration(cls, path: str | Path = DEFAULT_CALIBRATION) -> "BoardGeometry":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"보정 파일이 없다: {path} (calibration/run.sh로 먼저 보정한다)")
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        by_name = {c["name"]: c for c in data["corners"]}
        missing = [n for n in CORNER_NAMES if n not in by_name]
        if missing:
            raise ValueError(f"보정 파일에 모서리가 없다: {missing}")
        corners = np.array([by_name[n]["base_xyz"] for n in CORNER_NAMES], dtype=float) * 1000.0
        # 네 모서리를 같은 툴 자세로 찍었으므로 평균 자세를 쓴다 (ZYZ 각을 직접 평균하지 않고 첫 점 자세 사용)
        abc = np.array(by_name["top_left"]["robot_posx"][3:6], dtype=float)
        offset = np.array(data.get("robot", {}).get("tcp_offset_mm", [0.0, 0.0, 0.0]), dtype=float)
        return cls(corners, abc, offset)

    # ------------------------------------------------------------------ 좌표 변환
    def board_to_base(self, x: float, y: float) -> np.ndarray:
        """장기판 좌표 (x, y) → 판 윗면 위 팁 위치 (base, mm). 이중선형 보간."""
        check_range(x, y)
        u, v = x / MAX_X, y / MAX_Y
        tl, tr, br, bl = self.corners_mm
        top = tl + (tr - tl) * u
        bottom = bl + (br - bl) * u
        return top + (bottom - top) * v

    def normal(self) -> np.ndarray:
        """판 윗면 법선 (위쪽, base). 네 모서리로 평면을 맞춘다."""
        c = self.corners_mm - self.corners_mm.mean(axis=0)
        n = np.linalg.svd(c)[2][-1]
        return n if n[2] > 0 else -n

    def tip_target(self, x: float, y: float, height_mm: float) -> np.ndarray:
        """(x, y) 교차점에서 판 법선 방향으로 height_mm 띄운 팁 목표 (base, mm)."""
        return self.board_to_base(x, y) + self.normal() * height_mm

    def flange_pose(self, tip_mm: np.ndarray) -> list[float]:
        """팁 목표 → move_line에 줄 posx [x, y, z, A, B, C] (플랜지 기준, TCP 미설정 가정)."""
        r = zyz_to_matrix(*self.tool_abc)
        flange = np.asarray(tip_mm, float) - r @ self.tcp_offset_mm
        return [*map(float, flange), *map(float, self.tool_abc)]

    def tip_from_posx(self, posx) -> np.ndarray:
        r = zyz_to_matrix(*posx[3:6])
        return np.asarray(posx[:3], float) + r @ self.tcp_offset_mm

    def spacing_mm(self) -> tuple[float, float]:
        tl, tr, br, bl = self.corners_mm
        w = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2 / MAX_X
        h = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2 / MAX_Y
        return float(w), float(h)


def check_range(x: float, y: float):
    if not (0 <= x <= MAX_X and 0 <= y <= MAX_Y):
        raise ValueError(f"장기판 좌표 범위 밖: ({x}, {y}). x는 0~{MAX_X}, y는 0~{MAX_Y}")
