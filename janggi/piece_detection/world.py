"""카메라 픽셀 ↔ 로봇 base 좌표 (카메라-로봇 보정 결과 사용).

YOLO 박스 중심은 장기판 교차점과 꼭 맞지 않는다 (말이 칸 가운데서 비켜 있거나 손으로 대충 놓음).
로봇이 말을 집을 때는 교차점 대신 이 변환으로 구한 '실제 말 위치'로 간다 (action.move_abs).

    cam = CameraToBase.from_calibration()
    top = cam.pixel_to_base(u, v, height_mm=12)   # 박스 중심 → 말 윗면 중심 (base, mm)

픽셀 → 왜곡 제거(OpenCV, RealSense inverse_brown_conrady와 같은 결과) → 카메라 광선 → T_base_camera로 base에 옮김
→ 판 평면을 법선 방향으로 height_mm 올린 평면과 교차.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

from .board_mapping import DEFAULT_CALIBRATION


class CameraToBase:
    def __init__(self, K, dist, T_base_camera, plane_normal, plane_d_m):
        self.K = np.asarray(K, float)
        self.dist = np.asarray(dist, float)
        T = np.asarray(T_base_camera, float)
        self.R, self.t_mm = T[:3, :3], T[:3, 3] * 1000.0
        n = np.asarray(plane_normal, float)
        self.n = n / np.linalg.norm(n)
        self.d_mm = float(plane_d_m) * 1000.0 / np.linalg.norm(n)   # n·p + d = 0 (p mm)

    @classmethod
    def from_calibration(cls, path: str | Path = DEFAULT_CALIBRATION) -> "CameraToBase":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"보정 파일이 없다: {path}")
        d = yaml.safe_load(path.read_text(encoding="utf-8"))
        i = d["camera"]["color_intrinsics"]
        K = [[i["fx"], 0, i["ppx"]], [0, i["fy"], i["ppy"]], [0, 0, 1]]
        return cls(K, i.get("coeffs", [0] * 5), d["T_base_camera"]["matrix"],
                   d["board_plane_base"]["normal"], d["board_plane_base"]["d"])

    def ray(self, u: float, v: float) -> np.ndarray:
        """픽셀 → base 좌표계 광선 방향 (단위 벡터). 원점은 self.t_mm."""
        xy = cv2.undistortPoints(np.array([[[u, v]]], float), self.K, self.dist).ravel()
        d = self.R @ np.array([xy[0], xy[1], 1.0])
        return d / np.linalg.norm(d)

    def pixel_to_base(self, u: float, v: float, height_mm: float = 0.0) -> np.ndarray:
        """픽셀이 가리키는 점 중 판 위 height_mm 평면에 있는 점 (base, mm)."""
        d = self.ray(u, v)
        den = self.n @ d
        if abs(den) < 1e-9:
            raise ValueError("광선이 판과 평행하다")
        s = -(self.n @ self.t_mm + self.d_mm - height_mm) / den
        if s <= 0:
            raise ValueError("판이 카메라 뒤에 있다")
        return self.t_mm + s * d

    def base_to_pixel(self, p_mm) -> np.ndarray:
        """base 점(mm) → 픽셀 (왜곡 포함). 시뮬레이터에서 가짜 검출 박스를 만들 때 쓴다."""
        pc = self.R.T @ (np.asarray(p_mm, float) - self.t_mm)
        uv, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.dist)
        return uv.ravel()

    def height_above(self, p_mm) -> float:
        return float(self.n @ np.asarray(p_mm, float) + self.d_mm)
