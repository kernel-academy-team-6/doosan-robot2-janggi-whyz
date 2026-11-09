"""YOLO 장기말 인식기 (models/weights.pt).

    from piece_detection import PieceDetector
    det = PieceDetector()              # GPU가 있으면 자동으로 씀
    pieces = det.detect(bgr_image)     # list[Detection]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .labels import CLASS_NAMES, parse_label

WS = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = WS / "models" / "weights.pt"


@dataclass
class Detection:
    cls_id: int
    name: str            # 모델 라벨 ('blue horse')
    side: str            # 'blue' | 'red'
    kind: str            # 'cha' 'horse' 'king' 'po' 'sa' 'sang' 'soldier'
    conf: float
    box: tuple[float, float, float, float]  # x1 y1 x2 y2 (pixel)
    board: tuple[int, int] | None = field(default=None)   # 할당된 장기판 교차점 (x, y)
    board_dist: float | None = field(default=None)        # 교차점까지 거리 (칸 단위)

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2, (y1 + y2) / 2


class PieceDetector:
    def __init__(self, model_path: str | Path = DEFAULT_MODEL, conf: float = 0.5, iou: float = 0.5,
                 imgsz: int = 640, device: str | None = None):
        from ultralytics import YOLO
        import torch

        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"모델 파일이 없다: {model_path}")
        self.model = YOLO(str(model_path))
        names = [self.model.names[i] for i in range(len(self.model.names))]
        if names != CLASS_NAMES:
            raise ValueError(f"모델 클래스가 labels.CLASS_NAMES와 다르다: {names}")
        self.device = device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.conf, self.iou, self.imgsz = conf, iou, imgsz
        # 첫 추론은 느리므로 미리 한 번 돌린다
        self.detect(np.zeros((480, 640, 3), np.uint8))

    def detect(self, bgr: np.ndarray, conf: float | None = None, roi=None) -> list[Detection]:
        """roi=(x0, y0, x1, y1)이면 그 영역만 잘라 인식한다. 말이 모델 입력에서 더 크게 보여 작은 말을 잘 찾는다.
        박스 좌표는 항상 원본 이미지 기준이다."""
        ox = oy = 0
        if roi is not None:
            x0, y0, x1, y1 = (int(v) for v in roi)
            bgr = bgr[y0:y1, x0:x1]
            ox, oy = x0, y0
        r = self.model.predict(bgr, conf=self.conf if conf is None else conf, iou=self.iou, imgsz=self.imgsz,
                               device=self.device, verbose=False)[0]
        out = []
        if r.boxes is None or len(r.boxes) == 0:
            return out
        xyxy = r.boxes.xyxy.cpu().numpy()
        cls = r.boxes.cls.cpu().numpy().astype(int)
        cf = r.boxes.conf.cpu().numpy()
        for b, c, p in zip(xyxy, cls, cf):
            side, kind = parse_label(CLASS_NAMES[c])
            box = (float(b[0]) + ox, float(b[1]) + oy, float(b[2]) + ox, float(b[3]) + oy)
            out.append(Detection(int(c), CLASS_NAMES[c], side, kind, float(p), box))
        return out
