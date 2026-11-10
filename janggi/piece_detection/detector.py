"""YOLO 장기말 위치 인식기 (models/weights.pt). 기존 14클래스 모델도 지원한다.

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
    side: str            # 'blue' | 'red' | 'unknown' (위치만 검출)
    kind: str            # 말 종류 또는 'piece' (종류 미분류)
    conf: float
    box: tuple[float, float, float, float]  # x1 y1 x2 y2 (pixel)
    board: tuple[int, int] | None = field(default=None)   # 할당된 장기판 교차점 (x, y)
    board_dist: float | None = field(default=None)        # 교차점까지 거리 (칸 단위)
    classification_conf: float | None = field(default=None)  # 2단계 분류 확률; conf는 YOLO 검출 확률 유지

    piece_id: str | None = field(default=None)
    geometric_size_mm: float | None = field(default=None)
    tracking_only: bool = field(default=False)
    classification_preview: bool = field(default=False)

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2, (y1 + y2) / 2


def deduplicate_detections(detections: list[Detection], iou_threshold: float = 0.5) -> list[Detection]:
    """클래스 무관 NMS. 표시/통계용이며 원본 검출/교차점 할당은 바꾸지 않는다."""
    if not detections:
        return []
    boxes = np.asarray([d.box for d in detections], dtype=float)
    areas = np.prod(np.maximum(boxes[:, 2:] - boxes[:, :2], 0), axis=1)
    order = np.argsort(-np.asarray([d.conf for d in detections]), kind="stable")
    kept = []
    while len(order):
        chosen, rest = order[0], order[1:]
        kept.append(detections[chosen])
        lo = np.maximum(boxes[chosen, :2], boxes[rest, :2])
        hi = np.minimum(boxes[chosen, 2:], boxes[rest, 2:])
        intersection = np.prod(np.maximum(hi - lo, 0), axis=1)
        union = areas[chosen] + areas[rest] - intersection
        iou = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
        order = rest[iou < iou_threshold]
    return kept


def count_unique_detections(detections: list[Detection], iou_threshold: float = 0.5) -> int:
    return len(deduplicate_detections(detections, iou_threshold))


class PieceDetector:
    def __init__(self, model_path: str | Path = DEFAULT_MODEL, conf: float = 0.5, iou: float = 0.5,
                 imgsz: int = 640, device: str | None = None):
        from ultralytics import YOLO
        import torch

        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"모델 파일이 없다: {model_path}")
        self.model = YOLO(str(model_path))
        self.names = [self.model.names[i] for i in range(len(self.model.names))]
        self.position_only = self.names == ["janggi_dol"]
        if not self.position_only and self.names != CLASS_NAMES:
            raise ValueError(f"지원하지 않는 모델 클래스: {self.names} (janggi_dol 1종 또는 기존 14종 필요)")
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
            side, kind = ("unknown", "piece") if self.position_only else parse_label(self.names[c])
            box = (float(b[0]) + ox, float(b[1]) + oy, float(b[2]) + ox, float(b[3]) + oy)
            out.append(Detection(int(c), self.names[c], side, kind, float(p), box))
        return out
