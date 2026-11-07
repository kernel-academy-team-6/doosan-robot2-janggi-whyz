"""장기말 인식 (YOLO, models/best.pt) + 장기판 교차점 매핑.

    from piece_detection import PieceDetector, BoardMapper, BoardStabilizer
    det = PieceDetector()
    mapper = BoardMapper.from_calibration()
    dets = det.detect(bgr)
    board = mapper.assign(dets)     # {(x, y): ('blue', 'horse'), ...}
"""

from .board_mapping import BoardMapper, BoardStabilizer, COLS, ROWS
from .labels import CLASS_NAMES, piece_char, parse_label


def __getattr__(name):  # PieceDetector는 torch/ultralytics를 불러오므로 필요할 때만 import
    if name in ("PieceDetector", "Detection"):
        from . import detector
        return getattr(detector, name)
    raise AttributeError(name)


__all__ = ["PieceDetector", "Detection", "BoardMapper", "BoardStabilizer", "CLASS_NAMES", "COLS", "ROWS",
           "piece_char", "parse_label"]
