"""분류를 꾸며내지 않는 위치 검출과 기존 14클래스 호환성을 검증한다."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from piece_detection.detector import PieceDetector
from piece_detection.board_mapping import BoardMapper, BoardStabilizer
from piece_detection.labels import CLASS_NAMES, piece_char


class Array:
    def __init__(self, value):
        self.value = np.array(value)

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class Boxes(SimpleNamespace):
    def __len__(self):
        return 1


@pytest.mark.parametrize('position_only,expected', [(True, ('unknown', 'piece')), (False, ('blue', 'cha'))])
def test_detection_crop_offsets_and_board_mapping(position_only, expected):
    boxes = Boxes(xyxy=Array([[10, 10, 30, 30]]), cls=Array([0]), conf=Array([.9]))
    detector = PieceDetector.__new__(PieceDetector)
    detector.position_only = position_only
    detector.names = ['janggi_dol'] if position_only else CLASS_NAMES
    detector.conf, detector.iou, detector.imgsz, detector.device = .35, .5, 640, 'cpu'
    detector.model = SimpleNamespace(predict=lambda *a, **kw: [SimpleNamespace(boxes=boxes)])
    detection, = detector.detect(np.zeros((200, 200, 3), np.uint8), roi=(20, 30, 100, 100))
    assert detection.box == (30, 40, 50, 60)
    assert (detection.side, detection.kind) == expected
    mapper = BoardMapper([(0, 0), (80, 0), (80, 90), (0, 90)])
    board = mapper.assign([detection])
    assert board == {(4, 5): expected}
    assert BoardStabilizer().update(board) == board
    assert piece_char(*expected) == ('알' if position_only else '차')
