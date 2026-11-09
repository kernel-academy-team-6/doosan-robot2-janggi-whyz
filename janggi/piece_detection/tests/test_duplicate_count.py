import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from piece_detection.detector import Detection, count_unique_detections


def test_cross_class_duplicates_count_once_without_changing_detections():
    detections = [
        Detection(0, 'blue cha', 'blue', 'cha', .9, (0, 0, 20, 20), (0, 0)),
        Detection(3, 'blue po', 'blue', 'po', .8, (1, 1, 21, 21)),
        Detection(7, 'red cha', 'red', 'cha', .7, (30, 0, 50, 20)),
    ]
    before = [vars(d).copy() for d in detections]
    assert count_unique_detections(detections) == 2
    assert [vars(d) for d in detections] == before


def test_neighboring_pieces_with_small_overlap_stay_separate():
    detections = [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, box)
                  for box in [(0, 0, 20, 20), (18, 0, 38, 20)]]
    assert count_unique_detections(detections) == 2


def test_empty_frame_and_exact_duplicates():
    assert count_unique_detections([]) == 0
    detections = [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, (0, 0, 20, 20)) for _ in range(3)]
    assert count_unique_detections(detections) == 1
