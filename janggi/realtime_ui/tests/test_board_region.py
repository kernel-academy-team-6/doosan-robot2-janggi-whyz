from pathlib import Path
from types import SimpleNamespace
import sys
import time

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2])]
from board_region import BoardRecognitionRegion
from piece_detection.board_mapping import BoardMapper
from piece_detection.detector import Detection
from app import App, Worker

POLYGON = np.array([[100,20], [180,100], [100,180], [20,100]], dtype=float)


def detection(x, y, half=5):
    return Detection(0, 'blue sa', 'blue', 'sa', .9, (x-half,y-half,x+half,y+half))


def test_rotated_board_excludes_bounding_rectangle_corners_and_keeps_edge_pieces():
    region = BoardRecognitionRegion(POLYGON)
    center, edge, near, corner = detection(100,100), detection(100,20,15), detection(100,15), detection(40,40)
    assert region.filter([center,edge,near,corner], 5) == [center,edge,near]
    assert region.filter([near], 0) == []


def test_mask_is_cached_but_frame_pixels_remain_live_and_original_is_unchanged():
    region = BoardRecognitionRegion(POLYGON)
    frame = np.full((200,200,3), 200, np.uint8)
    masked, roi = region.prepare(frame, 5)
    assert roi == (15,15,186,186)
    assert np.all(masked[40,40] == 114) and np.all(masked[100,100] == 200)
    assert np.all(frame == 200)
    mask = region.mask
    next_frame, _ = region.prepare(np.full_like(frame, 90), 5)
    assert region.mask is mask and np.all(next_frame[100,100] == 90)
    region.prepare(frame, 35)
    assert region.mask is not mask and region.mask[40,40] == 255
    region.prepare(np.zeros((100,100,3), np.uint8), 0)
    assert region.mask.shape == (100,100)
    with pytest.raises(ValueError): region.prepare(frame, -1)


def test_board_outside_camera_frame_produces_empty_region():
    region = BoardRecognitionRegion(POLYGON + 500)
    _, roi = region.prepare(np.zeros((200,200,3), np.uint8), 30)
    assert roi == (200,200,200,200)
    assert not np.any(region.mask)


@pytest.mark.parametrize('full_frame', [False,True])
def test_worker_filters_before_board_assignment_and_keeps_original_display_frame(full_frame):
    class Source:
        def __init__(self): self.sent = False
        def read(self):
            if self.sent:
                time.sleep(.01); return None
            self.sent = True
            return np.full((200,200,3), 200, np.uint8)
    class Detector:
        def detect(self, frame, **kwargs):
            assert frame[40,40,0] == (200 if full_frame else 114)
            return [detection(100,100), detection(40,40), detection(100,20)]
    args = SimpleNamespace(window=2, full_frame=full_frame, calibration=None, log_counts=False,
                           roi_margin=5, conf=.35, max_dist=.45)
    worker = Worker(Source(), Detector(), BoardMapper(POLYGON), args)
    try:
        deadline = time.monotonic() + 2
        while not worker.frames and worker.error is None and time.monotonic() < deadline:
            time.sleep(.01)
        assert worker.error is None and worker.frames == 1
        frame, dets, raw, stable = worker.get()
        assert len(dets) == (3 if full_frame else 2)
        assert np.all(frame == 200)
    finally:
        worker.stop()


def test_ui_margin_rejects_negative_or_invalid_edits_without_changing_worker_input():
    class Var:
        def __init__(self,value): self.value=value
        def get(self): return self.value
        def set(self,value): self.value=value
    ui = App.__new__(App); ui.args = SimpleNamespace(roi_margin=30)
    ui.roi_margin_var = Var('10'); ui._set_roi_margin()
    assert ui.args.roi_margin == 10
    for bad in ('-1', 'bad', ''):
        ui.roi_margin_var.set(bad); ui._set_roi_margin()
        assert ui.args.roi_margin == 10 and ui.roi_margin_var.get() == '10'
