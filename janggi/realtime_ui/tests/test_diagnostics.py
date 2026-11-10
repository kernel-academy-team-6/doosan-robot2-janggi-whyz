import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from diagnostics import ObstructionMonitor, BoxSizeEstimator, box_label
from app import draw_overlay
from piece_detection.detector import Detection


def stable_monitor():
    monitor = ObstructionMonitor()
    frame = np.zeros((100, 100, 3), np.uint8)
    for t in (0., .4, .9):
        status = monitor.update(frame, 32, 32, t)
    assert status['state'] == 'stable'
    return monitor, frame


def test_obstruction_motion_only_inside_board_and_settles():
    monitor = ObstructionMonitor()
    frame = np.zeros((100, 100, 3), np.uint8)
    polygon = [(30, 30), (70, 30), (70, 70), (30, 70)]
    for t in (0., .4, .9):
        status = monitor.update(frame, 32, 32, t, polygon)
    outside = frame.copy()
    outside[:20, :20] = 255
    status = monitor.update(outside, 32, 32, 1., polygon)
    assert status['state'] == 'stable' and status['motion'] == 0
    inside = outside.copy()
    inside[40:60, 40:60] = 255
    status = monitor.update(inside, 32, 32, 1.1, polygon)
    assert status['state'] == 'suspected' and status['motion'] > .03
    assert '움직임' in status['reason']
    for t in (1.2, 1.7, 2.1):
        status = monitor.update(inside, 32, 32, t, polygon)
    assert status['state'] == 'stable'


def test_count_oscillation_persistent_loss_and_reference_reset():
    monitor, frame = stable_monitor()
    monitor.update(frame, 31, 31, 1.)
    status = monitor.update(frame, 32, 32, 1.1)
    assert status['state'] == 'suspected' and '진동' in status['reason']
    for t in (3.2, 3.6, 4.1):
        status = monitor.update(frame, 32, 32, t)
    assert status['state'] == 'stable'
    for t in (4.2, 5., 7.):
        status = monitor.update(frame, 29, 29, t)
    assert status['state'] == 'suspected' and '감소' in status['reason']
    monitor.reset()
    for t in (8., 8.4, 8.9):
        status = monitor.update(frame, 29, 29, t)
    assert status['state'] == 'stable'


def test_single_capture_is_not_count_oscillation_and_empty_is_unknown():
    monitor, frame = stable_monitor()
    for t in (1., 1.4, 1.9):
        status = monitor.update(frame, 31, 31, t)
    assert status['state'] == 'stable'
    monitor.reset()
    for t in (0., .4, .9):
        status = monitor.update(frame, 0, None, t)
    assert status['state'] == 'unavailable'


def test_size_estimator_box_labels_and_overlay_selection():
    class Camera:
        def pixel_to_base(self, u, v, height_mm):
            return np.array([u*2, v*3, height_mm])

    estimator = BoxSizeEstimator(Camera(), (100, 100))
    detection = Detection(0, 'blue cha', 'blue', 'cha', .9, (20, 20, 40, 50), classification_conf=.8)
    assert estimator.measure(detection.box, (100, 100)) == (40., 90.)
    assert estimator.measure(detection.box, (200, 200)) is None
    assert box_label(detection) == 'D90/C80'
    assert box_label(detection, 'size-px') == '20.0x30.0px'
    assert box_label(detection, 'size-mm', estimator, (100, 100)) == '40.0x90.0mm~'
    assert 'mm N/A' in box_label(detection, 'size-mm', estimator, (200, 200))
    frame = np.zeros((100, 100, 3), np.uint8)
    class Glyphs:
        def put(self, *args):
            pass
    # 문자열 함수만이 아니라 실제 오버레이가 선택값을 적용하는지 확인한다.
    confidence = draw_overlay(frame, [detection], None, Glyphs())
    pixels = draw_overlay(frame, [detection], None, Glyphs(), label_mode='size-px')
    millimeters = draw_overlay(frame, [detection], None, Glyphs(), label_mode='size-mm', size_estimator=estimator)
    assert not np.array_equal(confidence, pixels)
    assert not np.array_equal(pixels, millimeters)
