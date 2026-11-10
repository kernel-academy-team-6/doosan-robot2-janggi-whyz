import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from piece_motion import PieceMotionMonitor, draw_piece_regions
from piece_detection.detector import Detection


def detection(box=(30, 30, 60, 60), confidence=.9):
    # 교차점 미할당인 말도 센서가 된다.
    return Detection(0, 'janggi_dol', 'unknown', 'piece', confidence, box)


def warm(monitor):
    for t in (0., .1, .2, .3):
        assert monitor.update([detection()], t) == []


def test_confidence_drop_is_local_and_independent_of_board_or_class():
    monitor = PieceMotionMonitor()
    warm(monitor)
    region, = monitor.update([detection(confidence=.7)], .4)
    assert region['reasons'] == ['CONF'] and region['visible']
    assert region['baseline_confidence'] == .9
    assert region['sensor_id'] == 1
    # 비교 기준이 낮아진 값으로 오염되지 않는다.
    region, = monitor.update([detection(confidence=.7)], 1.0)
    assert region['baseline_confidence'] == .9
    assert monitor.update([detection()], 1.1)  # 짧은 정상 복귀 뒤 경고 유지
    assert monitor.update([detection()], 1.6) == []


def test_size_change_keeps_original_region_and_sensor_id():
    monitor = PieceMotionMonitor()
    warm(monitor)
    region, = monitor.update([detection((37, 37, 53, 53))], .4)
    assert region['reasons'] == ['SIZE']
    assert region['sensor_id'] == 1 and region['box'] == [30, 30, 60, 60]
    assert region['geometric_size_px'] == 16


def test_moves_follow_box_then_clear_after_stopping():
    monitor = PieceMotionMonitor()
    warm(monitor)
    region, = monitor.update([detection((38, 30, 68, 60))], .4)
    assert region['reasons'] == ['MOVE'] and region['sensor_id'] == 1
    # 낮거나 불규칙한 FPS에서도 직전 관측으로부터의 이동을 감지한다.
    region, = monitor.update([detection((46, 30, 76, 60))], 1.0)
    assert region['reasons'] == ['MOVE'] and region['sensor_id'] == 1
    for t in (1.1, 1.3, 1.7):
        regions = monitor.update([detection((46, 30, 76, 60))], t)
    assert regions == []


def test_missing_region_expires_and_returning_piece_reconnects():
    monitor = PieceMotionMonitor()
    warm(monitor)
    assert monitor.update([], .4) == []
    region, = monitor.update([], .5)
    assert region['reasons'] == ['MISS'] and not region['visible']
    assert monitor.update([detection()], .6) == []
    assert len(monitor.sensors) == 1
    monitor.update([], .7)
    assert monitor.update([], .8)
    assert monitor.update([], 3.0) == [] and not monitor.sensors


def test_multiple_piece_sensors_and_reset():
    monitor = PieceMotionMonitor()
    for t in (0., .1, .2, .3):
        assert monitor.update([detection(), detection((100, 100, 130, 130))], t) == []
    region, = monitor.update([detection(confidence=.5), detection((100, 100, 130, 130))], .4)
    assert region['sensor_id'] == 1 and region['reasons'] == ['CONF']
    monitor.reset()
    assert monitor.update([detection(confidence=.5)], .5) == []


def test_region_renderer_crop_coordinates_and_input_preserved():
    frame = np.full((100, 100, 3), 100, np.uint8)
    original = frame.copy()
    region = dict(sensor_id=1, box=[100, 100, 130, 130], visible=False, reasons=['MISS'])
    result = draw_piece_regions(frame.copy(), [region], offset=(80, 80))
    assert np.array_equal(frame, original)
    assert not np.array_equal(result, original)
    assert result[25, 25, 2] > result[25, 25, 0]  # 빨간 의심 영역이 잘라낸 화면의 올바른 위치에 표시됨


def test_worker_runs_piece_sensors_during_global_obstruction_and_toggles():
    import time
    from types import SimpleNamespace
    from app import Worker

    class Source:
        source_loop = 0
        def __init__(self):
            self.start = time.monotonic()
            self.n = 0
        def read(self):
            time.sleep(.01)
            phase = int(time.monotonic()-self.start >= .15)
            self.n += 1
            frame = np.full((100, 100, 3), 200 if phase and self.n % 2 else 0, np.uint8)
            frame[:, :, 0] = phase
            return frame

    class Detector:
        def detect(self, frame, **kwargs):
            return [detection(confidence=.5 if frame[0, 0, 0] else .9)]

    args = SimpleNamespace(window=2, full_frame=False, log_counts=False, conf=.35, max_dist=.45,
                           show_piece_motion=True, obstruction_settle=.01)
    worker = Worker(Source(), Detector(), None, args)
    try:
        deadline = time.monotonic()+2
        found = False
        while time.monotonic() < deadline and worker.error is None:
            with worker.lock:
                found = worker.view_status['state'] == 'suspected' and any('CONF' in r['reasons'] for r in worker.local_regions)
            if found:
                break
            time.sleep(.005)
        assert found and worker.error is None
        worker.show_piece_motion = False
        start_frame = worker.frames
        while worker.frames < start_frame+3 and time.monotonic() < deadline:
            time.sleep(.01)
        with worker.lock:
            assert worker.local_regions == [] and not worker.piece_motion.sensors
    finally:
        worker.stop()
