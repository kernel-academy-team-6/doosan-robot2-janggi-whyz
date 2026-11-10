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
    assert region['sensor_id'] == 'piece_0001'
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
    assert region['sensor_id'] == 'piece_0001' and region['box'] == [30, 30, 60, 60]
    assert region['geometric_size_px'] == 16


def test_moves_follow_box_then_clear_after_stopping():
    monitor = PieceMotionMonitor()
    warm(monitor)
    region, = monitor.update([detection((38, 30, 68, 60))], .4)
    assert region['reasons'] == ['MOVE'] and region['sensor_id'] == 'piece_0001'
    # 낮거나 불규칙한 FPS에서도 직전 관측으로부터의 이동을 감지한다.
    region, = monitor.update([detection((46, 30, 76, 60))], 1.0)
    assert region['reasons'] == ['MOVE'] and region['sensor_id'] == 'piece_0001'
    for t in (1.1, 1.3, 1.7):
        regions = monitor.update([detection((46, 30, 76, 60))], t)
    assert regions == []


def test_missing_region_expires_but_identity_has_no_timeout():
    monitor = PieceMotionMonitor()
    warm(monitor)
    assert monitor.update([], .4) == []
    region, = monitor.update([], .5)
    assert region['reasons'] == ['MISS'] and not region['visible']
    assert monitor.update([], 600.) == []
    old = monitor.pieces['piece_0001']
    assert old.status == 'dormant' and not old.visible
    for t in (600.1, 600.2, 600.3, 600.4, 600.5, 600.6, 600.7):
        current = detection()
        monitor.update([current], t)
    assert current.piece_id == old.id and old.visible and old.revival_count == 1
    assert old.last_revival['absence_seconds'] > 599


def test_multiple_piece_sensors_and_reset():
    monitor = PieceMotionMonitor()
    for t in (0., .1, .2, .3):
        assert monitor.update([detection(), detection((100, 100, 130, 130))], t) == []
    region, = monitor.update([detection(confidence=.5), detection((100, 100, 130, 130))], .4)
    assert region['sensor_id'] == 'piece_0001' and region['reasons'] == ['CONF']
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


def test_size_recovers_from_small_initial_reference_at_high_fps():
    monitor = PieceMotionMonitor()
    small = detection((36, 36, 54, 54))
    for t in (0., .02, .04):
        monitor.update([small], t)
    piece = monitor.pieces['piece_0001']
    for _ in range(5):
        piece.add_size(18.)
    assert monitor.update([detection()], .06)[0]['reasons'] == ['SIZE']
    for i in range(1, 61):
        regions = monitor.update([detection()], .06+i*.02)
    assert regions == []
    assert np.median(monitor.pieces['piece_0001'].sensor_sizes) == 30
    assert piece.size_reference_rebuilds == 1 and not piece.sizes
    assert piece.add_size(30.)  # 과거의 잘못된 작은 크기가 새 정상 측정을 영구 거부하지 않음


def test_size_release_tolerates_normal_box_variation_after_partial_occlusion():
    monitor = PieceMotionMonitor()
    warm(monitor)  # 기준 기하평균 30px
    assert 'SIZE' in monitor.update([detection((36, 36, 54, 54))], .4)[0]['reasons']
    recovered = detection((32.5, 32.5, 57.5, 57.5))  # 25px로 회복: 기준 대비 16.7% 차이
    monitor.update([recovered], .6)
    assert monitor.update([recovered], 1.1) == []


def test_size_does_not_learn_stably_shrunken_or_low_confidence_box():
    for confidence in (.9, .5):
        monitor = PieceMotionMonitor()
        warm(monitor)
        for i in range(1, 81):
            regions = monitor.update([detection((36, 36, 54, 54), confidence)], .3+i*.02)
        assert 'SIZE' in regions[0]['reasons']
        assert np.median(monitor.pieces['piece_0001'].sensor_sizes) == 30
    # 크기가 복구되더라도 탐지 확률이 낮으면 기준을 변경하지 않는다.
    monitor = PieceMotionMonitor()
    for t in (0., .1, .2):
        monitor.update([detection((36, 36, 54, 54))], t)
    for i in range(1, 60):
        regions = monitor.update([detection(confidence=.5)], .2+i*.02)
    assert 'SIZE' in regions[0]['reasons'] and 'CONF' in regions[0]['reasons']
    assert np.median(monitor.pieces['piece_0001'].sensor_sizes) == 18


def test_sensor_and_semantic_state_are_same_object_with_optional_cell():
    from piece_state import PieceTracker
    tracker = PieceTracker()
    monitor = PieceMotionMonitor(pieces=tracker.pieces)
    warm(monitor)
    piece = tracker.pieces['piece_0001']
    assert piece is monitor.sensors[piece.id]
    tracker.update_visual(piece.id, {'probabilities': {'blue_cha': .95, 'blue_horse': .05}, 'size_mm': 29.}, .4)
    assert piece.label == 'blue_cha'
    assert not piece.board_assigned and piece.serialize()['cell'] is None
    d = detection()
    d.board = (3, 4)
    monitor.update([d], .5)
    assert d.piece_id == piece.id and piece.board_assigned and piece.cell == (3, 4)
    observations = {(3, 4): {'piece_id': piece.id, 'probabilities': {'blue_cha': .95, 'blue_horse': .05}}}
    tracker.apply(observations, .6, update_semantics=False)
    tracker.apply(observations, .7, update_semantics=False)
    assert tracker.pieces[tracker.occupancy[(3, 4)]] is piece
    piece.label = 'blue_horse'
    assert tracker.board() == {(3, 4): ('blue', 'cha')}  # 게임판은 라이브 객체 변경에 흔들리지 않는 스냅샷
    d.board = None
    monitor.update([d], .8)
    assert not piece.board_assigned and piece.cell is None
    assert tracker.board() == {(3, 4): ('blue', 'cha')}


def test_unassigned_healthy_piece_classifies_while_global_and_other_piece_blocked(tmp_path):
    import time
    from types import SimpleNamespace
    from app import Worker
    model = tmp_path/'model.pt'
    model.write_bytes(b'test')
    class Source:
        source_loop = 0
        def __init__(self):
            self.start, self.n = time.monotonic(), 0
        def read(self):
            time.sleep(.01)
            stage = int(time.monotonic()-self.start > .15)
            self.n += 1
            frame = np.full((100, 100, 3), 200 if stage and self.n%2 else 0, np.uint8)
            frame[:, :, 0] = stage
            return frame
    class Detector:
        classification_conf, device, iou, imgsz = .5, 'cpu', .5, 640
        classifier = SimpleNamespace(path=model, classes=['blue_cha', 'red_soldier'], size=32)
        def detect_positions(self, frame, **kwargs):
            return [detection((10, 10, 30, 30), .95), detection((60, 10, 80, 30), .5 if frame[0, 0, 0] else .95)]
        def classify_observations(self, frame, detections):
            return {d.piece_id: dict(piece_id=d.piece_id, box=list(d.box),
                                    probabilities={'blue_cha': .95, 'red_soldier': .05}) for d in detections}
    args = SimpleNamespace(window=2, full_frame=True, log_counts=False, calibration=None, model=model, source='test',
                           state_dir=tmp_path/'states', conf=.35, max_dist=.45, classification_interval=.04)
    worker = Worker(Source(), Detector(), None, args)
    try:
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline and worker.error is None:
            time.sleep(.01)
        with worker.lock:
            healthy = worker.tracker.pieces['piece_0001']
            blocked = worker.tracker.pieces['piece_0002']
            assert worker.view_status['state'] == 'suspected'
            assert healthy is worker.piece_motion.sensors[healthy.id]
            assert healthy.label == 'blue_cha' and healthy.last_classified > blocked.last_classified+.5
            assert 'CONF' in blocked.flags
            assert healthy.cell is None and not healthy.board_assigned
            assert len(worker.tracker.occupancy) == 0  # 격자 없는 인식은 게임판을 임의로 만들지 않는다.
            before = len(healthy.history)
        assert worker.error is None
        # 표시를 켜거나 끄더라도 통합 객체와 종류 이력은 유지한다.
        worker.show_piece_motion = True
        time.sleep(.05)
        worker.show_piece_motion = False
        time.sleep(.05)
        with worker.lock:
            assert worker.tracker.pieces[healthy.id] is healthy and len(healthy.history) >= before
    finally:
        worker.stop()


def test_result_from_before_local_block_is_rejected_after_recovery():
    import time
    from types import SimpleNamespace
    from app import Worker
    from piece_state import PieceTracker
    worker = Worker.__new__(Worker)
    worker.tracker = PieceTracker()
    monitor = worker.piece_motion = PieceMotionMonitor(pieces=worker.tracker.pieces)
    warm(monitor)
    piece = worker.tracker.pieces['piece_0001']
    old_epoch = piece.visual_epoch
    monitor.update([detection(confidence=.6)], .4)
    current = detection()
    monitor.update([current], 1.)
    assert piece.eligible and piece.visual_epoch != old_epoch
    observation = dict(piece_id=piece.id, box=list(current.box), probabilities={'blue_cha': .9, 'blue_horse': .1})
    class Runner:
        error = None
        result = dict(observations={piece.id: observation}, metadata=dict(epochs={piece.id: old_epoch},
                                                                       observed_at=time.monotonic(), frame_size=[100, 100]))
        def poll(self):
            result, self.result = self.result, None
            return result
        def submit(self, frame, detections, metadata):
            self.metadata = metadata
    worker.classifier_worker = Runner()
    worker.args = SimpleNamespace(classification_interval=.2)
    worker.source = SimpleNamespace()
    worker.robot_motion = False
    worker.expected_robot_move = worker.game_turn = None
    worker.size_estimator = None
    worker.state_store = SimpleNamespace(save=lambda *args: None)
    frame = np.zeros((100, 100, 3), np.uint8)
    worker._update_tracks(frame, [current], {}, {'state': 'suspected'})
    assert piece.label is None
    worker.classifier_worker.result = dict(observations={piece.id: observation}, metadata=worker.classifier_worker.metadata)
    worker._update_tracks(frame, [current], {}, {'state': 'suspected'})
    assert piece.label == 'blue_cha'  # 전역 방해는 정상 객체의 새 결과를 막지 않는다.


def dormant_known():
    monitor = PieceMotionMonitor()
    warm(monitor)
    old = monitor.pieces['piece_0001']
    old.label, old.probabilities = 'blue_cha', {'blue_cha': .95, 'blue_sang': .05}
    old.sizes.extend([29., 29.1, 28.9])
    monitor.update([], .4)
    monitor.update([], .5)
    monitor.update([], 600.)
    return monitor, old


def probe(piece, at, label='blue_cha'):
    piece.last_raw = dict(piece_id=piece.id, box=list(piece.box), probabilities={label: .95})
    piece.last_classified = at


def test_known_reappearance_needs_two_fresh_probes_and_keeps_history_isolated():
    monitor, old = dormant_known()
    for t in (600.1, 600.2, 600.3):
        current = detection()
        monitor.update([current], t)
    provisional = monitor.pieces[current.piece_id]
    assert provisional.id != old.id and provisional.probabilities == {} and not provisional.sizes
    assert provisional.eligible and not provisional.game_eligible
    probe(provisional, 600.31)
    for t in (600.4, 600.5, 600.6):
        current = detection()
        monitor.update([current], t)
    assert current.piece_id == provisional.id  # 같은 CNN 결과는 두 번 세지 않는다
    probe(provisional, 600.61)
    current = detection()
    monitor.update([current], 600.7)
    assert current.piece_id == old.id and old.sizes and old.probabilities['blue_cha'] == .95
    assert provisional.status == 'superseded' and old.game_eligible
    events, links = monitor.drain_lifecycle()
    assert any(e['event'] == 'revived' for e in events)
    assert links[0]['piece_id'] == old.id and links[0]['provisional_id'] == provisional.id


def test_different_class_rejects_dormant_identity():
    monitor, old = dormant_known()
    for t in (600.1, 600.2, 600.3):
        current = detection()
        monitor.update([current], t)
    provisional = monitor.pieces[current.piece_id]
    probe(provisional, 600.31, 'red_sang')
    monitor.update([current], 600.4)
    assert provisional.revival_state == 'new_identity' and not old.visible
    assert any(e['event'] == 'revival_rejected' for e in monitor.drain_lifecycle()[0])


def test_ambiguous_histories_do_not_select_an_arbitrary_old_id():
    from piece_state import PieceState
    monitor, old = dormant_known()
    other = PieceState('piece_0099', box=old.box, good_box=old.box, label=old.label, status='dormant', misses=2)
    other.sensor_sizes.extend([30., 30., 30.])
    monitor.pieces[other.id] = other
    for t in (600.1, 600.2, 600.3):
        current = detection()
        monitor.update([current], t)
    provisional = monitor.pieces[current.piece_id]
    for t in (600.4, 600.7):
        probe(provisional, t-.01)
        monitor.update([current], t)
    assert current.piece_id == provisional.id
    assert provisional.revival_state == 'ambiguous_new_identity'
    assert not old.visible and not other.visible


def test_captured_piece_is_not_a_reappearance_candidate():
    monitor, old = dormant_known()
    old.status = 'captured'
    current = detection()
    monitor.update([current], 600.1)
    assert current.piece_id != old.id
    assert monitor.pieces[current.piece_id].revival_candidates == []
    assert old.status == 'captured'


def test_local_obstruction_resets_reappearance_evidence():
    monitor, old = dormant_known()
    for t in (600.1, 600.2, 600.3):
        current = detection()
        monitor.update([current], t)
    provisional = monitor.pieces[current.piece_id]
    probe(provisional, 600.31)
    monitor.update([current], 600.4)
    assert provisional.revival_votes[old.id] == 1
    monitor.update([detection(confidence=.5)], 600.5)
    assert not provisional.revival_votes
    monitor.update([current], 601.1)
    monitor.update([current], 601.2)
    probe(provisional, 601.21)
    monitor.update([current], 601.3)
    assert current.piece_id == provisional.id and provisional.revival_votes[old.id] == 1


def test_current_track_has_priority_over_nearby_dormant_history():
    monitor, old = dormant_known()
    old.box = (25, 30, 55, 60)
    current = detection((45, 30, 75, 60))
    monitor.update([current], 600.1)
    active_id = current.piece_id
    for t in (600.2, 600.3, 600.4):
        monitor.update([current], t)
    assert current.piece_id == active_id and not old.visible
