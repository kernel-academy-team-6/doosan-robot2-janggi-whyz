import json
import sys
from pathlib import Path
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piece_state import PieceState, PieceTracker, StateStore, SIZE_REFERENCES_MM, SIZE_PRIOR_SIGMA_MM
from classification_async import LatestClassification


LABELS = ['blue_cha', 'red_soldier', 'blue_horse', 'blue_soldier']


def observation(label='blue_cha', size=29., confidence=.95):
    return dict(probabilities={k: confidence if k == label else (1-confidence)/(len(LABELS)-1) for k in LABELS},
                size_mm=size, box=[0, 0, 30, 30])


def initialize(tracker, observations):
    assert not tracker.apply(observations, 1.)
    assert tracker.apply(observations, 2.)


def test_history_fusion_size_weight_and_outlier_rejection():
    piece = PieceState('a', (0, 0))
    piece.update(observation(), 1.)
    original = dict(piece.probabilities)
    piece.update(observation('blue_horse'), 2.)
    assert piece.label == 'blue_cha'
    assert piece.probabilities['blue_cha'] < original['blue_cha']
    for _ in range(20):
        piece.update(observation('blue_horse'), 3.)
    assert piece.label == 'blue_horse' and len(piece.history) == 20
    small = PieceState('b', (1, 1))
    for value in (23., 23.1, 22.9, 23., 23.2):
        assert small.add_size(value)
    assert not small.add_size(12.)
    assert 22.9 <= small.size_mm <= 23.2
    small.update(dict(probabilities={'blue_cha': .51, 'blue_soldier': .49}, size_mm=23.), 1.)
    assert small.label == 'blue_soldier'
    assert len(small.sizes) == 6


@pytest.mark.parametrize('group,label,wrong', [
    ('small', 'blue_soldier', 'blue_cha'),
    ('medium', 'blue_cha', 'blue_king'),
    ('king', 'blue_king', 'blue_cha'),
])
def test_empirical_size_groups_overrule_a_wrong_cnn_group(group, label, wrong):
    piece = PieceState('a')
    piece.update(dict(size_mm=SIZE_REFERENCES_MM[group], probabilities={label: .1, wrong: .9}), 1.)
    assert piece.label == label
    assert piece.probabilities[label] > .98


def test_size_prior_preserves_within_group_odds_and_zero_cnn_probability():
    piece = PieceState('a')
    piece.update(dict(size_mm=SIZE_REFERENCES_MM['medium'],
                      probabilities={'blue_cha': .3, 'red_horse': .1, 'blue_king': .6, 'blue_po': 0.}), 1.)
    assert piece.probabilities['blue_cha']/piece.probabilities['red_horse'] == pytest.approx(3.)
    assert piece.probabilities['blue_po'] == 0.
    assert piece.probabilities['blue_king'] < .01


@pytest.mark.parametrize('size,strength', [(None, .5), (SIZE_REFERENCES_MM['small'], 0.)])
def test_missing_geometry_or_disabled_size_prior_keeps_cnn_probabilities(size, strength):
    raw = {'blue_cha': .6, 'blue_soldier': .4}
    piece = PieceState('a')
    piece.update(dict(size_mm=size, probabilities=raw), 1., size_strength=strength)
    assert piece.probabilities == raw


def test_no_floor_prior_normalizes_distant_sizes_without_underflow():
    piece = PieceState('a')
    piece.update(dict(size_mm=10000., probabilities={
        'blue_king': 1e-300, 'blue_cha': .9, 'blue_soldier': .1}), 1.)
    assert all(np.isfinite(v) for v in piece.probabilities.values())
    assert sum(piece.probabilities.values()) == pytest.approx(1.)
    assert piece.probabilities['blue_king'] > .999


def test_size_spread_softens_prior_and_empirical_width_keeps_room_for_measurement_noise():
    raw = {'blue_cha': .5, 'blue_king': .5}
    stable, spread = PieceState('stable'), PieceState('spread')
    size = SIZE_REFERENCES_MM['medium']
    stable.sizes.extend([size, size])
    spread.sizes.extend([size-1., size+1.])
    for p in (stable, spread):
        p.update(dict(size_mm=None, probabilities=raw), 1.)
    assert spread.probabilities['blue_king'] > stable.probabilities['blue_king']
    assert SIZE_PRIOR_SIGMA_MM > 3*.3694940477107863


def test_snapshot_records_no_floor_prior_configuration():
    fusion = PieceTracker().snapshot()['fusion']
    assert fusion['size_references_mm'] == SIZE_REFERENCES_MM
    assert fusion['size_likelihood_floor'] == 0.
    assert fusion['size_sigma_mm'] == SIZE_PRIOR_SIGMA_MM
    assert fusion['size_weighting'] == 'gaussian_no_floor'


def test_move_preserves_identity_and_history_without_ghost_positions():
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation()})
    pid = tracker.occupancy[(0, 0)]
    new = {(0, 1): observation()}
    assert not tracker.apply(new, 3.)
    assert tracker.apply(new, 4.)
    assert tracker.occupancy[(0, 1)] == pid and len(tracker.pieces[pid].history) == 2
    assert tracker.board(visible=set()) == {}
    assert [e['event'] for e in tracker.events].count('move') == 1


def test_capture_requires_destination_replacement_and_preserves_attacker_prior():
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation(), (0, 3): observation('red_soldier', 23.)})
    attacker, victim = tracker.occupancy[(0, 0)], tracker.occupancy[(0, 3)]
    assert not tracker.apply({(0, 3): observation()}, 3.)
    assert tracker.pieces[victim].label == 'red_soldier'
    assert tracker.apply({(0, 3): observation()}, 4.)
    assert tracker.occupancy == {(0, 3): attacker}
    assert tracker.pieces[victim].status == 'captured'
    assert tracker.pieces[attacker].label == 'blue_cha'
    assert tracker.pieces[attacker].size_mm == 29.
    event = next(e for e in tracker.events if e['event'] == 'capture')
    assert event['captured_id'] == victim and event['count'] == 1


def test_capture_reconciles_same_visual_objects_and_restores_victim_history():
    tracker = PieceTracker()
    a = PieceState('piece_0001', (0, 0), box=(0, 0, 20, 20), visible=True)
    b = PieceState('piece_0002', (0, 3), box=(0, 30, 20, 50), visible=True)
    tracker.pieces.update({a.id: a, b.id: b})
    tracker.update_visual(a.id, observation(), 1.)
    tracker.update_visual(b.id, observation('red_soldier', 23.), 1.)
    initial = {(0, 0): dict(observation(), piece_id=a.id), (0, 3): dict(observation('red_soldier', 23.), piece_id=b.id)}
    tracker.apply(initial, 2., update_semantics=False)
    tracker.apply(initial, 3., update_semantics=False)
    # 가림 뒤 목적지의 현재 박스가 잠시 상대 ID로 연결된 상황을 재현한다.
    for _ in range(20):
        tracker.update_visual(b.id, observation(), 4.)
    assert b.label == 'blue_cha'
    assert tracker.board()[(0, 3)] == ('red', 'soldier')
    current = {(0, 3): dict(observation(), piece_id=b.id)}
    tracker.apply(current, 5., update_semantics=False)
    assert tracker.apply(current, 6., update_semantics=False)
    assert tracker.pieces[a.id] is a and tracker.pieces[b.id] is b
    assert tracker.occupancy == {(0, 3): a.id}
    assert a.label == 'blue_cha' and a.visible
    assert b.status == 'captured' and b.label == 'red_soldier' and not b.visible
    assert tracker.resolve_id(b.id) == a.id
    assert tracker.board() == {(0, 3): ('blue', 'cha')}


def test_persistent_miss_does_not_create_capture_and_reinitializes():
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation(), (0, 3): observation('red_soldier', 23.)})
    previous_ids = set(tracker.pieces)
    unchanged_destination = {(0, 3): observation('red_soldier', 23.)}
    for i in range(3):
        assert not tracker.apply(unchanged_destination, i+3.)
        assert len(tracker.occupancy) == 2
    assert tracker.apply(unchanged_destination, 6.)
    assert not any(e['event'] == 'capture' for e in tracker.events)
    assert any(e['event'] == 'reset' for e in tracker.events)
    assert not set(tracker.pieces) & previous_ids


def test_pause_preserves_all_evidence_and_resets_confirmation():
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation()})
    previous = tracker.pieces[tracker.occupancy[(0, 0)]].serialize()
    tracker.apply({(0, 1): observation()}, 3.)
    tracker.pause()
    assert tracker.pieces[tracker.occupancy[(0, 0)]].serialize() == previous
    assert not tracker.apply({(0, 1): observation()}, 4.)
    assert tracker.apply({(0, 1): observation()}, 5.)


def test_game_reinitialization_does_not_erase_unrelated_offboard_object():
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation()})
    unrelated = PieceState('offboard', None, visible=True)
    unrelated.update(observation('blue_horse'), 1.)
    tracker.pieces[unrelated.id] = unrelated
    before = unrelated.semantic_snapshot()
    original = tracker.pieces['piece_0001'].semantic_snapshot()
    epoch = tracker.pieces['piece_0001'].visual_epoch
    tracker.reset('여러 말 재배치', keep_objects=True)
    assert unrelated.semantic_snapshot() == before
    assert tracker.pieces['piece_0001'].label is not None
    assert tracker.pieces['piece_0001'].visual_epoch == epoch
    assert tracker.pieces['piece_0001'].semantic_snapshot() == original


def test_turn_rule_and_robot_expected_move():
    tracker = PieceTracker(validator=lambda board, src, dst: dst == (0, 1))
    initialize(tracker, {(0, 0): observation()})
    tracker.turn = 'red'
    assert tracker._transition({(0, 1): observation()}) is None
    tracker.expected_move = ((0, 0), (0, 1))
    assert tracker._transition({(0, 1): observation()}) == ((0, 0), (0, 1), None)
    assert tracker._transition({(0, 2): observation()}) is None


def test_state_persistence_and_event_log(tmp_path):
    tracker = PieceTracker()
    initialize(tracker, {(0, 0): observation()})
    store = StateStore(tmp_path/'session')
    store.save(tracker, 1.)
    tracker.pause()
    store.close(tracker, 2.)
    state = json.loads((tmp_path/'session/current_state.json').read_text())
    assert state['expected_count'] == 1 and state['pieces'][0]['geometric_size_mm'] == 29.
    assert '보류' in state['status']
    events = [json.loads(line) for line in (tmp_path/'session/events.jsonl').read_text().splitlines()]
    assert {e['event'] for e in events} == {'initialized', 'classification'}
    assert not (tmp_path/'session/current_state.tmp').exists()


def test_reinitialized_board_requires_explicit_new_game():
    from app import Worker
    worker = Worker.__new__(Worker)
    worker.lock = threading.Lock()
    worker.tracker = PieceTracker()
    initialize(worker.tracker, {(0, 0): observation()})
    worker._visible_cells = frozenset({(0, 0)})
    worker._stable_gate = True
    worker.tracking_status = '확정 배치 갱신됨'
    worker.mark_game_generation()
    assert worker.tracking_ready()
    worker.tracker.reset('재배치')
    initialize(worker.tracker, {(0, 0): observation()})
    assert not worker.tracking_ready()
    assert worker.tracking_ready(new_game=True)
    worker.mark_game_generation()
    assert worker.tracking_ready()
    worker._stable_gate = False
    assert not worker.tracking_ready(new_game=True)


def test_async_discards_blocked_result_and_keeps_latest_request():
    started, release = threading.Event(), threading.Event()
    calls = []
    def classify(frame, detections):
        calls.append(int(frame[0]))
        if len(calls) == 1:
            started.set()
            assert release.wait(2.)
        return {'value': int(frame[0])}
    runner = LatestClassification(classify, .03)
    try:
        runner.submit(np.array([1]), [], {'frame': 1})
        assert started.wait(1.)
        runner.invalidate()
        runner.submit(np.array([2]), [], {'frame': 2})
        runner.submit(np.array([3]), [], {'frame': 3})
        release.set()
        deadline = time.monotonic()+2
        result = None
        while result is None and time.monotonic() < deadline:
            time.sleep(.01)
            result = runner.poll()
        assert result['observations'] == {'value': 3}
        assert calls == [1, 3]
    finally:
        release.set()
        runner.close()


def test_worker_tracks_move_and_capture_with_position_stream(tmp_path):
    from types import SimpleNamespace
    from app import Worker
    from piece_detection.detector import Detection
    from piece_detection.board_mapping import BoardMapper
    mapper = BoardMapper([(10, 10), (90, 10), (90, 90), (10, 90)])
    model_path = tmp_path/'model.pt'
    model_path.write_bytes(b'test classifier metadata')

    class Detector:
        position_only, device, iou, imgsz, classification_conf = False, 'cpu', .5, 640, .5
        classifier = SimpleNamespace(path=model_path, classes=LABELS, size=32)
        def detect_positions(self, frame, **kwargs):
            stage = int(frame[0, 0, 0])
            cells = [(0, 0), (0, 3)] if stage == 0 else [(0, 1), (0, 3)] if stage == 1 else [(0, 3)]
            result = []
            for cell in cells:
                x, y = mapper.board_to_pixel(*cell)
                result.append(Detection(0, 'janggi_dol', 'unknown', 'piece', .95, (x-3, y-3, x+3, y+3)))
            return result
        def classify_observations(self, frame, detections):
            stage = int(frame[0, 0, 0])
            return {d.piece_id: {**observation('red_soldier' if d.board == (0, 3) and stage < 2 else 'blue_cha'),
                              'box': list(d.box), 'piece_id': d.piece_id} for d in detections}

    class Source:
        source_loop = 0
        source_frame = source_time = None
        def __init__(self):
            self.start = time.monotonic()
            self.n = 0
        def read(self):
            time.sleep(.01)
            elapsed = time.monotonic()-self.start
            self.source_time, self.source_frame = elapsed, self.n
            self.n += 1
            stage = 0 if elapsed < .35 else 1 if elapsed < .7 else 2
            return np.full((100, 100, 3), stage, np.uint8)

    args = SimpleNamespace(window=2, full_frame=True, log_counts=False, calibration=None, model=model_path,
                           conf=.35, max_dist=.45, state_dir=tmp_path/'states', source='synthetic',
                           obstruction_settle=.03, classification_interval=.04)
    worker = Worker(Source(), Detector(), mapper, args)
    try:
        deadline = time.monotonic()+3
        while time.monotonic() < deadline and worker.error is None:
            with worker.lock:
                captured = any(p.status == 'captured' for p in worker.tracker.pieces.values())
                if worker.latest:
                    assert len(worker.latest[2]) == len(worker.latest[3])
            if captured:
                break
            time.sleep(.01)
    finally:
        worker.stop()
    assert worker.error is None
    events = [json.loads(line) for line in (worker.state_store.directory/'events.jsonl').read_text().splitlines()]
    assert [e['event'] for e in events if e['event'] in ('move', 'capture')] == ['move', 'capture']
    move = next(e for e in events if e['event'] == 'move')
    capture = next(e for e in events if e['event'] == 'capture')
    assert move['piece_id'] == capture['piece_id']
    assert worker.tracker.pieces[capture['captured_id']].status == 'captured'
    assert len(worker.tracker.occupancy) == 1


def test_revival_probes_are_logged_without_rewriting_full_snapshot(tmp_path, monkeypatch):
    tracker = PieceTracker()
    store = StateStore(tmp_path/'session')
    snapshots = []
    original = tracker.snapshot
    def snapshot():
        snapshots.append(True)
        return original()
    monkeypatch.setattr(tracker, 'snapshot', snapshot)
    store.save(tracker, 0.)
    for t in (.1, .2, .3):
        tracker.event('revival_probe', piece_id='piece_0001', candidate='blue_cha')
        store.save(tracker, t)
    assert len(snapshots) == 1
    tracker.event('revival_verified', piece_id='piece_0001')
    store.save(tracker, .4)
    assert len(snapshots) == 2
    store.close(tracker, .5)
    events = [json.loads(line) for line in (store.directory/'events.jsonl').read_text().splitlines()]
    assert len([e for e in events if e['event'] == 'revival_probe']) == 3


def test_cancelling_pending_classification_preserves_running_result():
    started, release = threading.Event(), threading.Event()
    calls = []
    def classify(frame, detections):
        calls.append(int(frame[0]))
        started.set()
        assert release.wait(2.)
        return {'value': int(frame[0])}
    runner = LatestClassification(classify, .01)
    try:
        runner.submit(np.array([1]), [], {'frame': 1})
        assert started.wait(1.)
        runner.submit(np.array([2]), [], {'frame': 2})
        runner.cancel_pending()
        release.set()
        deadline = time.monotonic()+1
        result = None
        while result is None and time.monotonic() < deadline:
            result = runner.poll()
            time.sleep(.005)
        assert result['observations'] == {'value': 1}
        assert calls == [1] and runner.pending is None
    finally:
        release.set()
        runner.close()


def test_urgent_unknown_bypasses_regular_classifier_wait():
    started = threading.Event()
    calls = []
    def classify(frame, detections):
        calls.append(int(frame[0]))
        started.set()
        return {}
    runner = LatestClassification(classify, interval=10.)
    try:
        runner.last_start = time.monotonic()
        runner.submit(np.array([1]), [], {})
        assert not started.wait(.03)
        runner.submit(np.array([2]), [], {'urgent_ids': ['new']})
        assert started.wait(.5)
        assert calls == [2]
    finally:
        runner.close()


def test_emergency_preview_threshold_does_not_change_fused_history():
    tracker = PieceTracker()
    piece = PieceState('new', visible=True, flags=('SIZE', 'CONF'))
    tracker.pieces[piece.id] = piece
    tracker.update_preview(piece.id, {'probabilities': {'blue_cha': .49, 'red_soldier': .48, 'red_king': .03}}, {'observed_at': 1.})
    assert piece.display_label is None
    tracker.update_preview(piece.id, {'probabilities': {'blue_cha': .5, 'red_soldier': .5}}, {'observed_at': 2.})
    assert piece.display_label == 'blue_cha' and piece.display_confidence == .5
    assert piece.label is None and not piece.probabilities and not piece.history and not piece.sizes
    assert not piece.game_eligible
    piece.label, piece.probabilities = 'red_king', {'red_king': .95}
    tracker.update_preview(piece.id, {'probabilities': {'blue_cha': 1.}}, {'observed_at': 3.})
    assert piece.display_label == 'red_king'
