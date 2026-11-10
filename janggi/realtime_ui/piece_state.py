"""검출 기반 통합 말 객체와 별도의 안정된 게임판 스냅샷·이벤트 저장."""
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
import json
import copy
import math
from pathlib import Path

import numpy as np


@dataclass
class PieceState:
    id: str
    cell: tuple | None = None
    box: tuple | None = None
    visible: bool = False
    last_seen: float = 0.
    detection_confidence: float = 0.
    association_confidence: float = 1.
    visual_epoch: int = 0
    good_box: tuple | None = None
    tracking_center: tuple | None = None
    tracking_velocity: tuple = (0., 0.)
    tracking_only: bool = False
    missing_box: tuple | None = None
    missing_since: float | None = None
    revival_observations: deque = field(default_factory=lambda: deque(maxlen=2))
    sensor_confidence: deque = field(default_factory=lambda: deque(maxlen=20))
    sensor_sizes: deque = field(default_factory=lambda: deque(maxlen=20))
    positions: deque = field(default_factory=lambda: deque(maxlen=20))
    misses: int = 0
    seen_count: int = 0
    revival_state: str = 'active'
    revival_candidates: list = field(default_factory=list)
    revival_votes: dict = field(default_factory=dict)
    revival_frames: int = 0
    revival_since: float | None = None
    revival_last_probe: float = float('-inf')
    revival_count: int = 0
    last_revival: dict | None = None
    flags: tuple = ()
    last_alert: float = float("-inf")
    recovery_sizes: deque = field(default_factory=lambda: deque(maxlen=20))
    recovery_since: float | None = None
    size_reference_rebuilds: int = 0
    reference_reset_evidence: dict | None = None
    last_raw: dict | None = None
    last_classified: float = float("-inf")
    confirmed_cell: tuple | None = None
    game_bound: bool = False
    superseded_by: str | None = None
    probabilities: dict = field(default_factory=dict)
    label: str | None = None
    preview_label: str | None = None
    preview_confidence: float = 0.
    preview_observed_at: float | None = None
    last_regular_attempt: float = float('-inf')
    status: str = 'on_board'
    sizes: deque = field(default_factory=lambda: deque(maxlen=20))
    history: deque = field(default_factory=lambda: deque(maxlen=20))
    size_outlier_count: int = 0
    previous_cell: tuple | None = None
    captured_by: str | None = None
    captured_at: object = None
    moves: deque = field(default_factory=lambda: deque(maxlen=50))
    alternative: str | None = None
    alternative_count: int = 0

    @property
    def display_label(self):
        return self.label or self.preview_label

    @property
    def display_confidence(self):
        return self.probabilities.get(self.label, 0.) if self.label is not None else self.preview_confidence

    @property
    def board_assigned(self):
        return self.visible and self.cell is not None

    @property
    def eligible(self):
        return (self.visible and not self.tracking_only and not self.flags and len(self.sensor_confidence) >= 3
                and self.association_confidence >= .25 and self.status not in ('captured', 'superseded'))

    @property
    def game_eligible(self):
        return self.eligible and self.revival_state not in ('pending', 'tentative')

    def clear_semantics(self):
        self.visual_epoch += 1
        self.probabilities.clear()
        self.label = None
        self.preview_label, self.preview_confidence, self.preview_observed_at = None, 0., None
        self.last_regular_attempt = float('-inf')
        self.sizes.clear()
        self.history.clear()
        self.alternative, self.alternative_count = None, 0
        self.last_raw, self.last_classified = None, float('-inf')

    def semantic_snapshot(self):
        return dict(probabilities=dict(self.probabilities), label=self.label, sizes=list(self.sizes), history=list(self.history),
                    last_raw=copy.deepcopy(self.last_raw), last_classified=self.last_classified)

    def restore_semantics(self, saved):
        self.probabilities = dict(saved['probabilities'])
        self.label = saved['label']
        self.sizes = deque(saved['sizes'], maxlen=20)
        self.history = deque(saved['history'], maxlen=20)
        self.last_raw, self.last_classified = copy.deepcopy(saved.get('last_raw')), saved.get('last_classified', float('-inf'))
        self.alternative, self.alternative_count = None, 0

    @property
    def size_mm(self):
        return float(np.median(self.sizes)) if self.sizes else None

    def add_size(self, value):
        if value is None or not math.isfinite(value) or value <= 0:
            return False
        if len(self.sizes) >= 5:
            median = self.size_mm
            mad = float(np.median(np.abs(np.asarray(self.sizes)-median)))
            if abs(value-median) > max(2., 3*1.4826*mad):
                self.size_outlier_count += 1
                return False
        self.sizes.append(float(value))
        return True

    def update(self, observation, stamp, threshold=.5, alpha=.2, size_strength=.5):
        accepted = self.add_size(observation.get('size_mm'))
        raw = observation.get('probabilities')
        if not raw:
            return dict(id=self.id, time=stamp, size_accepted=accepted, label=self.label)
        current = dict(raw)
        if self.size_mm is not None and size_strength > 0:
            median = self.size_mm
            spread = float(np.median(np.abs(np.asarray(self.sizes)-median)))
            strength = size_strength/(1+spread)
            for label in current:
                kind = label.split('_', 1)[1]
                reference = 35. if kind == 'king' else 23. if kind in ('sa', 'soldier') else 29.
                likelihood = .25+.75*math.exp(-.5*((median-reference)/1.8)**2)
                current[label] *= likelihood**strength
            total = sum(current.values())
            current = {k: v/total for k, v in current.items()}
        if not self.probabilities:
            self.probabilities = current
        else:
            self.probabilities = {k: (1-alpha)*self.probabilities.get(k, 0)+alpha*current.get(k, 0)
                                  for k in set(self.probabilities) | set(current)}
        winner = max(self.probabilities, key=self.probabilities.get)
        probability = self.probabilities[winner]
        if self.label is None:
            if probability >= threshold:
                self.label = winner
        elif winner != self.label and probability >= threshold:
            self.alternative_count = self.alternative_count+1 if self.alternative == winner else 1
            self.alternative = winner
            if self.alternative_count >= 3:
                self.label = winner
                self.alternative, self.alternative_count = None, 0
        else:
            self.alternative, self.alternative_count = None, 0
        record = dict(time=stamp, label=self.label, confidence=self.probabilities.get(self.label, 0),
                      raw_label=max(raw, key=raw.get), raw_confidence=max(raw.values()),
                      size_mm=observation.get('size_mm'), size_accepted=accepted, median_size_mm=self.size_mm,
                      width_px=observation.get('width_px'), height_px=observation.get('height_px'),
                      geometric_size_px=observation.get('geometric_size_px'), box=observation.get('box'),
                      width_mm=observation.get('width_mm'), height_mm=observation.get('height_mm'))
        self.history.append(record)
        return dict(id=self.id, **record)

    def serialize(self):
        return dict(id=self.id, cell=list(self.cell) if self.cell is not None else None,
                    board_assigned=self.board_assigned, confirmed_cell=self.confirmed_cell, box=self.box, visible=self.visible,
                    last_seen=self.last_seen, last_classified=self.last_classified if math.isfinite(self.last_classified) else None,
                    detection_confidence=self.detection_confidence, flags=list(self.flags),
                    association_confidence=self.association_confidence, seen_count=self.seen_count,
                    tracking_center=self.tracking_center, tracking_velocity=self.tracking_velocity,
                    tracking_only=self.tracking_only, good_box=self.good_box,
                    missing_box=self.missing_box, missing_since=self.missing_since,
                    revival_state=self.revival_state, revival_candidates=list(self.revival_candidates),
                    revival_count=self.revival_count, last_revival=self.last_revival,
                    status=self.status, label=self.label, display_label=self.display_label,
                    preview_label=self.preview_label, preview_confidence=self.preview_confidence,
                    preview_observed_at=self.preview_observed_at,
                    last_regular_attempt=self.last_regular_attempt if math.isfinite(self.last_regular_attempt) else None,
                    confidence=self.probabilities.get(self.label, 0), probabilities=self.probabilities,
                    geometric_size_mm=self.size_mm, size_samples_mm=list(self.sizes), history=list(self.history),
                    size_outlier_count=self.size_outlier_count, size_sample_count=len(self.sizes),
                    size_mad_mm=float(np.median(np.abs(np.asarray(self.sizes)-self.size_mm))) if self.sizes else None,
                    size_reference_rebuilds=self.size_reference_rebuilds,
                    sensor_baseline_size_px=float(np.median(self.sensor_sizes)) if self.sensor_sizes else None,
                    sensor_baseline_confidence=float(np.median(self.sensor_confidence)) if self.sensor_confidence else None,
                    previous_cell=self.previous_cell, captured_by=self.captured_by, captured_at=self.captured_at, moves=list(self.moves))


class PieceTracker:
    def __init__(self, threshold=.5, alpha=.2, size_strength=.5, validator=None):
        self.threshold, self.alpha, self.size_strength = threshold, alpha, size_strength
        self.validator = validator
        self.pieces, self.occupancy = {}, {}
        self.confirmed_labels, self.confirmed_semantics = {}, {}
        self.aliases = {}
        self.next_id, self.revision, self.generation = 1, 0, 0
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.status = '안정된 초기 배치 확인 중'
        self.expected_move = None
        self.turn = None
        self.events = []

    def event(self, event, **values):
        self.events.append(dict(event=event, **values))

    def reset(self, reason, keep_objects=False):
        self.generation += 1
        self.event('reset', reason=reason, previous_pieces=[p.serialize() for p in self.pieces.values()])
        if keep_objects:
            for piece in self.pieces.values():
                # 게임 규칙과 맞지 않는 배치는 시각 추적/종류 기억을 초기화하지 않는다.
                piece.confirmed_cell, piece.game_bound = None, False
        else:
            self.pieces.clear()
        self.occupancy = {}
        self.confirmed_labels, self.confirmed_semantics, self.aliases = {}, {}, {}
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.expected_move = None
        self.status = '재인식 중: '+reason

    def pause(self):
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.status = '게임 배치 확정 보류 · 말별 관측은 별도 갱신'

    def board(self, visible=None):
        return {cell: label for cell, label in self.confirmed_labels.items() if visible is None or cell in visible}

    def _new(self, cell, observation=None):
        pid = (observation or {}).get('piece_id')
        if pid is not None and pid in self.pieces:
            piece = self.pieces[pid]
        else:
            piece = PieceState(f'piece_{self.next_id:04}', cell)
            self.next_id += 1
            self.pieces[piece.id] = piece
        piece.game_bound, piece.confirmed_cell = True, cell
        self.occupancy[cell] = piece.id

    def resolve_id(self, pid):
        while pid in self.aliases:
            pid = self.aliases[pid]
        return pid

    def _bind_visual(self, pid, observed_id):
        observed_id = self.resolve_id(observed_id)
        if observed_id == pid or observed_id not in self.pieces:
            return
        target, observed = self.pieces[pid], self.pieces[observed_id]
        for name in ('box', 'cell', 'visible', 'last_seen', 'detection_confidence', 'association_confidence',
                     'tracking_center', 'tracking_velocity', 'tracking_only', 'missing_box', 'missing_since', 'positions', 'misses', 'flags', 'last_alert', 'recovery_sizes'):
            setattr(target, name, getattr(observed, name))
        target.good_box = observed.good_box or observed.box
        target.revival_state = 'active'
        target.revival_observations.clear()
        target.visual_epoch += 1
        observed.visible, observed.status, observed.superseded_by = False, 'superseded', pid
        self.aliases[observed_id] = pid
        self.event('identity_reconciled', observed_id=observed_id, piece_id=pid)

    def update_preview(self, pid, observation, stamp):
        """미분류 객체를 즉시 표시하되 정상 종류/크기/이동평균 이력은 변경하지 않는다."""
        piece = self.pieces[pid]
        raw = observation.get('probabilities') or {}
        if piece.label is not None or not raw:
            return
        winner = max(raw, key=raw.get)
        confidence = raw[winner]
        piece.preview_label = winner if confidence >= .5 else None
        piece.preview_confidence = confidence
        piece.preview_observed_at = stamp['observed_at']
        self.event('classification_preview', piece_id=pid, label=piece.preview_label,
                   confidence=confidence, observed_at=stamp['observed_at'], flags=list(piece.flags))

    def update_visual(self, pid, observation, stamp):
        piece = self.pieces[pid]
        observed_at = stamp['observed_at'] if isinstance(stamp, dict) else float(stamp)
        if piece.revival_state == 'tentative':
            if not piece.eligible or observed_at <= piece.revival_last_probe:
                return
            piece.revival_last_probe = observed_at
        piece.last_raw = observation
        if isinstance(stamp, dict):
            stamp = {k: v for k, v in stamp.items() if k != 'epochs'}
        piece.last_classified = stamp['observed_at'] if isinstance(stamp, dict) else float(stamp)
        if piece.revival_state == 'tentative':
            raw = observation.get('probabilities') or {}
            winner = max(raw, key=raw.get) if raw else None
            required = .6 if piece.label is None or winner == piece.label else .8
            if winner is None or raw[winner] < required:
                piece.revival_observations.clear()
                return
            if piece.revival_observations and piece.revival_observations[-1][0] != winner:
                piece.revival_observations.clear()
            piece.revival_observations.append((winner, copy.deepcopy(observation), stamp))
            self.event('revival_probe', piece_id=pid, candidate=winner, confidence=raw[winner],
                       observations=len(piece.revival_observations))
            if len(piece.revival_observations) < 2:
                return
            evidence = list(piece.revival_observations)
            if piece.label is not None and winner != piece.label:
                previous = piece.semantic_snapshot()
                piece.clear_semantics()
                self.event('revival_reclassified', piece_id=pid, previous=previous, label=winner)
            piece.revival_state = 'active'
            piece.revival_observations.clear()
            # 두 번 검증한 새 관측만 반영한다. 부분 가림 중에는 과거 이력을 오염시키지 않는다.
            for _, confirmed, confirmed_stamp in evidence:
                piece.last_raw, piece.last_classified = confirmed, confirmed_stamp['observed_at'] if isinstance(confirmed_stamp, dict) else float(confirmed_stamp)
                self.event('classification', cell=piece.cell, **piece.update(confirmed, confirmed_stamp, self.threshold, self.alpha, self.size_strength))
            self.event('revival_verified', piece_id=pid, label=piece.label)
            return
        self.event('classification', cell=piece.cell, **piece.update(observation, stamp, self.threshold, self.alpha, self.size_strength))

    def _label_at(self, cell):
        value = self.confirmed_labels.get(cell)
        return '_'.join(value) if value and value[0] != 'unknown' else None

    def _compatible(self, label, observation):
        p = observation.get('probabilities') or {}
        return label is not None and p.get(label, 0) >= self.threshold

    def _legal(self, src, dst):
        label = self._label_at(src)
        if label is None:
            return False
        if self.expected_move is not None:
            if (src, dst) != self.expected_move:
                return False
        elif self.turn is not None and label.split('_')[0] != self.turn:
            return False
        return self.validator(self.board(), src, dst) if self.validator else True

    def _transition(self, observations):
        old, new = set(self.occupancy), set(observations)
        removed, added = old-new, new-old
        if len(old) == len(new) and len(removed) == len(added) == 1:
            src, dst = next(iter(removed)), next(iter(added))
            label = self._label_at(src)
            if self._compatible(label, observations[dst]) and self._legal(src, dst):
                return src, dst, None
        if len(old)-len(new) == 1 and len(removed) == 1 and not added:
            src = next(iter(removed))
            label = self._label_at(src)
            destinations = []
            for dst in old & new:
                victim_label = self._label_at(dst)
                p = observations[dst].get('probabilities') or {}
                if (victim_label and label and victim_label.split('_')[0] != label.split('_')[0]
                        and self._compatible(label, observations[dst])
                        and p.get(label, 0)-p.get(victim_label, 0) >= .15 and self._legal(src, dst)):
                    destinations.append(dst)
            if len(destinations) == 1:
                dst = destinations[0]
                return src, dst, self.occupancy[dst]
        return None

    def apply(self, observations, stamp, update_semantics=True):
        # 점유 배치뿐 아니라 목적지의 현재 분류가 두 관측에서 같은 이동을 가리켜야 한다.
        if not observations:
            self.pause()
            return False
        cells = frozenset(observations)
        transition = None
        if self.occupancy and cells != frozenset(self.occupancy):
            transition = self._transition(observations)
            signature = (cells, transition)
        else:
            signature = (cells, None)
        if signature == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate, self.candidate_count = signature, 1
        if not self.occupancy or cells != frozenset(self.occupancy):
            if self.candidate_count < 2:
                self.status = '새 배치 재확인 중'
                return False
            if self.occupancy and transition is None:
                self.bad_count += 1
                self.status = '한 수로 설명되지 않는 배치 · 재확인 중'
                if self.bad_count < 3:
                    return False
                self.reset('안정된 배치가 이동/포획 한 수와 일치하지 않음', keep_objects=not update_semantics)
            if not self.occupancy:
                for cell in sorted(observations):
                    self._new(cell, observations[cell])
                self.event('initialized', time=stamp, count=len(self.occupancy))
            elif transition:
                src, dst, captured_id = transition
                pid = self.occupancy.pop(src)
                # 목적지에 잘못 이어진 시각 추적 이력이 포획 전 상대 말에 섞이지 않도록 복원한다.
                if not update_semantics:
                    for identity in (pid, captured_id):
                        if identity in self.confirmed_semantics:
                            self.pieces[identity].restore_semantics(self.confirmed_semantics[identity])
                    self._bind_visual(pid, observations[dst].get('piece_id', pid))
                    self.update_visual(pid, observations[dst], stamp)
                if captured_id:
                    self.pieces[captured_id].status = 'captured'
                    self.pieces[captured_id].visible = False
                    self.pieces[captured_id].cell = None
                    self.pieces[captured_id].captured_by = pid
                    self.pieces[captured_id].captured_at = stamp
                self.occupancy[dst] = pid
                self.pieces[pid].previous_cell = src
                self.pieces[pid].cell = dst
                self.pieces[pid].confirmed_cell = dst
                self.pieces[pid].status = "on_board"
                self.pieces[pid].moves.append(dict(src=src, dst=dst, time=stamp, captured_id=captured_id))
                self.event('capture' if captured_id else 'move', time=stamp, piece_id=pid, src=src, dst=dst,
                           captured_id=captured_id, captured_label=self.pieces[captured_id].label if captured_id else None,
                           count=len(self.occupancy))
                self.expected_move = None
        self.bad_count = 0
        for cell, observation in observations.items():
            pid = self.occupancy[cell]
            if not update_semantics:
                observed_id = observation.get('piece_id', pid)
                if (observed_id != pid and observed_id in self.pieces
                        and self._compatible(self._label_at(cell), observation)):
                    self._bind_visual(pid, observed_id)
                    self.update_visual(pid, observation, stamp)
            if update_semantics:
                self.event('classification', cell=cell, **self.pieces[pid].update(
                    observation, stamp, self.threshold, self.alpha, self.size_strength))
            piece = self.pieces[pid]
            self.confirmed_labels[cell] = tuple(piece.label.split('_', 1)) if piece.label else ('unknown', 'piece')
            self.confirmed_semantics[pid] = piece.semantic_snapshot()
        self.confirmed_labels = {c: p for c, p in self.confirmed_labels.items() if c in self.occupancy}
        self.revision += 1
        self.status = '확정 배치 갱신됨'
        return True

    def snapshot(self):
        return dict(schema_version=2, generation=self.generation, updated_at=datetime.now().astimezone().isoformat(), revision=self.revision,
                    status=self.status, expected_count=len(self.occupancy),
                    visible_count=sum(p.visible for p in self.pieces.values()),
                    classified_visible_count=sum(p.visible and p.label is not None for p in self.pieces.values()),
                    preview_visible_count=sum(p.visible and p.label is None and p.preview_label is not None for p in self.pieces.values()),
                    local_paused_count=sum(p.visible and not p.eligible for p in self.pieces.values()),
                    fusion=dict(current_weight=self.alpha, past_weight=1-self.alpha, size_strength=self.size_strength,
                                size_references_mm=dict(small=23, medium=29, king=35)),
                    board=[dict(cell=list(cell), piece_id=pid, piece=self.confirmed_labels.get(cell, ('unknown', 'piece')))
                           for cell, pid in sorted(self.occupancy.items())],
                    pieces=[p.serialize() for p in self.pieces.values()])


class StateStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.events = (self.directory/'events.jsonl').open('a', encoding='utf-8')
        self.last_write = float('-inf')

    def save(self, tracker, now, force=False):
        events, tracker.events = tracker.events, []
        important = any(e['event'] not in ('classification', 'revival_probe', 'classification_preview') for e in events)
        for event in events:
            self.events.write(json.dumps(event, ensure_ascii=False)+'\n')
        if events:
            self.events.flush()
        if force or important or now-self.last_write >= 1:
            path = self.directory/'current_state.json'
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(tracker.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')
            temporary.replace(path)
            self.last_write = now

    def close(self, tracker, now):
        self.save(tracker, now, force=True)
        self.events.close()
