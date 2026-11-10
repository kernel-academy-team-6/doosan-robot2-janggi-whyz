"""안정된 관측에서만 말 ID·분류 확률·크기를 갱신하고 한 수의 변화를 확정한다."""
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path

import numpy as np


@dataclass
class PieceState:
    id: str
    cell: tuple
    probabilities: dict = field(default_factory=dict)
    label: str | None = None
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
        return dict(id=self.id, cell=list(self.cell), status=self.status, label=self.label,
                    confidence=self.probabilities.get(self.label, 0), probabilities=self.probabilities,
                    geometric_size_mm=self.size_mm, size_samples_mm=list(self.sizes), history=list(self.history),
                    size_outlier_count=self.size_outlier_count, size_sample_count=len(self.sizes),
                    size_mad_mm=float(np.median(np.abs(np.asarray(self.sizes)-self.size_mm))) if self.sizes else None,
                    previous_cell=self.previous_cell, captured_by=self.captured_by, captured_at=self.captured_at, moves=list(self.moves))


class PieceTracker:
    def __init__(self, threshold=.5, alpha=.2, size_strength=.5, validator=None):
        self.threshold, self.alpha, self.size_strength = threshold, alpha, size_strength
        self.validator = validator
        self.pieces, self.occupancy = {}, {}
        self.next_id, self.revision, self.generation = 1, 0, 0
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.status = '안정된 초기 배치 확인 중'
        self.expected_move = None
        self.turn = None
        self.events = []

    def event(self, event, **values):
        self.events.append(dict(event=event, **values))

    def reset(self, reason):
        self.generation += 1
        self.event('reset', reason=reason, previous_pieces=[p.serialize() for p in self.pieces.values()])
        self.pieces = {}
        self.occupancy = {}
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.expected_move = None
        self.status = '재인식 중: '+reason

    def pause(self):
        self.candidate, self.candidate_count, self.bad_count = None, 0, 0
        self.status = '화면 방해/안정 대기 · 이력 갱신 보류'

    def board(self, visible=None):
        return {cell: tuple(self.pieces[pid].label.split('_', 1)) if self.pieces[pid].label else ('unknown', 'piece')
                for cell, pid in self.occupancy.items() if visible is None or cell in visible}

    def _new(self, cell):
        piece = PieceState(f'piece_{self.next_id:04}', cell)
        self.next_id += 1
        self.pieces[piece.id] = piece
        self.occupancy[cell] = piece.id

    def _compatible(self, piece, observation):
        p = observation.get('probabilities') or {}
        return piece.label is not None and p.get(piece.label, 0) >= self.threshold

    def _legal(self, src, dst):
        piece = self.pieces[self.occupancy[src]]
        if piece.label is None:
            return False
        if self.expected_move is not None:
            if (src, dst) != self.expected_move:
                return False
        elif self.turn is not None and piece.label.split('_')[0] != self.turn:
            return False
        return self.validator(self.board(), src, dst) if self.validator else True

    def _transition(self, observations):
        old, new = set(self.occupancy), set(observations)
        removed, added = old-new, new-old
        if len(old) == len(new) and len(removed) == len(added) == 1:
            src, dst = next(iter(removed)), next(iter(added))
            piece = self.pieces[self.occupancy[src]]
            if self._compatible(piece, observations[dst]) and self._legal(src, dst):
                return src, dst, None
        if len(old)-len(new) == 1 and len(removed) == 1 and not added:
            src = next(iter(removed))
            piece = self.pieces[self.occupancy[src]]
            destinations = []
            for dst in old & new:
                victim = self.pieces[self.occupancy[dst]]
                p = observations[dst].get('probabilities') or {}
                if (victim.label and piece.label and victim.label.split('_')[0] != piece.label.split('_')[0]
                        and self._compatible(piece, observations[dst])
                        and p.get(piece.label, 0)-p.get(victim.label, 0) >= .15 and self._legal(src, dst)):
                    destinations.append(dst)
            if len(destinations) == 1:
                dst = destinations[0]
                return src, dst, self.occupancy[dst]
        return None

    def apply(self, observations, stamp):
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
                self.reset('안정된 배치가 이동/포획 한 수와 일치하지 않음')
            if not self.occupancy:
                for cell in sorted(observations):
                    self._new(cell)
                self.event('initialized', time=stamp, count=len(self.occupancy))
            elif transition:
                src, dst, captured_id = transition
                pid = self.occupancy.pop(src)
                if captured_id:
                    self.pieces[captured_id].status = 'captured'
                    self.pieces[captured_id].captured_by = pid
                    self.pieces[captured_id].captured_at = stamp
                self.occupancy[dst] = pid
                self.pieces[pid].previous_cell = src
                self.pieces[pid].cell = dst
                self.pieces[pid].moves.append(dict(src=src, dst=dst, time=stamp, captured_id=captured_id))
                self.event('capture' if captured_id else 'move', time=stamp, piece_id=pid, src=src, dst=dst,
                           captured_id=captured_id, captured_label=self.pieces[captured_id].label if captured_id else None,
                           count=len(self.occupancy))
                self.expected_move = None
        self.bad_count = 0
        for cell, observation in observations.items():
            self.event('classification', cell=cell, **self.pieces[self.occupancy[cell]].update(
                observation, stamp, self.threshold, self.alpha, self.size_strength))
        self.revision += 1
        self.status = '확정 배치 갱신됨'
        return True

    def snapshot(self):
        return dict(schema_version=1, generation=self.generation, updated_at=datetime.now().astimezone().isoformat(), revision=self.revision,
                    status=self.status, expected_count=len(self.occupancy),
                    fusion=dict(current_weight=self.alpha, past_weight=1-self.alpha, size_strength=self.size_strength,
                                size_references_mm=dict(small=23, medium=29, king=35)),
                    board=[dict(cell=list(cell), piece_id=pid) for cell, pid in sorted(self.occupancy.items())],
                    pieces=[p.serialize() for p in self.pieces.values()])


class StateStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.events = (self.directory/'events.jsonl').open('a', encoding='utf-8')
        self.last_write = float('-inf')

    def save(self, tracker, now, force=False):
        events, tracker.events = tracker.events, []
        important = any(e['event'] != 'classification' for e in events)
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
