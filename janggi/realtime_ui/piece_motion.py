"""통합 말 객체의 위치·신뢰도·크기 관찰. 교차점 없이 박스를 연결한다."""
from collections import deque
import math

import numpy as np
from piece_state import PieceState


def geometry(box):
    x1, y1, x2, y2 = box
    return np.array([(x1+x2)/2, (y1+y2)/2]), math.sqrt((x2-x1)*(y2-y1))


def iou(a, b):
    w = max(0., min(a[2], b[2])-max(a[0], b[0]))
    h = max(0., min(a[3], b[3])-max(a[1], b[1]))
    intersection = w*h
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union > 0 else 0.


class PieceMotionMonitor:
    def __init__(self, confidence_drop=.15, size_change=.25, move_ratio=.2, hold=.5, miss_display_seconds=2., pieces=None):
        if (not all(math.isfinite(v) for v in (confidence_drop, size_change, move_ratio, hold, miss_display_seconds))
                or not 0 < confidence_drop <= 1 or size_change <= 0 or move_ratio <= 0 or hold < 0 or miss_display_seconds <= 0):
            raise ValueError('국소 방해 감지 설정값을 확인하세요.')
        self.confidence_drop, self.size_change, self.move_ratio = confidence_drop, size_change, move_ratio
        self.hold, self.miss_display_seconds = hold, miss_display_seconds
        self.pieces = pieces if pieces is not None else {}
        self.sensors, self.next_id = {}, 1
        self.lifecycle_events, self.revival_links = [], []

    def reset(self):
        self.sensors.clear()
        self.pieces.clear()
        self.lifecycle_events.clear()
        self.revival_links.clear()

    def reset_baselines(self):
        for piece in self.pieces.values():
            if 'SIZE' in piece.flags:
                piece.reference_reset_evidence = dict(old_size_mm=piece.size_mm, label=piece.label,
                                                       probabilities=dict(piece.probabilities), reason='manual_reset')
                piece.sizes.clear()
                piece.size_reference_rebuilds += 1
            piece.sensor_confidence.clear()
            piece.sensor_sizes.clear()
            piece.positions.clear()
            piece.recovery_sizes.clear()
            piece.recovery_since = None
            piece.flags = ()
            piece.visual_epoch += 1

    def drain_lifecycle(self):
        events, links = self.lifecycle_events, self.revival_links
        self.lifecycle_events, self.revival_links = [], []
        return events, links

    @staticmethod
    def _dormant_score(old, box):
        if old.box is None:
            return None
        center, current_size = geometry(box)
        old_center, last_size = geometry(old.box)
        size = float(np.median(old.sensor_sizes)) if old.sensor_sizes else last_size
        reference = old.good_box or old.box
        aspect = (reference[2]-reference[0])/(reference[3]-reference[1])
        width, height = size*math.sqrt(aspect), size/math.sqrt(aspect)
        expected = (old_center[0]-width/2, old_center[1]-height/2,
                    old_center[0]+width/2, old_center[1]+height/2)
        distance = float(np.linalg.norm(center-old_center))/max(size, current_size)
        overlap = iou(expected, box)
        ratio = current_size/size
        if distance > .5 or overlap < .3 or not .75 <= ratio <= 1.33:
            return None
        return 2*overlap-distance, overlap, distance, ratio

    def _confirm_revivals(self, now, valid):
        # 새 관측은 독립 ID로 분류하며 확인 전에는 과거 이력을 섞지 않는다.
        for current in list(self.sensors.values()):
            if not current.visible or current.revival_state != 'pending':
                continue
            if not current.eligible or current.detection_confidence < .7:
                current.revival_frames, current.revival_since = 0, None
                current.revival_votes.clear()
                current.revival_last_probe = current.last_classified
                continue
            current.revival_frames += 1
            if current.revival_since is None:
                current.revival_since = now
            raw = (current.last_raw or {}).get('probabilities') or {}
            new_probe = bool(raw) and current.last_classified > current.revival_last_probe
            if new_probe:
                current.revival_last_probe = current.last_classified
            matches, remaining, scores = [], [], []
            for pid in current.revival_candidates:
                old = self.pieces.get(pid)
                if old is None or old.visible or old.status in ('captured', 'superseded'):
                    continue
                match = self._dormant_score(old, current.box)
                if match is None:
                    continue
                score, overlap, distance, ratio = match
                if old.label is not None:
                    if new_probe:
                        winner = max(raw, key=raw.get)
                        if winner != old.label and raw[winner] >= .7:
                            self.lifecycle_events.append(dict(event='revival_rejected', piece_id=current.id,
                                dormant_id=pid, reason='class_mismatch', observed_at=now))
                            continue
                        current.revival_votes[pid] = current.revival_votes.get(pid, 0)+1 if winner == old.label and raw[old.label] >= .6 else 0
                    ready = (overlap >= .5 and distance <= .35 and .8 <= ratio <= 1.25
                             and current.revival_votes.get(pid, 0) >= 2 and current.revival_frames >= 3
                             and now-current.revival_since >= .2)
                else:
                    ready = (overlap >= .7 and distance <= .2 and .85 <= ratio <= 1.18
                             and current.detection_confidence >= .8 and current.revival_frames >= 5
                             and now-current.revival_since >= .3)
                remaining.append(pid)
                scores.append((score, pid))
                if ready:
                    matches.append((score, pid))
            current.revival_candidates = remaining
            if not remaining:
                current.revival_state = 'new_identity'
                continue
            matches.sort(reverse=True)
            if len(matches) > 1 and matches[0][0]-matches[1][0] < .12:
                current.revival_state = 'ambiguous_new_identity'
                self.lifecycle_events.append(dict(event='revival_ambiguous', piece_id=current.id,
                                                  candidates=[pid for _, pid in matches], observed_at=now))
                continue
            if not matches:
                continue
            # 미확인 후보도 포함하여 같은 위치의 과거 ID가 경쟁하면 연결을 보류한다.
            best_score, best_id = matches[0]
            if any(pid != best_id and score >= best_score-.12 for score, pid in scores):
                continue
            old = self.pieces[best_id]
            absence = now-old.last_seen
            for name in ('box', 'cell', 'visible', 'last_seen', 'detection_confidence', 'association_confidence',
                         'good_box', 'sensor_confidence', 'sensor_sizes', 'positions', 'misses', 'flags', 'last_alert'):
                setattr(old, name, getattr(current, name))
            old.visual_epoch += 1
            old.recovery_sizes.clear()
            old.recovery_since = None
            old.seen_count += current.seen_count
            old.status = 'on_board' if old.cell is not None else 'visible'
            old.revival_state = 'active'
            old.revival_count += 1
            old.last_revival = dict(observed_at=now, absence_seconds=absence, provisional_id=current.id)
            current.visible, current.cell, current.status = False, None, 'superseded'
            current.superseded_by = old.id
            self.sensors.pop(current.id, None)
            self.sensors[old.id] = old
            for _, _, detection in valid:
                if detection.piece_id == current.id:
                    detection.piece_id = old.id
            self.revival_links.append(dict(piece_id=old.id, provisional_id=current.id,
                observation=current.last_raw, stamp=current.history[-1]['time'] if current.history else None))
            self.lifecycle_events.append(dict(event='revived', piece_id=old.id, provisional_id=current.id,
                absence_seconds=absence, observations=current.revival_frames, observed_at=now))

    def update(self, detections, now):
        valid = []
        for detection in detections:
            box = tuple(float(v) for v in detection.box)
            if (len(box) == 4 and all(math.isfinite(v) for v in box) and box[2] > box[0] and box[3] > box[1]
                    and math.isfinite(detection.conf)):
                valid.append((box, float(detection.conf), detection))
        self.sensors = {pid: piece for pid, piece in self.pieces.items()
                        if (piece.visible or (piece.misses <= 1 and now-piece.last_seen <= .5)) and piece.box is not None
                        and piece.status not in ('captured', 'superseded')}
        for piece in self.pieces.values():
            if piece.visible:
                piece.visible = False
                piece.cell = None
                if piece.status not in ('captured', 'superseded'):
                    piece.status = 'unobserved'
        # 위치와 박스 겹침으로만 연결한다. 종류 예측의 흔들림은 센서 ID 연결에 사용하지 않는다.
        candidates = []
        for sid, sensor in self.sensors.items():
            old_center, old_size = geometry(sensor.box)
            for index, (box, confidence, detection) in enumerate(valid):
                center, size = geometry(box)
                distance = float(np.linalg.norm(center-old_center))/max(old_size, size)
                overlap = iou(sensor.box, box)
                if distance <= 1.25 and .2 <= size/old_size <= 5:
                    candidates.append((2*overlap-distance, sid, index))
        used_sensors, used_detections, matches = set(), set(), []
        for score, sid, index in sorted(candidates, reverse=True):
            if sid not in used_sensors and index not in used_detections:
                alternatives = [s for s, other, i in candidates if i == index and other != sid and other not in used_sensors]
                if alternatives and score-max(alternatives) < .08:
                    continue  # 가까운 말끼리 구분이 애매하면 이력을 억지로 연결하지 않는다.
                used_sensors.add(sid)
                used_detections.add(index)
                self.sensors[sid].association_confidence = float(np.clip((score+1.25)/3.25, 0, 1))
                matches.append((self.sensors[sid], index))
        for index, (box, confidence, detection) in enumerate(valid):
            if index not in used_detections:
                while f'piece_{self.next_id:04}' in self.pieces:
                    self.next_id += 1
                sensor = PieceState(f'piece_{self.next_id:04}', box=box, last_seen=now, good_box=box, status='visible')
                self.next_id += 1
                self.sensors[sensor.id] = self.pieces[sensor.id] = sensor
                sensor.revival_candidates = [pid for pid, old in self.pieces.items()
                    if pid != sensor.id and pid not in used_sensors and old.status not in ('captured', 'superseded')
                    and (old.seen_count >= 3 or old.label is not None or old.game_bound)
                    and self._dormant_score(old, box) is not None]
                if sensor.revival_candidates:
                    sensor.revival_state = 'pending'
                    self.lifecycle_events.append(dict(event='revival_started', piece_id=sensor.id,
                                                      candidates=list(sensor.revival_candidates), observed_at=now))
                matches.append((sensor, index))
                used_sensors.add(sensor.id)
        regions = []
        for sensor, index in matches:
            box, confidence, detection = valid[index]
            center, size = geometry(box)
            # 최근 0.3초의 이동 누적으로 느린 움직임도 감지한다.
            previous_center, _ = geometry(sensor.box)
            frame_movement = float(np.linalg.norm(center-previous_center))/size
            sensor.positions.append((now, center))
            while sensor.positions and now-sensor.positions[0][0] > .3:
                sensor.positions.popleft()
            movement = max(frame_movement, max((float(np.linalg.norm(center-point))/size for _, point in sensor.positions), default=0.))
            reference_conf = float(np.median(sensor.sensor_confidence)) if sensor.sensor_confidence else confidence
            reference_size = float(np.median(sensor.sensor_sizes)) if sensor.sensor_sizes else size
            flags = []
            if movement >= self.move_ratio:
                flags.append('MOVE')
            if len(sensor.sensor_confidence) >= 3 and reference_conf-confidence >= self.confidence_drop:
                flags.append('CONF')
            size_limit = min(.2, self.size_change*.8) if 'SIZE' in sensor.flags else self.size_change
            size_abnormal = len(sensor.sensor_sizes) >= 3 and abs(size/reference_size-1) >= size_limit
            if size_abnormal:
                flags.append('SIZE')
            # 작은 임시 기준이 정상 복귀를 영구 SIZE로 만들지 않게 회복 후보를 별도로 검증한다.
            # 기준을 자동 축소하지 않는다. MOVE/CONF/MISS 없이 신뢰도가 높고 더 큰 박스가 일정해야 한다.
            recovery = (flags == ['SIZE'] and 1 < size/reference_size <= 2
                        and confidence >= max(.8, reference_conf-.05)
                        and max((box[2]-box[0])/(box[3]-box[1]), (box[3]-box[1])/(box[2]-box[0])) <= 1.6)
            if recovery:
                if sensor.recovery_since is None:
                    sensor.recovery_since = now
                sensor.recovery_sizes.append((now, size))
                values = [v for _, v in sensor.recovery_sizes]
                if (max(values)-min(values))/float(np.median(values)) > .05:
                    sensor.recovery_sizes.clear()
                    sensor.recovery_sizes.append((now, size))
                    sensor.recovery_since = now
                if (len(values) >= 5 and now-sensor.recovery_since >= .8
                        and (max(values)-min(values))/float(np.median(values)) <= .05):
                    sensor.sensor_sizes.clear()
                    sensor.sensor_sizes.extend(values)
                    reference_size = float(np.median(values))
                    flags.remove('SIZE')
                    sensor.flags = tuple(f for f in sensor.flags if f != 'SIZE')
                    sensor.reference_reset_evidence = dict(old_size_mm=sensor.size_mm, label=sensor.label,
                                                           probabilities=dict(sensor.probabilities))
                    sensor.sizes.clear()  # 잘못된 작은 mm 이력이 이후 관측을 특이값으로 계속 거부하지 않게 한다.
                    sensor.visual_epoch += 1
                    sensor.size_reference_rebuilds += 1
                    sensor.recovery_sizes.clear()
                    sensor.recovery_since = None
            else:
                sensor.recovery_sizes.clear()
                sensor.recovery_since = None
            if flags and not sensor.flags:
                sensor.visual_epoch += 1
            if flags:
                sensor.flags, sensor.last_alert = tuple(flags), now
            elif sensor.flags == ('MISS',) or now-sensor.last_alert >= self.hold:
                sensor.flags = ()
            sensor.box, sensor.last_seen, sensor.misses = box, now, 0
            sensor.seen_count += 1
            sensor.visible, sensor.cell = True, detection.board
            sensor.detection_confidence = confidence
            sensor.status = 'on_board' if sensor.cell is not None else 'visible'
            detection.piece_id = sensor.id
            if not sensor.flags:
                sensor.sensor_confidence.append(confidence)
                sensor.sensor_sizes.append(size)
                sensor.good_box = box
            if sensor.flags:
                region_box = box
                if any(flag in sensor.flags for flag in ('CONF', 'SIZE')):
                    region_box = (min(box[0], sensor.good_box[0]), min(box[1], sensor.good_box[1]),
                                  max(box[2], sensor.good_box[2]), max(box[3], sensor.good_box[3]))
                regions.append(dict(sensor_id=sensor.id, box=list(region_box), visible=True, reasons=list(sensor.flags),
                                    confidence=confidence, baseline_confidence=reference_conf,
                                    geometric_size_px=size, baseline_size_px=reference_size))
        for sid, sensor in self.sensors.items():
            if sid not in used_sensors:
                sensor.misses += 1
                sensor.status = 'dormant'
                if sensor.misses == 1:
                    self.lifecycle_events.append(dict(event='dormant', piece_id=sid, last_seen=sensor.last_seen, observed_at=now))
                if sensor.misses == 1:
                    sensor.visual_epoch += 1
                if sensor.misses >= 2:
                    sensor.flags = ('MISS',)
        # 검색/보관에 시간 제한을 두지 않는다. 보이지 않는 객체는 휴면 상태로 남긴다.
        for pid, piece in self.pieces.items():
            if not piece.visible and piece.status not in ('captured', 'superseded'):
                piece.status = 'dormant'
                if now-piece.last_seen > .5 and piece.misses == 0:
                    piece.misses = 2
                    piece.flags = ('MISS',)
                    piece.visual_epoch += 1
                if piece.misses >= 2 and now-piece.last_seen <= self.miss_display_seconds:
                    regions.append(dict(sensor_id=pid, box=list(piece.box), visible=False, reasons=['MISS'],
                                        confidence=None, missing_seconds=now-piece.last_seen))
        self._confirm_revivals(now, valid)
        return regions


def draw_piece_regions(image, regions, offset=(0, 0)):
    import cv2
    for region in regions:
        box = region['box']
        pad = max(3, round(geometry(box)[1]*.12))
        x1, y1 = round(box[0]-offset[0])-pad, round(box[1]-offset[1])-pad
        x2, y2 = round(box[2]-offset[0])+pad, round(box[3]-offset[1])+pad
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(image.shape[1]-1, x2), min(image.shape[0]-1, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        warning = any(reason != 'MOVE' for reason in region['reasons'])
        color = (40, 50, 230) if warning else (0, 195, 255)
        roi = image[y1:y2+1, x1:x2+1]
        roi[:] = (roi.astype(np.float32)*.8+np.asarray(color)*.2).astype(np.uint8)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        tag = str(region['sensor_id'])+' '+ '/'.join(region['reasons'])
        (w, h), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, .4, 1)
        x, y = max(0, min(x1, image.shape[1]-w-2)), min(image.shape[0]-3, y2+h+3)
        cv2.putText(image, tag, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(image, tag, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
    return image
