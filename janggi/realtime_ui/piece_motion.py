"""통합 말 객체의 위치·신뢰도·크기 관찰. 교차점 없이 박스를 연결한다."""
from collections import deque
import math

import numpy as np
from piece_state import PieceState
from assignment import assign_candidates


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
            piece.tracking_velocity = (0., 0.)
            piece.recovery_sizes.clear()
            piece.recovery_since = None
            piece.flags = ()
            piece.visual_epoch += 1

    def drain_lifecycle(self):
        events, links = self.lifecycle_events, self.revival_links
        self.lifecycle_events, self.revival_links = [], []
        return events, links

    @staticmethod
    def reference_box(piece):
        """마지막 정상 크기로 만든 추적 박스. 부분 검출 크기로 기준을 축소하지 않는다."""
        reference = piece.good_box or piece.box
        center = np.asarray(piece.tracking_center) if piece.tracking_center is not None else geometry(reference)[0]
        size = float(np.median(piece.sensor_sizes)) if piece.sensor_sizes else geometry(reference)[1]
        aspect = (reference[2]-reference[0])/(reference[3]-reference[1])
        width, height = size*math.sqrt(aspect), size/math.sqrt(aspect)
        return (center[0]-width/2, center[1]-height/2, center[0]+width/2, center[1]+height/2)

    @classmethod
    def _remember_missing(cls, piece, now):
        # 누락 직전 추적 중심과 정상 크기로 영역을 고정한다. 휴면 중에는 이동시키지 않는다.
        reference = cls.reference_box(piece)
        pad = .35*geometry(reference)[1]
        piece.missing_box = (reference[0]-pad, reference[1]-pad, reference[2]+pad, reference[3]+pad)
        piece.missing_since = now

    @classmethod
    def _connection_score(cls, piece, box, dormant=False, reference=None, metrics=None, prediction=None):
        reference = cls.reference_box(piece) if reference is None else reference
        cx, cy, size = metrics if metrics is not None else ((box[0]+box[2])/2, (box[1]+box[3])/2,
                                                          math.sqrt((box[2]-box[0])*(box[3]-box[1])))
        ax, ay = (reference[0]+reference[2])/2, (reference[1]+reference[3])/2
        standard_size = math.sqrt((reference[2]-reference[0])*(reference[3]-reference[1]))
        distance = math.hypot(cx-ax, cy-ay)/standard_size
        ratio = size/standard_size
        search = (piece.missing_box or reference) if dormant else reference
        search_area = (search[2]-search[0])*(search[3]-search[1])
        intersection = max(0., min(search[2], box[2])-max(search[0], box[0]))*max(0., min(search[3], box[3])-max(search[1], box[1]))
        containment = intersection/min(search_area, size**2)
        if dormant:
            if distance > .95 or containment < .6 or not .08 <= ratio <= 1.4:
                return None
        else:
            predicted_distance = distance
            if prediction is not None:
                px, py = prediction
                predicted_distance = math.hypot(cx-px, cy-py)/standard_size
                dx, dy = px-ax, py-ay
                if dx or dy:
                    projected = (reference[0]+dx, reference[1]+dy, reference[2]+dx, reference[3]+dy)
                    area = max(0., min(projected[2], box[2])-max(projected[0], box[0]))*max(0., min(projected[3], box[3])-max(projected[1], box[1]))
                    containment = max(containment, area/min(standard_size**2, size**2))
            if (distance > 1.25 and predicted_distance > .8) or not .08 <= ratio <= 5:
                return None
            distance = .4*distance+.6*predicted_distance
        return 2*containment-distance

    def _confirm_revivals(self, now, valid):
        # 종류가 있는 객체는 Worker의 독립 CNN 결과로 사후 검증한다.
        for piece in self.sensors.values():
            if piece.revival_state != 'tentative':
                continue
            if not piece.eligible:
                piece.revival_frames, piece.revival_since = 0, None
                piece.revival_observations.clear()
                continue
            piece.revival_frames += 1
            if piece.revival_since is None:
                piece.revival_since = now
            if piece.label is None and piece.revival_frames >= 5 and now-piece.revival_since >= .3:
                piece.revival_state = 'active'
                self.lifecycle_events.append(dict(event='revival_verified', piece_id=piece.id, reason='geometry', observed_at=now))

    @staticmethod
    def predict_center(piece, now, reference):
        center, size = geometry(reference)
        elapsed = min(.3, max(0., now-piece.last_seen))
        delta = np.asarray(piece.tracking_velocity)*elapsed
        length = float(np.linalg.norm(delta))
        if length > size:
            delta *= size/length
        return float(center[0]+delta[0]), float(center[1]+delta[1])

    def update(self, detections, now, confidence_threshold=0.):
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
        # 프레임 안에서는 같은 기준/박스 수치를 한 번만 계산한다.
        references = {pid: self.reference_box(piece) for pid, piece in self.sensors.items()}
        metrics = [((box[0]+box[2])/2, (box[1]+box[3])/2, math.sqrt((box[2]-box[0])*(box[3]-box[1])))
                   for box, _, _ in valid]
        predictions = {pid: self.predict_center(piece, now, references[pid]) for pid, piece in self.sensors.items()}
        used_sensors, used_detections, matches = set(), set(), []
        # 높은 신뢰도부터 전체 배정하고 남은 기존 객체에만 낮은 신뢰도 관측을 사용한다.
        for weak in (False, True):
            candidates = []
            for sid, sensor in self.sensors.items():
                if sid in used_sensors:
                    continue
                for index, (box, confidence, detection) in enumerate(valid):
                    if index in used_detections or (confidence < confidence_threshold) != weak:
                        continue
                    score = self._connection_score(sensor, box, reference=references[sid], metrics=metrics[index],
                                                   prediction=predictions[sid])
                    if score is not None:
                        candidates.append((score, sid, index))
            for score, sid, index in assign_candidates(candidates):
                used_sensors.add(sid)
                used_detections.add(index)
                self.sensors[sid].association_confidence = float(np.clip((score+1.25)/3.25, 0, 1))
                matches.append((self.sensors[sid], index))
        # 활성 ID가 우선이다. 남은 검출만 휴면 ID에 즉시 임시 연결한다.
        dormant_candidates = []
        unmatched = [i for i in range(len(valid)) if i not in used_detections]
        for pid, old in self.pieces.items() if unmatched else ():
            if pid in used_sensors or old.status in ('captured', 'superseded') or old.box is None:
                continue
            if old.seen_count < 3 and old.label is None and not old.game_bound:
                continue
            reference = references.get(pid)
            if reference is None:
                reference = references[pid] = self.reference_box(old)
            for index in unmatched:
                score = self._connection_score(old, valid[index][0], dormant=True,
                                               reference=reference, metrics=metrics[index])
                if score is not None:
                    dormant_candidates.append((score, pid, index))
        for score, pid, index in sorted(dormant_candidates, reverse=True):
            if pid in used_sensors or index in used_detections:
                continue
            alternatives = [s for s, other, i in dormant_candidates
                            if (i == index and other != pid and other not in used_sensors)
                            or (other == pid and i != index and i not in used_detections)]
            if alternatives and score-max(alternatives) < .12:
                continue
            old = self.pieces[pid]
            absence = now-old.last_seen
            old.revival_state = 'tentative'
            old.revival_last_probe = old.last_classified
            old.revival_observations.clear()
            old.revival_frames, old.revival_since = 0, None
            old.visual_epoch += 1
            old.revival_count += 1
            old.last_revival = dict(observed_at=now, absence_seconds=absence, provisional=True)
            old.positions.clear()
            old.association_confidence = float(np.clip((score+1.25)/3.25, 0, 1))
            self.lifecycle_events.append(dict(event='revival_attached', piece_id=pid,
                                              absence_seconds=absence, missing_box=old.missing_box, observed_at=now))
            self.sensors[pid] = old
            used_sensors.add(pid)
            used_detections.add(index)
            matches.append((old, index))
        for index, (box, confidence, detection) in enumerate(valid):
            if index not in used_detections and confidence >= confidence_threshold:
                while f'piece_{self.next_id:04}' in self.pieces:
                    self.next_id += 1
                sensor = PieceState(f'piece_{self.next_id:04}', box=box, last_seen=now, good_box=box, status='visible')
                self.next_id += 1
                self.sensors[sensor.id] = self.pieces[sensor.id] = sensor
                matches.append((sensor, index))
                used_sensors.add(sensor.id)
                self.lifecycle_events.append(dict(event='created', piece_id=sensor.id, box=box,
                                                  confidence=confidence, reason='no_existing_assignment', observed_at=now))
        regions = []
        for sensor, index in matches:
            box, confidence, detection = valid[index]
            center, size = geometry(box)
            # 최근 0.3초의 이동 누적으로 느린 움직임도 감지한다.
            previous_center, _ = geometry(sensor.box)
            reference = references.get(sensor.id)
            if reference is None:
                reference = self.reference_box(sensor)
            tracking_size = math.sqrt((reference[2]-reference[0])*(reference[3]-reference[1]))
            frame_movement = float(np.linalg.norm(center-previous_center))/tracking_size
            sensor.positions.append((now, center))
            while sensor.positions and now-sensor.positions[0][0] > .3:
                sensor.positions.popleft()
            movement = max(frame_movement, max((float(np.linalg.norm(center-point))/tracking_size for _, point in sensor.positions), default=0.))
            reference_conf = float(np.median(sensor.sensor_confidence)) if sensor.sensor_confidence else confidence
            reference_size = float(np.median(sensor.sensor_sizes)) if sensor.sensor_sizes else size
            sensor.tracking_only = confidence < confidence_threshold
            flags = ['CONF'] if sensor.tracking_only else []
            if movement >= self.move_ratio:
                flags.append('MOVE')
            if len(sensor.sensor_confidence) >= 3 and reference_conf-confidence >= self.confidence_drop:
                if 'CONF' not in flags:
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
                sensor.revival_observations.clear()
            if flags:
                sensor.flags, sensor.last_alert = tuple(flags), now
            elif sensor.flags == ('MISS',) or now-sensor.last_alert >= self.hold:
                sensor.flags = ()
            elapsed = now-sensor.last_seen
            if 0 < elapsed <= .5:
                instantaneous = (center-previous_center)/elapsed
                # 부분 박스 변형에 의한 속도 급증을 제한하고 속도는 부드럽게 추정한다.
                speed = float(np.linalg.norm(instantaneous))
                limit = 8*tracking_size
                if speed > limit:
                    instantaneous *= limit/speed
                gain = .2 if flags else .5
                sensor.tracking_velocity = tuple((1-gain)*np.asarray(sensor.tracking_velocity)+gain*instantaneous)
            elif elapsed > .5:
                sensor.tracking_velocity = (0., 0.)
            sensor.box, sensor.last_seen, sensor.misses = box, now, 0
            sensor.seen_count += 1
            sensor.visible, sensor.cell = True, detection.board
            sensor.detection_confidence = confidence
            sensor.status = 'on_board' if sensor.cell is not None else 'visible'
            detection.piece_id = sensor.id
            detection.tracking_only = sensor.tracking_only
            # ID의 위치 추적은 방해 플래그와 독립이다. 느린 부분 이동도 매 검출마다 따라간다.
            # 크기/종류 필터는 아래의 정상 관측에서만 갱신한다.
            sensor.tracking_center = tuple(center)
            if not sensor.flags:
                sensor.sensor_confidence.append(confidence)
                sensor.sensor_sizes.append(size)
                sensor.good_box = box
            if sensor.flags:
                region_box = box
                if any(flag in sensor.flags for flag in ('CONF', 'SIZE')):
                    reference = self.reference_box(sensor)
                    region_box = (min(box[0], reference[0]), min(box[1], reference[1]),
                                  max(box[2], reference[2]), max(box[3], reference[3]))
                regions.append(dict(sensor_id=sensor.id, box=list(region_box), visible=True, reasons=list(sensor.flags),
                                    confidence=confidence, baseline_confidence=reference_conf,
                                    geometric_size_px=size, baseline_size_px=reference_size))
        for sid, sensor in self.sensors.items():
            if sid not in used_sensors:
                sensor.misses += 1
                sensor.status = 'dormant'
                if sensor.misses == 1:
                    self._remember_missing(sensor, now)
                    self.lifecycle_events.append(dict(event='dormant', piece_id=sid, last_seen=sensor.last_seen, missing_box=sensor.missing_box, observed_at=now))
                if sensor.misses == 1:
                    sensor.visual_epoch += 1
                if sensor.misses >= 2:
                    sensor.flags = ('MISS',)
        # 검색/보관에 시간 제한을 두지 않는다. 보이지 않는 객체는 휴면 상태로 남긴다.
        for pid, piece in self.pieces.items():
            if not piece.visible and piece.status not in ('captured', 'superseded'):
                piece.status = 'dormant'
                if now-piece.last_seen > .5 and piece.misses == 0:
                    self._remember_missing(piece, now)
                    piece.misses = 2
                    piece.flags = ('MISS',)
                    piece.visual_epoch += 1
                if piece.misses >= 2 and now-piece.last_seen <= self.miss_display_seconds:
                    regions.append(dict(sensor_id=pid, box=list(piece.missing_box or piece.box), visible=False, reasons=['MISS'],
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
