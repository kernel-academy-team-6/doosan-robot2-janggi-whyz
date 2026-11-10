"""실험용 말 박스 센서. 교차점·종류·게임 추적 ID와 독립적으로 동작한다."""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


def geometry(box):
    x1, y1, x2, y2 = box
    return np.array([(x1+x2)/2, (y1+y2)/2]), math.sqrt((x2-x1)*(y2-y1))


def iou(a, b):
    w = max(0., min(a[2], b[2])-max(a[0], b[0]))
    h = max(0., min(a[3], b[3])-max(a[1], b[1]))
    intersection = w*h
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union > 0 else 0.


@dataclass
class Sensor:
    id: int
    box: tuple
    last_seen: float
    good_box: tuple
    confidence: deque = field(default_factory=lambda: deque(maxlen=20))
    sizes: deque = field(default_factory=lambda: deque(maxlen=20))
    positions: deque = field(default_factory=lambda: deque(maxlen=20))
    misses: int = 0
    flags: tuple = ()
    last_alert: float = float('-inf')


class PieceMotionMonitor:
    def __init__(self, confidence_drop=.15, size_change=.25, move_ratio=.2, hold=.5, max_missing=2.):
        if (not all(math.isfinite(v) for v in (confidence_drop, size_change, move_ratio, hold, max_missing))
                or not 0 < confidence_drop <= 1 or size_change <= 0 or move_ratio <= 0 or hold < 0 or max_missing <= 0):
            raise ValueError('국소 방해 감지 설정값을 확인하세요.')
        self.confidence_drop, self.size_change, self.move_ratio = confidence_drop, size_change, move_ratio
        self.hold, self.max_missing = hold, max_missing
        self.reset()

    def reset(self):
        self.sensors, self.next_id = {}, 1

    def update(self, detections, now):
        valid = []
        for detection in detections:
            box = tuple(float(v) for v in detection.box)
            if (len(box) == 4 and all(math.isfinite(v) for v in box) and box[2] > box[0] and box[3] > box[1]
                    and math.isfinite(detection.conf)):
                valid.append((box, float(detection.conf)))
        self.sensors = {sid: sensor for sid, sensor in self.sensors.items() if now-sensor.last_seen <= self.max_missing}
        # 위치와 박스 겹침으로만 연결한다. 종류 예측의 흔들림은 센서 ID 연결에 사용하지 않는다.
        candidates = []
        for sid, sensor in self.sensors.items():
            old_center, old_size = geometry(sensor.box)
            for index, (box, confidence) in enumerate(valid):
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
                matches.append((self.sensors[sid], index))
        for index, (box, confidence) in enumerate(valid):
            if index not in used_detections:
                sensor = Sensor(self.next_id, box, now, box)
                self.next_id += 1
                self.sensors[sensor.id] = sensor
                matches.append((sensor, index))
                used_sensors.add(sensor.id)
        regions = []
        for sensor, index in matches:
            box, confidence = valid[index]
            center, size = geometry(box)
            # 최근 0.3초의 이동 누적으로 느린 움직임도 감지한다.
            previous_center, _ = geometry(sensor.box)
            frame_movement = float(np.linalg.norm(center-previous_center))/size
            sensor.positions.append((now, center))
            while sensor.positions and now-sensor.positions[0][0] > .3:
                sensor.positions.popleft()
            movement = max(frame_movement, max((float(np.linalg.norm(center-point))/size for _, point in sensor.positions), default=0.))
            reference_conf = float(np.median(sensor.confidence)) if sensor.confidence else confidence
            reference_size = float(np.median(sensor.sizes)) if sensor.sizes else size
            flags = []
            if movement >= self.move_ratio:
                flags.append('MOVE')
            if len(sensor.confidence) >= 3 and reference_conf-confidence >= self.confidence_drop:
                flags.append('CONF')
            if len(sensor.sizes) >= 3 and abs(size/reference_size-1) >= self.size_change:
                flags.append('SIZE')
            if flags:
                sensor.flags, sensor.last_alert = tuple(flags), now
            elif now-sensor.last_alert >= self.hold:
                sensor.flags = ()
            sensor.box, sensor.last_seen, sensor.misses = box, now, 0
            if not sensor.flags:
                sensor.confidence.append(confidence)
                sensor.sizes.append(size)
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
                if sensor.misses >= 2:
                    regions.append(dict(sensor_id=sid, box=list(sensor.box), visible=False, reasons=['MISS'],
                                        confidence=None, missing_seconds=now-sensor.last_seen))
        if len(self.sensors) > 128:
            retained = sorted(self.sensors, key=lambda sid: self.sensors[sid].last_seen, reverse=True)[:128]
            self.sensors = {sid: self.sensors[sid] for sid in retained}
            regions = [r for r in regions if r["sensor_id"] in self.sensors]
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
        tag = f'S{region["sensor_id"]} '+ '/'.join(region['reasons'])
        (w, h), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, .4, 1)
        x, y = max(0, min(x1, image.shape[1]-w-2)), min(image.shape[0]-3, y2+h+3)
        cv2.putText(image, tag, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(image, tag, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
    return image
