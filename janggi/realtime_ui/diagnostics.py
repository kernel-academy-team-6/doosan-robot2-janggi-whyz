"""표시 전용 시야 방해 휴리스틱과 검출 박스 크기 추정. 게임 상태를 변경하지 않는다."""
from collections import deque
from functools import lru_cache
from pathlib import Path
import math

import cv2
import numpy as np
import yaml

from piece_detection.world import CameraToBase


class ObstructionMonitor:
    def __init__(self, window=2., settle=.8, motion_threshold=.03):
        if window <= 0 or settle < 0 or not 0 < motion_threshold <= 1:
            raise ValueError('방해 감지 창은 양수, 안정 시간은 0 이상, 움직임 비율은 0~1입니다.')
        self.window, self.settle, self.motion_threshold = window, settle, motion_threshold
        self.reset()

    def reset(self):
        self.previous = None
        self.history = deque()
        self.quiet_since = None
        self.baseline = None
        self.mask_key = None
        self.mask = None

    @staticmethod
    def oscillates(values):
        values = [v for v in values if v is not None]
        changes = [b-a for a, b in zip(values, values[1:]) if b != a]
        return any(a*b < 0 for a, b in zip(changes, changes[1:]))

    def update(self, frame, detected, assigned, now, polygon=None):
        scale = min(1., 320/max(frame.shape[:2]))
        size = (max(1, round(frame.shape[1]*scale)), max(1, round(frame.shape[0]*scale)))
        small = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        polygon_key = None if polygon is None else tuple(np.asarray(polygon).ravel())
        key = (frame.shape[:2], gray.shape, polygon_key)
        if key != self.mask_key:
            self.reset()
            self.mask_key = key
            self.mask = np.ones(gray.shape, np.uint8)
            if polygon is not None:
                self.mask.fill(0)
                cv2.fillConvexPoly(self.mask, np.round(np.asarray(polygon)*scale).astype(np.int32), 1)
        if not self.mask.any():
            return dict(state='unavailable', reason='보정 영역이 화면 밖입니다.', motion=0.)
        motion = 0.
        if self.previous is not None:
            changed = (cv2.absdiff(gray, self.previous) >= 20) & (self.mask > 0)
            motion = float(changed.sum()/max(1, self.mask.sum()))
        self.previous = gray
        self.history.append((now, detected, assigned))
        while self.history and now-self.history[0][0] > self.window:
            self.history.popleft()
        jitter = self.oscillates([r[1] for r in self.history]) or self.oscillates([r[2] for r in self.history])
        loss = self.baseline is not None and (
            detected <= self.baseline[0]-2 or
            (assigned is not None and self.baseline[1] is not None and assigned <= self.baseline[1]-2))
        reasons = []
        if motion >= self.motion_threshold:
            reasons.append('판 영역 움직임')
        if jitter:
            reasons.append('검출/판 할당 수 진동')
        if loss:
            reasons.append('안정 기준보다 말 수 감소')
        if reasons:
            self.quiet_since = None
            state, reason = 'suspected', ' · '.join(reasons)
        elif detected == 0 or assigned == 0:
            self.quiet_since = None
            state, reason = 'unavailable', '말 검출 또는 판 할당 없음 · 원인 미확인'
        else:
            if self.quiet_since is None:
                self.quiet_since = now
            if now-self.quiet_since < self.settle or len(self.history) < 3:
                state, reason = 'settling', '화면 안정 여부 확인 중'
            else:
                state, reason = 'stable', '움직임·개수 진동 징후 없음'
                self.baseline = (detected, assigned)
        return dict(state=state, reason=reason, motion=motion, detected=detected, assigned=assigned,
                    scope='장기판' if polygon is not None else '전체 화면')


class BoxSizeEstimator:
    def __init__(self, camera, image_size, height_mm=12.):
        self.camera, self.image_size, self.height_mm = camera, image_size, height_mm

    @classmethod
    def from_calibration(cls, path, height_mm=12.):
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        intr = data['camera']['color_intrinsics']
        return cls(CameraToBase.from_calibration(path), (intr['width'], intr['height']), height_mm)

    @lru_cache(maxsize=256)
    def measure(self, box, frame_size):
        if tuple(frame_size) != tuple(self.image_size):
            return None
        x1, y1, x2, y2 = box
        if not all(math.isfinite(v) for v in box) or x2 <= x1 or y2 <= y1:
            return None
        cx, cy = (x1+x2)/2, (y1+y2)/2
        try:
            points = [self.camera.pixel_to_base(u, v, self.height_mm)
                      for u, v in ((x1, cy), (x2, cy), (cx, y1), (cx, y2))]
            width, height = float(np.linalg.norm(points[1]-points[0])), float(np.linalg.norm(points[3]-points[2]))
            return (width, height) if np.isfinite([width, height]).all() else None
        except ValueError:
            return None


def box_label(detection, mode='confidence', estimator=None, frame_size=None):
    if mode in ('geometric-px', 'geometric-mm'):
        x1, y1, x2, y2 = detection.box
        if mode == 'geometric-mm':
            if detection.geometric_size_mm is not None:
                return f'G{detection.geometric_size_mm:.1f}mm~'
            if estimator is not None and frame_size is not None:
                size = estimator.measure(tuple(detection.box), tuple(frame_size))
                if size is not None:
                    return f'G{math.sqrt(size[0]*size[1]):.1f}mm~'
        return f'G{math.sqrt(max(0, (x2-x1)*(y2-y1))):.1f}px'+(' (mm N/A)' if mode == 'geometric-mm' else '')
    if mode in ('size-px', 'size-mm'):
        x1, y1, x2, y2 = detection.box
        if mode == 'size-mm' and estimator is not None and frame_size is not None:
            size = estimator.measure(tuple(detection.box), tuple(frame_size))
            if size is not None:
                return f'{size[0]:.1f}x{size[1]:.1f}mm~'
        return f'{x2-x1:.1f}x{y2-y1:.1f}px'+(' (mm N/A)' if mode == 'size-mm' else '')
    probability = detection.classification_conf
    return (f'D{detection.conf*100:.0f}/C{probability*100:.0f}' if probability is not None
            else f'{detection.conf*100:.0f}')
