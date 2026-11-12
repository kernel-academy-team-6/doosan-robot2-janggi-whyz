"""Calibrated board polygon plus a pixel margin, independent of rendering zoom."""
import cv2
import numpy as np


class BoardRecognitionRegion:
    def __init__(self, polygon):
        self.polygon = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        self._key = None
        self.mask = None

    def prepare(self, frame, margin):
        """Mask detector input only; keep original pixels for display/classification."""
        margin = int(margin)
        if margin < 0:
            raise ValueError('판 영역 여유는 0 이상의 픽셀이어야 합니다.')
        h, w = frame.shape[:2]
        key = (h, w, margin)
        if key != self._key:
            inside = np.zeros((h, w), np.uint8)
            cv2.fillPoly(inside, [np.rint(self.polygon).astype(np.int32)], 255)
            # Euclidean dilation follows the rotated/perspective board edges.
            outside_distance = cv2.distanceTransform(255-inside, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
            self.mask = np.asarray(outside_distance <= margin, np.uint8) * 255
            lo = np.floor(self.polygon.min(axis=0) - margin).astype(int)
            hi = np.ceil(self.polygon.max(axis=0) + margin).astype(int) + 1
            self.roi = (int(np.clip(lo[0], 0, w)), int(np.clip(lo[1], 0, h)),
                        int(np.clip(hi[0], 0, w)), int(np.clip(hi[1], 0, h)))
            self._key = key
        masked = cv2.copyTo(frame, self.mask, np.full_like(frame, 114))
        return masked, self.roi

    def filter(self, detections, margin):
        # Use original-frame centers so corner pieces crossing the boundary
        # remain detectable while objects farther outside never reach tracking.
        return [d for d in detections
                if cv2.pointPolygonTest(self.polygon, tuple(map(float, d.center)), True) >= -margin]
