"""최신 프레임 하나만 예약하며 오래된 분류 결과를 버리는 비동기 실행기."""
from dataclasses import replace
import threading
import time


class LatestClassification:
    def __init__(self, function, interval=.2):
        if interval <= 0:
            raise ValueError('분류 주기는 양수여야 합니다.')
        self.function, self.interval = function, interval
        self.condition = threading.Condition()
        self.pending, self.completed = None, None
        self.generation, self.last_start = 0, float('-inf')
        self.closed, self.busy, self.error = False, False, None
        self.thread = threading.Thread(target=self._run, daemon=True, name='piece-classification')
        self.thread.start()

    def invalidate(self):
        with self.condition:
            self.generation += 1
            self.pending = self.completed = None

    def submit(self, frame, detections, metadata):
        with self.condition:
            if self.closed:
                return
            self.pending = (self.generation, frame, [replace(d) for d in detections], dict(metadata))
            self.condition.notify()

    def poll(self):
        with self.condition:
            result, self.completed = self.completed, None
            return result

    def _run(self):
        while True:
            with self.condition:
                while not self.closed:
                    delay = self.interval-(time.monotonic()-self.last_start)
                    if self.pending is not None and delay <= 0:
                        break
                    self.condition.wait(timeout=max(.001, delay) if self.pending is not None else None)
                if self.closed:
                    return
                generation, frame, detections, metadata = self.pending
                self.pending = None
                self.last_start, self.busy = time.monotonic(), True
            try:
                observations = self.function(frame, detections)
                with self.condition:
                    if generation == self.generation:
                        self.completed = dict(observations=observations, metadata=metadata)
            except Exception as exc:
                with self.condition:
                    self.error = repr(exc)
                    self.closed = True
            finally:
                with self.condition:
                    self.busy = False

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout=5.)
