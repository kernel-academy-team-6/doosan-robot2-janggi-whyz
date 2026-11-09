"""프레임별 개수 기록. 정답 위치 없이 검출 정확도 대신 개수 충족률을 집계한다."""
from __future__ import annotations

import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np


def confidence_stats(confidences):
    values = sorted(float(v) for v in confidences) if confidences is not None else []
    n = len(values)
    k = math.ceil(n * .3)
    return dict(count=n, mean=sum(values) / n if n else None,
                min=values[0] if n else None,
                p30=float(np.percentile(values, 30)) if n else None,
                bottom30_count=k, bottom30_mean=sum(values[:k]) / k if k else None)


CONF_FIELDS = tuple(confidence_stats([]))


class CountLog:
    def __init__(self, directory, expected, metadata):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        session = directory / f'counts_{stamp}'
        session.mkdir(exist_ok=False)
        self.path = session / 'counts.csv'
        self.file = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=[
            'frame', 'elapsed_s', 'source_frame', 'source_time_s', 'source_loop',
            'expected', 'detected', 'deduplicated', 'assigned', 'stable_assigned',
            'deduplicated_equals_expected', 'assigned_equals_expected', 'confidence_threshold', 'roi_used',
            *[prefix + key for prefix in ('conf_', 'dedup_conf_') for key in CONF_FIELDS]])
        self.writer.writeheader()
        self.expected, self.metadata = expected, metadata
        self.started = time.monotonic()
        self.frames = 0
        self.closed = False
        self.thresholds = set()
        self.metrics = {name: dict(samples=0, total=0, minimum=None, maximum=None, equal=0, below=0, above=0)
                        for name in ('detected', 'deduplicated', 'assigned', 'stable_assigned')}
        self.confidence = {name: dict(frames=0, count=0, total=0., minimum=None,
                                     frame_p30_total=0., frame_bottom30_mean_total=0.)
                           for name in ('raw', 'deduplicated')}

    def record(self, detected, deduplicated, assigned, stable_assigned, confidence, roi_used,
               source_frame=None, source_time=None, source_loop=None,
               confidences=None, deduplicated_confidences=None):
        self.frames += 1
        values = dict(detected=detected, deduplicated=deduplicated,
                      assigned=assigned, stable_assigned=stable_assigned)
        row = dict(frame=self.frames, elapsed_s=round(time.monotonic() - self.started, 6),
                   source_frame=source_frame, source_time_s=source_time, source_loop=source_loop,
                   expected=self.expected, **values,
                   deduplicated_equals_expected=int(deduplicated == self.expected),
                   assigned_equals_expected=int(assigned == self.expected) if assigned is not None else None,
                   confidence_threshold=confidence, roi_used=int(roi_used))
        for name, prefix, scores in (('raw', 'conf_', confidences),
                                     ('deduplicated', 'dedup_conf_', deduplicated_confidences)):
            stats = confidence_stats(scores)
            row.update({prefix + key: value for key, value in stats.items()})
            if stats['count']:
                m = self.confidence[name]
                m['frames'] += 1
                m['count'] += stats['count']
                m['total'] += stats['mean'] * stats['count']
                m['minimum'] = stats['min'] if m['minimum'] is None else min(m['minimum'], stats['min'])
                m['frame_p30_total'] += stats['p30']
                m['frame_bottom30_mean_total'] += stats['bottom30_mean']
        self.writer.writerow(row)
        self.thresholds.add(float(confidence))
        for name, value in values.items():
            if value is None:
                continue
            m = self.metrics[name]
            m['samples'] += 1
            m['total'] += value
            m['minimum'] = value if m['minimum'] is None else min(m['minimum'], value)
            m['maximum'] = value if m['maximum'] is None else max(m['maximum'], value)
            m['equal'] += value == self.expected
            m['below'] += value < self.expected
            m['above'] += value > self.expected
        if self.frames % 30 == 0:
            self.file.flush()

    def close(self, error=None):
        if self.closed:
            return
        self.closed = True
        self.file.close()
        metrics = {}
        for name, m in self.metrics.items():
            n = m['samples']
            metrics[name] = {**m, 'mean': m['total'] / n if n else None,
                             'equal_expected_fraction': m['equal'] / n if n else None}
        confidence = {}
        for name, m in self.confidence.items():
            n, f = m['count'], m['frames']
            confidence[name] = dict(frames_with_detections=f, detection_samples=n,
                                    mean=m['total'] / n if n else None, min=m['minimum'],
                                    mean_frame_p30=m['frame_p30_total'] / f if f else None,
                                    mean_frame_bottom30_mean=m['frame_bottom30_mean_total'] / f if f else None)
        summary = dict(metadata=self.metadata, expected=self.expected, processed_frames=self.frames,
                       elapsed_s=time.monotonic() - self.started, confidence_thresholds=sorted(self.thresholds),
                       metrics=metrics, confidence=confidence, error=error,
                       interpretation='Count agreement only, not detection accuracy; misses and false positives can cancel.')
        self.path.with_suffix('.summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'[개수 기록] {self.path}')
        for name in ('deduplicated', 'assigned'):
            fraction = metrics[name]['equal_expected_fraction']
            if fraction is not None:
                print(f'[개수 기록] {name}: {self.expected}개와 일치한 프레임 {fraction:.1%}')
