import csv
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from count_log import CountLog, confidence_stats
from app import Worker
from piece_detection.detector import Detection


def test_count_agreement_summary_and_missing_mapping(tmp_path):
    log = CountLog(tmp_path, 32, {'source': 'example.mp4'})
    log.record(33, 32, 32, 32, .35, True, 0, 0., 0)
    log.record(31, 31, 30, 31, .35, True, 1, 1/30, 0)
    log.record(33, 33, None, None, .4, False, 0, 0., 1)
    log.close()
    log.close()  # 종료 처리가 중복돼도 요약을 덮어쓰지 않는다.
    with log.path.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert rows[2]['assigned_equals_expected'] == ''
    assert rows[2]['source_loop'] == '1'
    summary = json.loads(log.path.with_suffix('.summary.json').read_text())
    assert summary['metrics']['deduplicated']['equal_expected_fraction'] == 1/3
    assert summary['metrics']['assigned']['equal_expected_fraction'] == .5
    assert summary['metrics']['assigned']['samples'] == 2
    assert summary['metrics']['deduplicated']['below'] == 1
    assert summary['metrics']['deduplicated']['above'] == 1
    assert summary['confidence_thresholds'] == [.35, .4]


def test_worker_records_each_inference_without_ui_polling(tmp_path, monkeypatch):
    import app
    monkeypatch.setattr(app, 'HERE', tmp_path)
    model = tmp_path / 'model.pt'
    model.write_bytes(b'test model')

    class Source:
        source_frame = source_time = None
        source_loop = 0

        def read(self):
            i = 0 if self.source_frame is None else self.source_frame + 1
            if i >= 3:
                time.sleep(.01)
                return None
            self.source_frame, self.source_time = i, i / 30
            return np.full((5, 5, 3), i, np.uint8)

    class Detector:
        iou, imgsz = .5, 640

        def detect(self, frame, **kwargs):
            count = [2, 1, 3][int(frame[0, 0, 0])]
            return [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, (i*20, 0, i*20+10, 10))
                    for i in range(count)]

    args = SimpleNamespace(window=8, full_frame=False, log_counts=True, expected_pieces=2,
                           source='example.mp4', model=model, calibration=None,
                           playback_fps=0, max_dist=.45, conf=.35)
    worker = Worker(Source(), Detector(), None, args)
    try:
        deadline = time.monotonic() + 2
        while worker.frames < 3 and worker.error is None and time.monotonic() < deadline:
            time.sleep(.01)
    finally:
        worker.stop()
    assert worker.error is None
    with worker.count_log.path.open() as f:
        rows = list(csv.DictReader(f))
    assert [int(r['detected']) for r in rows] == [2, 1, 3]
    assert [int(r['source_frame']) for r in rows] == [0, 1, 2]
    assert all(float(r['conf_mean']) == pytest.approx(.9) for r in rows)
    assert all(float(r['dedup_conf_min']) == pytest.approx(.9) for r in rows)
    summary = json.loads(worker.count_log.path.with_suffix('.summary.json').read_text())
    assert summary['processed_frames'] == 3
    assert summary['metrics']['deduplicated']['equal_expected_fraction'] == 1/3
    video = summary['metadata']['videos']
    assert video['frames'] == len(rows)
    assert {p.name for p in worker.count_log.path.parent.iterdir()} == {
        'counts.csv', 'counts.summary.json', 'overlay.mp4', 'board.mp4'}
    import cv2
    for path in video['files'].values():
        capture = cv2.VideoCapture(path)
        assert capture.isOpened() and int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == len(rows)
        capture.release()


def test_confidence_lower_tail_and_percentile_are_distinct():
    values = [i / 10 for i in range(1, 11)]
    stats = confidence_stats(values)
    assert stats['mean'] == pytest.approx(.55)
    assert stats['min'] == .1
    assert stats['p30'] == pytest.approx(.37)
    assert stats['bottom30_count'] == 3
    assert stats['bottom30_mean'] == pytest.approx(.2)
    assert confidence_stats([.9] * 32)['bottom30_count'] == 10
    assert confidence_stats([.8])['bottom30_mean'] == .8
    assert confidence_stats([]) == dict(count=0, mean=None, min=None, p30=None,
                                        bottom30_count=0, bottom30_mean=None)


def test_confidence_csv_and_weighted_summary_with_empty_frame(tmp_path):
    log = CountLog(tmp_path, 32, {})
    log.record(4, 2, None, None, .1, False,
               confidences=[.4, .6, .8, 1.], deduplicated_confidences=[.8, 1.])
    log.record(1, 1, None, None, .1, False, confidences=[.2], deduplicated_confidences=[.2])
    log.record(0, 0, None, None, .1, False, confidences=[], deduplicated_confidences=[])
    log.close()
    with log.path.open() as f:
        rows = list(csv.DictReader(f))
    assert float(rows[0]['conf_mean']) == pytest.approx(.7)
    assert float(rows[0]['conf_bottom30_mean']) == pytest.approx(.5)
    assert float(rows[0]['dedup_conf_mean']) == pytest.approx(.9)
    assert rows[2]['conf_mean'] == rows[2]['conf_min'] == rows[2]['conf_p30'] == ''
    assert rows[2]['conf_count'] == '0'
    summary = json.loads(log.path.with_suffix('.summary.json').read_text())
    raw = summary['confidence']['raw']
    assert raw['mean'] == pytest.approx(.6)  # 전체 박스 5개의 평균; 프레임 평균의 평균과 구분
    assert raw['min'] == .2
    assert raw['frames_with_detections'] == 2 and raw['detection_samples'] == 5
    assert raw['mean_frame_p30'] == pytest.approx(.39)
    assert raw['mean_frame_bottom30_mean'] == pytest.approx(.35)
    assert summary['confidence']['deduplicated']['mean'] == pytest.approx(2 / 3)
