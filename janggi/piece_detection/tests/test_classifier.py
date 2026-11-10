import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from piece_detection.detector import Detection
from piece_detection.classifier import TwoStageDetector


class Detector:
    position_only, device, iou, imgsz = True, 'cpu', .5, 640

    def detect(self, frame, **kwargs):
        self.arguments = kwargs
        return [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, (20, 20, 40, 40)),
                Detection(0, 'janggi_dol', 'unknown', 'piece', .8, (21, 21, 41, 41)),
                Detection(0, 'janggi_dol', 'unknown', 'piece', .7, (60, 20, 80, 40))]


class Classifier:
    classes, size = ['blue_cha', 'red_sang'], 32

    def predict(self, images):
        self.images = images
        return [('blue_cha', .95), ('red_sang', .2)]


def test_two_stage_batch_rgb_dedup_and_uncertainty():
    detector, classifier = Detector(), Classifier()
    pipeline = TwoStageDetector(detector, classifier, .5)
    frame = np.zeros((100, 100, 3), np.uint8)
    frame[20:40, 20:40] = (10, 20, 200)
    result = pipeline.detect(frame, conf=.3, roi=(5, 5, 90, 90))
    assert len(result) == 2 and len(classifier.images) == 2
    assert np.array_equal(np.asarray(classifier.images[0])[16, 16], [200, 20, 10])
    assert result[0].side == 'blue' and result[0].kind == 'cha'
    assert result[0].box == (20, 20, 40, 40) and result[0].conf == .9
    assert result[0].classification_conf == .95
    assert result[1].side == 'unknown' and result[1].classification_conf == .2
    assert detector.arguments == dict(conf=.3, roi=(5, 5, 90, 90))
    assert not pipeline.robot_compatible


def test_two_stage_invalid_crop_remains_unknown():
    detector, classifier = Detector(), Classifier()
    detector.detect = lambda *a, **k: [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, (0, 0, 10, 30))]
    classifier.predict = lambda images: pytest.fail('원본 밖의 Crop을 분류하면 안 됨')
    result = TwoStageDetector(detector, classifier).detect(np.zeros((100, 100, 3), np.uint8))
    assert result[0].side == 'unknown' and result[0].classification_conf is None
    detector.position_only = False
    with pytest.raises(ValueError, match='1종'):
        TwoStageDetector(detector, classifier)


def test_two_stage_invalid_predictions_are_rejected():
    detector, classifier = Detector(), Classifier()
    classifier.predict = lambda images: [('unrelated', .9)]*len(images)
    with pytest.raises(ValueError, match='잘못된 분류'):
        TwoStageDetector(detector, classifier).detect(np.zeros((100, 100, 3), np.uint8))
    classifier.predict = lambda images: []
    with pytest.raises(ValueError, match='개수'):
        TwoStageDetector(detector, classifier).detect(np.zeros((100, 100, 3), np.uint8))
