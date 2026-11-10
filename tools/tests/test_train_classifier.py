import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

def training_modules():
    torch = pytest.importorskip('torch')
    pytest.importorskip('torchvision')
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import train_piece_classifier as training
    import piece_detection.classifier as classifiers
    return torch, training, classifiers


def test_classification_metrics_include_confusing_pair():
    _, training, _ = training_modules()
    report = training.metrics_from_confusion([[8, 2], [1, 9]], ['blue_cha', 'blue_sang'])
    assert report['accuracy'] == .85
    assert report['cho_cha_as_sang'] == 2 and report['cho_sang_as_cha'] == 1
    assert report['per_class']['blue_cha']['recall'] == .8


def test_model_checkpoint_batch_and_full_image_transform(tmp_path):
    torch, _, classifiers = training_modules()
    torch.set_num_threads(1)
    classes = ['blue_cha', 'blue_sang']
    path = tmp_path/'best.pt'
    model = classifiers.build_model(2).eval()
    torch.save(dict(architecture='mobilenet_v3_small', preprocessing=classifiers.PREPROCESSING, classes=classes,
                    image_size=64, state_dict=model.state_dict()), path)
    classifier = classifiers.MobileNetClassifier(path, 'cpu', batch_size=2)
    image = Image.new('RGB', (64, 64), 'white')
    predictions = classifier.predict([image]*3)
    assert len(predictions) == 3 and all(c in classes and 0 <= p <= 1 for c, p in predictions)
    assert classifier.predict([]) == []
    image = Image.new('RGB', (64, 64), 'white')
    image.putpixel((0, 0), (255, 0, 0))
    transformed = classifiers.input_transform(64)(image)
    assert transformed.shape == (3, 64, 64)
    assert transformed[1, 0, 0] < 0 and transformed[1, 1, 1] > 0  # 모서리를 잘라내지 않는다.
    checkpoint = torch.load(path, weights_only=True)
    checkpoint['preprocessing'] = 'old-padding'
    torch.save(checkpoint, path)
    with pytest.raises(ValueError, match='전처리'):
        classifiers.MobileNetClassifier(path)
