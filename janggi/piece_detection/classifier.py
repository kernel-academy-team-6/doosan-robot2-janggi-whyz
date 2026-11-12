"""교체 가능한 Crop 분류기와 위치 YOLO → 분류 어댑터."""
from pathlib import Path
from dataclasses import replace
from typing import Protocol

import numpy as np
from PIL import Image

from .crops import prepare_image
from .detector import deduplicate_detections
from .labels import SIDES, KINDS

DEFAULT_CLASSIFIER = Path(__file__).resolve().parents[2]/'models/piece_classifier_pieces2_224/best.pt'
PREPROCESSING = 'tight-square-frame-rgb-v1'
VALID_CLASSES = {f'{side}_{kind}' for side in SIDES for kind in KINDS}


class CropClassifier(Protocol):
    classes: list[str]
    size: int
    def predict(self, images: list[Image.Image]) -> list[tuple[str, float]]: ...
    def predict_proba(self, images: list[Image.Image]) -> list[list[float]]: ...


def input_transform(size=224):
    from torchvision import transforms
    # 이미 정사각형인 전체 입력만 축소한다. 기본 weights.transforms()의 CenterCrop을 사용하지 않는다.
    return transforms.Compose([transforms.Resize((size, size), interpolation=transforms.InterpolationMode.BICUBIC),
                               transforms.ToTensor(),
                               transforms.Normalize([.485, .456, .406], [.229, .224, .225])])


def build_model(classes, pretrained=False):
    import torch.nn as nn
    from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
    model = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT if pretrained else None)
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, classes)
    return model


class MobileNetClassifier:
    def __init__(self, path=DEFAULT_CLASSIFIER, device=None, batch_size=32, *, allowed_classes=None):
        import torch
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(f'분류 모델이 없습니다: {path}. 먼저 tools/train_piece_classifier.sh를 실행하세요.')
        checkpoint = torch.load(self.path, map_location='cpu', weights_only=True)
        if checkpoint.get('architecture') != 'mobilenet_v3_small' or checkpoint.get('preprocessing') != PREPROCESSING:
            raise ValueError('지원하지 않는 분류 모델/전처리입니다. 이 프로젝트의 학습 스크립트로 만든 best.pt를 사용하세요.')
        self.classes = checkpoint['classes']
        if (not isinstance(self.classes, list) or not self.classes or
                len(set(self.classes)) != len(self.classes) or not set(self.classes) <= (VALID_CLASSES if allowed_classes is None else set(allowed_classes))):
            raise ValueError('분류 모델의 클래스 목록이 잘못되었습니다.')
        self.size = int(checkpoint['image_size'])
        if not 32 <= self.size <= 1024 or batch_size < 1:
            raise ValueError('분류 입력 크기/배치 크기가 잘못되었습니다.')
        self.device = device or ('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.batch_size = batch_size
        self.model = build_model(len(self.classes))
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.to(self.device).eval()
        self.transform = input_transform(self.size)

    def predict_proba(self, images):
        import torch
        results = []
        with torch.inference_mode():
            for start in range(0, len(images), self.batch_size):
                batch = torch.stack([self.transform(image) for image in images[start:start+self.batch_size]]).to(self.device)
                results.extend(self.model(batch).softmax(1).cpu().tolist())
        return results

    def predict(self, images):
        return [(self.classes[int(np.argmax(p))], float(max(p))) for p in self.predict_proba(images)]


class UltralyticsClassifier:
    """같은 Crop/클래스 규약으로 학습한 Ultralytics Classification 가중치 어댑터."""
    def __init__(self, path, device=None, batch_size=32, size=224):
        import torch
        from ultralytics import YOLO
        from torchvision import transforms
        if not Path(path).is_file():
            raise FileNotFoundError(f'분류 모델이 없습니다: {path}')
        self.path, self.size, self.batch_size = Path(path), size, batch_size
        self.device = device or ('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(str(path))
        if self.model.task != 'classify':
            raise ValueError('Ultralytics 분류 가중치가 필요합니다. 탐지 가중치를 사용할 수 없습니다.')
        self.classes = [self.model.names[i].replace(' ', '_').replace('_solder', '_soldier')
                        for i in range(len(self.model.names))]
        if not self.classes or len(set(self.classes)) != len(self.classes) or not set(self.classes) <= VALID_CLASSES:
            raise ValueError('Ultralytics 분류 모델의 클래스가 장기말 라벨과 다릅니다.')
        # 해당 모델도 같은 전체 Crop 및 [0,1] RGB 전처리로 학습했어야 한다.
        self.model.model.transforms = transforms.Compose([
            transforms.Resize((size, size), interpolation=transforms.InterpolationMode.BICUBIC), transforms.ToTensor()])

    def predict_proba(self, images):
        results = []
        for start in range(0, len(images), self.batch_size):
            for result in self.model.predict(images[start:start+self.batch_size], imgsz=self.size,
                                             device=self.device, verbose=False):
                results.append(result.probs.data.cpu().tolist())
        return results

    def predict(self, images):
        return [(self.classes[int(np.argmax(p))], float(max(p))) for p in self.predict_proba(images)]


class TwoStageDetector:
    positions_deduplicated = True

    def __init__(self, detector, classifier: CropClassifier, classification_conf=.5):
        if not detector.position_only:
            raise ValueError('2단계 모드의 탐지 모델은 janggi_dol 1종 모델이어야 합니다.')
        if not 0 <= classification_conf <= 1:
            raise ValueError('분류 신뢰도 기준은 0~1입니다.')
        self.detector, self.classifier = detector, classifier
        self.classification_conf = classification_conf
        self.device, self.iou, self.imgsz = detector.device, detector.iou, detector.imgsz
        self.position_only = False
        self.robot_compatible = set(classifier.classes) == VALID_CLASSES
        self.names = classifier.classes

    def detect_positions(self, bgr, conf=None, roi=None):
        return [replace(d, side='unknown', kind='piece', name='janggi_dol', classification_conf=None)
                for d in deduplicate_detections(self.detector.detect(bgr, conf=conf, roi=roi))]

    def detect_tracking_positions(self, bgr, conf=None, roi=None, low_conf=.1):
        threshold = self.detector.conf if conf is None else conf
        return self.detect_positions(bgr, conf=min(threshold, low_conf), roi=roi)

    def classify_observations(self, bgr, detections):
        frame = Image.fromarray(np.ascontiguousarray(bgr[:, :, ::-1]))
        observations, crops, identities = {}, [], []
        for detection in detections:
            if detection.piece_id is None:
                continue
            pid = detection.piece_id
            observations[pid] = dict(box=list(detection.box), probabilities=None, piece_id=pid)
            try:
                crops.append(prepare_image(frame, self.classifier.size, box=detection.box))
                identities.append(pid)
            except ValueError:
                continue
        probabilities = self.classifier.predict_proba(crops) if crops else []
        if len(probabilities) != len(identities):
            raise ValueError('분류 확률 개수가 Crop 개수와 다릅니다.')
        for pid, distribution in zip(identities, probabilities):
            p = np.asarray(distribution, float)
            if p.shape != (len(self.classifier.classes),) or not np.isfinite(p).all() or (p < 0).any() or p.sum() <= 0:
                raise ValueError('잘못된 종류별 분류 확률입니다.')
            observations[pid]['probabilities'] = dict(zip(self.classifier.classes, (p/p.sum()).tolist()))
        return observations

    def detect(self, bgr, conf=None, roi=None):
        # 검출 좌표는 전체 프레임 좌표. Crop도 ROI가 아닌 전체 RGB 프레임에서 가져온다.
        detections = [replace(d, side='unknown', kind='piece', name='janggi_dol')
                      for d in deduplicate_detections(self.detector.detect(bgr, conf=conf, roi=roi))]
        if not detections:
            return []
        frame = Image.fromarray(np.ascontiguousarray(bgr[:, :, ::-1]))
        valid, crops = [], []
        for i, detection in enumerate(detections):
            try:
                crops.append(prepare_image(frame, self.classifier.size, box=detection.box))
                valid.append(i)
            except ValueError:
                # 원본 밖의 영역을 패딩해 학습 조건을 바꾸지 않는다. 위치만 표시한다.
                continue
        predictions = self.classifier.predict(crops) if crops else []
        if len(predictions) != len(valid):
            raise ValueError('분류 결과 개수가 Crop 개수와 다릅니다.')
        for i, (label, probability) in zip(valid, predictions):
            if label not in VALID_CLASSES or not np.isfinite(probability) or not 0 <= probability <= 1:
                raise ValueError('잘못된 분류 결과입니다.')
            detection = detections[i]
            detection.classification_conf = probability
            if probability >= self.classification_conf:
                detection.side, detection.kind = label.split('_', 1)
                detection.name = label
        return detections
