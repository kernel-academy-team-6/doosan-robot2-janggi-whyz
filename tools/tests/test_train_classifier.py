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


def test_kind_only_preserves_splits_and_merges_side_labels(tmp_path):
    import json
    _,training,_=training_modules()
    classes=['blue_cha','blue_sang','red_cha','red_sang']
    (tmp_path/'dataset.json').write_text(json.dumps(dict(schema_version=3,
        preprocessing='tight square frame',classes=classes)))
    manifest=[]
    for si,split in enumerate(['train','val','test']):
        for ci,label in enumerate(classes):
            folder=tmp_path/split/label;folder.mkdir(parents=True)
            Image.new('RGB',(32,32),(20+si*20,30+ci*20,100)).save(folder/'sample.png')
            manifest.append(json.dumps(dict(sample_id=f'{si}-{ci}',split=split)))
    (tmp_path/'manifest.jsonl').write_text('\n'.join(manifest))
    datasets,info=training.load_datasets(tmp_path,64,['cha','sang'])
    assert info['classes']==['cha','sang'] and info['source_classes']==classes
    for split,dataset in datasets.items():
        assert dataset.classes==['cha','sang'] and len(dataset)==4
        assert dataset.targets==[0,1,0,1]
        assert all(Path(file).relative_to(tmp_path).parts[0]==split for file,_ in dataset.samples)


def test_two_kind_checkpoint_requires_explicit_expert_loader(tmp_path):
    torch,_,classifiers=training_modules()
    path=tmp_path/'expert.pt'
    torch.save(dict(architecture='mobilenet_v3_small',preprocessing=classifiers.PREPROCESSING,
                    classes=['cha','sang'],image_size=64,
                    state_dict=classifiers.build_model(2).state_dict()),path)
    with pytest.raises(ValueError,match='클래스'):
        classifiers.MobileNetClassifier(path,'cpu')
    expert=classifiers.MobileNetClassifier(path,'cpu',allowed_classes={'cha','sang'})
    assert expert.classes==['cha','sang']
