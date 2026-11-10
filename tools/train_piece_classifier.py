#!/usr/bin/env python3
"""내보낸 장기말 데이터셋으로 MobileNetV3-Small 전이학습. 로봇/카메라 사용 없음."""
import argparse
import csv
import hashlib
import json
import os
import random
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision.datasets import ImageFolder

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'janggi'))
os.environ.setdefault('TORCH_HOME', str(ROOT/'tools/datasets/.torch'))
from piece_detection.classifier import build_model, input_transform, PREPROCESSING, VALID_CLASSES


def metrics_from_confusion(matrix, classes):
    matrix = np.asarray(matrix)
    support, predicted = matrix.sum(1), matrix.sum(0)
    tp = np.diag(matrix)
    recall = np.divide(tp, support, out=np.zeros(len(classes), float), where=support > 0)
    precision = np.divide(tp, predicted, out=np.zeros(len(classes), float), where=predicted > 0)
    f1 = np.divide(2*precision*recall, precision+recall, out=np.zeros(len(classes), float), where=precision+recall > 0)
    return dict(accuracy=float(tp.sum()/max(1, matrix.sum())), macro_f1=float(f1.mean()),
                balanced_accuracy=float(recall.mean()), samples=int(matrix.sum()),
                per_class={c: dict(precision=float(precision[i]), recall=float(recall[i]), f1=float(f1[i]),
                                   support=int(support[i])) for i, c in enumerate(classes)},
                confusion_matrix=matrix.tolist(), classes=classes,
                cho_cha_as_sang=int(matrix[classes.index('blue_cha'), classes.index('blue_sang')]) if {'blue_cha', 'blue_sang'} <= set(classes) else None,
                cho_sang_as_cha=int(matrix[classes.index('blue_sang'), classes.index('blue_cha')]) if {'blue_cha', 'blue_sang'} <= set(classes) else None)


def load_datasets(path, size):
    info = json.loads((path/'dataset.json').read_text())
    if info.get('schema_version', 0) < 2 or not info.get('preprocessing', '').startswith('tight square'):
        raise ValueError('원본 프레임에서 회전 추출한 새 데이터셋을 내보내 주세요. 기존 회색 패딩 데이터셋은 지원하지 않습니다.')
    classes = info['classes']
    if not classes or not set(classes) <= VALID_CLASSES or len(set(classes)) != len(classes):
        raise ValueError('데이터셋 클래스 목록이 잘못되었습니다.')
    datasets = {split: ImageFolder(path/split, transform=input_transform(size)) for split in ('train', 'val', 'test')}
    for split, dataset in datasets.items():
        if dataset.classes != classes:
            raise ValueError(f'{split} 클래스 순서/폴더가 dataset.json과 다릅니다.')
    # 알려진 동일 원본/동일 Crop이 분할에 걸쳐 있으면 학습하지 않는다.
    seen = {}
    manifest = path/'manifest.jsonl'
    if not manifest.is_file():
        raise ValueError('출처 검증을 위한 manifest.jsonl이 필요합니다.')
    for line in manifest.read_text().splitlines():
        row = json.loads(line)
        key = row['sample_id']
        if key in seen and seen[key] != row['split']:
            raise ValueError('같은 원본의 파생 이미지가 여러 split에 포함되어 있습니다.')
        seen[key] = row['split']
    hashes = {}
    for split, dataset in datasets.items():
        for filename, label in dataset.samples:
            digest = hashlib.sha256(Path(filename).read_bytes()).hexdigest()
            if digest in hashes and hashes[digest] != (split, label):
                raise ValueError('완전히 동일한 이미지가 다른 split 또는 다른 정답에 있습니다.')
            hashes[digest] = (split, label)
    return datasets, info


@torch.inference_mode()
def evaluate(model, loader, device, classes):
    model.eval()
    matrix = np.zeros((len(classes), len(classes)), dtype=np.int64)
    predictions = []
    total_loss = 0.
    for images, target in loader:
        logits = model(images.to(device))
        target = target.to(device)
        total_loss += nn.functional.cross_entropy(logits, target, reduction='sum').item()
        confidence, index = logits.softmax(1).max(1)
        for truth, guess, probability in zip(target.cpu().tolist(), index.cpu().tolist(), confidence.cpu().tolist()):
            matrix[truth, guess] += 1
            predictions.append((truth, guess, probability))
    result = metrics_from_confusion(matrix, classes)
    result['loss'] = total_loss/max(1, matrix.sum())
    return result, predictions


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def train(args):
    if args.epochs < 1 or not 0 <= args.freeze_epochs <= args.epochs or args.batch_size < 2 or args.workers < 0 or args.threads < 1 or not 32 <= args.size <= 1024:
        raise ValueError('epochs≥1, 0≤freeze-epochs≤epochs, batch-size≥2, workers≥0, threads≥1, size=32~1024이어야 합니다.')
    torch.set_num_threads(args.threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    device = ('cuda:0' if torch.cuda.is_available() else 'cpu') if args.device == 'auto' else args.device
    if device.startswith('cuda') and not torch.cuda.is_available():
        raise ValueError('현재 환경에서 CUDA GPU를 사용할 수 없습니다. --device cpu로 실행하세요.')
    data = args.dataset.resolve()
    datasets, info = load_datasets(data, args.size)
    classes = info['classes']
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f'결과 폴더가 비어 있지 않습니다: {output}. --output으로 새 폴더를 선택하세요.')
    print(f'[학습] {device} · 클래스 {len(classes)} · '+str({s: len(d) for s, d in datasets.items()}), flush=True)
    if info.get('split', {}).get('unit') == 'sample':
        print('[평가 조건] 사진별 분배 데이터입니다. 같은 촬영의 유사 사진으로 인해 독립 촬영 성능보다 높게 나올 수 있습니다.', flush=True)
    # 다운로드 실패 시 임의 초기화로 조용히 대체하지 않는다.
    model = build_model(len(classes), pretrained=True).to(device)
    output.mkdir(parents=True, exist_ok=True)
    generator = torch.Generator().manual_seed(args.seed)
    loaders = {s: DataLoader(d, batch_size=args.batch_size, shuffle=s == 'train', num_workers=args.workers,
                            generator=generator if s == 'train' else None, pin_memory=device.startswith('cuda'))
               for s, d in datasets.items()}
    counts = np.bincount(datasets['train'].targets, minlength=len(classes))
    weight = torch.tensor(counts.sum()/(len(classes)*counts), dtype=torch.float32, device=device)
    criterion = nn.CrossEntropyLoss(weight=weight)
    config = dict(architecture='mobilenet_v3_small', pretrained='IMAGENET1K_V1', preprocessing=PREPROCESSING,
                  image_size=args.size, classes=classes, dataset=str(data), dataset_sha256=hashlib.sha256((data/'dataset.json').read_bytes()).hexdigest(),
                  dataset_manifest_sha256=hashlib.sha256((data/'manifest.jsonl').read_bytes()).hexdigest(),
                  dataset_split=info.get('split'), epochs=args.epochs, freeze_epochs=args.freeze_epochs, seed=args.seed,
                  batch_size=args.batch_size, device=device, lr_head=.001, lr_backbone=.0001,
                  torch_version=str(torch.__version__), torchvision_version=str(__import__('torchvision').__version__),
                  augmentation='exported images only; no random crop, flips or additional rotation',
                  class_weight=weight.cpu().tolist())
    write_json(output/'config.json', config)
    optimizer = None
    best, history, start = -1., [], time.monotonic()
    for epoch in range(args.epochs):
        frozen = epoch < args.freeze_epochs
        if epoch == 0 or epoch == args.freeze_epochs:
            for parameter in model.features.parameters():
                parameter.requires_grad = not frozen
            optimizer = torch.optim.AdamW([
                {'params': model.features.parameters(), 'lr': .0001},
                {'params': model.classifier.parameters(), 'lr': .001}], weight_decay=.0001)
        model.train()
        # 작은 배치에서도 사전학습 BN 통계를 보존하고 마지막 1장 배치 오류를 방지한다.
        for module in model.modules():
            if isinstance(module, nn.modules.batchnorm._BatchNorm):
                module.eval()
        total_loss, correct, total = 0., 0, 0
        for images, target in loaders['train']:
            images, target = images.to(device), target.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(images)
            loss = criterion(logits, target)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()*len(target)
            correct += int((logits.argmax(1) == target).sum())
            total += len(target)
        validation, _ = evaluate(model, loaders['val'], device, classes)
        row = dict(epoch=epoch+1, stage='head' if frozen else 'fine_tune', train_loss=total_loss/total,
                   train_accuracy=correct/total, val_loss=validation['loss'], val_accuracy=validation['accuracy'],
                   val_macro_f1=validation['macro_f1'], elapsed_s=time.monotonic()-start)
        history.append(row)
        print(f'[epoch {epoch+1}/{args.epochs}] {row["stage"]} train={correct/total:.3f} val={validation["accuracy"]:.3f} macroF1={validation["macro_f1"]:.3f}', flush=True)
        checkpoint = dict(architecture='mobilenet_v3_small', classes=classes, image_size=args.size,
                          preprocessing=PREPROCESSING, state_dict={k: v.detach().cpu().clone() for k, v in model.state_dict().items()},
                          epoch=epoch+1, val_macro_f1=validation['macro_f1'], config=config)
        torch.save(checkpoint, output/'last.pt')
        if validation['macro_f1'] > best:
            best = validation['macro_f1']
            torch.save(checkpoint, output/'best.pt')
        with (output/'history.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            writer.writerows(history)
    model.load_state_dict(torch.load(output/'best.pt', map_location=device, weights_only=True)['state_dict'])
    for split in ('val', 'test'):
        report, predictions = evaluate(model, loaders[split], device, classes)
        write_json(output/f'{split}_metrics.json', report)
        with (output/f'{split}_confusion.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['true / predicted', *classes])
            writer.writerows([label, *row] for label, row in zip(classes, report['confusion_matrix']))
        with (output/f'{split}_predictions.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['file', 'truth', 'prediction', 'confidence', 'correct'])
            for (filename, _), (truth, guess, probability) in zip(datasets[split].samples, predictions):
                writer.writerow([str(Path(filename).relative_to(data)), classes[truth], classes[guess], probability, truth == guess])
        print(f'[{split}] accuracy={report["accuracy"]:.3f} macroF1={report["macro_f1"]:.3f}', flush=True)
    print(f'[완료] {output}/best.pt', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path, help='train/val/test와 dataset.json이 있는 내보내기 폴더')
    parser.add_argument('--output', type=Path, default=ROOT/'models/piece_classifier')
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--freeze-epochs', type=int, default=5)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--device', default='auto')
    parser.add_argument('--workers', type=int, default=0)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--size', type=int, default=224)
    parser.add_argument('--seed', type=int, default=42)
    train(parser.parse_args())


if __name__ == '__main__':
    main()
