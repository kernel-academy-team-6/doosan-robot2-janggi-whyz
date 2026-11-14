"""Train a UI-compatible piece classifier (MobileNetV3-Small, same checkpoint format).

Differences from tools/train_piece_classifier.py (left unchanged):
- Crops are re-extracted from the original frame every time with the live
  preprocessing (prepare_image) plus augmentation: any rotation, box scale/shift
  jitter (detector box noise), brightness/contrast/saturation, blur, glare patch.
- Old labelled catalogs and the 2026-11-13 video tracks are mixed by group share
  and balanced per class, so 25k near-duplicate video frames do not swamp the
  older, more varied data.
- Optional --pair-classes trains a 2-class kind expert (e.g. cha sang) on both sides.

Usage (see README.md in this folder):
  janggi/.venv/bin/python tools/cnn2/train.py --output models/X --train-groups ... [--epochs 24]
"""
import argparse, json, math, os, random, sys, time
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter
ROOT = Path(__file__).resolve().parents[2]; HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'janggi'))
os.environ.setdefault('TORCH_HOME', str(ROOT/'tools/datasets/.torch'))
import torch, torch.nn as nn
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from piece_detection.crops import prepare_image
from piece_detection.classifier import build_model, input_transform, PREPROCESSING

CONTEXT = 2.2  # context square (x box side) kept in memory for rotation/jitter


def context_crop(frame, box):
    x1, y1, x2, y2 = box; cx, cy = (x1+x2)/2, (y1+y2)/2; half = max(x2-x1, y2-y1)*CONTEXT/2
    L, T = max(0, int(cx-half)), max(0, int(cy-half)); R, B = min(frame.width, int(math.ceil(cx+half))), min(frame.height, int(math.ceil(cy+half)))
    return frame.crop((L, T, R, B)), [x1-L, y1-T, x2-L, y2-T]


class Crops(Dataset):
    def __init__(self, rows, classes, size, augment, label_of):
        self.size, self.augment, self.tf = size, augment, input_transform(size)
        self.targets = [classes.index(label_of(r)) for r in rows]
        cache = {}; self.items = []
        for r in rows:
            if r['frame'] not in cache:
                if len(cache) > 32: cache.clear()
                cache[r['frame']] = Image.open(r['frame']).convert('RGB')
            img, box = context_crop(cache[r['frame']], r['box'])
            self.items.append((np.asarray(img).copy(), box))

    def __len__(self): return len(self.items)

    def __getitem__(self, i):
        arr, box = self.items[i]; img = Image.fromarray(arr)
        if not self.augment:
            return self.tf(prepare_image(img, self.size, box=box)), self.targets[i]
        x1, y1, x2, y2 = box; side = max(x2-x1, y2-y1); cx, cy = (x1+x2)/2, (y1+y2)/2
        for _ in range(6):
            s = side*random.uniform(.9, 1.14); dx, dy = (random.uniform(-.07, .07)*side for _ in range(2))
            b = (cx+dx-s/2, cy+dy-s/2, cx+dx+s/2, cy+dy+s/2)
            try:
                out = prepare_image(img, self.size, angle=random.uniform(0, 360), box=b); break
            except ValueError:
                continue
        else:
            out = prepare_image(img, self.size, box=box)
        out = ImageEnhance.Brightness(out).enhance(random.uniform(.65, 1.4))
        out = ImageEnhance.Contrast(out).enhance(random.uniform(.75, 1.25))
        out = ImageEnhance.Color(out).enhance(random.uniform(.7, 1.3))  # saturation; no hue shift (side = red vs green)
        if random.random() < .25: out = out.filter(ImageFilter.GaussianBlur(random.uniform(.3, 1.6)))
        if random.random() < .2:  # specular glare patch like the right side of the 11-13 recordings
            over = Image.new('L', out.size, 0); r = random.uniform(.25, .6)*self.size; c = [random.uniform(0, self.size) for _ in range(2)]
            ImageDraw.Draw(over).ellipse([c[0]-r, c[1]-r, c[0]+r, c[1]+r], fill=int(random.uniform(60, 150)))
            out = Image.composite(Image.new('RGB', out.size, (255, 255, 255)), out, over.filter(ImageFilter.GaussianBlur(r/3)))
        return self.tf(out), self.targets[i]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--train-groups', nargs='+', required=True)
    p.add_argument('--group-share', nargs='*', default=[], help='GROUP=SHARE, default equal per group')
    p.add_argument('--pair-classes', nargs='*', default=None, help='kind expert, e.g. cha sang')
    p.add_argument('--epochs', type=int, default=24); p.add_argument('--samples-per-epoch', type=int, default=8000)
    p.add_argument('--batch-size', type=int, default=64); p.add_argument('--lr', type=float, default=6e-4)
    p.add_argument('--size', type=int, default=224); p.add_argument('--workers', type=int, default=6)
    p.add_argument('--seed', type=int, default=0); p.add_argument('--device', default='cuda:0')
    a = p.parse_args()
    if a.output.exists() and any(a.output.iterdir()): sys.exit(f'{a.output} is not empty')
    a.output.mkdir(parents=True, exist_ok=True)
    random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed)
    index = [r for r in json.load(open(HERE/'index.json')) if r['group'] in a.train_groups]
    if a.pair_classes:
        classes = sorted(a.pair_classes); label_of = lambda r: r['label'].split('_', 1)[1]
        index = [r for r in index if label_of(r) in classes]
    else:
        classes = sorted({r['label'] for r in index}); label_of = lambda r: r['label']
    share = {g: 1. for g in a.train_groups}
    for item in a.group_share: g, v = item.split('='); share[g] = float(v)
    counts = {}
    for r in index: counts[(r['group'], label_of(r))] = counts.get((r['group'], label_of(r)), 0)+1
    weights = [share[r['group']]/counts[(r['group'], label_of(r))]/sum(1 for k in counts if k[0] == r['group']) for r in index]
    print('classes', classes, '| samples', len(index), '| per group', {g: sum(1 for r in index if r['group'] == g) for g in a.train_groups}, flush=True)
    data = Crops(index, classes, a.size, True, label_of)
    loader = DataLoader(data, batch_size=a.batch_size, num_workers=a.workers, drop_last=True, persistent_workers=True,
                        sampler=WeightedRandomSampler(weights, a.samples_per_epoch, replacement=True))
    model = build_model(len(classes), pretrained=True).to(a.device)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs*len(loader); warm = len(loader)
    sched = torch.optim.lr_scheduler.LambdaLR(opt, lambda s: min(1., (s+1)/warm)*.5*(1+math.cos(math.pi*min(1., s/steps))))
    crit = nn.CrossEntropyLoss(label_smoothing=.05); scaler = torch.amp.GradScaler()
    config = dict(vars(a), output=str(a.output), classes=classes, group_counts={f'{g}|{c}': n for (g, c), n in counts.items()},
                  augmentation='rotation 0-360, box scale .90-1.14 shift +-7%, brightness .65-1.4, contrast .75-1.25, saturation .7-1.3, blur p.25, glare p.2',
                  optimizer='AdamW wd1e-4, 1 epoch warmup + cosine, label smoothing .05, AMP', pretrained='IMAGENET1K_V1', architecture='mobilenet_v3_small')
    json.dump(config, open(a.output/'config.json', 'w'), indent=1, default=str)
    history = []; t0 = time.monotonic()
    for epoch in range(a.epochs):
        model.train(); tot = cor = n = 0
        for x, y in loader:
            x, y = x.to(a.device, non_blocking=True), y.to(a.device, non_blocking=True)
            with torch.autocast('cuda', dtype=torch.float16):
                logits = model(x); loss = crit(logits, y)
            opt.zero_grad(set_to_none=True); scaler.scale(loss).backward(); scaler.step(opt); scaler.update(); sched.step()
            tot += loss.item()*len(y); cor += int((logits.argmax(1) == y).sum()); n += len(y)
        history.append(dict(epoch=epoch+1, loss=tot/n, train_accuracy=cor/n, lr=sched.get_last_lr()[0], elapsed_s=round(time.monotonic()-t0)))
        print(f"[{epoch+1}/{a.epochs}] loss {tot/n:.4f} train acc {cor/n:.4f} ({time.monotonic()-t0:.0f}s)", flush=True)
    model.eval()
    checkpoint = dict(architecture='mobilenet_v3_small', classes=classes, image_size=a.size, preprocessing=PREPROCESSING,
                      state_dict={k: v.detach().cpu().clone() for k, v in model.state_dict().items()}, epoch=a.epochs, config=config)
    torch.save(checkpoint, a.output/'best.pt')  # fixed epoch count; no selection on held-out videos
    json.dump(history, open(a.output/'history.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
