"""Frame-level accuracy of a classifier on index.json groups (default: the two 2026-11-13 videos).

--model: a MobileNet checkpoint (UI-compatible) or a hierarchical bundle json (see hierarchical.py).
"""
import argparse, json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]; HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'janggi'), str(HERE)]
from piece_detection.crops import prepare_image

def load(path, device):
    if str(path).endswith('.json'):
        from hierarchical import HierarchicalClassifier
        return HierarchicalClassifier.from_bundle(path, device)
    from piece_detection.classifier import MobileNetClassifier
    return MobileNetClassifier(path, device)

def run(model, rows, batch=256):
    frames = {}; preds = []
    for start in range(0, len(rows), batch):
        part = rows[start:start+batch]; crops = []
        for r in part:
            if r['frame'] not in frames:
                if len(frames) > 64: frames.clear()
                frames[r['frame']] = Image.open(r['frame']).convert('RGB')
            crops.append(prepare_image(frames[r['frame']], model.size, box=r['box']))
        preds += [model.classes[int(np.argmax(p))] for p in model.predict_proba(crops)]
    return preds

def report(name, rows, preds):
    ok = [p == r['label'] for p, r in zip(preds, rows)]
    side = [p.split('_')[0] == r['label'].split('_')[0] for p, r in zip(preds, rows)]
    kind = [p.split('_', 1)[1] == r['label'].split('_', 1)[1] for p, r in zip(preds, rows)]
    errs = Counter((r['label'], p) for p, r in zip(preds, rows) if p != r['label'])
    per_track = defaultdict(list)
    for p, r in zip(preds, rows): per_track[(r['label'], r.get('track'))].append(p == r['label'])
    worst = sorted(((np.mean(v), k) for k, v in per_track.items()))[:4]
    print(f"  {name:22s} n={len(rows):5d} acc {np.mean(ok)*100:6.2f}%  side {np.mean(side)*100:6.2f}%  kind {np.mean(kind)*100:6.2f}%"
          f"  errors {sum(not o for o in ok)}  top {errs.most_common(4)}  worst-tracks {[(k[0], k[1], round(a, 2)) for a, k in worst if a < 1]}")
    return dict(n=len(rows), accuracy=float(np.mean(ok)), side=float(np.mean(side)), kind=float(np.mean(kind)),
                errors=[[a, b, n] for (a, b), n in errs.most_common()])

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('models', nargs='+', type=Path)
    p.add_argument('--groups', nargs='+', default=['video_20261113_23043', 'video_20261113_23055'])
    p.add_argument('--every', type=int, default=1, help='use every Nth sample per group')
    p.add_argument('--device', default='cuda:0'); p.add_argument('--json', type=Path)
    a = p.parse_args(); index = json.load(open(HERE/'index.json')); out = {}
    for m in a.models:
        model = load(m, a.device); print(m); out[str(m)] = {}
        for g in a.groups:
            rows = [r for r in index if r['group'] == g][::a.every]
            out[str(m)][g] = report(g, rows, run(model, rows))
    if a.json: json.dump(out, open(a.json, 'w'), indent=1)
