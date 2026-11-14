"""Average a MobileNet checkpoint's probabilities per static track (for labelling help / baselines)."""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'janggi'))
from piece_detection.classifier import MobileNetClassifier
from piece_detection.crops import prepare_image
p = argparse.ArgumentParser(); p.add_argument('track_dir', type=Path); p.add_argument('--model', type=Path, required=True); p.add_argument('--every', type=int, default=10)
a = p.parse_args()
t = json.load(open(a.track_dir/'tracks.json')); clf = MobileNetClassifier(a.model, 'cuda:0')
cap = cv2.VideoCapture(t['video']); frames = {}; idx = 0
wanted = {o['frame'] for tr in t['tracks'] for o in tr['observations'] if (o['frame']//t['every']) % a.every == 0}
while True:
    ok, bgr = cap.read()
    if not ok: break
    if idx in wanted: frames[idx] = Image.fromarray(bgr[:, :, ::-1].copy())
    idx += 1
for tr in t['tracks']:
    obs = [o for o in tr['observations'] if o['frame'] in frames]
    if not obs: continue
    probs = np.array(clf.predict_proba([prepare_image(frames[o['frame']], clf.size, box=o['box']) for o in obs]))
    m = probs.mean(0); top = np.argsort(-m)[:2]; per = (probs.argmax(1) == top[0]).mean()
    c = np.mean([[(o['box'][0]+o['box'][2])/2, (o['box'][1]+o['box'][3])/2] for o in obs], 0)
    print(f"track {tr['id']:2d} @({c[0]:4.0f},{c[1]:4.0f}) n={len(obs):3d}  {clf.classes[top[0]]:13s} {m[top[0]]:.2f} | {clf.classes[top[1]]:13s} {m[top[1]]:.2f}  frames-agree {per:.2f}")
