"""Detect pieces in recorded videos with the position YOLO and group them into static tracks.

Writes <out>/<video_id>/tracks.json (per-track box per sampled frame) and a montage per
track for manual glyph labelling. Pieces are assumed static unless a box drifts > 0.5 box.
"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'janggi'))
from piece_detection.detector import deduplicate_detections, Detection
from piece_detection.crops import prepare_image

p = argparse.ArgumentParser()
p.add_argument('videos', nargs='+', type=Path)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--every', type=int, default=3)
p.add_argument('--conf', type=float, default=.25)
a = p.parse_args()
from ultralytics import YOLO
yolo = YOLO(str(ROOT/'models/weights_only_piece.pt'))
for video in a.videos:
    vid = video.parent.name
    out = a.out/vid; out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video)); frames = []; idx = 0
    while True:
        ok, bgr = cap.read()
        if not ok: break
        if idx % a.every == 0:
            r = yolo.predict(bgr, conf=a.conf, verbose=False, device=0)[0]
            dets = [Detection(cls_id=0, side='unknown', kind='piece', name='janggi_dol', conf=float(c), box=tuple(map(float, b)))
                    for b, c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy())]
            # Board area only (both recordings share the fixed camera); drops the case/robot.
            keep = [d for d in deduplicate_detections(dets) if 300 < (d.box[0]+d.box[2])/2 < 710 and 195 < (d.box[1]+d.box[3])/2 < 600]
            frames.append((idx, [(list(d.box), d.conf) for d in keep]))
        idx += 1
    # Tracks from the frame with the most detections, then nearest-centre association.
    ref = max(frames, key=lambda f: len(f[1]))
    centres = np.array([[(b[0]+b[2])/2, (b[1]+b[3])/2] for b, _ in ref[1]])
    tracks = [dict(id=i, observations=[]) for i in range(len(centres))]
    for fi, dets in frames:
        for box, conf in dets:
            c = np.array([(box[0]+box[2])/2, (box[1]+box[3])/2]); d = np.linalg.norm(centres-c, axis=1); j = int(d.argmin())
            if d[j] < .5*max(box[2]-box[0], box[3]-box[1]):
                tracks[j]['observations'].append(dict(frame=fi, box=box, conf=conf))
    json.dump(dict(video=str(video), every=a.every, frames=idx, sampled=len(frames), tracks=tracks), open(out/'tracks.json', 'w'))
    # Montage: one large crop per track from the reference frame, numbered, for labelling.
    cap = cv2.VideoCapture(str(video)); cap.set(cv2.CAP_PROP_POS_FRAMES, ref[0]); ok, bgr = cap.read()
    img = Image.fromarray(bgr[:, :, ::-1]); tiles = []
    order = sorted(range(len(tracks)), key=lambda i: (round(centres[i][0]/40), centres[i][1]))
    for i in order:
        crop = np.asarray(prepare_image(img, 160, box=ref[1][i][0]))[:, :, ::-1].copy()
        cv2.putText(crop, str(i), (4, 22), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0), 4); cv2.putText(crop, str(i), (4, 22), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 0), 2)
        tiles.append(crop)
    while len(tiles) % 8: tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[r:r+8]) for r in range(0, len(tiles), 8)])
    cv2.imwrite(str(out/'montage.jpg'), grid)
    vis = bgr.copy()
    for i, c in enumerate(centres): cv2.putText(vis, str(i), (int(c[0])-8, int(c[1])-14), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 2)
    cv2.imwrite(str(out/'reference.jpg'), vis)
    counts = [len(t['observations']) for t in tracks]
    print(vid, 'frames', idx, 'sampled', len(frames), 'tracks', len(tracks), 'obs min/max', min(counts), max(counts),
          'dets per frame min/max', min(len(f[1]) for f in frames), max(len(f[1]) for f in frames))
