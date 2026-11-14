"""Unified sample index: (frame image, detector box, label, group) from labelled catalogs + video tracks.

Video frames sampled by extract_video_tracks.py are written once as JPEG (q=97).
"""
import json, sqlite3, sys
from pathlib import Path
import cv2
ROOT = Path(__file__).resolve().parents[2]; HERE = Path(__file__).resolve().parent
rows = []
for name, group in (('janggi_pieces', 'video_20261109'), ('janggi_pieces2', 'photos_20261112')):
    base = ROOT/'tools/datasets'/name
    con = sqlite3.connect(base/'catalog.sqlite')
    for frame, box, label in con.execute("select frame_path, box, label from samples where status='confirmed'"):
        rows.append(dict(frame=str(base/frame), box=json.loads(box), label=label, group=group, track=None))
labels = json.load(open(HERE/'labels.json'))
for vid, mapping in labels.items():
    if vid.startswith('_'): continue
    t = json.load(open(HERE/'videos'/vid/'tracks.json'))
    fdir = HERE/'videos'/vid/'frames'; fdir.mkdir(exist_ok=True)
    needed = {o['frame'] for tr in t['tracks'] if str(tr['id']) in mapping for o in tr['observations']}
    cap = cv2.VideoCapture(t['video']); i = 0
    while True:
        ok, bgr = cap.read()
        if not ok: break
        path = fdir/f'{i:06d}.jpg'
        if i in needed and not path.exists(): cv2.imwrite(str(path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 97])
        i += 1
    for tr in t['tracks']:
        label = mapping.get(str(tr['id']))
        if label is None: continue
        for o in tr['observations']:
            rows.append(dict(frame=str(fdir/f"{o['frame']:06d}.jpg"), box=o['box'], label=label, group=vid[:20], track=tr['id'], frame_index=o['frame']))
json.dump(rows, open(HERE/'index.json', 'w'))
from collections import Counter
print(Counter(r['group'] for r in rows)); print(len(rows), 'samples')
