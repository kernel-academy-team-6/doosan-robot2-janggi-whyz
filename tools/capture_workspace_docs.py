#!/usr/bin/env python3
"""Capture current tools on a supplied display; never connect to cameras or robots.

Run with janggi/.venv/bin/python. Supply DISPLAY pointing at an isolated Xvfb.
Runtime state and the writable dataset catalog are temporary; original labels,
models and recorded evaluations are not changed. Each mode runs in a fresh process.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
from datetime import datetime
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def capture(root, path):
    from PIL import ImageGrab
    root.update_idletasks()
    root.update()
    x, y = root.winfo_rootx(), root.winfo_rooty()
    ImageGrab.grab(bbox=(x, y, x+root.winfo_width(), y+root.winfo_height())).save(path)
    print(f'Captured {path.name}: {root.winfo_width()}x{root.winfo_height()}', flush=True)

def source_frame():
    project = ROOT/'tools/datasets/janggi_pieces'
    with sqlite3.connect(f'file:{project}/catalog.sqlite?mode=ro', uri=True) as db:
        rel = db.execute("SELECT frame_path FROM samples WHERE status='confirmed' ORDER BY id LIMIT 1").fetchone()[0]
    return project/rel

def vision(mode, output, temp):
    import time
    import cv2
    import torch
    sys.path[:0] = [str(ROOT/'janggi'), str(ROOT/'janggi/realtime_ui')]
    import app
    from piece_detection import PieceDetector, BoardMapper
    from piece_detection.classifier import TwoStageDetector, MobileNetClassifier
    torch.set_num_threads(4)
    frame = source_frame()
    sys.argv = ['capture', '--model-mode', 'piece' if mode == 'piece' else 'hybrid',
                '--source', str(frame), '--device', 'cpu', '--state-dir', str(temp/'states'),
                '--show-piece-motion']
    args = app.parse_args()
    mapper = BoardMapper.from_calibration(args.calibration)
    detector = PieceDetector(args.model, conf=args.conf, device='cpu')
    if mode != 'piece':
        detector = TwoStageDetector(detector, MobileNetClassifier(args.classifier, 'cpu'), args.classifier_conf)
    source = app.CvSource(str(frame), args.width, args.height)
    worker = app.Worker(source, detector, mapper, args)
    ui = app.App(args, worker, mapper)
    ui.root.geometry('+0+0')
    errors = []
    def finish():
        try:
            if worker.error:
                raise RuntimeError(worker.error)
            capture(ui.root, output/f'{mode}.png')
            if mode != 'piece':
                with worker.lock:
                    states = [{ 'id': p.id, 'visible':p.visible, 'label':p.label,
                                'flags':list(p.flags), 'revival_state':p.revival_state}
                              for p in worker.tracker.pieces.values()]
                (output/f'{mode}_observations.json').write_text(json.dumps(states, ensure_ascii=False, indent=2))
        except Exception as exc:
            errors.append(str(exc))
        finally:
            ui.quit()
    if mode == 'obstruction':
        def obstruct():
            # Deliberate input perturbation; predictions and widgets remain real.
            height, width = source.image.shape[:2]
            cv2.rectangle(source.image, (int(width*.28), int(height*.32)),
                          (int(width*.56), int(height*.66)), (25,25,25), -1)
        ui.root.after(7500, obstruct)
        ui.root.after(8800, finish)
    else:
        ui.root.after(7500 if mode == 'hybrid' else 2500, finish)
    try:
        ui.run()
    finally:
        worker.stop()
        source.close()
    if errors: raise RuntimeError('; '.join(errors))

def dataset(output, temp):
    import tkinter as tk
    from types import SimpleNamespace
    sys.path.insert(0, str(ROOT/'tools'))
    from piece_dataset.app import DatasetApp
    original = ROOT/'tools/datasets/janggi_pieces'
    clone = temp/'dataset'; clone.mkdir()
    with sqlite3.connect(f'file:{original}/catalog.sqlite?mode=ro', uri=True) as src:
        with sqlite3.connect(clone/'catalog.sqlite') as dest:
            src.backup(dest)
    for name in ('frames','crops'):
        (clone/name).symlink_to(original/name, target_is_directory=True)
    args = SimpleNamespace(project=clone, interval=2., ratios=(70,15,15), min_per_class=(1,1,1),
                           split_mode='random', split_unit='group', seed=42, augment=5, size=224,
                           angle_step=.1, margin=0., model=ROOT/'models/weights_only_piece.pt', device='cpu')
    r = tk.Tk(); ui = DatasetApp(r,args);r.geometry('1460x1000+0+0')
    def show():
        ui.filter.set('confirmed');ui.refresh()
    def finish():
        capture(r,output/'dataset.png');ui.quit()
    r.after(400,show);r.after(1600,finish);r.mainloop()

def evaluation(output, temp):
    import tkinter as tk
    from types import SimpleNamespace
    sys.path.insert(0,str(ROOT/'tools'))
    from evaluation_compare import ComparisonViewer
    candidates=[]
    for path in sorted((ROOT/'janggi/realtime_ui/evaluations').glob('counts_*')):
        if all((path/n).exists() for n in ('counts.csv','board.mp4','overlay.mp4')):
            candidates.append(path)
    # Select readable logs with at least 60 corresponding frames.
    from evaluation_viewer import Session
    selected=[]
    for path in candidates:
        try:
            s=Session(path);n=s.length;s.close()
            if n>=60:selected.append(path)
        except Exception:continue
        if len(selected)==2:break
    if len(selected)<2:raise RuntimeError('Two recorded evaluation sessions are required')
    (output/'evaluation_inputs.json').write_text(json.dumps([str(p.relative_to(ROOT)) for p in selected],indent=2))
    args=SimpleNamespace(session=selected[0],add_session=selected[1:],video_count=4,axis='frame',
                         window=120.,mode='page',confidence='deduplicated',speed=1.)
    r=tk.Tk();ui=ComparisonViewer(r,args);r.geometry('1900x1120+0+0')
    def finish():
        ui.index=min(45,ui.session.length-1);ui.redraw()
        capture(r,output/'evaluation.png');ui.quit()
    r.after(1800,finish);r.mainloop()

def tcp(output, temp):
    sys.path[:0]=[str(ROOT/'src/janggi_pick_place2'),str(ROOT/'src/janggi_tcp_test')]
    from janggi_tcp_test.profile import Profile, CompensationPlan, endpoints
    from janggi_tcp_test.report import save_report
    profile=Profile(*endpoints(40.,60.))
    plan=CompensationPlan(profile,[400.,50.,450.,0.,180.,0.])
    rows=[]
    for i in range(201):
        t=profile.duration*i/200
        rows.append({'kind':'model','elapsed_sec':t,**plan.sample(t)})
    save_report(output/'tcp_model',rows,[],{'backend':'offline_model_only','source':'capture_workspace_docs.py',
        'initial_width_mm':40,'target_width_mm':60,'gripper_feedback_received':False,
        'arm_pose_received':False,'profile_settings_sent':False,
        'note':'Pure profile and geometry calculation; no ROS node or hardware was started.'})
    print('Generated actual report.py output, model only',flush=True)

def simulation(output, temp):
    result = subprocess.run(
        ['bash', str(ROOT/'src/janggi/robot_game/run_sim_app.sh'),
         '--turns', '2', '--seed', '1', '--no-motion',
         '--screenshot', str(output/'simulation.png')],
        cwd=ROOT, capture_output=True, text=True, timeout=60)
    (output/'simulation.log').write_text(result.stdout+result.stderr)
    print(result.stdout, end='')
    if result.returncode:
        raise RuntimeError(result.stderr or 'Simulation failed; see simulation.log')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',required=True,choices=['piece','hybrid','obstruction','dataset','evaluation','tcp','simulation'])
    p.add_argument('--output',type=Path,default=ROOT/'docs/assets/workspace_captures')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='janggi-doc-') as name:
        temp=Path(name)
        if a.mode in ('piece','hybrid','obstruction'):vision(a.mode,a.output,temp)
        else:globals()[a.mode](a.output,temp)
    manifest=a.output/'capture_manifest.json'
    data=json.loads(manifest.read_text()) if manifest.exists() else {}
    data[a.mode]={'captured_at':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
        'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'command':f'janggi/.venv/bin/python tools/capture_workspace_docs.py --mode {a.mode}',
        'display':'isolated Xvfb','robot_connected':False,'camera_connected':False}
    if a.mode in ('piece','hybrid','obstruction'):
        f=source_frame();data[a.mode].update(input=str(f.relative_to(ROOT)),input_sha256=digest(f),
            detector='models/weights_only_piece.pt',detector_sha256=digest(ROOT/'models/weights_only_piece.pt'),
            device='cpu',synthetic_obstruction=a.mode=='obstruction')
        if a.mode!='piece':
            data[a.mode].update(classifier='models/piece_classifier/best.pt',classifier_sha256=digest(ROOT/'models/piece_classifier/best.pt'))
    manifest.write_text(json.dumps(data,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
