from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import App, Worker
from manipulation import ManipulationOptions


def test_manipulation_observations_use_only_new_frames_and_preserve_unassigned_objects():
    worker=Worker.__new__(Worker);worker.lock=threading.Lock();worker.error=None
    worker._robot_views=deque([dict(stamp=0,objects=[])],maxlen=12)
    added=[]
    def check():
        if not added:
            added.append(True)
            for _ in range(3):
                worker._robot_views.append(dict(stamp=time.perf_counter(),objects=[
                    dict(piece_id='p1',cell=None,piece=('unknown','piece'),pixel=(100,200),confidence=.9)]))
    result=worker.robot_snapshot(check=check,settle_sec=0.)
    assert len(result)==3 and all(v['objects'][0]['cell'] is None for v in result)
    result[0]['objects'][0]['piece_id']='changed'
    assert worker._robot_views[1]['objects'][0]['piece_id']=='p1'


def test_fresh_camera_wait_respects_cancellation_and_timeout():
    worker=Worker.__new__(Worker);worker.lock=threading.Lock();worker.error=None;worker._robot_views=deque()
    def cancelled():raise RuntimeError('cancelled')
    with pytest.raises(RuntimeError,match='cancelled'):worker.robot_snapshot(check=cancelled)
    with pytest.raises(RuntimeError,match='새 카메라'):worker.robot_snapshot(timeout=0.)


def test_retry_and_direction_checkboxes_update_next_job_and_refuse_mid_job_mutations():
    class Var:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    ui=App.__new__(App);ui.manipulation_options=ManipulationOptions();ui._msgs=deque();ui.robot_busy=False
    ui.rg=SimpleNamespace(cfg=SimpleNamespace(manipulation=None))
    ui.manipulation_vars={name:Var(False) for name in ('adaptive_grasp_enabled','refresh_before_pick','retry_move_enabled')}
    ui._set_manipulation_options()
    assert not ui.rg.cfg.manipulation.retry_move_enabled
    before=ui.manipulation_options;ui.robot_busy=True
    ui.manipulation_vars['retry_move_enabled'].set(True);ui._set_manipulation_options()
    assert ui.manipulation_options is before and not ui.manipulation_vars['retry_move_enabled'].get()


def test_cli_can_disable_each_manipulation_feature(monkeypatch):
    import app
    monkeypatch.setattr(sys,'argv',['app.py','--robot','--no-adaptive-grasp',
                                    '--no-refresh-before-pick','--no-auto-retry'])
    args=app.parse_args()
    assert args.adaptive_grasp is False and args.refresh_before_pick is False and args.auto_retry is False
