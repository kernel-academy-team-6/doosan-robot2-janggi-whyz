"""Home is an explicit command independent of board recognition/game start."""
from collections import deque
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import app


@pytest.mark.parametrize('dry_run',[False,True])
def test_home_uses_pickplace2_j1_zero_without_reading_board_or_starting_game(dry_run):
    ui=app.App.__new__(app.App)
    ui.args=SimpleNamespace(no_motion=dry_run,
                            pick_place_config=Path(__file__).resolve().parents[3]/'src/janggi_pick_place2/config/pick_place.yaml')
    ui._msgs=deque()
    calls=[]
    ui.worker=SimpleNamespace(robot_motion=False,
                              fresh_state=lambda **kwargs:pytest.fail('home must not read board'))
    ui.rg=SimpleNamespace(picker=SimpleNamespace(go_observe=lambda *args:calls.append(args)),
                          cfg=SimpleNamespace(observe_vel=30.),
                          start=lambda *args:pytest.fail('home must not start game'))
    ui._ensure_robot=lambda:None
    jobs=[]
    def run(phase,job,**kwargs):
        jobs.append((phase,kwargs));job()
    ui._robot_job=run
    ui.robot_home()
    if dry_run:
        assert not calls and not ui.worker.robot_motion
    else:
        assert calls==[([0.,-13.694,134.216,0.,59.485,90.],30.,30.)]
        assert ui.worker.robot_motion
    assert jobs and '호밍' in jobs[0][1]['done_phase']


def test_home_respects_existing_busy_operation():
    ui=app.App.__new__(app.App)
    ui.args=SimpleNamespace(no_motion=False)
    ui.robot_busy=True
    ui._ensure_robot=lambda:pytest.fail('busy home must not create another controller')
    ui.robot_home()
