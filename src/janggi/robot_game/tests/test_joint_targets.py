"""Reference interpolation must preserve endpoints and stop on stale feedback."""
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from joint_targets import JointTargetBuffer


def test_interpolation_never_extrapolates_or_changes_latest_goal():
    buffer=JointTargetBuffer(delay_sec=.025)
    assert buffer.sample(0)=={}
    buffer.add({'joint_1':1.,'joint_6':-2.},1.)
    buffer.add({'joint_1':2.,'joint_6':-1.},1.02)
    assert buffer.sample(1.035)==pytest.approx({'joint_1':1.5,'joint_6':-1.5})
    assert buffer.sample(.9)=={'joint_1':1.,'joint_6':-2.}
    assert buffer.sample(100.)=={'joint_1':2.,'joint_6':-1.}


def test_multiple_samples_advance_and_reject_invalid_or_old_samples():
    buffer=JointTargetBuffer(delay_sec=0.)
    values={'joint_1':0.}
    buffer.add(values,1.);values['joint_1']=99.
    buffer.add({'joint_1':math.nan},2.)
    buffer.add({'joint_1':4.},.5)
    assert buffer.sample(100.)=={'joint_1':0.}
    for i in range(1,20):buffer.add({'joint_1':float(i)},1.+i*.02)
    assert buffer.sample(1.21)==pytest.approx({'joint_1':10.5})
    assert buffer.sample(100.)=={'joint_1':19.}


def test_cubic_uses_measured_velocities_and_stays_bounded():
    buffer=JointTargetBuffer(0,cubic=True)
    for i in range(5):buffer.add({'q':i*.02},i*.02,{'q':1.})
    for at in [.025,.03,.035,.045,.05]:
        q,v=buffer.sample_state(at)
        assert q['q']==pytest.approx(at)
        assert v['q']==pytest.approx(1.)
    assert buffer.sample_state(1.)==({'q':.08},{'q':0.})
    buffer=JointTargetBuffer(0,cubic=True)
    buffer.add({'q':0.},0.,{'q':1000.})
    buffer.add({'q':1.},1.,{'q':-1000.})
    for i in range(101):assert 0<=buffer.sample(i/100)['q']<=1.


def test_feedback_gap_resumes_without_position_jump():
    buffer=JointTargetBuffer(.035,cubic=True,continuous=True)
    for i in range(6):buffer.add({'q':i*.02},i*.02,{'q':1.})
    buffer.sample(.13)
    old,_=buffer.sample_state(.2)
    buffer.add({'q':.22},.22,{'q':1.})
    new,v=buffer.sample_state(.205)
    assert 0<=new['q']-old['q']<=1.05*.005+1e-12
    assert v['q']<=1.05+1e-12


def test_stationary_tail_can_realign_but_never_discards_pending_motion():
    buffer=JointTargetBuffer(delay_sec=0.,max_samples=8192)
    buffer.add({'joint_1':0.},1.)
    buffer.add({'joint_1':1.},2.)
    buffer.add({'joint_1':1.},3.)
    assert not buffer.align_stationary_tail(10.,['joint_1'])
    buffer.sample(2.5)
    assert buffer.align_stationary_tail(10.,['joint_1'])
    assert buffer.sample(10.)=={'joint_1':1.}
    buffer.add({'joint_1':2.},10.02)
    assert buffer.sample(10.01)['joint_1']==pytest.approx(1.5)
