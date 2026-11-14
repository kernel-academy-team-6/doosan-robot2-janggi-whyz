"""Camera recovery retries only observations, never completed pick/place commands."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2])]
from observations import SnapshotTimeout, TargetNotFound, stable_views
from manipulation import ManipulationCoordinator, ManipulationOptions, SeenPiece


def frame(t,x=100.,*,clear=True):
    return dict(stamp=t,clear=clear,objects=[dict(cell=(0,0),piece_id='p',pixel=(x,100.),piece=('blue','soldier'))])


def test_stability_requires_clear_stationary_fresh_window_and_retains_high_fps_frames():
    assert not stable_views([frame(1.),frame(1.1),frame(1.2)],3)
    assert len(stable_views([frame(1.+i/90) for i in range(40)],3))==3
    assert not stable_views([frame(1.),frame(1.2,clear=False),frame(1.4),frame(1.5)],3)
    assert not stable_views([frame(1.),frame(1.2,120),frame(1.4,130)],3)
    assert not stable_views([dict(stamp=1.,objects=[],clear=True)]*10,3)
    views=[frame(1.),frame(1.2),frame(1.4)]
    views[-1]['objects'][0]['piece']=('red','cha')
    assert len(stable_views(views,3))==3  # CNN label flicker is not physical motion.


def coordinator(check=lambda:None):
    events=[];notices=[]
    p=SimpleNamespace(set_piece_kind=lambda kind:None,pick_abs=lambda pt:events.append(('pick',pt[0])))
    g=SimpleNamespace(picker=p,log=lambda text:None,go_observe=lambda:events.append(('home',)),
                      on_manipulation_retry=notices.append,
                      geom=SimpleNamespace(board_to_base=lambda *dst:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(),check)
    c.orient=lambda *a:None
    return c,g,p,events,notices


def seen(x):
    return [[SeenPiece('own',(0,0),('blue','soldier'),np.array([x,0.,0.]),12.)]]*3


def test_frame_timeouts_rehome_and_retry_with_larger_observation_budget():
    c,g,p,events,notices=coordinator();timeouts=[]
    def scenes():
        timeouts.append(c._snapshot_timeout)
        if len(timeouts)<3:raise SnapshotTimeout('no stable frame')
        return seen(10.)
    c.scenes=scenes
    assert c.observe_home()[0][0].point[0]==10.
    assert timeouts==[3.,6.,9.]
    assert events==[('home',)]*3 and notices==['vision']*2


def test_persistent_frame_failure_is_bounded_and_never_sends_gripper_commands():
    c,g,p,events,notices=coordinator()
    def scenes():raise SnapshotTimeout('still missing')
    c.scenes=scenes
    with pytest.raises(SnapshotTimeout):c.observe_home()
    assert events==[('home',)]*3 and notices==['vision']*2


@pytest.mark.parametrize('stage',['home','camera','stop'])
def test_robot_and_fatal_camera_errors_and_user_stop_are_never_auto_retried(stage):
    c,g,p,events,notices=coordinator()
    def fail():raise RuntimeError('stop or robot failure')
    c.scenes=lambda:seen(0.)
    if stage=='home':g.go_observe=fail
    elif stage=='camera':c.scenes=fail
    else:c.check=fail
    with pytest.raises(RuntimeError):c.observe_home()
    assert not notices and len(events)<2


def test_ambiguous_target_reacquires_from_home_and_uses_new_center_once():
    c,g,p,events,notices=coordinator();c.scenes=lambda:seen(12.)
    c.pick(('blue','soldier'),np.zeros(3),'own',home_scenes=[[]]*3)
    assert events==[('home',),('pick',12.)] and notices==['vision']


def test_persistent_target_failure_never_uses_old_coordinates():
    c,g,p,events,notices=coordinator();c.scenes=lambda:[[]]*3
    with pytest.raises(TargetNotFound):c.pick(('blue','soldier'),np.zeros(3),home_scenes=[[]]*3)
    assert events==[('home',)]*2 and notices==['vision']*2


def test_post_place_video_failure_recovers_without_repeating_pick_or_place():
    c,g,p,events,notices=coordinator();calls=[]
    p.place=lambda *dst:events.append(('place',dst))
    def scenes():
        calls.append(True)
        if len(calls)==2:raise SnapshotTimeout('post-place timeout')
        return seen(0. if len(calls)==1 else 50.)
    c.scenes=scenes
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert len([e for e in events if e[0]=='pick'])==1
    assert len([e for e in events if e[0]=='place'])==1
    assert len([e for e in events if e[0]=='home'])==3
    assert notices==['vision'] and g._execution_observed


def test_missing_plan_center_rehomes_and_replans_before_any_pick(monkeypatch):
    from session import RobotGame,RobotGameConfig,RobotPlan
    g=RobotGame.__new__(RobotGame);g.cfg=RobotGameConfig();g.picker=object();events=[]
    g.log=lambda *a:None;g.on_manipulation_retry=lambda kind:events.append(kind)
    g.vision=SimpleNamespace(fresh_state=lambda:({}, {'fresh':(12,34)}))
    monkeypatch.setattr(ManipulationCoordinator,'observe_home',lambda self:events.append('home'))
    def plan(centers):
        if not centers:raise TargetNotFound('center unavailable')
        events.append(centers);return RobotPlan('a1a2')
    g.plan=plan
    assert g.plan_with_recovery({},lambda:False).uci=='a1a2'
    assert events==['vision','home',{'fresh':(12,34)}]
