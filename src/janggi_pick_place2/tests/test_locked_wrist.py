"""Fixed-J6 planning and exact outgoing spline requests, without a robot connection."""
from types import SimpleNamespace
import math
import pytest

from janggi_pick_place2.executor import Settings
from janggi_pick_place2.locked_wrist import LockedWristPlanner, line_distance
from janggi_pick_place2.robot_client import RobotClient


def inverse(pose,solution,check):
    check();x,y,z,yaw,_,_=pose
    return [x/10,y/10,z/10,0.,0.,x/10-yaw+90.]


def forward(q,check):
    check()
    return [q[0]*10,q[1]*10,q[2]*10,q[0]-(q[5]-90),180.,0.]


START=[200.,0.,200.,20.,180.,0.]
JOINTS=[20.,0.,20.,0.,0.,90.]


@pytest.mark.parametrize('field', ['angular_velocity_deg_s', 'angular_acceleration_deg_s2',
                                 'joint_velocity_deg_s', 'joint_acceleration_deg_s2',
                                 'linear_velocity_mm_s', 'linear_acceleration_mm_s2'])
def test_each_motion_limit_changes_fixed_wrist_spline_duration(field):
    from dataclasses import replace
    settings=Settings(joint6_lock_enabled=True)
    target=[240,30,200,0,180,0]
    ordinary=LockedWristPlanner(settings,inverse,forward).plan(START,JOINTS,target,0,lambda:None)
    slower=LockedWristPlanner(replace(settings,**{field:.01}),inverse,forward).plan(
        START,JOINTS,target,0,lambda:None)
    assert slower.duration > ordinary.duration
    assert slower.joints == ordinary.joints


def test_plan_keeps_every_joint6_at_90_and_verifies_cartesian_interiors():
    path=LockedWristPlanner(Settings(joint6_lock_enabled=True),inverse,forward).plan(START,JOINTS,[240,30,200,0,180,0],0,lambda:None)
    assert len(path.joints)>=2 and path.duration>=2*50/Settings(joint6_lock_enabled=True).linear_velocity_mm_s
    assert all(q[5]==90. for q in path.joints)
    assert path.target_pose[:3]==pytest.approx([240,30,200])
    assert path.target_pose[3]!=START[3]  # Free task yaw; fixed J6 is not fixed world yaw.
    for q in path.joints:
        assert line_distance(forward(q,lambda:None)[:3],START[:3],[240,30,200])<1e-6


def test_planner_refuses_wrong_start_angle_and_too_many_waypoints():
    planner=LockedWristPlanner(Settings(joint6_lock_enabled=True),inverse,forward)
    with pytest.raises(ValueError,match='home button'):
        planner.plan(START,[*JOINTS[:5],0.],[240,0,200,0,180,0],0,lambda:None)
    with pytest.raises(ValueError,match='100 waypoints'):
        planner.plan(START,JOINTS,[800,0,200,0,180,0],0,lambda:None)


def test_fk_failure_or_unsolvable_yaw_prevents_plan():
    def bad_fk(q,check):
        p=forward(q,check);p[0]+=2;return p
    with pytest.raises(ValueError,match='FK did not verify'):
        LockedWristPlanner(Settings(joint6_lock_enabled=True),inverse,bad_fk).plan(START,JOINTS,[210,0,200,0,180,0],0,lambda:None)
    with pytest.raises(ValueError,match='cannot solve free yaw'):
        LockedWristPlanner(Settings(joint6_lock_enabled=True),lambda *args:[20,0,20,0,0,80],forward).plan(START,JOINTS,[210,0,200,0,180,0],0,lambda:None)


@pytest.mark.parametrize('j6_delta',[0.,2.5,3.,3.1,-3.1])
def test_robot_sends_one_fixed_joint_spline_and_stops_on_lock_deviation(j6_delta):
    bad_feedback=abs(j6_delta)>3.
    robot=RobotClient.__new__(RobotClient)
    robot.settings=Settings(joint6_lock_enabled=True);robot.motion_timeout=60.
    state={'pose':START.copy(),'joints':JOINTS.copy()};requests=[];stopped=[]
    robot.current_pose=lambda check:state['pose'].copy()
    robot.current_joints=lambda check:state['joints'].copy()
    robot._solution=lambda check:0
    robot._inverse=inverse;robot._forward=forward;robot.stop=lambda:stopped.append(True)
    def call(kind,path,req,check):
        assert path=='/motion/move_spline_joint'
        requests.append(req)
        state['joints']=list(req.pos[-1].data)
        state['pose']=forward(state['joints'],check)
        state['joints'][5]+=j6_delta
        return SimpleNamespace(success=True)
    robot.io=SimpleNamespace(call=call)
    def wait(reached,check,monitor=None):
        monitor();assert reached()
    robot._wait=wait
    target=[240,30,200,0,180,0]
    robot.prepare_locked_path([target],lambda:None)
    robot._inverse=lambda *args:pytest.fail('preflight path should be reused')
    if bad_feedback:
        with pytest.raises(RuntimeError,match='stop requested'):
            robot.move(target,lambda:None)
        assert stopped==[True]
    else:
        robot.move(target,lambda:None)
        assert not stopped
    assert len(requests)==1
    req=requests[0]
    assert 2<=req.pos_cnt<=100 and req.sync_type==1 and req.mode==0 and req.time>0
    assert all(list(point.data)[5]==90. for point in req.pos)


def test_linked_limits_change_pickplace_duration_including_vertical_paths():
    from dataclasses import replace
    settings=Settings(joint6_lock_enabled=True)
    faster=replace(settings,linear_velocity_mm_s=settings.linear_velocity_mm_s*2,
                   linear_acceleration_mm_s2=settings.linear_acceleration_mm_s2*2,
                   angular_velocity_deg_s=settings.angular_velocity_deg_s*2,
                   angular_acceleration_deg_s2=settings.angular_acceleration_deg_s2*2,
                   joint_velocity_deg_s=settings.joint_velocity_deg_s*2,
                   joint_acceleration_deg_s2=settings.joint_acceleration_deg_s2*2)
    for target in ([240,30,200,0,180,0], [200,0,140,0,180,0], [200,0,260,0,180,0]):
        slow=LockedWristPlanner(settings,inverse,forward).plan(START,JOINTS,target,0,lambda:None)
        fast=LockedWristPlanner(faster,inverse,forward).plan(START,JOINTS,target,0,lambda:None)
        assert fast.duration < slow.duration
        assert slow.duration == max(slow.timing_limits.values())


def test_runtime_monitor_detects_curve_leaving_cartesian_line():
    assert line_distance([200,1,200],[200,0,200],[240,0,200])==pytest.approx(1.)
    assert line_distance([245,0,200],[200,0,200],[240,0,200])==pytest.approx(5.)
