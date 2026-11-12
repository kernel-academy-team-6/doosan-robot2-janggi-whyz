"""UI and action server must use identical backend construction."""
from types import SimpleNamespace
from dataclasses import replace
import pytest
from janggi_pick_place2 import runtime
from janggi_pick_place2.executor import Settings


def test_backend_factory_selects_same_virtual_arm_and_gripper_as_action_server(monkeypatch):
    calls=[]
    monkeypatch.setattr(runtime,'VirtualRobotClient',lambda *args:calls.append(('arm',args)) or 'virtual_arm')
    monkeypatch.setattr(runtime,'VirtualGripperClient',lambda *args:calls.append(('hand',args)) or 'virtual_hand')
    params=dict(service_timeout_sec=5.,motion_timeout_sec=60.,virtual_gripper_delay_sec=.4)
    robot,hand=runtime.create_clients('node','/dsr01',Settings(),params,'virtual')
    assert (robot,hand)==('virtual_arm','virtual_hand')
    assert calls[1][1]==('node','/dsr01',.4)


def test_move_line_trace_records_exact_ros_request_without_altering_pose():
    from janggi_pick_place2.robot_client import RobotClient
    robot=RobotClient.__new__(RobotClient);robot.settings=Settings(joint6_lock_enabled=False)
    requests=[];traces=[]
    robot.io=SimpleNamespace(call=lambda kind,path,req,check:requests.append(req))
    robot.current_pose=lambda check:[400.,-119.455,140.,0.,180.,0.]
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,0.]
    robot._solution=lambda check:0
    robot._preview_line=lambda *args:None
    robot._wait=lambda reached,check:None
    robot.command_observer=lambda operation,payload:traces.append((operation,payload))
    target=[406.5872,-119.455,140.,0.,180.,0.]
    robot.move(target,lambda:None)
    assert list(requests[0].pos)==traces[0][1]['flange_pose']==target
    assert traces[0][0]=='move_line' and traces[0][1]['ref']==0


@pytest.mark.parametrize('velocity',[30.,120.,400.])
def test_default_cartesian_path_passes_requested_speed_and_acceleration_without_duration_override(velocity):
    from janggi_pick_place2.robot_client import RobotClient
    robot=RobotClient.__new__(RobotClient)
    assert not Settings().joint6_lock_enabled
    robot.settings=replace(Settings(),linear_velocity_mm_s=velocity,
                           linear_acceleration_mm_s2=velocity*2,
                           angular_velocity_deg_s=velocity/2,
                           angular_acceleration_deg_s2=velocity)
    robot.current_pose=lambda check:[400.,0.,200.,90.,180.,0.]
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,90.]
    robot._solution=lambda check:0
    robot._preview_line=lambda *args:None
    robot._locked_planner=lambda:pytest.fail('default motion must not use timed J6 spline')
    robot._wait=lambda *args,**kwargs:None
    requests=[]
    robot.io=SimpleNamespace(call=lambda kind,path,req,check:requests.append((path,req)))
    for point in ([450.,0.,200.,90.,180.,0.],[450.,0.,142.,90.,180.,0.]):
        robot.move(point,lambda:None)
    assert len(requests)==2
    for path,request in requests:
        assert path=='/motion/move_line' and request.time==0.
        assert list(request.vel)==[velocity,velocity/2]
        assert list(request.acc)==[velocity*2,velocity]
        assert list(request.pos[3:])==[90.,180.,0.]


def test_cartesian_preflight_is_reused_for_matching_state_but_rechecked_after_drift():
    from janggi_pick_place2.robot_client import RobotClient
    robot=RobotClient.__new__(RobotClient); robot.settings=Settings()
    state={'pose':[400.,0.,200.,90.,180.,0.]}; checks=[]; requests=[]
    joints=[0.,0.,90.,0.,90.,90.]
    robot.current_pose=lambda check:list(state['pose'])
    robot.current_joints=lambda check:list(joints)
    robot._solution=lambda check:0
    def preview(start,target,seed,*args):
        checks.append((list(start),list(target)))
        return list(seed)
    robot._preview_line=preview
    robot._joint_move=lambda *args:pytest.fail('preserved home attitude needs no wrist alignment')
    def call(kind,path,req,check):
        requests.append(req);state['pose']=list(req.pos)
    robot.io=SimpleNamespace(call=call)
    robot._wait=lambda reached,check:reached()
    targets=[[450.,0.,200.,90.,180.,0.],[450.,0.,142.,90.,180.,0.]]
    robot.align_for_path(list(state['pose']),targets,lambda:None)
    assert len(checks)==2
    robot.move(targets[0],lambda:None)
    assert len(checks)==2  # No duplicate per-waypoint IK queries.
    state['pose'][0]+=1.
    robot.move(targets[1],lambda:None)
    assert len(checks)==3 and len(requests)==2
