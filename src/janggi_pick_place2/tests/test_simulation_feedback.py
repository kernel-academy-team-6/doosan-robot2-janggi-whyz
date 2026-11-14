"""Only real Gazebo joint feedback can complete simulated motions."""
from types import SimpleNamespace
import math
import pytest
from janggi_pick_place2.simulation_feedback import SimulationFeedback
from janggi_pick_place2.simulation_geometry import angle_from_stroke
from janggi_pick_place2.robot_client import RobotClient, VirtualRobotClient


def feedback():
    class Node:
        def create_subscription(self,*args,**kwargs):return object()
    return SimulationFeedback(Node(),'/dsr01',timeout=.02)


def test_arm_completion_requires_all_six_actual_positions_and_low_velocities():
    f=feedback();checks=[]
    f.wait=lambda reached,check:checks.append(reached)
    f.arm([0,0,90,0,90,90],lambda:None)
    ready=checks[0];q={f'joint_{i}':math.radians(v) for i,v in enumerate([0,0,90,0,90,90],1)};v=dict.fromkeys(q,0.)
    assert ready(q,v)
    assert not ready({},v)
    v['joint_6']=.1;assert not ready(q,v)
    v['joint_6']=0.;q['joint_3']+=.02;assert not ready(q,v)


def test_gripper_completion_accepts_stationary_contact_but_not_motion_or_missing_jaw():
    f=feedback();checks=[];f.wait=lambda reached,check:checks.append(reached)
    f.gripper(20.,lambda:None);ready=checks[0]
    q=dict.fromkeys(['gripper_rh_r1','gripper_rh_l1'],angle_from_stroke(619.));v=dict.fromkeys(q,0.)
    assert ready(q,v)
    assert not ready({},v)
    v['gripper_rh_l1']=.3;assert not ready(q,v)


def test_virtual_move_waits_for_physics_after_emulator_completion(monkeypatch):
    calls=[];robot=VirtualRobotClient.__new__(VirtualRobotClient)
    monkeypatch.setattr(RobotClient,'move',lambda self,*args:calls.append('emulator_complete'))
    robot.current_joints=lambda check:[0]*6
    robot.physics_feedback=SimpleNamespace(arm=lambda *args:calls.append('physical_settled'))
    robot.move([400,0,140,0,180,0],lambda:None)
    assert calls==['emulator_complete','physical_settled']


def test_absent_physics_times_out_instead_of_claiming_completion():
    with pytest.raises(TimeoutError,match='Gazebo physical motion'):
        feedback().wait(lambda q,v:True,lambda:None)
