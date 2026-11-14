"""Desired real-model gap must equal the geometric Gazebo CAD pad gap."""
from pathlib import Path
from types import SimpleNamespace
import sys
import math
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gripper_gap import angle_from_stroke,angle_from_ros_target,cad_gap_mm
from janggi_pick_place2.geometry import width_from_command


@pytest.mark.parametrize('c',[0.,50.,200.,350.,500.,619.,650.,700.])
def test_model_width_is_matched_without_changing_joint_limits(c):
    angle=angle_from_stroke(c)
    assert 0<=angle<=1.101
    assert cad_gap_mm(angle)==pytest.approx(width_from_command(c),abs=1e-10)
    assert angle_from_ros_target(c/700)==pytest.approx(angle)


def test_continuous_mapping_preserves_motion_direction_and_velocity_limit():
    angles=np.array([angle_from_stroke(c) for c in np.linspace(0,700,701)])
    assert np.all(np.diff(angles)>0)
    # Existing 8 stroke / 20ms animation and 0.65rad/s physics joint limit.
    assert np.max(np.diff(angles))*400<.65
    with pytest.raises(ValueError):angle_from_ros_target(math.nan)


def test_arm_only_feedback_does_not_recalibrate_old_gripper_value():
    from physics_robot import PhysicsRobot
    robot=PhysicsRobot(SimpleNamespace())
    robot._on_js(SimpleNamespace(name=['gripper_rh_r1'],position=[1.]))
    expected=angle_from_stroke(700.)
    assert robot.q['gripper_rh_r1']==pytest.approx(expected)
    robot._on_js(SimpleNamespace(name=['joint_1'],position=[.2]))
    assert robot.q['gripper_rh_r1']==pytest.approx(expected)
    assert robot.q['joint_1']==.2


def test_gazebo_mount_matches_flange_tool_length_across_opening_arc():
    from gripper_gap import calibrated_mount_mm,cad_tip_length_mm
    from janggi_pick_place2.geometry import tcp_length_from_command
    assert calibrated_mount_mm()==pytest.approx(15.37754565,abs=1e-6)
    for c in range(701):
        assert abs(calibrated_mount_mm()+cad_tip_length_mm(c)-tcp_length_from_command(c))<.12


def test_slow_physics_clock_does_not_compress_source_trajectory():
    from physics_robot import PhysicsRobot
    robot=PhysicsRobot(SimpleNamespace());robot._feedback_required=True
    assert robot.reference_time(10.) is None
    robot.physics_state=(100.,{},{});assert robot.reference_time(10.)==10.
    robot.physics_state=(100.2,{},{});assert robot.reference_time(11.)==pytest.approx(10.2)
    # Paused simulation: the wall clock keeps running but the reference stays put.
    assert robot.reference_time(12.)==pytest.approx(10.2)
