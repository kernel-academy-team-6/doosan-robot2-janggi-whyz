"""Verify gravity sign/SI units and the sum of independent torque budgets."""
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sim_dynamics import GravityModel
from physics_robot import arm_command, ARM_SPEEDS, GRAVITY_BUDGET


@pytest.fixture(scope='module')
def gravity():
    source=Path(__file__).resolve().parents[3]/'e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'
    return GravityModel(subprocess.check_output(['xacro',str(source)],text=True))


@pytest.mark.parametrize('angles', [[0,-13.694,134.216,0,59.485,90],[40,-25,110,30,75,-45],[0,0,90,0,90,0]])
def test_gravity_is_potential_gradient(gravity,angles):
    q=dict(zip([f'joint_{i}' for i in range(1,7)],np.radians(angles)))
    q['gripper_rh_r1']=.8
    def potential(positions):
        links,_=gravity._frames(positions)
        return sum(mass*9.80665*(links[n][:3,:3]@com+links[n][:3,3])[2]
                   for n,(mass,com) in gravity.masses.items())
    torque=gravity.torques(q)
    for name in torque:
        plus=q.copy();minus=q.copy();plus[name]+=1e-6;minus[name]-=1e-6
        assert torque[name]==pytest.approx((potential(plus)-potential(minus))/2e-6,abs=1e-5)


def test_command_clips_velocity_and_gravity_without_changing_positions():
    from types import SimpleNamespace
    names=[f'joint_{i}' for i in range(1,7)];q=dict.fromkeys(names,.2)
    limits=dict.fromkeys(names,66.)
    gravity=SimpleNamespace(torques=lambda positions:dict.fromkeys(names,1000.))
    msg=arm_command(q,dict.fromkeys(names,-1000.),gravity,limits)
    assert list(msg.joint_names)==names
    assert list(msg.points[0].positions)==[.2]*6
    assert list(msg.points[0].velocities)==pytest.approx([-v for v in ARM_SPEEDS])
    assert list(msg.points[0].effort)==pytest.approx([66*GRAVITY_BUDGET]*6)
    assert arm_command({}, {}, gravity,limits) is None
