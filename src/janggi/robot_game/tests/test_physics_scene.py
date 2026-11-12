"""Physical properties and controller paths must not bypass the physics solver."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import math
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import sim_app
from gazebo_scene import RenderedWorld
from gz_scene import GzScene
from simulation_calibration import ensure_simulation_calibration
from piece_visuals import piece_physics, DIAMETERS_MM, DEFAULT_DIAMETER_MM
from physics_robot import physical_robot_sdf, PhysicsRobot
from scene_models import CalibratedLayout


def test_all_pieces_have_mass_inertia_gravity_and_collisions():
    cal=ensure_simulation_calibration();w=RenderedWorld(cal)
    scene=GzScene.__new__(GzScene);scene.w=w
    scene.layout=CalibratedLayout.from_world(w,cal)
    scene.n,scene.R,scene.center=scene.layout.normal,scene.layout.rotation,scene.layout.center
    models=[ET.fromstring(sdf).find('model') for sdf in scene.board_model_sdfs()]
    assert models[0].find('link/collision') is not None
    assert models[1].find('link/collision') is not None
    for model,piece in zip(models[2:],w.pieces):
        assert model.findtext('static')=='false'
        link=model.find('link')
        radius=DIAMETERS_MM.get(piece['kind'],DEFAULT_DIAMETER_MM)/2000
        expected=1200*math.pi*radius**2*.005
        assert float(link.findtext('inertial/mass'))==pytest.approx(expected)
        assert link.findtext('gravity')=='true'
        assert float(link.findtext('collision/geometry/cylinder/length'))==.005
        for axis in ('ixx','iyy','izz'):
            assert float(link.findtext('inertial/inertia/'+axis))>0


def test_physical_robot_uses_torque_pid_and_preserves_collision_and_inertia():
    urdf='''<robot name="test"><link name="world"/><link name="base_link">
    <inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial></link>
    <joint name="world_fixed" type="fixed"><parent link="world"/><child link="base_link"/></joint>
    <link name="moving"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
    <joint name="joint_1" type="revolute"><parent link="base_link"/><child link="moving"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="20" velocity="1"/></joint>
    <ros2_control name="must_not_start" type="system"/></robot>'''
    sdf,joints=physical_robot_sdf(urdf);model=ET.fromstring(sdf).find('model')
    assert joints==['joint_1'] and model.findtext('static')=='false'
    assert model.find("link[@name='moving']/collision") is not None
    plugins=model.findall('plugin')
    controller=next(p for p in plugins if 'JointPositionController' in p.get('name'))
    assert controller.findtext('use_velocity_commands')=='false'
    assert float(controller.findtext('cmd_max'))>0
    assert model.find("joint[@name='world_fixed']/parent").text=='world'
    assert 'ros2_control' not in sdf


def test_target_forwarding_only_sends_observed_joints_and_no_entity_poses():
    from types import SimpleNamespace
    robot=PhysicsRobot.__new__(PhysicsRobot);sent=[]
    robot.publishers={name:SimpleNamespace(publish=lambda msg,n=name:sent.append((n,msg.data)))
                      for name in ['joint_1','gripper_rh_r1']}
    robot.q={'gripper_rh_r1':.3};robot.sync();assert not sent
    robot.q.update({f'joint_{i}':.1*i for i in range(1,7)})
    robot.sync()
    assert sent==[('joint_1',.1),('gripper_rh_r1',.3)]


@pytest.mark.parametrize('override,expected_mm', [(None, 13.766), ('20.5', 20.5)])
def test_gripper_attachment_offsets_along_joint6_flange_axis(override, expected_mm):
    import subprocess
    source = Path(__file__).resolve().parents[3]/'e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'
    command = ['xacro', str(source)]
    if override is not None:
        command.append(f'gripper_mount_offset_mm:={override}')
    root = ET.fromstring(subprocess.check_output(command, text=True))
    attachment=root.find("joint[@name='gripper_attach_joint']")
    assert attachment.find('parent').get('link')=='link_6'
    assert attachment.find('child').get('link')=='gripper_rh_p12_rn_base'
    assert [float(x) for x in attachment.find('origin').get('xyz').split()]==pytest.approx([0.,0.,expected_mm/1000.])
    assert attachment.find('origin').get('rpy')=='0 0 1.5708'
