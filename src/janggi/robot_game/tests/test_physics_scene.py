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
from piece_visuals import piece_physics, ASSETS, SUPPORT_CLEARANCE_M
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
        width,height=({'king':(34.,8.),'sa':(22.,6.),'soldier':(22.,6.)}.get(piece['kind'],(28.,7.)))
        expected=1200*2*(math.sqrt(2)-1)*(width/1000)**2*(height/1000)
        assert float(link.findtext('inertial/mass'))==pytest.approx(expected)
        assert link.findtext('gravity')=='true'
        contacts=link.findall('collision')
        assert len(contacts)==4
        for index,contact in enumerate(contacts):
            size=[float(v) for v in contact.findtext('geometry/box/size').split()]
            # Only the first box supports the piece; the others are inset top/bottom.
            inset=0. if index==0 else 2*SUPPORT_CLEARANCE_M
            assert size==pytest.approx([width/1000,width/1000*math.tan(math.pi/8),height/1000-inset])
            assert float(contact.findtext('pose').split()[-1])==pytest.approx(index*math.pi/4)
        visual_scale=[float(v) for v in link.findtext("visual[@name='body']/geometry/mesh/scale").split()]
        assert visual_scale==pytest.approx([width/1000/math.cos(math.pi/8)]*2+[height/1000])
        center=[float(v) for v in model.findtext('pose').split()[:3]]
        assert center[2]-height/2000==pytest.approx(piece['pos'][2]/1000)
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
    controller=next(p for p in plugins if 'JointTrajectoryController' in p.get('name'))
    assert float(controller.findtext('position_p_gain'))==600.
    assert float(controller.findtext('position_i_gain'))==5.
    assert float(controller.findtext('position_d_gain'))==0.
    assert float(controller.findtext('velocity_p_gain'))==35.
    assert float(controller.findtext('position_cmd_max'))==10.
    assert model.find("joint[@name='world_fixed']/parent").text=='world'
    assert 'ros2_control' not in sdf


def test_target_forwarding_only_sends_observed_joints_and_no_entity_poses():
    from types import SimpleNamespace
    from joint_targets import JointTargetBuffer
    import time
    robot=PhysicsRobot(SimpleNamespace());sent=[]
    robot.target_buffer=JointTargetBuffer(delay_sec=0.)
    robot.publishers={name:SimpleNamespace(publish=lambda msg,n=name:sent.append((n,msg.data)))
                      for name in ['gripper_rh_r1']}
    robot.arm_limits={f'joint_{i}':66. for i in range(1,7)}
    robot.gravity=SimpleNamespace(torques=lambda goals:{})
    robot.arm_publisher=SimpleNamespace(publish=lambda msg:sent.append(('arm',list(msg.points[0].positions))))
    robot.q={'gripper_rh_r1':.3};robot.sync();assert not sent
    robot.q.update({f'joint_{i}':.1*i for i in range(1,7)})
    robot.target_buffer.add(robot.q,time.monotonic())
    robot.sync()
    assert sent[0][0]=='arm' and sent[0][1]==pytest.approx([.1*i for i in range(1,7)])
    assert sent[1]==('gripper_rh_r1',.3)


def test_arm_feedback_is_interpolated_but_gripper_keeps_latest_stroke(monkeypatch):
    from types import SimpleNamespace
    from joint_targets import JointTargetBuffer
    import physics_robot
    clock=[1.]
    monkeypatch.setattr(physics_robot.time,'monotonic',lambda:clock[0])
    robot=PhysicsRobot(SimpleNamespace())
    sent={}
    robot.publishers={name:SimpleNamespace(publish=lambda msg,n=name:sent.update({n:msg.data}))
                      for name in ['gripper_rh_r1']}
    robot.arm_limits={f'joint_{i}':66. for i in range(1,7)}
    robot.gravity=SimpleNamespace(torques=lambda goals:{})
    robot.arm_publisher=SimpleNamespace(publish=lambda msg:sent.update(joint_1=msg.points[0].positions[0]))
    names=[f'joint_{i}' for i in range(1,7)]+['gripper_rh_r1']
    robot._on_js(SimpleNamespace(name=names,position=[0.]*6+[.3]))
    clock[0]=1.02
    robot._on_js(SimpleNamespace(name=names,position=[1.]*6+[.8]))
    # A change right after rest becomes a full source-period segment (1.00-1.10);
    # sample its midpoint after the interpolation delay.
    clock[0]=1.+robot.target_stream.period/2+robot.target_buffer.delay_sec;robot.sync()
    from gripper_gap import angle_from_stroke
    assert sent==pytest.approx({'joint_1':.5,'gripper_rh_r1':angle_from_stroke(560.)})
    clock[0]=10.;robot.sync()
    assert sent==pytest.approx({'joint_1':1.,'gripper_rh_r1':angle_from_stroke(560.)})


def test_observed_piece_centers_use_each_kind_thickness():
    from types import SimpleNamespace
    w=RenderedWorld(ensure_simulation_calibration())
    original=[piece['pos'].copy() for piece in w.pieces]
    poses=[]
    for i,piece in enumerate(w.pieces):
        height={'sa':6.,'soldier':6.,'king':8.}.get(piece['kind'],7.)
        xyz=piece['pos']/1000+w.geom.normal()*height/2000
        poses.append(SimpleNamespace(name=f'piece_{i}',position=SimpleNamespace(x=xyz[0],y=xyz[1],z=xyz[2])))
    w.update_poses(SimpleNamespace(pose=poses))
    for piece,expected in zip(w.pieces,original):
        assert piece['pos']==pytest.approx(expected)


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


def test_gripper_inertia_closed_geometry_and_specification_limits():
    import subprocess
    import numpy as np
    source = Path(__file__).resolve().parents[3]/'e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'
    root = ET.fromstring(subprocess.check_output(['xacro',str(source)],text=True))
    links = [el for el in root.findall('link') if el.get('name').startswith('gripper_')]
    assert sum(float(el.find('inertial/mass').get('value')) for el in links)==pytest.approx(.500)
    for link in links:
        el=link.find('inertial/inertia');v={k:float(x) for k,x in el.attrib.items()}
        inertia=np.array([[v['ixx'],v['ixy'],v['ixz']],[v['ixy'],v['iyy'],v['iyz']],[v['ixz'],v['iyz'],v['izz']]])
        eigen=np.linalg.eigvalsh(inertia)
        assert eigen.min()>0 and eigen[2]<=eigen[0]+eigen[1]+1e-12
    limit=root.find("joint[@name='gripper_rh_r1']/limit")
    angle=float(limit.get('upper'))
    # Actual inner pad faces in the supplied CAD: 0.057 m arm, 30 degree offset.
    gap=2*(.008+.0493634*math.cos(angle)-.0285*math.sin(angle)-.0039)
    # Restore the original limit and 700 -> 1.0 rad mapping. CAD closure is
    # not evidence that a real gripper should be driven to a different stroke.
    assert angle==pytest.approx(1.101)
    assert gap*1000==pytest.approx(2.069436,abs=1e-5)
    assert .114*float(limit.get('velocity'))<=.075
    sdf,_=physical_robot_sdf(ET.tostring(root,encoding='unicode'));model=ET.fromstring(sdf).find('model')
    for i,speed in enumerate([120,120,150,225,225,225],1):
        assert float(model.findtext(f"joint[@name='joint_{i}']/axis/limit/velocity"))==pytest.approx(math.radians(speed))
    assert float(model.findtext("joint[@name='joint_5']/axis/limit/upper"))==pytest.approx(math.radians(155))
    controllers=[p for p in model.findall('plugin') if 'JointPositionController' in p.get('name')]
    assert len(controllers)==1
    arm=next(p for p in model.findall('plugin') if 'JointTrajectoryController' in p.get('name'))
    for index in range(6):
        limit=194. if index<3 else 66.
        assert float(arm.findall('position_d_gain')[index].text)==0.
        assert float(arm.findall('position_cmd_max')[index].text)+float(arm.findall('velocity_cmd_max')[index].text)+.35*limit==pytest.approx(limit)
    for side in ('r','l'):
        collisions=model.findall(f"link[@name='gripper_rh_p12_rn_{side}2']/collision")
        assert len(collisions)==2 and all(c.find('geometry/box') is not None for c in collisions)
    for name in ('gripper_rh_r2','gripper_rh_l1','gripper_rh_l2'):
        assert model.find(f"joint[@name='{name}']/axis/mimic").get('joint')=='gripper_rh_r1'
    for plugin in controllers:
        joint=plugin.findtext('joint_name');effort=float(plugin.findtext('cmd_max'))
        assert effort<=float(model.findtext(f"joint[@name='{joint}']/axis/limit/effort"))


def _emulator_robot(monkeypatch):
    from types import SimpleNamespace
    import physics_robot
    clock=[0.]
    monkeypatch.setattr(physics_robot.time,'monotonic',lambda:clock[0])
    return PhysicsRobot(SimpleNamespace()),clock


def test_virtual_driver_held_updates_give_smooth_monotone_reference(monkeypatch):
    """DRCF pose updates ~10 Hz but /joint_states repeats it at 100 Hz (zero velocity)."""
    from types import SimpleNamespace
    robot,clock=_emulator_robot(monkeypatch)
    names=[f'joint_{i}' for i in range(1,7)]
    def pose(t):  # 0.5 rad/s ramp held for 0.1 s per update, starting at t=1
        return .5*max(0.,min(3.,round(int((t-1.)/.1)*.1,6)))
    t=0.
    references=[]
    while t<5.:
        clock[0]=t
        robot._on_js(SimpleNamespace(name=names,position=[pose(t)]*6,velocity=[0.]*6))
        if t>.5:
            q,v=robot.target_buffer.sample_state(t)
            references.append((t,q['joint_1'],v['joint_1']))
        t=round(t+.01,6)
    q=[r[1] for r in references];v=[r[2] for r in references]
    assert all(b>=a-1e-12 for a,b in zip(q,q[1:]))           # no reversals
    assert q[-1]==pytest.approx(1.5)                            # endpoint preserved
    steady=[s for at,_,s in references if 1.6<at<3.6]
    assert min(steady)>.45 and max(steady)<.55                  # no hold-then-jump pulses
    # Start/stop ease from rest needs <1.5x the ramp speed; the old path gave ~3-8x pulses.
    assert max(abs(s) for s in v)<.75


def test_stale_combined_arm_copies_are_ignored_while_arm_only_source_is_alive(monkeypatch):
    from types import SimpleNamespace
    robot,clock=_emulator_robot(monkeypatch)
    arm=[f'joint_{i}' for i in range(1,7)]
    fingers=['gripper_rh_r1','gripper_rh_r2','gripper_rh_l1','gripper_rh_l2']
    clock[0]=1.;robot._on_js(SimpleNamespace(name=arm,position=[.2]*6))
    clock[0]=1.009;robot._on_js(SimpleNamespace(name=arm+fingers,position=[.1]*6+[0.]*4))
    assert robot.q['joint_1']==.2
    # Without a live arm-only source (e.g. validation scripts) combined messages drive the arm.
    clock[0]=2.;robot._on_js(SimpleNamespace(name=arm+fingers,position=[.3]*6+[0.]*4))
    assert robot.q['joint_1']==.3


def test_continuous_playback_never_moves_backwards_when_delay_grows():
    from joint_targets import JointTargetBuffer
    buffer=JointTargetBuffer(.05,cubic=True,continuous=True)
    for i in range(30):buffer.add({'q':i*.01},i*.01)
    previous=None
    for i in range(40):
        now=.1+i*.005
        if i==10:buffer.delay_sec=.2
        q=buffer.sample(now)['q']
        if previous is not None:assert q>=previous-1e-12
        previous=q


def test_arm_links_are_filtered_from_pieces_but_gripper_side_links_are_not():
    from collision_masks import ARM_CATEGORY, ARM_COLLIDE
    inertial='<inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>'
    box='<collision><geometry><box size=".1 .1 .1"/></geometry></collision>'
    urdf=(f'<robot name="test"><link name="world"/><link name="base_link">{inertial}{box}</link>'
          '<joint name="world_fixed" type="fixed"><parent link="world"/><child link="base_link"/></joint>'
          f'<link name="link_2">{inertial}{box}</link><link name="link_6">{inertial}{box}</link>'
          '<joint name="joint_2" type="revolute"><parent link="base_link"/><child link="link_2"/><axis xyz="0 0 1"/>'
          '<limit lower="-3" upper="3" effort="20" velocity="1"/></joint>'
          '<joint name="joint_6" type="revolute"><parent link="link_2"/><child link="link_6"/><axis xyz="0 0 1"/>'
          '<limit lower="-3" upper="3" effort="20" velocity="1"/></joint></robot>')
    model=ET.fromstring(physical_robot_sdf(urdf)[0]).find('model')
    for name in ('base_link','link_2'):
        contact=model.find(f"link[@name='{name}']/collision/surface/contact")
        assert int(contact.findtext('category_bitmask'))==ARM_CATEGORY
        assert int(contact.findtext('collide_bitmask'))==ARM_COLLIDE
    # Wrist / gripper keep the default (unfiltered) groups so they still touch pieces.
    assert model.find("link[@name='link_6']/collision/surface/contact") is None


def test_create_accepts_entity_that_appears_after_reply_timeout(monkeypatch):
    """A timed-out create/blocking reply must not kill the scene (and robot loop) if the model exists."""
    import gz_scene
    scene=object.__new__(GzScene);scene.gz_world='empty';scene._existing_models=set()
    monkeypatch.setattr(scene,'_svc',lambda *a,**k:False,raising=False)
    calls=[0]
    def refresh():
        calls[0]+=1
        if calls[0]>=2:scene._existing_models={'intel_realsense_d455f'}
    monkeypatch.setattr(scene,'_refresh_existing_models',refresh,raising=False)
    monkeypatch.setattr(gz_scene.time,'sleep',lambda s:None)
    assert scene._create('<sdf version="1.9"><model name="intel_realsense_d455f"/></sdf>')
    # A model that never appears still fails loudly.
    scene._existing_models=set();calls[0]=-10**9
    clock=[0.]
    monkeypatch.setattr(gz_scene.time,'monotonic',lambda:clock.__setitem__(0,clock[0]+10.) or clock[0])
    with pytest.raises(RuntimeError):
        scene._create('<sdf version="1.9"><model name="missing"/></sdf>')
