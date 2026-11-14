"""Full E0509 + CAD gripper physical pick/place, isolated transport, no ROS moves."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi'),str(ROOT/'src/janggi_pick_place2'),'/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python','/opt/ros/jazzy/opt/gz_transport_vendor/lib/python']
import physics_robot
from physics_robot import PhysicsRobot,physical_robot_sdf,calibrated_robot_urdf,ARM_TOPIC
from sim_dynamics import GravityModel
from gz_scene import origin_T,axis_rot
from piece_visuals import piece_physics,piece_height_m
from janggi_pick_place2.geometry import command_from_width,tcp_length
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
p=argparse.ArgumentParser();p.add_argument('--kind',default='cha');args=p.parse_args();kind=args.kind
os.environ['GZ_PARTITION']=f'janggi_full_pick_{os.getpid()}'
urdf=subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'),'gripper_mount_offset_mm:=0'],text=True)
xml=ET.fromstring(urdf);chain=[xml.find(f"joint[@name='joint_{i}']") for i in range(1,7)];names=[f'joint_{i}' for i in range(1,7)]
def fk(q):
 t=np.eye(4)
 for j,a in zip(chain,q):
  t=t@origin_T(j.find('origin'));r=np.eye(4);r[:3,:3]=axis_rot([float(v) for v in j.find('axis').get('xyz').split()],a);t=t@r
 return t
home=np.radians([0,-13.694,134.216,0,59.485,90]);attitude=fk(home)[:3,:3];qprev=home.copy()
def ik(xyz):
 global qprev
 def error(q):
  t=fk(q);return np.r_[t[:3,3]-xyz,.2*Rotation.from_matrix(attitude.T@t[:3,:3]).as_rotvec()]
 sol=least_squares(error,qprev,bounds=(np.radians([-180,-180,-150,-180,-150,-270]),np.radians([180,180,150,180,150,270])),ftol=1e-11,xtol=1e-11,gtol=1e-11)
 if np.linalg.norm(error(sol.x))>1e-6:raise RuntimeError('IK failed')
 qprev=sol.x;return qprev.copy()
width={'soldier':20.,'cha':26.,'king':32.}[kind];closed=command_from_width(width);opened=command_from_width(width+10);height=piece_height_m(kind);low=(-18+tcp_length(width))/1000
# Fixed-attitude Cartesian line references; arm settling and contacts are measured.
stages=[('home',home,opened,3.,1.),('approach',ik([.465,0,.23]),opened,3.,1.),('descend',ik([.465,0,low]),opened,2.,1.),('close',qprev.copy(),closed,0.,3.),('lift',ik([.465,0,.23]),closed,2.,1.),('transport',ik([.525,0,.23]),closed,2.,1.),('place',ik([.525,0,low]),closed,2.,1.),('release',qprev.copy(),opened,0.,3.),('retreat',ik([.525,0,.23]),opened,2.,1.)]
# Reconstruct straight XYZ MoveL-like references at fixed flange attitude.
splines={};qfrom=np.zeros(6)
for name,qto,c,duration,hold in stages:
 if duration and name!='home':
  qprev=qfrom.copy();a=fk(qfrom)[:3,3];b=fk(qto)[:3,3];u=np.linspace(0,1,61);qs=[]
  for at in u:
   blend=at**3*(10-15*at+6*at**2);qs.append(ik(a+(b-a)*blend))
  splines[name]=CubicSpline(u,qs,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
 qfrom=qto.copy()
sdf=ET.Element('sdf',version='1.11');w=ET.SubElement(sdf,'world',name='full_pick');ph=ET.SubElement(w,'physics',name='1ms',type='ignored');ET.SubElement(ph,'max_step_size').text='.0005';ET.SubElement(ph,'real_time_factor').text='1'
pl=ET.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');ET.SubElement(ET.SubElement(pl,'engine'),'filename').text='gz-physics-bullet-featherstone-plugin'
ET.SubElement(w,'plugin',filename='gz-sim-scene-broadcaster-system',name='gz::sim::systems::SceneBroadcaster')
w.append(ET.fromstring(physical_robot_sdf(urdf)[0]).find('model'))
w.append(ET.fromstring(f'<model name="piece"><pose>.465 0 {-.02+height/2} 0 0 0</pose><link name="piece_body">{piece_physics(kind)}</link></model>'))
w.append(ET.fromstring('<model name="board"><static>true</static><pose>.465 0 -.025 0 0 0</pose><link name="board"><collision name="board"><geometry><box><size>.4 .4 .01</size></box></geometry></collision></link></model>'))
path=OUT/f'robot_{kind}.sdf';path.write_text(ET.tostring(sdf,encoding='unicode'))
node=Node();robot=PhysicsRobot(SimpleNamespace(node=node));robot.gravity=GravityModel(calibrated_robot_urdf(urdf));robot.arm_limits={n:min(194.,float(xml.find(f"joint[@name='{n}']/limit").get('effort'))) for n in names};robot.arm_publisher=node.advertise(ARM_TOPIC,JointTrajectory);robot.publishers={'gripper_rh_r1':node.advertise('/janggi_sim/physics/gripper_rh_r1/target',Double)}
clock=[0.];physics_robot.time=SimpleNamespace(monotonic=lambda:clock[0])
latest=[None];piece=[None];snapshots={};tracking=[]
def receive(msg):
 robot._physics_feedback(msg);latest[0]=robot.physics_state[0]
def poses(msg):
 for p in msg.pose:
  if p.name=='piece':piece[0]=[p.position.x,p.position.y,p.position.z]
node.subscribe(Model,'/janggi_sim/physics/joint_states',receive);node.subscribe(Pose_V,'/world/full_pick/dynamic_pose/info',poses)
q0=np.zeros(6);stroke=0.
with (OUT/f'robot_{kind}.log').open('w') as log:
 process=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+50
  while latest[0] is None or piece[0] is None:
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No physics feedback')
   time.sleep(.02)
  origin=latest[0]
  for stage,q1,c,duration,hold in stages:
   start=latest[0];next_source=0.;deadline=time.monotonic()+max(20,(duration+hold)*10)
   while (at:=latest[0]-start)<duration+hold:
    if time.monotonic()>deadline:raise RuntimeError('Simulation stalled')
    clock[0]=latest[0]-origin
    u=min(1.,at/duration) if duration else 1.;blend=u**3*(10-15*u+6*u*u);q=q0+(q1-q0)*blend;v=(q1-q0)*(30*u*u-60*u**3+30*u**4)/duration if duration and u<1 else np.zeros(6)
    if stage in splines:
     q=splines[stage](u);v=splines[stage](u,1)/duration if u<1 else np.zeros(6)
    if at>=next_source:
     stroke+=float(np.clip(c-stroke,-8,8));robot._on_js(SimpleNamespace(name=names+['gripper_rh_r1'],position=list(q)+[stroke/700],velocity=list(v)+[0.]));next_source=at+.02
    robot.sync()
    if at>=duration+.5:
     measured=robot.physics_state[1];tracking.append(max(abs(measured[n]-q1[i]) for i,n in enumerate(names)))
    time.sleep(.004)
   snapshots[stage]=list(piece[0]);q0=q1.copy();print(stage,snapshots[stage],flush=True)
 finally:
  process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()
report=dict(kind=kind,partition=os.environ['GZ_PARTITION'],snapshots=snapshots,lift_mm=(snapshots['lift'][2]-snapshots['approach'][2])*1000,transport_x_mm=snapshots['transport'][0]*1000,placed_xyz_mm=(np.array(snapshots['retreat'])*1000).tolist(),hold_joint_error_peak_deg=float(np.degrees(max(tracking))))
report['success']=report['lift_mm']>65 and abs(report['transport_x_mm']-525)<5 and abs(report['placed_xyz_mm'][0]-525)<5 and abs(report['placed_xyz_mm'][2]-(-20+height*500))<2
(OUT/f'robot_{kind}.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
