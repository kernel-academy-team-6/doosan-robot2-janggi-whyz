"""Exercise the production PhysicsRobot sync/SDF in isolated Gazebo, without ROS.
Cartesian references are generated offline, with constant flange attitude.
"""
import csv,gzip,json,os,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
os.environ['GZ_PARTITION']=f'janggi_production_validation_{os.getpid()}'
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi')]
import physics_robot
from physics_robot import PhysicsRobot,physical_robot_sdf,ARM_TOPIC
from sim_dynamics import GravityModel
from gz_scene import GzScene,origin_T,axis_rot
from gazebo_scene import RenderedWorld
from simulation_calibration import ensure_simulation_calibration
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
from gz.msgs10.double_pb2 import Double
urdf=subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'),'gripper_mount_offset_mm:=0'],text=True)
xml=ET.fromstring(urdf);chain=[xml.find(f"joint[@name='joint_{i}']") for i in range(1,7)]
names=[f'joint_{i}' for i in range(1,7)]
def fk(q):
 t=np.eye(4)
 for j,a in zip(chain,q):
  t=t@origin_T(j.find('origin'));r=np.eye(4);r[:3,:3]=axis_rot([float(v) for v in j.find('axis').get('xyz').split()],a);t=t@r
 return t
home=np.radians([0,-13.694,134.216,0,59.485,90]);attitude=fk(home)[:3,:3]
lo=np.radians([-180,-180,-150,-180,-150,-270]);hi=-lo
segments=[];cursor=0.;qprev=np.zeros(6)
def joint_move(name,target,duration=3.,hold=1.):
 global cursor,qprev
 spl=CubicSpline([0,1],[qprev,target],bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
 segments.append((cursor,name,duration,hold,spl));cursor+=duration+hold;qprev=target.copy()
def cart_move(name,xyz):
 global cursor,qprev
 start=fk(qprev)[:3,3];duration=max(1.2,1.875*np.linalg.norm(xyz-start)/.12)
 u=np.linspace(0,1,61);qs=[]
 for at in u:
  blend=at**3*(10-15*at+6*at**2);point=start+(xyz-start)*blend
  def error(q):
   t=fk(q);return np.r_[t[:3,3]-point,.2*Rotation.from_matrix(attitude.T@t[:3,:3]).as_rotvec()]
  solved=least_squares(error,qprev,bounds=(lo,hi),gtol=1e-11,ftol=1e-11,xtol=1e-11)
  if np.linalg.norm(error(solved.x))>1e-6:raise RuntimeError('IK failed '+name)
  qprev=solved.x;qs.append(qprev.copy())
 spl=CubicSpline(u,qs,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
 segments.append((cursor,name,duration,1.2,spl));cursor+=duration+1.2
joint_move('main_home',home)
for name,xyz in [('board_approach',[.34,-.08,.205]),('X_250mm',[.59,-.08,.205]),('Y_260mm',[.59,.18,.205]),('X_return',[.34,.18,.205]),('Y_return',[.34,-.08,.205]),('Z_down35',[.34,-.08,.17]),('Z_up35',[.34,-.08,.205])]:cart_move(name,np.array(xyz))
joint_move('main_home_return',home,3.,2.)
joint_move('tool_home',np.radians([0,0,90,0,90,0]),3.,2.)
joint_move('main_home_final',home,3.,3.)
sdf=ET.Element('sdf',version='1.11');world=ET.SubElement(sdf,'world',name='production_test')
ph=ET.SubElement(world,'physics',name='1ms',type='ignored');ET.SubElement(ph,'max_step_size').text='.001';ET.SubElement(ph,'real_time_factor').text='1'
p=ET.SubElement(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');ET.SubElement(ET.SubElement(p,'engine'),'filename').text='gz-physics-bullet-featherstone-plugin'
world.append(ET.fromstring(physical_robot_sdf(urdf)[0]).find('model'))
scene=GzScene(RenderedWorld(ensure_simulation_calibration()))
for body in scene.board_model_sdfs():world.append(ET.fromstring(body).find('model'))
path=OUT/'production.sdf';path.write_text(ET.tostring(sdf,encoding='unicode'))
node=Node();robot=PhysicsRobot(SimpleNamespace(node=node));robot.gravity=GravityModel(urdf)
robot.arm_limits={n:min(194.,float(xml.find(f"joint[@name='{n}']/limit").get('effort'))) for n in names}
robot.arm_publisher=node.advertise(ARM_TOPIC,JointTrajectory)
robot.publishers={'gripper_rh_r1':node.advertise('/janggi_sim/physics/gripper_rh_r1/target',Double)}
clock=[0.];physics_robot.time=SimpleNamespace(monotonic=lambda:clock[0])
latest=[None];rows=[];active={'case':'startup','phase':'startup','target':np.zeros(6)}
def receive(msg):
 state={j.name:j for j in msg.joint}
 if not all(n in state for n in names):return
 now=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9;latest[0]=now
 q=np.array([state[n].axis1.position for n in names]);v=np.array([state[n].axis1.velocity for n in names])
 if started[0] is not None:
  row={'sim_s':now,'case':active['case'],'phase':active['phase']}
  for i in range(6):row.update({f'q{i+1}':q[i],f'v{i+1}':v[i],f'target{i+1}':active['target'][i]})
  rows.append(row)
started=[None];node.subscribe(Model,'/janggi_sim/physics/joint_states',receive)
with (OUT/'production.log').open('w') as log:
 process=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+60
  while latest[0] is None:
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No simulation feedback')
   time.sleep(.02)
  started[0]=latest[0];next_source=0.;printed=-10;deadline=time.monotonic()+cursor*4+30
  while (elapsed:=latest[0]-started[0])<cursor:
   if time.monotonic()>deadline:raise RuntimeError('Simulation stalled')
   clock[0]=elapsed
   at,name,duration,hold,spline=next(s for s in reversed(segments) if elapsed>=s[0]);u=min(1.,(elapsed-at)/duration)
   q=spline(u);v=spline(u,1)/duration if u<1 else np.zeros(6)
   active['case']=name;active['phase']='move' if u<1 else ('settle' if elapsed-at-duration<.4 else 'hold')
   if elapsed>=next_source:
    robot._on_js(SimpleNamespace(name=names+['gripper_rh_r1'],position=list(q)+[.9],velocity=list(v)+[0.]))
    next_source=elapsed+.02
   robot.sync()
   goal=robot.target_buffer.sample(elapsed)
   if goal:active['target']=np.array([goal[n] for n in names])
   if elapsed-printed>10:print(f'{elapsed:.1f}/{cursor:.1f}s {name}',flush=True);printed=elapsed
   time.sleep(.004)
 finally:
  process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()
with gzip.open(OUT/'production.csv.gz','wt') as f:
 writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
q=np.array([[r[f'q{i}'] for i in range(1,7)] for r in rows]);goal=np.array([[r[f'target{i}'] for i in range(1,7)] for r in rows]);v=np.array([[r[f'v{i}'] for i in range(1,7)] for r in rows]);err=np.degrees(q-goal)
report={'duration_sim_s':cursor,'partition':os.environ['GZ_PARTITION'],'samples':len(rows),'cases':{}}
for name in dict.fromkeys(r['case'] for r in rows):
 mask=np.array([r['case']==name and r['phase']=='hold' for r in rows]);moving=np.array([r['case']==name and r['phase']=='move' for r in rows])
 if mask.any():report['cases'][name]={'hold_peak_deg':np.max(np.abs(err[mask]),axis=0).tolist(),'hold_swing_deg':np.degrees(np.ptp(q[mask],axis=0)).tolist(),'hold_speed_peak_deg_s':np.degrees(np.max(np.abs(v[mask]),axis=0)).tolist(),'moving_peak_deg':np.max(np.abs(err[moving]),axis=0).tolist()}
(OUT/'production.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
