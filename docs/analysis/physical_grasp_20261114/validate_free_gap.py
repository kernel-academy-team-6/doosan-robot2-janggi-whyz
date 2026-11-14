"""Isolated Gazebo pad-pose ruler validation. No ROS or real device connection."""
import copy,json,os,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
os.environ['GZ_PARTITION']=f'janggi_gripper_width_{os.getpid()}'
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi'),str(ROOT/'tools'),str(ROOT/'src/janggi_pick_place2'),'/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python','/opt/ros/jazzy/opt/gz_transport_vendor/lib/python']
from physics_robot import PhysicsRobot,physical_robot_sdf
from gripper_gap import angle_from_stroke
from janggi_pick_place2.geometry import width_from_command
from gazebo_gripper_gap import inner_pad,transform,pose_matrix,measure_pads,LINKS
from gz_scene import origin_T
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
full=ET.fromstring(subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro')],text=True))
urdf=ET.Element('robot',name='gripper_only');ET.SubElement(urdf,'link',name='world')
for child in full:
 if child.tag=='link' and child.get('name','').startswith('gripper_'):urdf.append(copy.deepcopy(child))
 if child.tag=='joint' and child.get('name') in ('gripper_rh_r1','gripper_rh_r2','gripper_rh_l1','gripper_rh_l2'):urdf.append(copy.deepcopy(child))
j=ET.SubElement(urdf,'joint',name='mount',type='fixed');ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link='gripper_rh_p12_rn_base');ET.SubElement(j,'origin',xyz='0 0 .3',rpy='0 3.141592653589793 0')
sdf=ET.Element('sdf',version='1.11');w=ET.SubElement(sdf,'world',name='gripper_width_check')
p=ET.SubElement(w,'physics',name='1ms',type='ignored');ET.SubElement(p,'max_step_size').text='.0005';ET.SubElement(p,'real_time_factor').text='1'
p=ET.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');ET.SubElement(ET.SubElement(p,'engine'),'filename').text='gz-physics-bullet-featherstone-plugin'
ET.SubElement(w,'plugin',filename='gz-sim-scene-broadcaster-system',name='gz::sim::systems::SceneBroadcaster')
w.append(ET.fromstring(physical_robot_sdf(ET.tostring(urdf,encoding='unicode'))[0]).find('model'))
path=OUT/'calibration_validation.sdf';path.write_text(ET.tostring(sdf,encoding='unicode'))
pads={}
for name,side,mesh in zip(LINKS,('right','left'),('r2','l2')):
 visual=full.find(f"link[@name='{name}']/visual")
 scale=[float(v) for v in visual.find('geometry/mesh').get('scale').split()]
 pads[name]=transform(inner_pad(ROOT/f'src/RH-P12-RN-A/rh_p12_rn_a_description/meshes/{mesh}.stl',scale,side),origin_T(visual.find('origin')))
latest=[None]
def poses(message):
 by_name={p.name.split('::')[-1]:p for p in message.pose}
 if all(n in by_name for n in LINKS):latest[0]=(time.monotonic(),measure_pads(*(transform(pads[n],pose_matrix(by_name[n])) for n in LINKS))[0])
node=Node();node.subscribe(Pose_V,'/world/gripper_width_check/dynamic_pose/info',poses)
pub=node.advertise('/janggi_sim/physics/gripper_rh_r1/target',Double)
robot=PhysicsRobot(SimpleNamespace());stroke=0.;report=[]
with (OUT/'calibration_validation.log').open('w') as log:
 server=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+45
  while latest[0] is None:
   if server.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No physical pose feedback')
   time.sleep(.02)
  for command in (0,200,400,619,700,350,0):
   start=time.monotonic();samples=[];last_stamp=-1
   while (elapsed:=time.monotonic()-start)<4.:
    stroke+=float(np.clip(command-stroke,-8.,8.))
    robot._on_js(SimpleNamespace(name=['gripper_rh_r1'],position=[stroke/700.]))
    pub.publish(Double(data=robot.q['gripper_rh_r1']))
    if elapsed>3. and latest[0][0]!=last_stamp:last_stamp=latest[0][0];samples.append(latest[0][1]['gap_mm'])
    time.sleep(.02)
   if not samples:raise RuntimeError('No fresh settled gap samples')
   expected=width_from_command(command)
   row=dict(command=command,desired_width_mm=expected,measured_gap_mm=float(np.mean(samples)),gap_error_mm=float(np.mean(samples)-expected),gap_swing_mm=float(np.ptp(samples)),target_joint_rad=angle_from_stroke(command),samples=len(samples))
   report.append(row);print(json.dumps(row),flush=True)
 finally:
  server.terminate()
  try:server.wait(timeout=8)
  except subprocess.TimeoutExpired:server.kill();server.wait()
(OUT/'calibration_results.json').write_text(json.dumps(dict(partition=os.environ['GZ_PARTITION'],measurements=report),indent=2)+'\n')
