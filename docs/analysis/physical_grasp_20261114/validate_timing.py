"""Wall-time emulator source versus slow Gazebo physics; no ROS motion clients."""
import json,os,subprocess,sys,time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi'),str(ROOT/'src/janggi_pick_place2'),'/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python','/opt/ros/jazzy/opt/gz_transport_vendor/lib/python']
from physics_robot import PhysicsRobot,calibrated_robot_urdf,ARM_TOPIC
from sim_dynamics import GravityModel
from janggi_pick_place2.simulation_feedback import SimulationFeedback
from sensor_msgs.msg import JointState
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
from gz.msgs10.double_pb2 import Double
os.environ['GZ_PARTITION']=f'janggi_slow_timing_{os.getpid()}'
urdf=subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'),'gripper_mount_offset_mm:=0'],text=True)
sdf=ET.parse(OUT/'robot_soldier.sdf');sdf.find('world/physics/real_time_factor').text='.4';path=OUT/'slow_timing.sdf';sdf.write(path)
node=Node();robot=PhysicsRobot(SimpleNamespace());robot.gravity=GravityModel(calibrated_robot_urdf(urdf));names=[f'joint_{i}' for i in range(1,7)];robot.arm_limits={n:194. if i<3 else 66. for i,n in enumerate(names)};robot.arm_publisher=node.advertise(ARM_TOPIC,JointTrajectory);robot.publishers={'gripper_rh_r1':node.advertise('/janggi_sim/physics/gripper_rh_r1/target',Double)}
feedback=SimulationFeedback(SimpleNamespace(create_subscription=lambda *a,**k:None),'/isolated',timeout=20.)
latest=[None];peak=np.zeros(6)
def receive(message):
 robot._physics_feedback(message);latest[0]=robot.physics_state[0]
 q=robot.physics_state[1];v=robot.physics_state[2]
 global peak
 peak=np.maximum(peak,np.abs([v[n] for n in names]))
 msg=JointState();msg.header.stamp.sec=int(latest[0]);msg.header.stamp.nanosec=int((latest[0]-int(latest[0]))*1e9);msg.name=names;msg.position=[q[n] for n in names];msg.velocity=[v[n] for n in names];feedback.receive(msg)
node.subscribe(Model,'/janggi_sim/physics/joint_states',receive)
goal_deg=[0,-13.694,134.216,0,59.485,90];goal=np.radians(goal_deg);duration=3.;result={};done=[False]
import threading
with (OUT/'slow_timing.log').open('w') as log:
 process=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+50
  while latest[0] is None:
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No feedback')
   time.sleep(.02)
  wall0=time.monotonic();sim0=latest[0];next_source=0.;waiting=False
  def wait_physics():
   try:
    feedback.arm(goal_deg,lambda:None);result.update(completed_wall_s=time.monotonic()-wall0,completed_sim_s=latest[0]-sim0)
   except Exception as exc:result['error']=str(exc)
   finally:done[0]=True
  while not done[0]:
   elapsed=time.monotonic()-wall0
   if elapsed>25:raise RuntimeError('No physical completion')
   if elapsed>=next_source:
    u=min(1.,elapsed/duration);blend=u**3*(10-15*u+6*u*u);q=goal*blend;v=goal*(30*u*u-60*u**3+30*u**4)/duration if u<1 else np.zeros(6)
    robot._on_js(SimpleNamespace(name=names+['gripper_rh_r1'],position=list(q)+[.8],velocity=list(v)+[0.]));next_source=elapsed+.02
   robot.sync()
   if elapsed>=duration and not waiting:
    waiting=True;threading.Thread(target=wait_physics,daemon=True).start()
   time.sleep(.004)
 finally:
  process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()
result.update(source_duration_wall_s=duration,requested_real_time_factor=.4,peak_axis_speed_deg_s=np.degrees(peak).tolist(),partition=os.environ['GZ_PARTITION'])
result['success']='error' not in result and result['completed_wall_s']>5. and result['completed_sim_s']>2.8 and max(result['peak_axis_speed_deg_s'])<120
(OUT/'slow_timing.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
