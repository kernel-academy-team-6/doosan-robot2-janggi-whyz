"""Drop actual octagonal piece models in an isolated Bullet world; no ROS calls."""
import json, os, subprocess, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
os.environ['GZ_PARTITION']=f'janggi_piece_specs_{os.getpid()}'
sys.path.insert(0,str(ROOT/'src/janggi/robot_game'))
from piece_visuals import piece_physics,piece_visuals,piece_height_m
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
kinds=('sa','cha','king');positions={}
def callback(message):
 for pose in message.pose:
  if pose.name in kinds:positions[pose.name]=pose.position.z
node=Node();node.subscribe(Pose_V,'/piece_spec_check/poses',callback)
models=[]
for i,kind in enumerate(kinds):
 models.append(f'<model name="{kind}"><pose>{i*.06} 0 .05 0 0 0</pose><link name="l">'
  +piece_physics(kind)+piece_visuals('blue',kind)+'''</link><plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><topic>/piece_spec_check/poses</topic></plugin></model>''')
world='''<sdf version="1.11"><world name="isolated_piece_specs"><physics name="1ms" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"><engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine></plugin><model name="ground"><static>true</static><pose>0 0 -.005 0 0 0</pose><link name="l"><collision name="floor"><geometry><box><size>1 1 .01</size></box></geometry></collision></link></model>'''+''.join(models)+'</world></sdf>'
path=OUT/'world.sdf';path.write_text(world)
with (OUT/'server.log').open('w') as log:
 process=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+40
  while len(positions)!=3:
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No piece feedback')
   time.sleep(.05)
  time.sleep(2)
  result={kind:{'expected_center_z_m':piece_height_m(kind)/2,'measured_center_z_m':positions[kind]} for kind in kinds}
  for record in result.values():assert abs(record['expected_center_z_m']-record['measured_center_z_m'])<.00025
  for record in result.values():record['contact_error_mm']=1000*(record['measured_center_z_m']-record['expected_center_z_m'])
  (OUT/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 finally:
  process.terminate()
  try:process.wait(timeout=5)
  except subprocess.TimeoutExpired:process.kill();process.wait()
