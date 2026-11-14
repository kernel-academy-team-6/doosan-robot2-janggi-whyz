"""Isolated force-control regression; never connects to ROS or a running user world."""
import argparse, csv, json, os, subprocess, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
P=argparse.ArgumentParser();P.add_argument('label');P.add_argument('--no-collisions',action='store_true');P.add_argument('--gripper-only',action='store_true');P.add_argument('--model-file',type=Path);P.add_argument('--contact',action='store_true');P.add_argument('--contact-z',type=float,default=.2535);P.add_argument('--bullet',action='store_true');P.add_argument('--drop-test',action='store_true');args=P.parse_args()
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
os.environ['GZ_PARTITION']=f'janggi_dynamics_{os.getpid()}'
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi')]
from physics_robot import physical_robot_sdf
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double
urdf=ET.fromstring(subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro')],text=True))
if args.gripper_only:
 for el in list(urdf):
  if el.get('name','')!='world' and not el.get('name','').startswith('gripper_'):urdf.remove(el)
 urdf.remove(urdf.find("joint[@name='gripper_attach_joint']"))
 anchor=ET.SubElement(urdf,'joint',name='world_fixed',type='fixed')
 ET.SubElement(anchor,'parent',link='world');ET.SubElement(anchor,'child',link='gripper_rh_p12_rn_base');ET.SubElement(anchor,'origin',xyz='0 0 .4',rpy='0 3.141592653589793 0')
sdf,joints=physical_robot_sdf(ET.tostring(urdf,encoding='unicode'));model=ET.fromstring(sdf).find('model')
if args.model_file:
 model=ET.parse(args.model_file).find('.//model');joints=[j.get('name') for j in model.findall('joint') if j.get('type')=='revolute']
if args.no_collisions:
 for link in model.findall('link'):
  for collision in link.findall('collision'):link.remove(collision)
world=ET.Element('sdf',version='1.11');w=ET.SubElement(world,'world',name='isolated_dynamics')
ph=ET.SubElement(w,'physics',name='1ms',type='ignored');ET.SubElement(ph,'max_step_size').text='.001';ET.SubElement(ph,'real_time_factor').text='1'
ET.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');w.append(model)
if args.bullet:
 ET.SubElement(ET.SubElement(w.find('plugin'),'engine'),'filename').text='gz-physics-bullet-featherstone-plugin'
if args.contact:
 w.append(ET.fromstring('''<model name="contact_piece"><static>true</static><pose>0 0 CONTACT_Z 0 0 0</pose><link name="piece"><collision name="piece"><geometry><cylinder><radius>.0135</radius><length>.007</length></cylinder></geometry></collision></link></model>'''.replace('CONTACT_Z',str(args.contact_z))))
if args.drop_test:
 w.append(ET.fromstring('''<model name="drop_support"><static>true</static><pose>1.5 0 -.025 0 0 0</pose><link name="board"><collision name="board"><geometry><box><size>.4 .4 .01</size></box></geometry></collision></link></model>'''))
 w.append(ET.fromstring('''<model name="drop_piece"><pose>1.5 0 .03 0 0 0</pose><link name="piece"><inertial><mass>.0038997809329</mass><inertia><ixx>0.0000002103</ixx><iyy>0.0000002103</iyy><izz>0.0000004043</izz></inertia></inertial><collision name="piece"><geometry><cylinder><radius>.0143835</radius><length>.005</length></cylinder></geometry></collision></link><plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><topic>/diagnostic/drop_pose</topic></plugin></model>'''))
path=OUT/(args.label+'.sdf');path.write_text(ET.tostring(world,encoding='unicode'))
rows=[];start=None;ready=False;targets={j:0. for j in joints}
def cb(msg):
 global ready
 ready=True
 if start is not None:
  row={'wall_s':time.monotonic()-start}
  for j in msg.joint:
   if j.name in targets:row[j.name]=j.axis1.position;row[j.name+'_target']=targets[j.name]
  if all(j in row for j in joints):rows.append(row)
drop_z=[]
from gz.msgs10.pose_v_pb2 import Pose_V
def drop_cb(msg):
 for pose in msg.pose:
  if pose.name=='drop_piece':drop_z.append(pose.position.z)
node=Node();node.subscribe(Pose_V,'/diagnostic/drop_pose',drop_cb);node.subscribe(Model,'/janggi_sim/physics/joint_states',cb)
pubs={j:node.advertise('/janggi_sim/physics/'+j+'/target',Double) for j in joints}
home=[0.,-.239002,2.342507,0.,1.038202,1.570796]
with (OUT/(args.label+'.log')).open('w') as log:
 server=subprocess.Popen(['gz','sim','-s','-r']+(['--physics-engine','gz-physics-bullet-featherstone-plugin'] if args.bullet else [])+[str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+90
  while not ready:
   if server.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Gazebo did not become ready')
   time.sleep(.05)
  start=time.monotonic()
  while (elapsed:=time.monotonic()-start)<10:
   ramp=np.clip((elapsed-.5)/3,0,1);ramp=ramp*ramp*(3-2*ramp)
   for k,j in enumerate(joints):
    if j.startswith('joint_'):
     n=int(j.split('_')[1])-1;targets[j]=home[n]*ramp+(.08*np.sin(2*np.pi*(elapsed-5)/3) if 5<elapsed<8 else 0)
    else:targets[j]=min(1.,max(0.,(elapsed-4)/1.5))*1.0
    pubs[j].publish(Double(data=float(targets[j])))
   time.sleep(.05)
 finally:
  server.terminate()
  try:server.wait(timeout=8)
  except subprocess.TimeoutExpired:server.kill();server.wait()
if not rows:raise RuntimeError('No feedback: '+str(OUT/(args.label+'.log')))
with (OUT/(args.label+'.csv')).open('w') as f:
 writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
tail=[r for r in rows if r['wall_s']>9]
summary={'label':args.label,'partition':os.environ['GZ_PARTITION'],'samples':len(rows),'joints':{}}
for j in joints:
 e=np.array([r[j]-r[j+'_target'] for r in tail]);q=np.array([r[j] for r in tail]);summary['joints'][j]={'hold_error_deg':float(np.mean(e)*180/np.pi),'hold_peak_to_peak_deg':float(np.ptp(q)*180/np.pi)}
if args.drop_test:
 summary['drop_rest_z_m']=float(np.mean(drop_z[-100:])) if drop_z else None
 summary['drop_expected_z_m']=-.0175
(OUT/(args.label+'.json')).write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
