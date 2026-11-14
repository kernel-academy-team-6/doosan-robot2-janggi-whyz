"""Physical lift/transport/release test, isolated GZ_PARTITION; no ROS or pose resets."""
import argparse,copy,json,os,subprocess,sys,time,gzip
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi'),str(ROOT/'src/janggi_pick_place2'),str(ROOT/'tools'),'/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python','/opt/ros/jazzy/opt/gz_transport_vendor/lib/python']
from physics_robot import physical_robot_sdf
from gripper_gap import angle_from_stroke
from piece_visuals import piece_physics,piece_height_m
from janggi_pick_place2.geometry import command_from_width,tcp_length
from gazebo_gripper_gap import inner_pad,transform,pose_matrix,measure_pads,LINKS
from gz_scene import origin_T
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
parser=argparse.ArgumentParser();parser.add_argument('--label',required=True);parser.add_argument('--kind',default='soldier');parser.add_argument('--contact-trace',action='store_true');parser.add_argument('--mount',default='0');parser.add_argument('--shape',default='mesh');parser.add_argument('--pad-p',type=float);parser.add_argument('--pad-i',type=float);parser.add_argument('--pad-effort',type=float);parser.add_argument('--step',default='.0005');parser.add_argument('--engine',default='gz-physics-bullet-featherstone-plugin');parser.add_argument('--full-width',action='store_true');parser.add_argument('--collision-extra-mm',type=float,default=0.);args=parser.parse_args()
OUT=Path(__file__).parent;os.environ['GZ_PARTITION']=f'janggi_grasp_{os.getpid()}'
kind=args.kind;width={'soldier':20.,'cha':26.,'king':32.}[kind];opened=width+10
full=ET.fromstring(subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro'),f'gripper_mount_offset_mm:={args.mount}'],text=True))
u=ET.Element('robot',name='test');ET.SubElement(u,'link',name='world')
for name in ('anchor','test_x','test_z'):
 l=ET.SubElement(u,'link',name=name);inert=ET.SubElement(l,'inertial');ET.SubElement(inert,'mass',value='.02');ET.SubElement(inert,'inertia',ixx='.00001',iyy='.00001',izz='.00001',ixy='0',ixz='0',iyz='0')
j=ET.SubElement(u,'joint',name='anchor_fixed',type='fixed');ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link='anchor')
for name,parent,child,xyz,axis in [('slide_x','anchor','test_x','0 0 .23','1 0 0'),('slide_z','test_x','test_z','0 0 0','0 0 1')]:
 j=ET.SubElement(u,'joint',name=name,type='prismatic');ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child);ET.SubElement(j,'origin',xyz=xyz);ET.SubElement(j,'axis',xyz=axis);ET.SubElement(j,'limit',lower='-.3',upper='.3',effort='100',velocity='.2')
for child in full:
 if child.tag=='link' and child.get('name','').startswith('gripper_'):u.append(copy.deepcopy(child))
 if child.tag=='joint' and child.get('name') in ('gripper_rh_r1','gripper_rh_r2','gripper_rh_l1','gripper_rh_l2'):u.append(copy.deepcopy(child))
ET.SubElement(u,'link',name='flange')
j=ET.SubElement(u,'joint',name='flange_fixed',type='fixed');ET.SubElement(j,'parent',link='test_z');ET.SubElement(j,'child',link='flange');ET.SubElement(j,'origin',rpy='0 3.141592653589793 0')
j=copy.deepcopy(full.find("joint[@name='gripper_attach_joint']"));j.find('parent').set('link','flange');u.append(j)
# physical_robot_sdf may calibrate gripper_attach_joint in exactly the same way as production.
sdf=ET.Element('sdf',version='1.11');w=ET.SubElement(sdf,'world',name='grasp_test');p=ET.SubElement(w,'physics',name='1ms',type='ignored');ET.SubElement(p,'max_step_size').text=args.step;ET.SubElement(p,'real_time_factor').text='1'
p=ET.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');ET.SubElement(ET.SubElement(p,'engine'),'filename').text=args.engine
ET.SubElement(w,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
ET.SubElement(w,'plugin',filename='gz-sim-scene-broadcaster-system',name='gz::sim::systems::SceneBroadcaster')
model=ET.fromstring(physical_robot_sdf(ET.tostring(u,encoding='unicode'))[0]).find('model')
for plugin in model.findall('plugin'):
 if plugin.findtext('joint_name') in ('slide_x','slide_z'):
  for tag,value in [('p_gain',10000),('i_gain',2000),('d_gain',100),('cmd_max',100),('cmd_min',-100),('i_max',20),('i_min',-20)]:plugin.find(tag).text=str(value)
if 'dartsim' in args.engine:
 for joint in model.findall('joint'):
  mimic=joint.find('axis/mimic')
  if mimic is not None:
   joint.find('axis').remove(mimic)
   controller=ET.SubElement(model,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
   for name,value in dict(joint_name=joint.get('name'),topic='/janggi_sim/physics/gripper_rh_r1/target',p_gain=50,i_gain=.1,d_gain=.3,cmd_max=2,cmd_min=-2,i_max=1,i_min=-1,use_velocity_commands='false').items():ET.SubElement(controller,name).text=str(value)
w.append(model)
height=piece_height_m(kind)
w.append(ET.fromstring('<model name="piece"><pose>0 0 '+str(-.02+height/2)+' 0 0 0</pose><link name="piece_body">'+piece_physics(kind)+'</link></model>'))
w.append(ET.fromstring('<model name="board"><static>true</static><pose>0 0 -.025 0 0 0</pose><link name="board"><collision name="board"><geometry><box><size>.4 .4 .01</size></box></geometry><surface><friction><bullet><friction>.8</friction><friction2>.8</friction2></bullet></friction></surface></collision></link></model>'))
if args.pad_p is not None:
 plugin=next(p for p in model.findall('plugin') if p.findtext('joint_name')=='gripper_rh_r1')
 plugin.find('p_gain').text=str(args.pad_p)
if args.pad_i is not None:
 plugin=next(p for p in model.findall('plugin') if p.findtext('joint_name')=='gripper_rh_r1')
 plugin.find('i_gain').text=str(args.pad_i)
if args.pad_effort is not None:
 plugin=next(p for p in model.findall('plugin') if p.findtext('joint_name')=='gripper_rh_r1')
 plugin.find('cmd_max').text=str(args.pad_effort);plugin.find('cmd_min').text=str(-args.pad_effort)
if args.full_width or args.collision_extra_mm:
 scale=w.find("model[@name='piece']/link/collision/geometry/mesh/scale")
 values=list(map(float,scale.text.split()));values[:2]=[v*(width+(2 if args.full_width else args.collision_extra_mm))/width for v in values[:2]];scale.text=' '.join(map(str,values))
if args.shape=='cylinder':
 geo=w.find("model[@name='piece']/link/collision/geometry")
 for child in list(geo):geo.remove(child)
 cylinder=ET.SubElement(geo,'cylinder');ET.SubElement(cylinder,'radius').text=str((width+2)/2000.);ET.SubElement(cylinder,'length').text=str(height)
sensor=ET.SubElement(w.find("model[@name='piece']/link"),'sensor',name='contacts',type='contact')
ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'topic').text='/piece_contacts';ET.SubElement(sensor,'update_rate').text='100'
contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='piece_contact';ET.SubElement(contact,'topic').text='/piece_contacts'
path=OUT/(args.label+'_'+kind+'.sdf');path.write_text(ET.tostring(sdf,encoding='unicode'))
node=Node();pub={n:node.advertise('/janggi_sim/physics/'+n+'/target',Double) for n in ('slide_x','slide_z','gripper_rh_r1')}
latest={};rows=[];contact_rows=[];contact_counts=Counter();pad_points={};pad_frames={}
for name,side,mesh in zip(LINKS,('right','left'),('r2','l2')):
 visual=full.find(f"link[@name='{name}']/visual")
 pad_points[name]=transform(inner_pad(ROOT/f'src/RH-P12-RN-A/rh_p12_rn_a_description/meshes/{mesh}.stl',[.001]*3,side),origin_T(visual.find('origin')))

def poses(msg):
 stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
 for p in msg.pose:
  name=p.name.split('::')[-1]
  if name in LINKS:pad_frames[name]=pose_matrix(p)
  if name in ('piece','piece_body','test_z','gripper_rh_p12_rn_r2','gripper_rh_p12_rn_l2'):latest[name]=(stamp,np.array([p.position.x,p.position.y,p.position.z]))
node.subscribe(Pose_V,'/world/grasp_test/dynamic_pose/info',poses)
def contacts(msg):
 for c in msg.contact:
  contact_counts[(globals().get('stage','startup'),c.collision1.name,c.collision2.name)]+=1
  if args.contact_trace:contact_rows.append(dict(a=c.collision1.name,b=c.collision2.name,stage=globals().get('stage','startup'),depth=list(c.depth),normals=[[n.x,n.y,n.z] for n in c.normal],positions=[[n.x,n.y,n.z] for n in c.position],wrench=str(c.wrench)[:900]))
node.subscribe(Contacts,'/piece_contacts',contacts)

closed=command_from_width(width);openc=command_from_width(opened)
# Match executor flange descent: maximum H during close/open is the closed value here.
low=(-18.+tcp_length(width))/1000.-.23
stages=[('open',0.,0.,openc,2.5),('descend',0.,low,openc,2.),('close',0.,low,closed,2.5),('lift',0.,low+.06,closed,2.),('hold_lift',0.,low+.06,closed,1.),('transport',.06,low+.06,closed,2.),('hold_transport',.06,low+.06,closed,1.),('release',.06,low+.06,openc,2.5)]
q=np.array([0.,0.,angle_from_stroke(0)]);snapshots={}
with (OUT/(args.label+'_'+kind+'.log')).open('w') as log:
 proc=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+40
  while 'piece' not in latest:
   if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No Gazebo physical poses; '+str(path))
   time.sleep(.02)
  for stage,x,z,c,duration in stages:
   start=latest['piece'][0];initial=q.copy();goal=np.array([x,z,angle_from_stroke(c)]);deadline=time.monotonic()+duration*10+15
   while (elapsed:=latest['piece'][0]-start)<duration:
    if time.monotonic()>deadline:raise RuntimeError('Simulation stalled')
    # Physical carriage moves slowly; jaws match the production 8c/20ms rate approximately.
    u=min(1.,elapsed/1.);blend=u*u*(3-2*u);q=initial+(goal-initial)*blend
    for name,value in zip(pub,q):pub[name].publish(Double(data=float(value)))
    rows.append(dict(stage=stage,sim_s=latest['piece'][0],piece=latest['piece'][1].tolist()))
    time.sleep(.004)
   q=goal
   if len(pad_frames)==2:
    actual=[transform(pad_points[n],pad_frames[n]) for n in LINKS]
    print('pads',measure_pads(*actual)[0],[(a[:,2].min()*1000,a[:,2].max()*1000) for a in actual],flush=True)
   snapshots[stage]={n:p.tolist() for n,(_,p) in latest.items()};print(stage,snapshots[stage].get('piece'),flush=True)
 finally:
  proc.terminate()
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
(OUT/(args.label+'_'+kind+'_contacts_summary.json')).write_text(json.dumps([dict(stage=k[0],a=k[1],b=k[2],samples=v) for k,v in contact_counts.items()],indent=2)+'\n')
if args.contact_trace:
 with gzip.open(OUT/(args.label+'_'+kind+'_contacts.json.gz'),'wt') as stream:json.dump(contact_rows,stream)

base=snapshots['open']['piece'];lift=snapshots['hold_lift']['piece'];carried=snapshots['hold_transport']['piece'];released=snapshots['release']['piece']
report=dict(kind=kind,label=args.label,partition=os.environ['GZ_PARTITION'],lift_mm=(lift[2]-base[2])*1000,transport_mm=(carried[0]-base[0])*1000,release_z_mm=released[2]*1000,snapshots=snapshots)
report['success']=report['lift_mm']>45 and abs(report['transport_mm']-60)<5 and abs(report['release_z_mm']-(-20+height*500))<2
(OUT/(args.label+'_'+kind+'.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='snapshots'}),flush=True)
