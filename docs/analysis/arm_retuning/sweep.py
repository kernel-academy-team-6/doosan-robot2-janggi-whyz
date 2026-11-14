"""Multi-pose isolated Gazebo sweep. No ROS imports or real robot connections."""
import argparse,copy,csv,gzip,json,math,os,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
P=argparse.ArgumentParser();P.add_argument('--label',default='sweep');P.add_argument('--source-jitter',action='store_true');P.add_argument('--trajectory',action='store_true');P.add_argument('--cubic',action='store_true');P.add_argument('--source-velocity',action='store_true');P.add_argument('--fast',action='store_true');P.add_argument('--final',action='store_true');P.add_argument('--continuous',action='store_true');args=P.parse_args()
os.environ['GZ_PARTITION']=f'janggi_retune_{os.getpid()}'
sys.path[:0]=[str(ROOT/'src/janggi/robot_game'),str(ROOT/'src/janggi')]
from physics_robot import physical_robot_sdf
from joint_targets import JointTargetBuffer
from gz_scene import origin_T,axis_rot
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double
urdf=ET.fromstring(subprocess.check_output(['xacro',str(ROOT/'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro')],text=True))
chain=[urdf.find(f"joint[@name='joint_{i}']") for i in range(1,7)]
def fk(q):
 t=np.eye(4)
 for joint,angle in zip(chain,q):
  t=t@origin_T(joint.find('origin'));r=np.eye(4);r[:3,:3]=axis_rot([float(v) for v in joint.find('axis').get('xyz').split()],angle);t=t@r
 return t
home=np.radians([0.,-13.694,134.216,0.,59.485,90.]);pose=fk(home)
lo=np.radians([-360,-360,-155,-360,-155,-270]);hi=-lo
cases=[('home',home,3.,1.2)]
# Return home between large single-joint moves, covering both wrist directions.
for axis,angle in [(0,40),(0,-40),(1,12),(1,-12),(2,12),(2,-18),(3,35),(3,-35),(4,20),(4,-20),(5,70),(5,-70)]:
 q=home.copy();q[axis]+=math.radians(angle);cases.extend([(f'J{axis+1}_{angle:+}',q,1.5,1.),('home',home,1.5,.8)])
# Fixed tool attitude Cartesian moves; IK solved offline from the measured URDF.
qprev=home.copy()
for name,xyz in [('X+60',( .06,0,0)),('XY_square',(.06,.06,0)),('Y+60',(0,.06,0)),('XY_home',(0,0,0)),('Z_down80',(0,0,-.08)),('Z_home',(0,0,0))]:
 target=pose.copy();target[:3,3]+=xyz
 def residual(q):
  actual=fk(q);return np.r_[actual[:3,3]-target[:3,3],.2*Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
 solved=least_squares(residual,qprev,bounds=(lo,hi),xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=500)
 if np.linalg.norm(residual(solved.x))>1e-6:raise RuntimeError('IK failed: '+name)
 cases.append((name,solved.x,1.5,1.));qprev=solved.x
cases.extend([('tool_home',np.radians([0,0,90,0,90,0]),2.,1.2),('home',home,2.,2.)])
profiles={
 'recent':[(2400,200,56),(3600,1200,84),(3000,1200,56),(1000,200,11.2),(600,200,5.6),(240,200,1.12)],
 'previous':[(1200,100,40),(1800,600,60),(1500,600,40),(500,100,8),(300,100,4),(120,100,.8)],
 'low_integral':[(1200,20,40),(1800,120,60),(1500,120,40),(500,20,8),(300,20,4),(120,20,.8)],
 'soft':[(800,40,28),(1200,300,40),(1000,300,28),(300,40,5),(200,40,2.8),(90,40,.55)],
}
if args.trajectory:
 profiles={'recent':profiles['recent'],'previous':profiles['previous'],'traj_previous':profiles['previous'],'traj_gravity':[(1000,10,30),(1500,20,45),(1200,20,30),(350,10,6),(250,10,3),(100,10,.7)]}
if args.final:
 profiles={'recent':profiles['recent'],'previous':profiles['previous'],'traj_gravity':profiles['traj_gravity'],'traj_gravity_tuned':[(1000,10,40),(1500,20,45),(1200,20,30),(350,10,6),(250,10,4.5),(120,10,1.4)]}
base=ET.fromstring(physical_robot_sdf(ET.tostring(urdf,encoding='unicode'))[0]).find('model')
world=ET.Element('sdf',version='1.11');w=ET.SubElement(world,'world',name='retune')
ph=ET.SubElement(w,'physics',name='1ms',type='ignored');ET.SubElement(ph,'max_step_size').text='.001';ET.SubElement(ph,'real_time_factor').text='1'
pl=ET.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');ET.SubElement(ET.SubElement(pl,'engine'),'filename').text='gz-physics-bullet-featherstone-plugin'
for i,(label,gains) in enumerate(profiles.items()):
 m=copy.deepcopy(base);m.set('name','arm_'+label);ET.SubElement(m,'pose').text=f'{i*2} 0 0 0 0 0'
 for plugin in list(m.findall('plugin')):
  if plugin.findtext('joint_name','').startswith('joint_'):m.remove(plugin);continue
  topic=plugin.find('topic')
  if topic is not None:topic.text=topic.text.replace('/janggi_sim/physics','/tune/'+label)
 if not label.startswith('traj_'):
  for j,(kp,ki,kd) in enumerate(gains,1):
   limit=194. if j<=3 else 66.
   controller=ET.SubElement(m,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
   fields=dict(joint_name=f'joint_{j}',topic=f'/tune/{label}/joint_{j}/target',use_velocity_commands='false',p_gain=kp,i_gain=ki,d_gain=kd,i_max=min(100.,limit),i_min=-min(100.,limit),cmd_max=limit,cmd_min=-limit)
   for name,value in fields.items():ET.SubElement(controller,name).text=str(value)
 if label.startswith('traj_'):
  for plugin in list(m.findall('plugin')):
   if plugin.findtext('joint_name','').startswith('joint_'):m.remove(plugin)
  controller=ET.SubElement(m,'plugin',filename='gz-sim-joint-trajectory-controller-system',name='gz::sim::systems::JointTrajectoryController')
  ET.SubElement(controller,'topic').text='/tune/'+label+'/trajectory'
  for j,(kp,ki,kd) in enumerate(gains,1):
   limit=194. if j<=3 else 66.
   fields=dict(joint_name=f'joint_{j}',position_p_gain=kp,position_i_gain=ki,position_d_gain=0.,position_i_min=-limit/2,position_i_max=limit/2,position_cmd_min=-limit*(.5 if 'gravity' in label else .75),position_cmd_max=limit*(.5 if 'gravity' in label else .75),velocity_p_gain=kd,velocity_i_gain=0.,velocity_d_gain=0.,velocity_cmd_min=-limit*(.15 if 'gravity' in label else .25),velocity_cmd_max=limit*(.15 if 'gravity' in label else .25))
   for name,value in fields.items():ET.SubElement(controller,name).text=str(value)
 for c in m.findall('.//collision'):
  surface=c.find('surface')
  if surface is None:surface=ET.SubElement(c,'surface')
  contact=surface.find('contact')
  if contact is None:contact=ET.SubElement(surface,'contact')
  ET.SubElement(contact,'collide_bitmask').text=str(1<<i)
 w.append(m)
path=OUT/(args.label+'.sdf');path.write_text(ET.tostring(world,encoding='unicode'))
rows={p:[] for p in profiles};latest={};active={'case':'startup','phase':'startup','source':np.zeros(6),'target':np.zeros(6)}
node=Node()
def receive(label,message):
 state={j.name:j for j in message.joint};names=[f'joint_{i}' for i in range(1,7)]
 if not all(n in state for n in names):return
 now=message.header.stamp.sec+message.header.stamp.nsec*1e-9;latest[label]=now
 q=np.array([state[n].axis1.position for n in names]);v=np.array([state[n].axis1.velocity for n in names])
 if started is not None:
  row={'sim_s':now,'case':active['case'],'phase':active['phase']}
  for i in range(6):row.update({f'q{i+1}':q[i],f'v{i+1}':v[i],f'target{i+1}':active['target'][i]})
  rows[label].append(row)
started=None
for label in profiles:node.subscribe(Model,'/tune/'+label+'/joint_states',lambda m,l=label:receive(l,m))
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
traj_pubs={label:node.advertise('/tune/'+label+'/trajectory',JointTrajectory) for label in profiles if label.startswith('traj_')}
pubs={label:[node.advertise(f'/tune/{label}/joint_{i}/target',Double) for i in range(1,7)] for label in profiles if not label.startswith('traj_')}
schedule=[];cursor=0.;last=np.zeros(6)
for name,q,duration,hold in cases:
 if args.fast:duration*=.75
 schedule.append((cursor,name,last.copy(),q.copy(),duration,hold));cursor+=duration+hold;last=q
from sim_dynamics import GravityModel
gravity=GravityModel(ET.tostring(urdf,encoding='unicode'))
buffer=JointTargetBuffer(delay_sec=.035 if args.source_velocity else .045 if args.cubic else .025,cubic=args.cubic,continuous=args.continuous);next_source=0.;last_print=-10
with (OUT/(args.label+'.log')).open('w') as log:
 process=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=log,stderr=log)
 try:
  deadline=time.monotonic()+90
  while len(latest)<len(profiles):
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No Gazebo state')
   time.sleep(.02)
  started=min(latest.values());deadline=time.monotonic()+cursor*5+30
  while (elapsed:=min(latest.values())-started)<cursor:
   if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Simulation stalled')
   item=next(x for x in reversed(schedule) if elapsed>=x[0]);at,name,q0,q1,duration,hold=item;u=min(1.,(elapsed-at)/duration);blend=u*u*u*(10+u*(-15+6*u));source=q0+(q1-q0)*blend;source_velocity=(q1-q0)*(30*u*u-60*u**3+30*u**4)/duration if u<1 else np.zeros(6)
   active['case']=f'{at:.1f}_{name}';active['phase']='move' if u<1 else ('settle' if elapsed-at-duration<.4 else 'hold');active['source']=source
   jitter=args.source_jitter and int(elapsed*10)%37==0
   if elapsed>=next_source and not jitter:buffer.add(dict(zip(range(6),source)),elapsed,dict(zip(range(6),source_velocity)) if args.source_velocity else None);next_source=elapsed+.02
   target,velocity=buffer.sample_state(elapsed)
   if target:
    active['target']=np.array([target[i] for i in range(6)])
    for publishers in pubs.values():
     for p,q in zip(publishers,active['target']):p.publish(Double(data=float(q)))
   if target:
    message=JointTrajectory();message.joint_names.extend([f'joint_{i}' for i in range(1,7)]);point=message.points.add();point.positions.extend(active['target']);point.velocities.extend([velocity[i] for i in range(6)])
    for label,publisher in traj_pubs.items():
     command=JointTrajectory();command.CopyFrom(message)
     if 'gravity' in label:
      compensation=gravity.torques(dict(zip(message.joint_names,active['target'])))
      command.points[0].effort.extend([float(np.clip(compensation[f'joint_{i}'],-(.35*(194 if i<=3 else 66)),.35*(194 if i<=3 else 66))) for i in range(1,7)])
     publisher.publish(command)
   if elapsed-last_print>10:print(f'{elapsed:.1f}/{cursor:.1f}s {name}',flush=True);last_print=elapsed
   time.sleep(.004)
 finally:
  process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()
report={'profiles':profiles,'duration_sim_s':cursor,'partition':os.environ['GZ_PARTITION'],'home_flange_m':pose[:3,3].tolist(),'results':{}}
for label,data in rows.items():
 with gzip.open(OUT/(args.label+'_'+label+'.csv.gz'),'wt') as f:
  writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
 q=np.array([[r[f'q{i}'] for i in range(1,7)] for r in data]);goal=np.array([[r[f'target{i}'] for i in range(1,7)] for r in data]);err=np.degrees(q-goal);move=np.array([r['phase']=='move' for r in data]);hold=np.array([r['phase']=='hold' for r in data]);v=np.array([[r[f'v{i}'] for i in range(1,7)] for r in data])
 per_case={}
 for case in dict.fromkeys(r['case'] for r in data):
  mask=np.array([r['case']==case and r['phase']=='hold' for r in data])
  if mask.any():per_case[case]={'hold_peak_deg':np.max(np.abs(err[mask]),axis=0).tolist(),'hold_swing_deg':np.degrees(np.ptp(q[mask],axis=0)).tolist(),'hold_speed_peak_deg_s':np.degrees(np.max(np.abs(v[mask]),axis=0)).tolist()}
 report['results'][label]={'samples':len(data),'moving_rms_deg':np.sqrt(np.mean(err[move]**2,axis=0)).tolist(),'moving_peak_deg':np.max(np.abs(err[move]),axis=0).tolist(),'hold_rms_deg':np.sqrt(np.mean(err[hold]**2,axis=0)).tolist(),'hold_peak_deg':np.max(np.abs(err[hold]),axis=0).tolist(),'cases':per_case}
(OUT/(args.label+'.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:{m:v for m,v in x.items() if m!='cases'} for k,x in report['results'].items()},indent=2))
