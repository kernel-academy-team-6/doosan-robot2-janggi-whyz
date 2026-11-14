"""Compare skins above three measured body sizes in an isolated render world."""
import os,sys,subprocess,threading,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
os.environ['GZ_PARTITION']=f'janggi_skin_check_{os.getpid()}'
sys.path.insert(0,str(ROOT/'src/janggi/robot_game'))
from piece_visuals import piece_visuals,piece_height_m
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage
models=[]
all_pieces='--all-pieces' in sys.argv
for row,lift in enumerate([.0001,.001]):
 for col,kind in enumerate(['sa','cha','king']):
  height=piece_height_m(kind)
  visuals=ET.fromstring('<root>'+piece_visuals('red',kind)+'</root>')
  visuals.find("visual[@name='photo_top']/pose").text=f'0 0 {height/2+lift} 0 0 0'
  models.append(f'<model name="piece_{row}_{col}"><static>true</static><pose>{(row-.5)*.07} {(col-1)*.06} {height/2} 0 0 0</pose><link name="l">'+''.join(ET.tostring(v,encoding='unicode') for v in visuals)+'</link></model>')
if all_pieces:
 models=[]
 manifest=json.loads((ROOT/'src/janggi/robot_game/assets/pieces/photo_instances.json').read_text())
 for i,(key,entry) in enumerate(manifest.items()):
  side,kind=entry['label'].split('_',1);height=piece_height_m(kind)
  models.append(f'<model name="{key}"><static>true</static><pose>{(i//8-1.5)*.05} {(i%8-3.5)*.05} {height/2} 0 0 0</pose><link name="l">'+piece_visuals(side,kind,texture_key=key)+'</link></model>')
world='''<sdf version="1.11"><world name="skin_check"><physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><scene><ambient>.7 .7 .7 1</ambient><background>.5 .5 .5 1</background><shadows>false</shadows></scene><light name="sun" type="directional"><direction>0 0 -1</direction><diffuse>1 1 1 1</diffuse></light><model name="camera"><static>true</static><pose>0 0 .25 0 1.57079632679 0</pose><link name="l"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>5</update_rate><topic>/skin_check/image</topic><camera><horizontal_fov>.9</horizontal_fov><image><width>1024</width><height>768</height><format>R8G8B8</format></image><clip><near>.01</near><far>2</far></clip></camera></sensor></link></model>'''+''.join(models)+'</world></sdf>'
if all_pieces:world=world.replace('0 0 .25 0','0 0 .5 0').replace('<width>1024</width><height>768</height>','<width>1600</width><height>900</height>')
path=OUT/'render_world.sdf';path.write_text(world)
ready=threading.Event();images=[]
def callback(m):
 if len(images)<4:
  images.append((m.width,m.height,m.step,bytes(m.data)))
  if len(images)==4:ready.set()
node=Node();node.subscribe(GzImage,'/skin_check/image',callback)
with (OUT/'render.log').open('w') as log:
 server=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(path)],stdout=log,stderr=log)
 try:
  if not ready.wait(30):raise RuntimeError('No render frames; inspect render.log')
  w,h,step,data=images[-1];a=np.frombuffer(data,np.uint8).reshape(h,step)[:,:w*3].reshape(h,w,3)
  Image.fromarray(a).save(OUT/('all_32_skins.png' if all_pieces else 'skin_comparison.png'));print('Rendered',w,h)
 finally:
  server.terminate()
  try:server.wait(timeout=5)
  except subprocess.TimeoutExpired:server.kill();server.wait()
