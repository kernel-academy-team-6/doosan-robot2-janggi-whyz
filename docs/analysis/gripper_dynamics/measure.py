"""Read gripper tracking in an isolated Gazebo world; never commands ROS robots."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
# Set before importing Transport: this test cannot address the user's world.
os.environ['GZ_PARTITION'] = f'janggi_gripper_diagnostic_{os.getpid()}'
sys.path.insert(0, str(ROOT / 'src/janggi/robot_game'))
sys.path.insert(0, str(ROOT / 'src/janggi'))
from physics_robot import physical_robot_sdf
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double

full = ET.fromstring(subprocess.check_output([
    'xacro', str(ROOT / 'src/e0509_gripper_description/urdf/e0509_with_gripper.urdf.xacro')
], text=True))
robot = ET.Element('robot', name='gripper_test')
ET.SubElement(robot, 'link', name='world')
for child in full:
    if child.tag in ('link', 'joint') and child.get('name', '').startswith('gripper_') and child.get('name') != 'gripper_attach_joint':
        robot.append(child)
anchor = ET.SubElement(robot, 'joint', name='world_fixed', type='fixed')
ET.SubElement(anchor, 'parent', link='world')
ET.SubElement(anchor, 'child', link='gripper_rh_p12_rn_base')
ET.SubElement(anchor, 'origin', xyz='0 0 0.3', rpy='0 3.141592653589793 0')
sdf, joints = physical_robot_sdf(ET.tostring(robot, encoding='unicode'))
world = ET.Element('sdf', version='1.9')
w = ET.SubElement(world, 'world', name='gripper_diagnostic')
physics = ET.SubElement(w, 'physics', name='1ms', type='ignored')
ET.SubElement(physics, 'max_step_size').text = '0.001'
ET.SubElement(physics, 'real_time_factor').text = '1'
ET.SubElement(w, 'plugin', filename='gz-sim-physics-system', name='gz::sim::systems::Physics')
w.append(ET.fromstring(sdf).find('model'))
world_path = OUT / 'isolated_world.sdf'
world_path.write_text(ET.tostring(world, encoding='unicode'))

rows = []
started = None
target = 0.0
def callback(msg):
    if started is not None:
        row = {'wall_sec': time.monotonic()-started, 'target_rad': target}
        row.update({j.name: j.axis1.position for j in msg.joint if j.name in joints})
        rows.append(row)

node = Node()
node.subscribe(Model, '/janggi_sim/physics/joint_states', callback)
publishers = [node.advertise(f'/janggi_sim/physics/{j}/target', Double) for j in joints]
with (OUT / 'server.log').open('w') as log:
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world_path)], stdout=log, stderr=log)
    try:
        time.sleep(3)
        started = time.monotonic()
        while time.monotonic()-started < 6:
            elapsed = time.monotonic()-started
            target = min(.8, max(0., (elapsed-1)*.8/.3))
            for publisher in publishers:
                publisher.publish(Double(data=target))
            time.sleep(.02)
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
if not rows:
    raise RuntimeError('No isolated Gazebo joint feedback; inspect server.log')
with (OUT / 'tracking.csv').open('w') as f:
    writer = csv.DictWriter(f, fieldnames=['wall_sec', 'target_rad']+joints)
    writer.writeheader()
    writer.writerows(rows)
tail = [r for r in rows if r['wall_sec'] > 5]
summary = {'partition': os.environ['GZ_PARTITION'], 'samples': len(rows), 'target_rad': .8,
           'final_mean_rad': {j: sum(r[j] for r in tail)/len(tail) for j in joints},
           'max_pair_difference_rad': max(max(r[j] for j in joints)-min(r[j] for j in joints) for r in rows),
           'final_pair_difference_rad': max(max(r[j] for j in joints)-min(r[j] for j in joints) for r in tail)}
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
