"""Dynamic URDF robot, driven by bounded joint torques, never entity pose updates."""
import subprocess
import tempfile
import time
from pathlib import Path
import xml.etree.ElementTree as ET

from gz_scene import GzRobotMirror, _resolve_uri
from gz.msgs10.double_pb2 import Double

MODEL_NAME = 'janggi_physics_robot'


def physical_robot_sdf(urdf):
    root = ET.fromstring(urdf)
    # The driver URDF can include ros2_control plugins. This model is controlled
    # exclusively by Gazebo's force PID; never start a second hardware interface.
    for child in list(root):
        if child.tag in ('ros2_control', 'gazebo'):
            root.remove(child)
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', _resolve_uri(mesh.get('filename')))
    # Independent force servos track all four finger linkage joints. Do not let
    # SDF mimic constraints prescribe their positions outside the physics solver.
    for joint in root.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)
    with tempfile.TemporaryDirectory(prefix='janggi_physics_') as folder:
        path = Path(folder)/'robot.urdf'
        path.write_text(ET.tostring(root, encoding='unicode'))
        converted = subprocess.run(['gz', 'sdf', '-p', str(path)], check=True,
                                   capture_output=True, text=True, timeout=30)
    sdf = ET.fromstring(converted.stdout)
    model = sdf.find('model')
    model.set('name', MODEL_NAME)
    ET.SubElement(model, 'static').text = 'false'
    ET.SubElement(model, 'self_collide').text = 'false'
    joints = []
    for joint in model.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic', 'continuous'):
            continue
        name = joint.get('name');joints.append(name)
        arm = name.startswith('joint_')
        effort_limit = min(194., float(joint.findtext('axis/limit/effort', '194'))) if arm else 2.
        plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-position-controller-system',
                               name='gz::sim::systems::JointPositionController')
        values = dict(joint_name=name, topic=f'/janggi_sim/physics/{name}/target',
                      use_velocity_commands='false', p_gain=1200 if arm else 5,
                      i_gain=30 if arm else .1, d_gain=60 if arm else .1,
                      i_max=100 if arm else 1, i_min=-100 if arm else -1,
                      cmd_max=effort_limit, cmd_min=-effort_limit)
        for tag, value in values.items():
            ET.SubElement(plugin, tag).text = str(value)
    state_plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                                 name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(state_plugin, 'topic').text = '/janggi_sim/physics/joint_states'
    for collision in model.findall('.//collision'):
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        friction = ET.SubElement(surface, 'friction')
        ode = ET.SubElement(friction, 'ode')
        ET.SubElement(ode, 'mu').text = '1.0'
        ET.SubElement(ode, 'mu2').text = '1.0'
    return ET.tostring(sdf, encoding='unicode'), joints


class PhysicsRobot(GzRobotMirror):
    """Emulator feedback supplies setpoints; Gazebo determines actual link motion."""
    def build(self):
        self._ros()
        # Remove old static mirror bodies on a scene restart, not while moving.
        self.scene._refresh_existing_models()
        for name in list(self.scene._existing_models):
            if name.startswith('rb_') or name == MODEL_NAME:
                self.scene._remove(name)
        sdf, self.controlled_joints = physical_robot_sdf(self.urdf)
        self.scene._create(sdf)
        self.publishers = {name:self.scene.node.advertise(f'/janggi_sim/physics/{name}/target', Double)
                           for name in self.controlled_joints}

    def sync(self):
        # No zero substitution for missing arm feedback.
        if not all(f'joint_{i}' in self.q for i in range(1,7)):
            return
        for name, publisher in self.publishers.items():
            if name in self.q:
                publisher.publish(Double(data=self.q[name]))
