"""Dynamic URDF robot, driven by bounded joint torques, never entity pose updates."""
import subprocess
import tempfile
import time
from pathlib import Path
import xml.etree.ElementTree as ET

from gz_scene import GzRobotMirror, _resolve_uri
from gz.msgs10.double_pb2 import Double
from joint_targets import HeldStreamSampler, JointTargetBuffer
from gripper_gap import FINGER_JOINTS, angle_from_ros_target, calibrated_mount_mm
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
from gz.msgs10.model_pb2 import Model
from sim_dynamics import GravityModel
from collision_masks import ARM_CATEGORY, ARM_COLLIDE, ARM_LINKS

MODEL_NAME = 'janggi_physics_robot'

# Gazebo native force controller: position PI + measured-velocity damping.
# Lower stiffness than prior retuning; D(error) is disabled to avoid target kicks.
ARM_GAINS = {
    'joint_1': (600., 5., 35.), 'joint_2': (900., 10., 50.),
    'joint_3': (700., 10., 30.), 'joint_4': (150., 2., 4.),
    'joint_5': (120., 2., 3.), 'joint_6': (50., 1., 1.),
}
ARM_TOPIC = '/janggi_sim/physics/arm/trajectory'
JOINT_STATE_RATE_HZ = 250
# Worst-case sum of the three independent command budgets cannot exceed URDF effort.
POSITION_BUDGET, VELOCITY_BUDGET, GRAVITY_BUDGET = .5, .15, .35
# Official E0509 rated axis speeds, converted from degrees/s to radians/s.
ARM_SPEEDS = (2.094395102393, 2.094395102393, 2.617993877991,
              3.926990816987, 3.926990816987, 3.926990816987)



def calibrated_robot_urdf(urdf):
    root = ET.fromstring(urdf)
    # The command TCP model is flange based, whereas the supplied CAD is shorter.
    # Calibrate the physical simulation adapter rather than modifying real URDF,
    # command coordinates, or probe/camera calibration. One fixed adapter works
    # for the whole opening arc (residual <0.12 mm).
    attachment = root.find("joint[@name='gripper_attach_joint']/origin")
    if attachment is not None:
        xyz = [float(v) for v in attachment.get('xyz', '0 0 0').split()]
        xyz[2] = calibrated_mount_mm()/1000.
        attachment.set('xyz', ' '.join(map(str, xyz)))
    return ET.tostring(root, encoding='unicode')


def physical_robot_sdf(urdf):
    root = ET.fromstring(calibrated_robot_urdf(urdf))
    # The driver URDF can include ros2_control plugins. This model is controlled
    # exclusively by Gazebo's force PID; never start a second hardware interface.
    for child in list(root):
        if child.tag in ('ros2_control', 'gazebo'):
            root.remove(child)
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', _resolve_uri(mesh.get('filename')))
    # Preserve the visible CAD, but use its exact flat pad dimensions for contact.
    # A convex hull of the entire concave distal STL is not a rubber pad surface.
    for side, sign in (('r', 1), ('l', -1)):
        link = root.find(f"link[@name='gripper_rh_p12_rn_{side}2']")
        if link is None:
            continue
        for collision in list(link.findall('collision')):
            link.remove(collision)
        for name, xyz, size in (
            ('pad', (0., sign*.0011, .030), (.026, .010, .020)),
            ('back', (0., sign*.011, .002612835), (.026, .014, .03477433)),
        ):
            collision = ET.SubElement(link, 'collision', name=name)
            ET.SubElement(collision, 'origin', xyz=' '.join(map(str, xyz)))
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str, size)))
    # Apply E0509 specification corrections to the actual Gazebo model only.
    # Keep upstream driver files / its hardware interface unchanged.
    if all(root.find(f"joint[@name='joint_{i}']") is not None for i in range(1, 7)):
        for i, speed in enumerate(ARM_SPEEDS, 1):
            limit = root.find(f"joint[@name='joint_{i}']/limit")
            limit.set('velocity', str(speed))
            if i == 5:
                limit.set('lower', '-2.705260340591')
                limit.set('upper', '2.705260340591')
    # URDF conversion can discard mimic tags. Recreate SDF 1.11 constraints
    # explicitly below; the launch selects Bullet Featherstone to solve them.
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
    sdf.set('version', '1.11')
    # Bullet's untreated triangle meshes have a 10 mm collision margin in
    # gz-physics7. Convex hulls use 1 mm; retain visual meshes unchanged.
    for mesh in model.findall('.//collision/geometry/mesh'):
        mesh.set('optimization', 'convex_decomposition')
        decomposition = ET.SubElement(mesh, 'convex_decomposition')
        ET.SubElement(decomposition, 'max_convex_hulls').text = '1'
        ET.SubElement(decomposition, 'voxel_resolution').text = '10000'
    ET.SubElement(model, 'static').text = 'false'
    ET.SubElement(model, 'self_collide').text = 'false'
    joints = []
    fingers = {'gripper_rh_r1', 'gripper_rh_r2', 'gripper_rh_l1', 'gripper_rh_l2'}
    has_linkage = fingers.issubset({j.get('name') for j in model.findall('joint')})
    arm_limits = {}
    for joint in model.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic', 'continuous'):
            continue
        name = joint.get('name');joints.append(name)
        arm = name.startswith('joint_')
        if has_linkage and name in fingers - {'gripper_rh_r1'}:
            mimic = ET.SubElement(joint.find('axis'), 'mimic', joint='gripper_rh_r1', axis='axis')
            ET.SubElement(mimic, 'multiplier').text = '1'
            continue  # native physics constraint; no follower motor
        # Arm limits remain upstream URDF budgets, not verified peak ratings.
        # A softer 0.3 Nm finger drive avoids exciting the mimic/contact solver
        # on 3-9 g pieces. This is a simulation tuning value, not rated torque.
        effort_limit = min(194., float(joint.findtext('axis/limit/effort', '194'))) if arm else .3
        if arm:
            arm_limits[name] = effort_limit
            continue
        p_gain, i_gain, d_gain = (10., .1, .3)
        plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-position-controller-system',
                               name='gz::sim::systems::JointPositionController')
        values = dict(joint_name=name, topic=f'/janggi_sim/physics/{name}/target',
                      use_velocity_commands='false', p_gain=p_gain,
                      i_gain=i_gain, d_gain=d_gain,
                      i_max=min(100.,effort_limit) if arm else .15,
                      i_min=-min(100.,effort_limit) if arm else -.15,
                      cmd_max=effort_limit, cmd_min=-effort_limit)
        for tag, value in values.items():
            ET.SubElement(plugin, tag).text = str(value)
    if arm_limits:
        controller = ET.SubElement(model, 'plugin', filename='gz-sim-joint-trajectory-controller-system',
                                   name='gz::sim::systems::JointTrajectoryController')
        ET.SubElement(controller, 'topic').text = ARM_TOPIC
        for name, limit in arm_limits.items():
            kp, ki, kv = ARM_GAINS[name]
            values = dict(joint_name=name, position_p_gain=kp, position_i_gain=ki,
                          position_d_gain=0., position_i_min=-limit/2, position_i_max=limit/2,
                          position_cmd_min=-limit*POSITION_BUDGET, position_cmd_max=limit*POSITION_BUDGET,
                          velocity_p_gain=kv, velocity_i_gain=0., velocity_d_gain=0.,
                          velocity_cmd_min=-limit*VELOCITY_BUDGET, velocity_cmd_max=limit*VELOCITY_BUDGET)
            for tag,value in values.items():
                ET.SubElement(controller,tag).text=str(value)
    state_plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                                 name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(state_plugin, 'topic').text = '/janggi_sim/physics/joint_states'
    # Default is every 0.5 ms physics step (2 kHz): one Python core was busy just
    # decoding it. Consumers are the 200 Hz target loop and 50 Hz ROS feedback.
    ET.SubElement(state_plugin, 'update_rate').text = str(JOINT_STATE_RATE_HZ)
    arm_collisions = {id(c) for link in model.findall('link') if link.get('name') in ARM_LINKS
                      for c in link.findall('collision')}
    for collision in model.findall('.//collision'):
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        if id(collision) in arm_collisions:
            # Arm body vs pieces is filtered in the broadphase (see collision_masks).
            contact = surface.find('contact')
            if contact is None:
                contact = ET.SubElement(surface, 'contact')
            ET.SubElement(contact, 'collide_bitmask').text = str(ARM_COLLIDE)
            ET.SubElement(contact, 'category_bitmask').text = str(ARM_CATEGORY)
        friction = ET.SubElement(surface, 'friction')
        # Bullet defaults spinning friction to 1 m: inappropriate for millimetre
        # plastic parts and rubber pads. Keep ordinary Coulomb friction below.
        ET.SubElement(ET.SubElement(friction, 'torsional'), 'coefficient').text = '0'
        ode = ET.SubElement(friction, 'ode')
        ET.SubElement(ode, 'mu').text = '1.0'
        ET.SubElement(ode, 'mu2').text = '1.0'
        bullet = ET.SubElement(friction, 'bullet')
        ET.SubElement(bullet, 'friction').text = '1.0'
        ET.SubElement(bullet, 'friction2').text = '1.0'
    return ET.tostring(sdf, encoding='unicode'), joints


def arm_command(goals, velocities, gravity, limits):
    """One coherent six-axis force-controller reference; no ROS motion calls."""
    names=list(limits)
    if not names or not all(n in goals for n in names):
        return None
    compensation=gravity.torques(goals)
    message=JointTrajectory();message.joint_names.extend(names)
    point=message.points.add()
    for name in names:
        point.positions.append(goals[name])
        speed=ARM_SPEEDS[int(name.split('_')[1])-1]
        point.velocities.append(max(-speed,min(speed,velocities.get(name,0.))))
        budget=limits[name]*GRAVITY_BUDGET
        point.effort.append(max(-budget,min(budget,compensation.get(name,0.))))
    return message


class PhysicsRobot(GzRobotMirror):
    """Emulator feedback supplies setpoints; Gazebo determines actual link motion."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.target_stream = HeldStreamSampler()
        self.target_buffer = JointTargetBuffer(delay_sec=self.target_stream.delay_sec, cubic=True,
                                               continuous=True, max_samples=8192)
        self.physics_state = None
        self._sim_previous = self._target_time = None
        self._feedback_required = False
        self._last_feedback_publish = -1.
        self._arm_only_seen = None

    def _on_js(self, message):
        now = time.monotonic()
        # /joint_states mixes joint_state_broadcaster (arm only, fresh) and
        # gripper_joint_publisher (arm copied from dynamic_joint_states on its own
        # timer, plus fingers). While the arm-only source is alive, its copies are
        # ignored: they arrive just before the next fresh sample and can be older.
        fingers = any(name in FINGER_JOINTS for name in message.name)
        if not fingers and any(name.startswith('joint_') for name in message.name):
            self._arm_only_seen = now
        use_arm = not fingers or self._arm_only_seen is None or now-self._arm_only_seen > .5
        for name,value in zip(message.name,message.position):
            if name in FINGER_JOINTS:
                # Only fresh gripper entries are decoded, so arm-only messages cannot
                # convert an already calibrated angle twice. Real ROS/serial unchanged.
                self.q[name]=angle_from_ros_target(value)
            elif use_arm or not name.startswith('joint_'):
                self.q[name]=value
        arm = {f'joint_{i}':self.q.get(f'joint_{i}') for i in range(1, 7)}
        if use_arm and None not in arm.values():
            self.target_stream.add(self.target_buffer, arm, now)
            self.target_buffer.delay_sec = self.target_stream.delay_sec

    def build(self):
        self._ros()
        # Remove old static mirror bodies on a scene restart, not while moving.
        self.scene._refresh_existing_models()
        for name in list(self.scene._existing_models):
            if name.startswith('rb_') or name == MODEL_NAME:
                self.scene._remove(name)
        sdf, self.controlled_joints = physical_robot_sdf(self.urdf)
        self.gravity = GravityModel(calibrated_robot_urdf(self.urdf))
        xml = ET.fromstring(self.urdf)
        self.arm_limits = {n:min(194.,float(xml.find(f"joint[@name='{n}']/limit").get('effort')))
                           for n in self.controlled_joints if n.startswith('joint_')}
        self.arm_publisher = self.scene.node.advertise(ARM_TOPIC, JointTrajectory)
        from sensor_msgs.msg import JointState
        self.feedback_publisher = self.node.create_publisher(JointState,f'/{self.ns}/gazebo/joint_states',10)
        self._feedback_required = True
        self.scene.node.subscribe(Model,'/janggi_sim/physics/joint_states',self._physics_feedback)
        self.scene._create(sdf)
        self.publishers = {name:self.scene.node.advertise(f'/janggi_sim/physics/{name}/target', Double)
                           for name in self.controlled_joints
                           if name == 'gripper_rh_r1'}

    def _physics_feedback(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
        q = {j.name:j.axis1.position for j in message.joint}
        v = {j.name:j.axis1.velocity for j in message.joint}
        self.physics_state = (stamp, q, v)
        if hasattr(self, 'feedback_publisher') and stamp-self._last_feedback_publish >= .02:
            from sensor_msgs.msg import JointState
            msg=JointState();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
            msg.name=list(q);msg.position=[q[n] for n in msg.name];msg.velocity=[v[n] for n in msg.name]
            self.feedback_publisher.publish(msg)
            self._last_feedback_publish=stamp

    def reference_time(self, wall_now):
        """Replay emulator wall-time paths at simulation speed, including low RTF.

        Pausing / slow rendering must not compress a seconds-long arm trajectory
        into fewer physics seconds. End-of-motion physical feedback gates the UI.
        """
        if self.physics_state is None:
            return None if self._feedback_required else wall_now
        sim_now=self.physics_state[0]
        if self._target_time is None:
            self._target_time=wall_now
        elif self._sim_previous is not None:
            self._target_time += max(0.,sim_now-self._sim_previous)
        self._sim_previous=sim_now
        return self._target_time

    def sync(self):
        if not all(f'joint_{i}' in self.q for i in range(1,7)):
            return
        wall_now=time.monotonic()
        now=self.reference_time(wall_now)
        if now is None:
            return
        goals, velocities = self.target_buffer.sample_state(now)
        if self.physics_state is not None and wall_now-now>.1:
            _,actual,speed=self.physics_state
            names=[f'joint_{i}' for i in range(1,7)]
            settled=all(n in actual and n in speed and abs(actual[n]-self.q[n])<.0035
                        and abs(speed[n])<.02 for n in names)
            if settled and self.target_buffer.align_stationary_tail(wall_now,names):
                self._target_time=wall_now
                goals,velocities=self.target_buffer.sample_state(wall_now)
        goals.update({n:q for n,q in self.q.items() if not n.startswith('joint_')})
        command=arm_command(goals,velocities,self.gravity,self.arm_limits)
        if command is not None:
            self.arm_publisher.publish(command)
        for name,publisher in self.publishers.items():
            if name in goals:
                publisher.publish(Double(data=goals[name]))

    def close(self):
        if hasattr(self,'arm_publisher'):
            goals=self.target_buffer.sample(self._target_time or time.monotonic())
            goals.update({n:q for n,q in self.q.items() if not n.startswith('joint_')})
            command=arm_command(goals,{},self.gravity,self.arm_limits)
            if command is not None:self.arm_publisher.publish(command)
        if self._feedback_required:
            self.scene.node.unsubscribe('/janggi_sim/physics/joint_states')
        super().close()
