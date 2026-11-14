"""Gazebo physical-joint completion; never subscribes to hardware serial results."""
import math
import threading
import time
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from .simulation_geometry import cad_gap_mm


class SimulationFeedback:
    def __init__(self, node, prefix, timeout=120., *, position_deg=1.0, velocity_deg_s=5.0,
                 hold_sec=.05, gripper_mm=5.0, gripper_velocity_rad_s=.06, max_wait_sec=3.0):
        # Gazebo-only completion tolerances (real hardware never uses this class).
        # The physics arm converges slowly at low RTF; millimetre-level arrival is
        # not achievable there, so these are looser than the old 0.35 deg / 2 deg/s.
        # max_wait_sec: wall-clock cap on settling while Gazebo feedback is live. After it the
        # motion is accepted as-is (Gazebo only). Missing feedback still raises after `timeout`.
        values = (timeout, position_deg, velocity_deg_s, gripper_mm, gripper_velocity_rad_s, max_wait_sec)
        if not all(math.isfinite(v) and v > 0 for v in values) or not math.isfinite(hold_sec) or hold_sec < 0:
            raise ValueError('simulation settle tolerances must be positive and finite')
        self.timeout = timeout
        self.position_rad, self.velocity_rad_s = math.radians(position_deg), math.radians(velocity_deg_s)
        self.hold_sec, self.gripper_mm, self.gripper_velocity_rad_s = hold_sec, gripper_mm, gripper_velocity_rad_s
        self.max_wait_sec = max_wait_sec
        self.last_wait = None  # {'elapsed_sec', 'capped'} of the most recent wait
        self._condition = threading.Condition()
        self._sample = None
        self.subscription = node.create_subscription(
            JointState, prefix+'/gazebo/joint_states', self.receive, 10,
            callback_group=ReentrantCallbackGroup())

    def receive(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        q = dict(zip(message.name,message.position))
        v = dict(zip(message.name,message.velocity))
        with self._condition:
            self._sample = (stamp,q,v,time.monotonic())
            self._condition.notify_all()

    def wait(self, reached, check):
        start=time.monotonic()
        deadline=start+self.timeout
        settled=None
        while True:
            check()
            with self._condition:
                sample=self._sample
                now=time.monotonic()
                if sample is not None and now-sample[3]<1.:
                    stamp,q,v,_=sample
                    if reached(q,v):
                        if settled is None:settled=stamp
                        if stamp-settled>=self.hold_sec:
                            self.last_wait=dict(elapsed_sec=now-start,capped=False);return
                    else:settled=None
                    if now-start>=self.max_wait_sec:
                        # Gazebo is alive but still converging: do not stall the game.
                        self.last_wait=dict(elapsed_sec=now-start,capped=True);return
                else:settled=None
                if now>=deadline:
                    raise TimeoutError('Gazebo physical motion did not settle; check simulation pause / physics feedback')
                self._condition.wait(.02)

    def arm(self, joints_deg, check):
        goal={f'joint_{i}':math.radians(value) for i,value in enumerate(joints_deg,1)}
        def reached(q,v):
            return all(n in q and n in v and abs(q[n]-target)<self.position_rad
                       and abs(v[n])<self.velocity_rad_s for n,target in goal.items())
        self.wait(reached,check)

    def gripper(self, width_mm, check):
        names=('gripper_rh_r1','gripper_rh_l1')
        def reached(q,v):
            if not all(n in q and n in v for n in names):return False
            # A held object can stop the jaws before the unloaded width target.
            gap=cad_gap_mm(sum(q[n] for n in names)/2.)
            return abs(gap-width_mm)<self.gripper_mm and all(abs(v[n])<self.gripper_velocity_rad_s for n in names)
        self.wait(reached,check)
