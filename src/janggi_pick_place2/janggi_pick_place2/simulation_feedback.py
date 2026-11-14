"""Gazebo physical-joint completion; never subscribes to hardware serial results."""
import math
import threading
import time
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from .simulation_geometry import cad_gap_mm


class SimulationFeedback:
    def __init__(self, node, prefix, timeout=120.):
        self.timeout = timeout
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
        deadline=time.monotonic()+self.timeout
        settled=None
        while True:
            check()
            with self._condition:
                sample=self._sample
                if sample is not None and time.monotonic()-sample[3]<1.:
                    stamp,q,v,_=sample
                    if reached(q,v):
                        if settled is None:settled=stamp
                        if stamp-settled>=.10:return
                    else:settled=None
                else:settled=None
                if time.monotonic()>=deadline:
                    raise TimeoutError('Gazebo physical motion did not settle; check simulation pause / physics feedback')
                self._condition.wait(.02)

    def arm(self, joints_deg, check):
        goal={f'joint_{i}':math.radians(value) for i,value in enumerate(joints_deg,1)}
        def reached(q,v):
            return all(n in q and n in v and abs(q[n]-target)<math.radians(.35)
                       and abs(v[n])<math.radians(2.) for n,target in goal.items())
        self.wait(reached,check)

    def gripper(self, width_mm, check):
        names=('gripper_rh_r1','gripper_rh_l1')
        def reached(q,v):
            if not all(n in q and n in v for n in names):return False
            # A held object can stop the jaws before the unloaded width target.
            gap=cad_gap_mm(sum(q[n] for n in names)/2.)
            return abs(gap-width_mm)<3. and all(abs(v[n])<.02 for n in names)
        self.wait(reached,check)
