"""Command-only adapter to the existing gripper_service_node position topic."""
import time

from std_msgs.msg import Int32
from janggi_pick_place2.geometry import command_from_width


class TestGripper:
    __test__ = False

    def __init__(self, node, prefix, parameters):
        self.p = parameters
        self.topic = prefix+'/gripper/position_cmd'
        self.publisher = node.create_publisher(Int32, self.topic, 10)
        self.last_command_u = None

    def prepare(self, check):
        deadline = time.monotonic()+self.p['service_timeout_sec']
        while self.publisher.get_subscription_count() == 0:
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'no subscriber on {self.topic}; start existing gripper bringup')
            time.sleep(.02)
        check()

    def initialize(self, width_mm, check):
        target = command_from_width(width_mm)
        self.begin(target, lambda _stamp: None, check)
        deadline = time.monotonic()+self.p['gripper_command_wait_sec']
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.02, max(0., deadline-time.monotonic())))
        check()
        # This is a commanded endpoint, never a measured gripper position.
        return target

    def begin(self, target_u, hook, check):
        if not isinstance(target_u, int) or not 0 <= target_u <= 700:
            raise ValueError('position command must be an integer in [0, 700]')
        self.prepare(check)
        stamp = time.monotonic()
        self.publisher.publish(Int32(data=target_u))
        self.last_command_u = target_u
        hook(stamp)  # PC topic publication estimate, not motor onset.

    def close_session(self):
        # Existing gripper_service_node owns the serial port.
        pass
