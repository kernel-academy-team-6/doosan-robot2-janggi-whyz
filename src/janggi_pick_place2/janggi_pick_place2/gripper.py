"""Reuse the existing, working position_cmd path; no direct gripper serial I/O."""
import json
import time

from std_msgs.msg import Int32, String
from .geometry import command_from_width


def command_state(width_mm, current_ma, source):
    return {
        'source': source, 'feedback_received': False, 'grip_verified': False,
        'commanded_position': command_from_width(width_mm),
        'commanded_width_mm': width_mm, 'requested_current_ma': int(current_ma),
        'current_limit_command_sent': False, 'current_limit_verified': False,
    }


class GripperClient:
    def __init__(self, node, prefix, parameters):
        self.node, self.p = node, parameters
        self.last_sample = None
        self.last_command = None
        self.topic = prefix+'/gripper/position_cmd'
        self.position_cmd = node.create_publisher(Int32, self.topic, 10)
        self.state = node.create_publisher(String, '~/gripper/state', 10)

    def _sleep(self, seconds, check):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.02, max(0., deadline-time.monotonic())))
        check()

    def prepare(self, check):
        deadline = time.monotonic()+self.p['service_timeout_sec']
        while self.position_cmd.get_subscription_count() == 0:
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'no subscriber on {self.topic}; start existing gripper bringup first')
            time.sleep(.02)
        check()

    def _command(self, width_mm, current_ma, check):
        position = command_from_width(width_mm)
        self.prepare(check)
        self.last_command = command_state(width_mm, current_ma, 'position_cmd')
        self.last_command['stamp_ns'] = self.node.get_clock().now().nanoseconds
        self.last_command['wait_completed'] = False
        # The existing gripper_service_node performs torque enable + goal write.
        # It owns the serial port and stroke animation. We publish exactly once.
        self.position_cmd.publish(Int32(data=position))
        self.state.publish(String(data=json.dumps(self.last_command)))
        self._sleep(self.p['gripper_command_wait_sec'], check)
        self.last_command['wait_completed'] = True
        self.state.publish(String(data=json.dumps(self.last_command)))
        return dict(self.last_command)

    def open_to(self, width_mm, current_ma, check):
        return self._command(width_mm, current_ma, check)

    def grasp(self, grasp, check):
        return self._command(grasp.close_width_mm, grasp.current_ma, check)

    def close_session(self):
        # Existing gripper_service_node owns its serial lifecycle.
        pass


class MockGripper:
    def __init__(self):
        self.calls = []
        self.last_sample = None
        self.last_command = None

    def prepare(self, check):
        check()

    def open_to(self, width_mm, current_ma, check):
        check()
        self.calls.append(('open', width_mm, command_from_width(width_mm), current_ma))
        self.last_command = command_state(width_mm, current_ma, 'mock_command_only')
        self.last_command['wait_completed'] = True
        return dict(self.last_command)

    def grasp(self, grasp, check):
        check()
        self.calls.append(('grasp', grasp.close_width_mm,
                           command_from_width(grasp.close_width_mm), grasp.current_ma))
        self.last_command = command_state(grasp.close_width_mm, grasp.current_ma, 'mock_command_only')
        self.last_command['wait_completed'] = True
        return dict(self.last_command)

    def close_session(self):
        pass


class VirtualGripperClient(MockGripper):
    """Commanded-gap animation without claiming physical contact."""
    def __init__(self, node, prefix, delay_sec):
        super().__init__()
        self.node, self.delay_sec = node, delay_sec
        self.stroke = node.create_publisher(Int32, prefix+'/gripper/stroke', 10)
        self.state = node.create_publisher(String, '~/gripper/state', 10)

    def _show(self, check):
        check()
        self.last_command['source'] = 'virtual_command_only'
        self.last_command['stamp_ns'] = self.node.get_clock().now().nanoseconds
        self.last_command['wait_completed'] = False
        self.stroke.publish(Int32(data=self.last_command['commanded_position']))
        self.state.publish(String(data=json.dumps(self.last_command)))
        deadline = time.monotonic()+self.delay_sec
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.02, max(0., deadline-time.monotonic())))
        check()
        self.last_command['wait_completed'] = True
        self.state.publish(String(data=json.dumps(self.last_command)))
        return dict(self.last_command)

    def open_to(self, width_mm, current_ma, check):
        check()
        super().open_to(width_mm, current_ma, check)
        return self._show(check)

    def grasp(self, grasp, check):
        check()
        super().grasp(grasp, check)
        return self._show(check)
