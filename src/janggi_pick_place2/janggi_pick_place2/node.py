"""Home, pick, place, and combined action servers sharing one arm/gripper."""
from dataclasses import fields
from functools import partial
import json
import math
import signal
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.validate_namespace import validate_namespace
from std_srvs.srv import Trigger

from janggi_pick_place2.action import Home, Pick, Place, PickAndPlace
from .executor import OperationCancelled, PickPlaceExecutor, Settings
from .gripper import GripperClient, MockGripper, VirtualGripperClient
from .robot_client import MockRobot, RobotClient, VirtualRobotClient


EXTRA_DEFAULTS = {
    'backend': 'mock',
    'robot_namespace': 'dsr01',
    'expected_flange_tcp_name': '',
    'service_timeout_sec': 5.0,
    'motion_timeout_sec': 60.0,
    'gripper_command_wait_sec': 5.0,
    'mock_step_delay_sec': 0.05,
    'virtual_gripper_delay_sec': 0.4,
}


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('janggi_pick_place2')
        self._lock = threading.Lock()
        self._busy = False
        self._shutdown_requested = threading.Event()
        self._last_error = ''
        defaults = Settings()
        config = {f.name: getattr(defaults, f.name) for f in fields(Settings)}
        config.update(EXTRA_DEFAULTS)
        self.parameters = {}
        for name, value in config.items():
            if isinstance(value, tuple):
                value = list(value)
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.parameters[name] = self.get_parameter(name).value
        p = self.parameters
        for name, value in p.items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if p['backend'] not in ('mock', 'virtual', 'real'):
            raise ValueError('backend must be mock, virtual, or real')
        for key in ('service_timeout_sec', 'motion_timeout_sec', 'gripper_command_wait_sec'):
            if p[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if p['mock_step_delay_sec'] < 0 or p['virtual_gripper_delay_sec'] < 0:
            raise ValueError('simulation delays must be nonnegative')
        prefix = '/' + p['robot_namespace'].strip('/') if p['robot_namespace'].strip('/') else ''
        validate_namespace(prefix or '/')
        self.settings = Settings(**{f.name: p[f.name] for f in fields(Settings)}).validate()
        if p['backend'] == 'mock':
            self.robot, self.gripper = MockRobot(self.settings), MockGripper()
        elif p['backend'] == 'virtual':
            self.robot = VirtualRobotClient(
                self, prefix, self.settings, p['expected_flange_tcp_name'],
                p['service_timeout_sec'], p['motion_timeout_sec'])
            self.gripper = VirtualGripperClient(self, prefix, p['virtual_gripper_delay_sec'])
        else:
            self.robot = RobotClient(self, prefix, self.settings, p['expected_flange_tcp_name'],
                                     p['service_timeout_sec'], p['motion_timeout_sec'])
            self.gripper = GripperClient(self, prefix, p)
        self.runner = PickPlaceExecutor(self.robot, self.gripper, self.settings)
        group = ReentrantCallbackGroup()
        self.servers = []
        for operation, action in [('home', Home), ('pick', Pick), ('place', Place),
                                  ('pick_and_place', PickAndPlace)]:
            self.servers.append(ActionServer(
                self, action, '~/'+operation,
                execute_callback=partial(self._execute, operation=operation, action=action),
                goal_callback=partial(self._goal, operation=operation),
                cancel_callback=self._cancel,
                callback_group=group,
            ))
        self.status_service = self.create_service(Trigger, '~/status', self._status,
                                                  callback_group=group)
        self.get_logger().info(
            f'Actions ready; backend={p["backend"]}; no startup motion. '
            f'L(6mm)={self.settings.tcp_length_at_6mm_mm} mm; '
            'targets are base-frame [x,y,absolute TIP z,yaw] in mm/deg.'
        )
        if p['backend'] == 'real':
            self.get_logger().warn(
                f'Gripper uses existing {prefix}/gripper/position_cmd only. '
                'Current/speed requests are not sent; existing gripper settings remain in use. '
                'Wait for bringup initialization and avoid concurrent commands. '
                'Grip and gripper arrival are unverified.'
            )
        elif p['backend'] == 'virtual':
            self.get_logger().info(
                'Virtual arm services enabled; gripper animation goes to '
                f'{prefix}/gripper/stroke. Contact and current are simulated; no serial I/O.'
            )

    def _parse(self, request, operation):
        if operation == 'home':
            return None, None, None
        if operation == 'place':
            return list(request.target), None, None
        grasp = self.runner.resolve(request.object_width_mm, request.opening_allowance_mm,
                                    request.grip_current_ma)
        if operation == 'pick':
            return list(request.target), grasp, None
        return list(request.pick_target), grasp, list(request.place_target)

    def _goal(self, request, operation):
        with self._lock:
            if self._busy:
                self.get_logger().warn('Rejected concurrent goal: one arm/gripper operation at a time')
                return GoalResponse.REJECT
            try:
                target, grasp, place_target = self._parse(request, operation)
                self.runner.preflight(operation, target, grasp, place_target)
            except (ValueError, RuntimeError, TypeError) as exc:
                self._last_error = str(exc)
                self.get_logger().error(f'Rejected goal: {exc}')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal):
        self.robot.stop()
        return CancelResponse.ACCEPT

    def _execute(self, goal, operation, action):
        result = action.Result()
        def check():
            if goal.is_cancel_requested or self._shutdown_requested.is_set() or not self.context.ok():
                raise OperationCancelled('operation cancelled; gripper is not automatically opened')
        def report(stage, progress):
            check()
            if operation == 'pick_and_place' and stage != 'complete':
                progress = progress*.5 + (.5 if stage.startswith('place/') else 0.)
            feedback = action.Feedback()
            feedback.stage, feedback.progress = stage, progress
            goal.publish_feedback(feedback)
            self.get_logger().info(f'{operation}: {stage}')
            if self.parameters['backend'] == 'mock':
                deadline = time.monotonic()+self.parameters['mock_step_delay_sec']
                while time.monotonic() < deadline:
                    check()
                    time.sleep(.01)
        try:
            target, grasp, place_target = self._parse(goal.request, operation)
            report('initialize', 0.)
            final = self.runner.run(operation, target, grasp, check, report, place_target)
            check()
            result.success = True
            result.message = {
                'mock': 'MOCK simulation completed; not a physical grasp verification',
                'virtual': 'VIRTUAL arm completed; gripper/contact simulated, no current measurement',
                'real': ('Sequence completed using existing position_cmd; grip unverified; '
                         'requested current was not sent, existing gripper settings used'),
            }[self.parameters['backend']]
            result.final_flange_pose = list(final)
            self._last_error = ''
            goal.succeed()
        except OperationCancelled as exc:
            self.robot.stop()
            result.message = str(exc)
            self._last_error = str(exc)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        except Exception as exc:
            self.robot.stop()
            result.message = f'{self.runner.stage}: {exc}'
            self._last_error = result.message
            self.get_logger().error(result.message)
            goal.abort()
        finally:
            result.holding_state = self.runner.holding_state
            if not result.final_flange_pose:
                result.final_flange_pose = list(self.robot.last_pose)
            try:
                self.gripper.close_session()
            except Exception as exc:
                self.get_logger().error(f'Gripper cleanup failed: {exc}')
            with self._lock:
                self._busy = False
        return result

    def _status(self, _request, response):
        response.success = True
        data = {'package': 'janggi_pick_place2', 'backend': self.parameters['backend'],
                'busy': self._busy, 'stage': self.runner.stage,
                'holding_state': self.runner.holding_state,
                'last_error': self._last_error,
                'tcp_length_at_6mm_mm': self.settings.tcp_length_at_6mm_mm,
                'gripper_feedback_received': False,
                'grip_verified': False,
                'gripper_transport': 'position_cmd' if self.parameters['backend'] == 'real' else self.parameters['backend'],
                'current_limit_command_sent': False,
                'last_gripper_sample': None,
                'last_gripper_command': self.gripper.last_command,
                'gripper_command_wait_sec': self.parameters['gripper_command_wait_sec'],
                'last_flange_pose': list(self.robot.last_pose)}
        if self.parameters['backend'] == 'mock':
            data['mock_robot_events'] = self.robot.calls[-30:]
            data['mock_gripper_events'] = self.gripper.calls[-20:]
        elif self.parameters['backend'] == 'virtual':
            data['simulated_gripper_events'] = self.gripper.calls[-20:]
        response.message = json.dumps(data, ensure_ascii=False)
        return response


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node, executor = None, None
    try:
        node = PickPlaceNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        while node.context.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._shutdown_requested.set()
            stopping = node.robot.stop() if node._busy else None
            deadline = time.monotonic() + 15.
            while executor is not None and node.context.ok() and time.monotonic() < deadline:
                if not node._busy and (stopping is None or stopping.done()):
                    break
                executor.spin_once(timeout_sec=.1)
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
