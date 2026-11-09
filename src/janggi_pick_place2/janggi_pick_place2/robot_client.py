"""Arm backends. Real moves are asynchronous and checked to completion."""
import math
import time

from dsr_msgs2.srv import CheckMotion, GetCurrentPosj, GetCurrentPosx, GetCurrentTcp, GetRobotSystem, MoveJoint, MoveLine, MoveStop
from rclpy.callback_groups import ReentrantCallbackGroup

from .geometry import orientation_error_deg


class ServiceCaller:
    def __init__(self, node, prefix, timeout=10.):
        self.node, self.prefix, self.timeout = node, prefix, timeout
        self.group = ReentrantCallbackGroup()
        self.clients = {}

    def client(self, kind, suffix):
        if suffix not in self.clients:
            self.clients[suffix] = self.node.create_client(kind, self.prefix+suffix,
                                                          callback_group=self.group)
        return self.clients[suffix]

    def call(self, kind, suffix, request, check, timeout=None):
        client = self.client(kind, suffix)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while not client.service_is_ready():
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'service unavailable: {client.srv_name}')
            time.sleep(.02)
        check()
        future = client.call_async(request)
        try:
            while not future.done():
                check()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'service timeout: {client.srv_name}')
                time.sleep(.01)
            result = future.result()
        except BaseException:
            future.cancel()
            raise
        if result is None or not result.success:
            raise RuntimeError(f'service failed: {client.srv_name}')
        return result


class RobotClient:
    def __init__(self, node, prefix, settings, expected_tcp_name, service_timeout, motion_timeout):
        self.io = ServiceCaller(node, prefix, service_timeout)
        self.settings = settings
        self.expected_tcp_name, self.motion_timeout = expected_tcp_name, motion_timeout
        self.last_pose = []
        # Discover stop before accepting any motion, rather than lazily at cancellation.
        self.stop_client = self.io.client(MoveStop, '/motion/move_stop')

    def check_tcp(self, check):
        deadline = time.monotonic()+self.io.timeout
        while not self.stop_client.service_is_ready():
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError('move_stop service unavailable; no arm motion started')
            time.sleep(.02)
        result = self.io.call(GetCurrentTcp, '/tcp/get_current_tcp', GetCurrentTcp.Request(), check)
        if result.info != self.expected_tcp_name:
            raise RuntimeError(f'controller TCP name {result.info!r} differs from expected '
                               f'flange TCP {self.expected_tcp_name!r}; TCP is never changed here')

    def current_pose(self, check):
        req = GetCurrentPosx.Request()
        req.ref = 0
        result = self.io.call(GetCurrentPosx, '/aux_control/get_current_posx', req, check)
        if not result.task_pos_info or len(result.task_pos_info[0].data) < 6:
            raise RuntimeError('current pose response is incomplete')
        self.last_pose = list(result.task_pos_info[0].data[:6])
        if not all(math.isfinite(v) for v in self.last_pose):
            raise RuntimeError('current pose contains nonfinite values')
        return list(self.last_pose)

    def _wait(self, reached, check):
        deadline = time.monotonic() + self.motion_timeout
        while True:
            check()
            motion = self.io.call(CheckMotion, '/motion/check_motion', CheckMotion.Request(), check)
            if motion.status == 0 and reached():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError('arm did not reach the target before motion timeout')
            time.sleep(.05)

    def home(self, check):
        req = MoveJoint.Request()
        req.pos = list(self.settings.home_joint_deg)
        req.vel, req.acc = self.settings.joint_velocity_deg_s, self.settings.joint_acceleration_deg_s2
        req.mode, req.sync_type = 0, 1
        self.io.call(MoveJoint, '/motion/move_joint', req, check)
        def reached():
            actual = self.io.call(GetCurrentPosj, '/aux_control/get_current_posj', GetCurrentPosj.Request(), check).pos
            return all(abs(a-b) < .5 for a, b in zip(actual, req.pos))
        self._wait(reached, check)
        actual = self.current_pose(check)
        # Compare axis, not Euler components: ZYZ is singular at B=180 degrees.
        if abs(math.remainder(actual[4], 360.)) < 178.:
            raise RuntimeError('home did not produce a downward flange Z axis')

    def move(self, pose, check):
        req = MoveLine.Request()
        req.pos = list(map(float, pose))
        req.vel = [self.settings.linear_velocity_mm_s, self.settings.angular_velocity_deg_s]
        req.acc = [self.settings.linear_acceleration_mm_s2, self.settings.angular_acceleration_deg_s2]
        req.ref, req.mode, req.sync_type = 0, 0, 1
        self.io.call(MoveLine, '/motion/move_line', req, check)
        def reached():
            actual = self.current_pose(check)
            return math.dist(actual[:3], pose[:3]) <= .5 and orientation_error_deg(actual[3:], pose[3:]) <= 1.
        self._wait(reached, check)

    def stop(self):
        # Nonblocking: used from the action cancel callback as well as failures.
        client = self.stop_client
        if client.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1
            return client.call_async(req)
        self.io.node.get_logger().error('Cannot send quick-stop: move_stop service unavailable')
        return None


class VirtualRobotClient(RobotClient):
    """Move the emulator arm, refusing a controller in REAL robot-system mode."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._virtual_confirmed = False

    def check_tcp(self, check):
        self._virtual_confirmed = False
        system = self.io.call(GetRobotSystem, '/system/get_robot_system',
                              GetRobotSystem.Request(), check)
        if system.robot_system != 1:
            raise RuntimeError('backend=virtual requires ROBOT_SYSTEM_VIRTUAL (1); '
                               'no arm motion was sent. Use mode:=virtual bringup.')
        self._virtual_confirmed = True
        super().check_tcp(check)

    def stop(self):
        if self._virtual_confirmed:
            return super().stop()
        return None


class MockRobot:
    def __init__(self, settings):
        self.settings = settings
        self.last_pose = list(settings.mock_home_flange_pose)
        self.calls = []

    def check_tcp(self, check):
        check()

    def current_pose(self, check):
        check()
        return list(self.last_pose)

    def home(self, check):
        check()
        self.calls.append(('home', list(self.settings.home_joint_deg)))
        self.last_pose = list(self.settings.mock_home_flange_pose)

    def move(self, pose, check):
        check()
        self.calls.append(('move', list(pose)))
        self.last_pose = list(pose)

    def stop(self):
        self.calls.append(('stop',))
