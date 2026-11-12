"""Arm backends. Real moves are asynchronous and checked to completion."""
import math
import time

from dsr_msgs2.srv import (CheckMotion, GetCurrentPosj, GetCurrentPosx, GetCurrentTcp,
                          GetRobotSystem, GetCurrentSolutionSpace, Ikin, Fkin,
                          MoveJoint, MoveLine, MoveSplineJoint, MoveStop)
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup

from .geometry import (orientation_error_deg, CARTESIAN_ORIENTATION_TOLERANCE_DEG,
                       joint6_candidates, local_z_turn_deg)


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
                time.sleep(.001 if suffix in ('/motion/ikin','/motion/fkin') else .01)
            result = future.result()
        except BaseException:
            future.cancel()
            raise
        if result is None or not result.success:
            raise RuntimeError(f'service failed: {client.srv_name}')
        return result


class RobotClient:
    def __init__(self, node, prefix, settings, expected_tcp_name, service_timeout, motion_timeout):
        if expected_tcp_name != '':
            raise ValueError('TCP must be the unnamed/default joint-6 flange, without a tool offset')
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

    def _wait(self, reached, check, monitor=None):
        deadline = time.monotonic() + self.motion_timeout
        while True:
            check()
            self._check_joint6(self.current_joints(check)[5])
            if monitor is not None:
                monitor()
            motion = self.io.call(CheckMotion, '/motion/check_motion', CheckMotion.Request(), check)
            if motion.status == 0 and reached():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError('arm did not reach the target before motion timeout')
            time.sleep(.05)

    def _inverse(self, pose, solution, check):
        req=Ikin.Request();req.pos=list(pose);req.sol_space=solution;req.ref=0
        values=list(self.io.call(Ikin,'/motion/ikin',req,check).conv_posj)
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('invalid IK response; no motion sent')
        return values

    def _forward(self, joints, check):
        req=Fkin.Request();req.pos=list(joints);req.ref=0
        values=list(self.io.call(Fkin,'/motion/fkin',req,check).conv_posx)
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('invalid FK response; no motion sent')
        return values

    def _locked_planner(self):
        from .locked_wrist import LockedWristPlanner
        return LockedWristPlanner(self.settings,self._inverse,self._forward)

    def prepare_locked_path(self, waypoints, check):
        started = time.monotonic()
        pose,joints=self.current_pose(check),self.current_joints(check)
        self._check_joint6(joints[5])
        solution=self._solution(check)
        planner=self._locked_planner();paths=[]
        for target in waypoints:
            path=planner.plan(pose,joints,target,solution,check)
            if path.duration+2 >= self.motion_timeout:
                raise ValueError('fixed-J6 planned duration exceeds motion timeout; no motion sent')
            if path.joints:
                paths.append(path)
                joints=path.joints[-1]
                pose=path.target_pose
        self._locked_paths=paths
        observer=getattr(self,'command_observer',None)
        if observer:
            observer('path_preparation',dict(elapsed_sec=time.monotonic()-started,
                                           segments=len(paths)))

    def _move_locked(self, pose, check):
        from .locked_wrist import line_distance
        current,joints=self.current_pose(check),self.current_joints(check)
        self._check_joint6(joints[5])
        path=None
        for candidate in list(getattr(self,'_locked_paths',[])):
            if (math.dist(candidate.start_pose[:3],current[:3])<.1
                    and math.dist(candidate.target_pose[:3],pose[:3])<.1
                    and max(abs(a-b) for a,b in zip(candidate.start_joints,joints))<.1):
                path=candidate;self._locked_paths.remove(candidate);break
        if path is None:
            path=self._locked_planner().plan(current,joints,pose,self._solution(check),check)
        if not path.joints:
            return
        if path.duration+2 >= self.motion_timeout:
            raise ValueError('fixed-J6 planned duration exceeds motion timeout; no motion sent')
        req=MoveSplineJoint.Request()
        req.pos=[Float64MultiArray(data=list(q)) for q in path.joints]
        req.pos_cnt=len(req.pos)
        req.vel=[self.settings.joint_velocity_deg_s]*6
        req.acc=[self.settings.joint_acceleration_deg_s2]*6
        req.time=path.duration;req.mode=0;req.sync_type=1
        observer=getattr(self,'command_observer',None)
        if observer:
            observer('move_joint_spline',dict(joints_deg=path.joints,flange_start=current,
                       flange_target=path.target_pose,locked_joint6_deg=self.settings.joint6_locked_deg,
                       duration_sec=path.duration,
                       timing_limits_sec=path.timing_limits,
                       limiting_factor=max(path.timing_limits,key=path.timing_limits.get) if path.timing_limits else None,
                       cartesian_path_tolerance_mm=self.settings.joint6_locked_path_tolerance_mm))
        self.io.call(MoveSplineJoint,'/motion/move_spline_joint',req,check)
        def monitor():
            actual,q=self.current_pose(check),self.current_joints(check)
            j6_error=abs(q[5]-self.settings.joint6_locked_deg)
            path_error=line_distance(actual[:3],current[:3],pose[:3])
            tilt_error=abs(abs(math.remainder(actual[4],360.))-180.)
            if (j6_error>self.settings.joint6_lock_tolerance_deg
                    or path_error>self.settings.joint6_locked_path_tolerance_mm
                    or tilt_error>.1):
                self.stop()
                raise RuntimeError('fixed-J6 or Cartesian path feedback exceeded tolerance; stop requested: '
                                   f'J6 error={j6_error:.3f}deg (limit {self.settings.joint6_lock_tolerance_deg:g}), '
                                   f'path error={path_error:.3f}mm (limit {self.settings.joint6_locked_path_tolerance_mm:g}), '
                                   f'tilt error={tilt_error:.3f}deg (limit 0.1)')
        def reached():
            actual=self.current_pose(check)
            return math.dist(actual[:3],pose[:3])<=.1 and abs(self.current_joints(check)[5]-self.settings.joint6_locked_deg)<=self.settings.joint6_lock_tolerance_deg
        self._wait(reached,check,monitor=monitor)

    def current_joints(self, check):
        values = list(self.io.call(GetCurrentPosj, '/aux_control/get_current_posj',
                                   GetCurrentPosj.Request(), check).pos)
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('current joint response is incomplete or nonfinite')
        return values

    def _check_joint6(self, angle):
        if not self.settings.joint6_min_deg <= angle <= self.settings.joint6_max_deg:
            self.stop()
            raise RuntimeError(f'J6={angle:.3f} outside software band '
                               f'[{self.settings.joint6_min_deg}, {self.settings.joint6_max_deg}]; '
                               'no automatic recovery from an out-of-band starting position')

    def _solution(self, check):
        result = self.io.call(GetCurrentSolutionSpace, '/aux_control/get_current_solution_space',
                              GetCurrentSolutionSpace.Request(), check)
        if not 0 <= result.sol_space <= 7:
            raise RuntimeError('invalid IK solution space')
        return result.sol_space

    def _joint6_planning_band(self):
        return (self.settings.joint6_min_deg+self.settings.joint6_limit_margin_deg,
                self.settings.joint6_max_deg-self.settings.joint6_limit_margin_deg)

    def _preview_line(self, start, target, joints, solution, check):
        if orientation_error_deg(start[3:], target[3:]) > CARTESIAN_ORIENTATION_TOLERANCE_DEG:
            raise ValueError('align J6 at fixed flange XYZ before Cartesian translation')
        previous = list(joints)
        lower, upper = self._joint6_planning_band()
        count = max(1, math.ceil(math.dist(start[:3], target[:3])/self.settings.joint6_path_step_mm))
        for index in range(1, count+1):
            req = Ikin.Request()
            req.pos = [a+(b-a)*index/count for a,b in zip(start[:3], target[:3])] + list(target[3:])
            req.sol_space, req.ref = solution, 0
            result = self.io.call(Ikin, '/motion/ikin', req, check)
            values = list(result.conv_posj)
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise RuntimeError('IK returned invalid joints; no motion sent')
            # Follow the continuous winding. Wrapping each sample into the band
            # would hide the exact +360-degree limit failure being prevented.
            angle = previous[5] + math.remainder(values[5]-previous[5], 360.)
            if abs(angle-previous[5]) > 90.:
                raise ValueError('IK wrist branch is discontinuous; no Cartesian motion sent')
            if not lower <= angle <= upper:
                raise ValueError(f'Cartesian path predicts J6={angle:.3f} outside '
                                 f'planning band [{lower}, {upper}]')
            previous = values
            previous[5] = angle
        return previous

    def _joint_move(self, target, check, velocity, acceleration, purpose):
        self._check_joint6(self.current_joints(check)[5])
        self._check_joint6(target[5])
        req = MoveJoint.Request()
        req.pos, req.vel, req.acc = list(target), velocity, acceleration
        req.mode, req.sync_type = 0, 1
        observer = getattr(self, 'command_observer', None)
        if observer:
            observer('move_joint', dict(joints_deg=list(req.pos), velocity=req.vel,
                                       acceleration=req.acc, purpose=purpose))
        self.io.call(MoveJoint, '/motion/move_joint', req, check)
        self._wait(lambda: all(abs(a-b)<.5 for a,b in zip(self.current_joints(check), target)), check)

    def align_for_path(self, pose, waypoints, check, *, allow_rewind=False):
        self._line_paths=[]
        current, joints = self.current_pose(check), self.current_joints(check)
        self._check_joint6(joints[5])
        if math.dist(current[:3], pose[:3]) > .5:
            raise ValueError('wrist alignment must keep flange XYZ fixed')
        delta = local_z_turn_deg(current[3:], pose[3:])
        solution = self._solution(check)
        selected = None
        errors = []
        for angle in joint6_candidates(joints[5]+delta, joints[5], *self._joint6_planning_band()):
            if not allow_rewind and abs(angle-joints[5]) > 180.+1e-6:
                continue
            predicted = [*joints[:5], angle]
            start = list(pose)
            paths=[]
            try:
                for waypoint in waypoints:
                    paths.append((list(start),list(predicted),list(waypoint)))
                    predicted = self._preview_line(start, waypoint, predicted, solution, check)
                    start = list(waypoint)
            except ValueError as exc:
                errors.append(str(exc))
                continue
            selected = angle
            break
        if selected is None:
            raise ValueError('No continuous J6 path within software band; no alignment started. '
                             + '; '.join(errors))
        observer = getattr(self, 'command_observer', None)
        if observer:
            observer('joint6_path_selection', dict(current_deg=joints[5], selected_deg=selected,
                                                   travel_deg=selected-joints[5], allow_rewind=allow_rewind,
                                                   planning_band_deg=list(self._joint6_planning_band())))
        if abs(selected-joints[5]) > .01:
            self._joint_move([*joints[:5], selected], check,
                             self.settings.angular_velocity_deg_s,
                             self.settings.angular_acceleration_deg_s2, 'bounded_J6_alignment')
        actual = self.current_pose(check)
        if math.dist(actual[:3], pose[:3]) > .5 or orientation_error_deg(actual[3:], pose[3:]) > CARTESIAN_ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError('J6 alignment did not preserve flange position/orientation; no XY started')
        self._line_paths=paths

    def home(self, check, *, joints=None, velocity=None, acceleration=None):
        target = list(self.settings.home_joint_deg if joints is None else joints)
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise ValueError('home target must contain six finite joint angles')
        if self.settings.joint6_lock_enabled:
            if abs(target[5]-self.settings.joint6_locked_deg)>self.settings.joint6_lock_tolerance_deg:
                raise ValueError('home J6 must match joint6_locked_deg while wrist lock is enabled')
            target[5]=self.settings.joint6_locked_deg
        current = self.current_joints(check)
        self._check_joint6(current[5])
        candidates = joint6_candidates(target[5], current[5], *self._joint6_planning_band())
        if not candidates:
            raise ValueError('no equivalent home J6 angle within software band')
        target[5] = candidates[0]
        self._joint_move(target, check,
                         self.settings.joint_velocity_deg_s if velocity is None else velocity,
                         self.settings.joint_acceleration_deg_s2 if acceleration is None else acceleration,
                         'pick_place2_home_or_observe')
        self._locked_paths=[]
        self._line_paths=[]
        actual = self.current_pose(check)
        # Compare axis, not Euler components: ZYZ is singular at B=180 degrees.
        if abs(math.remainder(actual[4], 360.)) < 178.:
            raise RuntimeError('home did not produce a downward flange Z axis')

    def move(self, pose, check):
        if self.settings.joint6_lock_enabled:
            return self._move_locked(pose,check)
        current, joints = self.current_pose(check), self.current_joints(check)
        self._check_joint6(joints[5])
        validated=False
        for candidate in list(getattr(self,'_line_paths',[])):
            start,seed,target=candidate
            if (math.dist(start[:3],current[:3])<=.5
                    and orientation_error_deg(start[3:],current[3:])<=CARTESIAN_ORIENTATION_TOLERANCE_DEG
                    and math.dist(target[:3],pose[:3])<.01
                    and orientation_error_deg(target[3:],pose[3:])<.01
                    and max(abs(a-b) for a,b in zip(seed,joints))<=.5):
                self._line_paths.remove(candidate)
                validated=True
                break
        if not validated:
            self._preview_line(current, pose, joints, self._solution(check), check)
        req = MoveLine.Request()
        req.pos = list(map(float, pose))
        req.vel = [self.settings.linear_velocity_mm_s, self.settings.angular_velocity_deg_s]
        req.acc = [self.settings.linear_acceleration_mm_s2, self.settings.angular_acceleration_deg_s2]
        # No calculated duration or extra speed multiplier in the MoveL path.
        req.time = 0.0
        req.ref, req.mode, req.sync_type = 0, 0, 1
        observer = getattr(self, 'command_observer', None)
        if observer:
            observer('move_line', dict(flange_pose=list(req.pos), velocity=list(req.vel),
                                      acceleration=list(req.acc), ref=req.ref, mode=req.mode,
                                      sync_type=req.sync_type, time_sec=req.time))
        self.io.call(MoveLine, '/motion/move_line', req, check)
        def reached():
            actual = self.current_pose(check)
            return math.dist(actual[:3], pose[:3]) <= .5 and orientation_error_deg(actual[3:], pose[3:]) <= CARTESIAN_ORIENTATION_TOLERANCE_DEG
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

    def home(self, check, *, joints=None, velocity=None, acceleration=None):
        check()
        self.calls.append(('home', list(self.settings.home_joint_deg if joints is None else joints)))
        self.last_pose = list(self.settings.mock_home_flange_pose)

    def move(self, pose, check):
        check()
        self.calls.append(('move', list(pose)))
        self.last_pose = list(pose)

    def stop(self):
        self.calls.append(('stop',))
