"""Board-game adapter for the source janggi_pick_place2 executor.

No independent action server is needed. The UI reuses its ROS node/executor,
while sequencing, measured gap/TCP formulas and service completion checks come
from janggi_pick_place2 unchanged.
"""
from dataclasses import fields, replace
from pathlib import Path
import math
import sys
import json
import time
from types import SimpleNamespace

import numpy as np
import yaml

from .pick_place import PickConfig

PACKAGE_ROOT = Path(__file__).resolve().parents[2]/'janggi_pick_place2'
DEFAULT_PICK_PLACE_CONFIG = PACKAGE_ROOT/'config/pick_place.yaml'
# Import source modules so changes to the requested package are used immediately.
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
from janggi_pick_place2.executor import PickPlaceExecutor, Settings, OperationCancelled

PIECE_GROUPS = dict(sa=0, soldier=0, cha=1, horse=1, sang=1, po=1, king=2)
PIECE_WIDTHS_MM = {kind:(20.,26.,32.)[group] for kind,group in PIECE_GROUPS.items()}


def load_pick_place_settings(path=DEFAULT_PICK_PLACE_CONFIG):
    data = yaml.safe_load(Path(path).read_text())
    params = data['/janggi_pick_place2']['ros__parameters']
    for name in ('service_timeout_sec', 'motion_timeout_sec', 'gripper_command_wait_sec'):
        value = float(params[name])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if params.get('expected_flange_tcp_name', '') != '':
        raise ValueError('Only the unnamed/default J6 flange TCP is supported')
    settings = Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params}).validate()
    return settings, params


class BoardPickPlace2:
    def __init__(self, mover, config=DEFAULT_PICK_PLACE_CONFIG, *, virtual=False,
                 stop_check=lambda: False, log=lambda message: None,
                 robot=None, gripper=None, yaw=None, trace_path=None):
        self.mover, self.cfg = mover, PickConfig()
        self.settings, self.params = load_pick_place_settings(config)
        # Share Cartesian rotation limits with the fixed-J6 joint spline and home.
        self.settings = replace(self.settings, linear_velocity_mm_s=float(mover.vel[0]),
                                angular_velocity_deg_s=float(mover.vel[1]),
                                linear_acceleration_mm_s2=float(mover.acc[0]),
                                angular_acceleration_deg_s2=float(mover.acc[1]),
                                joint_velocity_deg_s=float(mover.vel[1]),
                                joint_acceleration_deg_s2=float(mover.acc[1])).validate()
        self.stop_check, self.log, self.discarded = stop_check, log, 0
        self.object_width_mm = self.settings.default_object_width_mm
        self.piece_kind = 'cha'
        self.held_grasp_height_mm = None
        self.operation_yaw = None
        self.operation_orientation_abc = None
        self.home_orientation_abc = None
        self.home_flange_z_mm = None
        self._last_place = None
        # Default: preserve measured home attitude, avoiding a yaw=0 wrist turn.
        # An explicit --pick-yaw remains an intentional orientation override.
        self.yaw = None if yaw is None else float(yaw)
        if self.yaw is not None and not math.isfinite(self.yaw):
            raise ValueError('pick yaw must be finite')
        if self.settings.joint6_lock_enabled and self.yaw is not None:
            raise ValueError('--pick-yaw conflicts with joint6_lock_enabled; use the configured J6 angle')
        self.backend = 'virtual' if virtual else 'real'
        # UI virtual mode is Gazebo: wait for physical motion, not emulator-only
        # completion. Real serial communication remains open-loop as requested.
        if virtual:
            self.params['virtual_physics_feedback'] = True
        self.trace_path = Path(trace_path) if trace_path else None
        self.trace_sequence = 0
        if robot is None or gripper is None:
            from janggi_pick_place2.runtime import create_clients
            prefix = getattr(mover, 'prefix', '/dsr01')
            default_robot, default_gripper = create_clients(mover.node, prefix, self.settings,
                                                            self.params, self.backend)
            robot = robot or default_robot
            gripper = gripper or default_gripper
        self.runner = PickPlaceExecutor(robot, gripper, self.settings)
        robot.command_observer = self.record_motion
        gripper.command_observer = self.record_motion
        mover.command_observer = self.record_motion
        self.record_motion('configuration', dict(backend=self.backend, yaw=self.yaw,
                           command_offset_mm=self.settings.tcp_length_at_6mm_mm,
                           grasp_yaw_offset_deg=self.settings.grasp_yaw_offset_deg,
                           calibration_offset_mm=mover.board.tcp_offset_mm.tolist(),
                           linear_velocity_mm_s=self.settings.linear_velocity_mm_s,
                           angular_velocity_deg_s=self.settings.angular_velocity_deg_s,
                           joint6_band_deg=[self.settings.joint6_min_deg,self.settings.joint6_max_deg],
                           joint6_limit_margin_deg=self.settings.joint6_limit_margin_deg,
                           orientation_mode='joint6_locked' if self.settings.joint6_lock_enabled else
                           'home_attitude' if self.yaw is None else 'explicit_yaw',
                           joint6_locked_deg=self.settings.joint6_locked_deg,
                           command_board_z_mm=self.settings.command_board_z_mm,
                           measured_board_height_source=(mover.board.height_map.source if mover.board.height_map is not None else None),
                           piece_grip_widths_mm=list(self.settings.piece_grip_widths_mm),
                           piece_grip_heights_mm=list(self.settings.piece_grip_heights_mm),
                           piece_min_widths_mm=list(self.settings.piece_min_widths_mm),
                           piece_thicknesses_mm=list(self.settings.piece_thicknesses_mm),
                           gripper_transport='stroke' if virtual else 'position_request' if self.params.get('gripper_wait_for_result',False) else 'position_cmd',
                           gripper_wait_sec=self.params['virtual_gripper_delay_sec'] if virtual else self.params['gripper_command_wait_sec']))
        self.log(f'집기: janggi_pick_place2 · J6 플랜지 기준값(+H(c))={self.settings.tcp_length_at_6mm_mm:g}mm · '
                 f'개방 여유={self.settings.default_opening_allowance_mm:g}mm')

    def set_motion_settings(self, settings):
        """Caller must serialize this with complete robot jobs, including homing."""
        settings.validate()
        self.settings = self.runner.settings = self.runner.robot.settings = settings
        self.mover.vel = (settings.linear_velocity_mm_s, settings.angular_velocity_deg_s)
        self.mover.acc = (settings.linear_acceleration_mm_s2, settings.angular_acceleration_deg_s2)
        # Cached splines include durations calculated from the previous limits.
        self.runner.robot._locked_paths = []
        self.runner.robot._line_paths = []
        self.record_motion('motion_settings', {
            name: getattr(settings, name) for name in (
                'linear_velocity_mm_s', 'linear_acceleration_mm_s2',
                'angular_velocity_deg_s', 'angular_acceleration_deg_s2',
                'joint_velocity_deg_s', 'joint_acceleration_deg_s2',
                'board_z_compensation_enabled', 'board_z_offset_mm', 'board_z_corner_offsets_mm')})

    def record_motion(self, operation, payload):
        event = dict(sequence=self.trace_sequence, wall_time=time.time(),
                     operation=operation, stage=getattr(getattr(self, 'runner', None), 'stage', 'initialize'),
                     **payload)
        self.trace_sequence += 1
        if self.trace_path:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
            with self.trace_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(event, ensure_ascii=False)+'\n')
        self.log('[로봇 명령] '+json.dumps(event, ensure_ascii=False))

    def go_observe(self, joints, velocity, acceleration):
        try:
            self._check()
            self.runner.home_to(self._check,
                                lambda stage,progress:self.log(f'집기/놓기: {stage}'),
                                joints=joints, velocity=velocity, acceleration=acceleration)
            self.operation_yaw = None
            self.operation_orientation_abc = None
            self.runner.orientation_abc = None  # Read the new home attitude at next pick.
            self.home_orientation_abc = None
            self.home_flange_z_mm = float(self.runner.robot.current_pose(self._check)[2])
        except BaseException:
            self.runner.robot.stop()
            raise

    def set_piece_kind(self, kind):
        self.object_width_mm = self.settings.piece_grip_widths_mm[PIECE_GROUPS[kind]]
        self.piece_kind = kind

    def grasp_height_for_kind(self, kind):
        return self.settings.piece_grip_heights_mm[PIECE_GROUPS[kind]]

    def piece_height_for_kind(self, kind):
        return self.settings.piece_thicknesses_mm[PIECE_GROUPS[kind]]

    def command_target_for_kind(self, point, kind):
        point = np.asarray(point, float).copy()
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Target must be three finite base coordinates in mm')
        # Keep calibrated camera XY and the original calibration plane untouched.
        point[2] = self.mover.board.command_surface_z(point, self.settings, virtual=self.backend == 'virtual') + self.grasp_height_for_kind(kind)
        return point

    def _check(self):
        if self.stop_check():
            raise OperationCancelled('로봇 정지 요청으로 집기/놓기를 중단했습니다.')

    def _target(self, point):
        point = np.asarray(point, float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Target must be three finite base coordinates in mm')
        yaw = self.operation_yaw if self.operation_yaw is not None else self.yaw
        if yaw is None:
            abc = self.runner.orientation_abc
            yaw = float(abc[0]-abc[2]) if abc is not None else 0.
        return [*map(float, point), yaw]

    def _run(self, operation, target, grasp=None):
        try:
            self._check()
            if not self.settings.joint6_lock_enabled:
                if self.home_orientation_abc is None:
                    self.home_orientation_abc = self.runner.robot.current_pose(self._check)[3:]
                self.runner.orientation_abc = ((self.operation_orientation_abc or [self.operation_yaw,180.,0.]) if self.operation_yaw is not None
                                               else self._home_grasp_orientation() if self.yaw is None else None)
            self.record_motion('tip_target', dict(action=operation, target=self._target(target),
                               orientation_abc=self.runner.orientation_abc,
                               object_width_mm=grasp.width_mm if grasp is not None else
                               self.runner.held.width_mm if self.runner.held is not None else None))
            self.runner.run(operation, self._target(target), grasp, self._check,
                            lambda stage, progress:self.log(f'집기/놓기: {stage}'))
            return np.asarray(target, float)
        except BaseException:
            self.runner.robot.stop()
            raise

    def _home_grasp_orientation(self):
        """Default flat world attitude, with configured home offset, never cumulative."""
        if self.home_orientation_abc is None:
            self.home_orientation_abc = self.runner.robot.current_pose(self._check)[3:]
        abc = list(self.home_orientation_abc)
        abc[0] = (abc[0] + self.settings.grasp_yaw_offset_deg + 180.) % 360. - 180.
        return abc

    def set_operation_yaw(self, yaw):
        if yaw is not None and (not math.isfinite(yaw) or self.settings.joint6_lock_enabled):
            raise ValueError('adaptive grasp yaw requires a finite angle and MoveL mode')
        self.operation_yaw = yaw
        self.operation_orientation_abc = None
        if yaw is not None:
            self.runner.orientation_abc = [float(yaw),180.,0.]
        else:
            if self.home_orientation_abc is None:
                self.home_orientation_abc = self.runner.robot.current_pose(self._check)[3:]
            self.runner.orientation_abc = self._home_grasp_orientation()

    def home_grasp_yaw(self):
        abc = self._home_grasp_orientation()
        return float(abc[0]-abc[2])

    def set_orthogonal_grasp(self, offset_deg):
        """Home-relative 0/90deg jaw direction, preserving the flange Z axis."""
        if offset_deg not in (0.,90.):
            raise ValueError('grasp direction offset must be 0 or 90 degrees')
        if offset_deg==0.:
            self.set_operation_yaw(None)
            return
        abc=self._home_grasp_orientation()
        # ZYZ: rotating local C preserves the flange's world Z direction.
        # For downward B=180, C -= 90 yields a +90deg opening direction.
        abc[2]=(abc[2]-offset_deg+180.)%360.-180.
        self.operation_yaw=float(abc[0]-abc[2])
        self.operation_orientation_abc=list(abc)
        self.runner.orientation_abc=list(abc)

    def current_yaw(self):
        abc = self.runner.robot.current_pose(self._check)[3:]
        return float(abc[0]-abc[2])

    def pick_abs(self, grasp_mm, *, refresh=None):
        try:
            self._check()
        except OperationCancelled:
            self.runner.robot.stop()
            raise
        # Open-loop UI command sequencing: inferred/unknown holding state must
        # not latch out a new operator-requested pick. This is not grip feedback.
        self.runner.holding_state = 'empty_unverified'
        self.runner.held = None
        self._last_place = None
        grasp = self.runner.resolve(object_width_mm=self.object_width_mm)
        target = self.command_target_for_kind(grasp_mm, self.piece_kind)
        self._last_pick_target = target
        def update_target():
            self._last_pick_target = self.command_target_for_kind(refresh(),self.piece_kind)
            self.record_motion('pick_target_refresh',dict(target=self._target(self._last_pick_target)))
            return self._target(self._last_pick_target)
        self.runner.refresh_pick_target = update_target if refresh else None
        try:
            result = self._run('pick', target, grasp)
        finally:
            self.runner.refresh_pick_target = None
        self.held_grasp_height_mm = self.grasp_height_for_kind(self.piece_kind)
        return self._last_pick_target if refresh else result

    def place(self, x, y):
        point = self.mover.board.tip_target(x, y, self.cfg.place_height_mm)
        self._check()
        if self.runner.held is None:
            self.runner.held = self.runner.resolve(object_width_mm=self.object_width_mm)
        self.runner.holding_state = 'holding_unverified'
        if self.held_grasp_height_mm is None:
            self.held_grasp_height_mm = self.grasp_height_for_kind(self.piece_kind)
        point[2] = self.mover.board.command_surface_z(point, self.settings, virtual=self.backend == 'virtual') + self.held_grasp_height_mm
        released = (self.runner.held,self.held_grasp_height_mm,(x,y))
        result = self._run('place', point)
        self._last_place = released
        self.held_grasp_height_mm = None
        return result

    def retry_place(self, x, y):
        """One release retry after visual evidence, without claiming verified holding."""
        self._check()
        if self._last_place is not None:
            self.runner.held,self.held_grasp_height_mm,_ = self._last_place
        self.runner.holding_state = 'holding_unverified'
        self.record_motion('release_retry_assumed_holding',dict(destination=[x,y],grip_verified=False))
        return self.place(x,y)

    def mark_holding_unknown(self):
        self.runner.holding_state = 'unknown'
        self.record_motion('holding_unknown_after_visual_check',dict(grip_verified=False))

    def discard_spot(self, index=None):
        # Same drop location every time: midpoint of (0,4)/(0,5), base +Y 70mm.
        point = self.mover.board.board_to_base(0.,4.5).copy()
        point[1] += 70.
        height = self.home_flange_z_mm
        if height is None:
            robot = self.runner.robot
            pose = (robot.home_flange_pose(self._check) if hasattr(robot,'home_flange_pose')
                    else robot.current_pose(self._check))
            height = self.home_flange_z_mm = float(pose[2])
        point[2] = height  # Flange height, not a tip-height target.
        return point

    def discard(self):
        self._check()
        point = self.discard_spot()
        point[2] = max(point[2], float(self.runner.robot.current_pose(self._check)[2]))
        grasp = self.runner.held or self.runner.resolve(object_width_mm=self.object_width_mm)
        try:
            self.record_motion('capture_drop',dict(flange_xyz=point.tolist(),height_mode='at_least_home_flange',
                grip_verified=False,release_attempts=2))
            self.runner.drop_at_height(point,grasp,self._check,
                lambda stage,progress:self.log(f'잡은 말 떨어뜨리기: {stage}'))
        except BaseException:
            self.runner.robot.stop()
            raise
        self.held_grasp_height_mm = None
        self._last_place = None
        self.discarded += 1
        return point
