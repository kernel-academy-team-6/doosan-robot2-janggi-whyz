"""Motion sequencing independent of ROS; a single executor owns arm/gripper use."""
from dataclasses import dataclass
import math

from .geometry import (finite, width, flange_pose, tcp_length, orientation_error_deg,
                       CARTESIAN_ORIENTATION_TOLERANCE_DEG)


class OperationCancelled(Exception):
    pass


@dataclass(frozen=True)
class Settings:
    tcp_length_at_6mm_mm: float = 160.0
    default_object_width_mm: float = 26.0
    default_opening_allowance_mm: float = 10.0
    default_grip_current_ma: float = 100.0
    release_extra_width_mm: float = 10.0
    grasp_overtravel_mm: float = 0.0
    command_board_z_mm: float = -20.0
    board_z_compensation_enabled: bool = False
    board_z_offset_mm: float = 0.0
    board_z_corner_offsets_mm: tuple = (0.0, 0.0, 0.0, 0.0)
    piece_min_widths_mm: tuple = (21.0, 27.0, 33.0)
    piece_thicknesses_mm: tuple = (6.0, 7.0, 8.0)
    piece_grip_widths_mm: tuple = (20.0, 26.0, 32.0)
    piece_grip_heights_mm: tuple = (2.0, 2.0, 3.0)
    home_joint_deg: tuple = (0.0, -13.694, 134.216, 0.0, 59.485, 90.0)
    joint_velocity_deg_s: float = 15.0
    joint_acceleration_deg_s2: float = 30.0
    linear_velocity_mm_s: float = 30.0
    linear_acceleration_mm_s2: float = 60.0
    angular_velocity_deg_s: float = 15.0
    angular_acceleration_deg_s2: float = 30.0
    mock_home_flange_pose: tuple = (400.0, 0.0, 450.0, 0.0, 180.0, 0.0)
    workspace_min_mm: tuple = (-900.0, -900.0, -300.0)
    workspace_max_mm: tuple = (900.0, 900.0, 1000.0)
    joint6_min_deg: float = -270.0
    joint6_max_deg: float = 270.0
    joint6_limit_margin_deg: float = 5.0
    joint6_path_step_mm: float = 10.0
    joint6_rewind_clearance_mm: float = 50.0
    joint6_lock_enabled: bool = False
    joint6_locked_deg: float = 90.0
    joint6_lock_tolerance_deg: float = 3.0
    joint6_locked_step_mm: float = 5.0
    joint6_locked_path_tolerance_mm: float = 0.5

    def validate(self):
        for name, value in self.__dict__.items():
            for number in value if isinstance(value, (tuple, list)) else [value]:
                finite(number, name)
        width(self.default_object_width_mm)
        if not isinstance(self.board_z_compensation_enabled, bool) or len(self.board_z_corner_offsets_mm) != 4:
            raise ValueError('board Z compensation requires a boolean and four corner offsets')
        tcp_length(self.default_object_width_mm, self.tcp_length_at_6mm_mm)
        for name in ('tcp_length_at_6mm_mm', 'default_grip_current_ma',
                     'joint_velocity_deg_s', 'joint_acceleration_deg_s2',
                     'linear_velocity_mm_s', 'linear_acceleration_mm_s2',
                     'angular_velocity_deg_s', 'angular_acceleration_deg_s2'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('default_opening_allowance_mm', 'release_extra_width_mm', 'grasp_overtravel_mm'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} must be nonnegative')
        if self.default_grip_current_ma > 1984:
            raise ValueError('default_grip_current_ma exceeds supported input range')
        if len(self.home_joint_deg) != 6 or len(self.mock_home_flange_pose) != 6:
            raise ValueError('home joints and mock pose must contain six values')
        if len(self.workspace_min_mm) != 3 or len(self.workspace_max_mm) != 3:
            raise ValueError('workspace bounds must contain three values')
        if any(a >= b for a, b in zip(self.workspace_min_mm, self.workspace_max_mm)):
            raise ValueError('workspace minimum must be below maximum')
        if not -360. <= self.joint6_min_deg < 0 < self.joint6_max_deg <= 360.:
            raise ValueError('J6 bounds must straddle zero within [-360, 360] degrees')
        if self.joint6_path_step_mm <= 0 or self.joint6_rewind_clearance_mm < 0:
            raise ValueError('J6 path step must be positive and rewind clearance nonnegative')
        if not 0 <= self.joint6_limit_margin_deg < (self.joint6_max_deg-self.joint6_min_deg)/2:
            raise ValueError('J6 limit margin must leave a nonempty planning band')
        for name in ('piece_min_widths_mm','piece_thicknesses_mm','piece_grip_widths_mm','piece_grip_heights_mm'):
            if len(getattr(self,name)) != 3:
                raise ValueError(f'{name} must contain small/medium/large values')
        for measured, thickness, closed, height in zip(self.piece_min_widths_mm,self.piece_thicknesses_mm,
                                                      self.piece_grip_widths_mm,self.piece_grip_heights_mm):
            width(measured);width(closed)
            if not closed <= measured or not 0 < height < thickness:
                raise ValueError('invalid piece grip width/height profile')
        if not isinstance(self.joint6_lock_enabled,bool):
            raise ValueError('joint6_lock_enabled must be boolean')
        if not self.joint6_min_deg+self.joint6_limit_margin_deg <= self.joint6_locked_deg <= self.joint6_max_deg-self.joint6_limit_margin_deg:
            raise ValueError('locked J6 must be inside the planning band')
        if not (0 < self.joint6_lock_tolerance_deg <= 3 and self.joint6_locked_step_mm > 0
                and 0 < self.joint6_locked_path_tolerance_mm <= 2):
            raise ValueError('invalid fixed-J6 tolerances')
        return self


@dataclass(frozen=True)
class Grasp:
    width_mm: float
    current_ma: int
    open_width_mm: float
    close_width_mm: float
    release_width_mm: float


class PickPlaceExecutor:
    def __init__(self, robot, gripper, settings):
        self.robot, self.gripper, self.settings = robot, gripper, settings.validate()
        self.holding_state = 'empty_unverified'
        self.held = None
        self.stage = 'idle'
        self.last_gripper_command = None
        # Optional adapter override: preserve the measured home flange attitude.
        # Standalone actions retain their explicit target yaw when this is None.
        self.orientation_abc = None
        self.refresh_pick_target = None

    def resolve(self, object_width_mm=-1., opening_allowance_mm=-1., grip_current_ma=-1.):
        def selected(value, default):
            value = finite(value, 'optional input')
            return default if value == -1.0 else value
        s = self.settings
        w = width(selected(object_width_mm, s.default_object_width_mm))
        allowance = selected(opening_allowance_mm, s.default_opening_allowance_mm)
        if allowance < 0:
            raise ValueError('opening_allowance_mm must be nonnegative, or -1 for default')
        current = selected(grip_current_ma, s.default_grip_current_ma)
        if not 1 <= current <= 1984 or not float(current).is_integer():
            raise ValueError('grip_current_ma must be an integer in [1, 1984], or -1')
        # Explicit error instead of silently reducing the requested clearances.
        return Grasp(w, int(current), width(w+allowance),
                     width(max(6., w-s.grasp_overtravel_mm)),
                     width(w+s.release_extra_width_mm))

    def target_pose(self, target, grasp):
        pose = flange_pose(target, grasp.width_mm, self.settings.tcp_length_at_6mm_mm)
        if self.orientation_abc is not None:
            if len(self.orientation_abc) != 3:
                raise ValueError('orientation override must contain three angles')
            pose[3:] = [finite(v,'orientation override') for v in self.orientation_abc]
        self.check_workspace(pose)
        return pose

    def check_workspace(self, pose):
        for axis, value, lower, upper in zip('xyz', pose[:3], self.settings.workspace_min_mm,
                                            self.settings.workspace_max_mm):
            if not lower <= value <= upper:
                raise ValueError(f'flange {axis}={value:.3f} outside [{lower}, {upper}] mm')

    def preflight(self, operation, target, grasp=None, place_target=None):
        if self.holding_state == 'unknown':
            raise RuntimeError('grasp state is unknown; inspect and recover manually before restarting')
        if operation == 'home':
            return
        if operation in ('pick', 'pick_and_place'):
            if self.holding_state != 'empty_unverified':
                raise RuntimeError('cannot pick while holding an object')
            self.target_pose(target, grasp)
            if place_target is not None:
                self.target_pose(place_target, grasp)
        else:
            if self.holding_state != 'holding_unverified' or self.held is None:
                raise RuntimeError('place requires a completed pick command in this server session (grip unverified)')
            self.target_pose(target, self.held)

    def run(self, operation, target, grasp, check, report, place_target=None):
        self.stage = 'initialize'
        self.preflight(operation, target, grasp, place_target)
        if operation == 'home':
            self.home_to(check, report)
        else:
            self.gripper.prepare(check)
        if operation in ('pick', 'pick_and_place'):
            self._pick(target, grasp, check, report, place_target)
        if operation in ('place', 'pick_and_place'):
            self._place(place_target if operation == 'pick_and_place' else target, check, report)
        self.stage = 'complete'
        report('complete', 1.)
        return self.robot.current_pose(check)

    def home_to(self, check, report, *, joints=None, velocity=None, acceleration=None):
        """Explicit home/observation operation; pick/place never homes implicitly."""
        self._step('home/move', .10, check, report)
        self.robot.check_tcp(check)
        if joints is None and velocity is None and acceleration is None:
            self.robot.home(check)
        else:
            self.robot.home(check, joints=joints, velocity=velocity, acceleration=acceleration)
        self.stage = 'complete'

    def _step(self, stage, progress, check, report):
        check()
        self.stage = stage
        report(stage, progress)

    def _approach(self, target, grasp, check):
        current = self.robot.current_pose(check)
        if abs(math.remainder(current[4], 360.)) < 178.:
            raise ValueError('pick/place must start with flange Z pointing down; '
                             'use the separate home action or orient the arm first')
        destination = self.target_pose(target, grasp)
        if destination[2] > current[2] + .1:
            raise ValueError('target flange height exceeds entry height; this action only descends')
        above = [destination[0], destination[1], current[2], *destination[3:]]
        self.check_workspace(above)
        return destination, above

    def _align_before_xy(self, above, check, report, phase, preview=None, allow_rewind=False):
        if self.settings.joint6_lock_enabled and hasattr(self.robot,'prepare_locked_path'):
            self._step(phase+'/fixed_wrist_check', .07, check, report)
            self.robot.prepare_locked_path(preview or [above],check)
            return
        current = self.robot.current_pose(check)
        aligned = [*current[:3], *above[3:]]
        # Compare rotation matrices: different A/C values can represent the same
        # orientation at B=180. Do not move merely to rewrite Euler coordinates.
        if hasattr(self.robot, 'align_for_path'):
            self._step(phase+'/align', .07, check, report)
            self.robot.align_for_path(aligned, preview or [above], check,
                                      allow_rewind=allow_rewind)
        elif orientation_error_deg(current[3:], aligned[3:]) > CARTESIAN_ORIENTATION_TOLERANCE_DEG:
            self._step(phase+'/align', .07, check, report)
            self.robot.move(aligned, check)  # Rotate at fixed flange XYZ first.
        measured = self.robot.current_pose(check)
        if orientation_error_deg(measured[3:], above[3:]) > CARTESIAN_ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError('flange orientation is not aligned; no XY translation started')

    def _pick(self, target, grasp, check, report, place_target=None):
        self._step('pick/prepare', .02, check, report)
        self.robot.check_tcp(check)
        destination, above = self._approach(target, grasp, check)
        if place_target is not None and self.target_pose(place_target, grasp)[2] > above[2] + .1:
            raise ValueError('place target flange height exceeds the current entry height')
        preview = [above, destination, above]
        if place_target is not None:
            place = self.target_pose(place_target, grasp)
            place_above = [*place[:2], above[2], *place[3:]]
            preview += [place_above, place, place_above]
        clearance = above[2] - max(p[2] for p in [destination] + ([place] if place_target is not None else []))
        self._align_before_xy(above, check, report, 'pick', preview,
                             allow_rewind=clearance >= self.settings.joint6_rewind_clearance_mm)
        self._step('pick/horizontal', .12, check, report)
        self.robot.move(above, check)
        if self.refresh_pick_target is not None:
            self._step('pick/refresh', .20, check, report)
            target = self.refresh_pick_target()
            check()
            destination, above = self._approach(target, grasp, check)
            self._align_before_xy(above, check, report, 'pick/refresh',
                                  [above,destination,above])
            self.robot.move(above, check)
        self._step('pick/open', .25, check, report)
        self.gripper.open_to(grasp.open_width_mm, grasp.current_ma, check)
        self._step('pick/descend', .40, check, report)
        self.robot.move(destination, check)
        self._step('pick/grasp', .55, check, report)
        self.holding_state = 'unknown'
        self.last_gripper_command = self.gripper.grasp(grasp, check)
        self.held = grasp
        self.holding_state = 'holding_unverified'
        self._step('pick/retreat', .75, check, report)
        self.robot.move(above, check)

    def _place(self, target, check, report):
        grasp = self.held
        self.robot.check_tcp(check)
        destination, above = self._approach(target, grasp, check)
        self._align_before_xy(above, check, report, 'place', [above, destination, above])
        self._step('place/horizontal', .10, check, report)
        self.robot.move(above, check)
        self._step('place/descend', .35, check, report)
        self.robot.move(destination, check)
        self._step('place/release', .60, check, report)
        self.holding_state = 'unknown'
        self.gripper.open_to(grasp.release_width_mm, grasp.current_ma, check)
        self.held = None
        self.holding_state = 'empty_unverified'
        self._step('place/retreat', .80, check, report)
        self.robot.move(above, check)
