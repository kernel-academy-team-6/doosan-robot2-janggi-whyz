"""Motion sequencing independent of ROS; a single executor owns arm/gripper use."""
from dataclasses import dataclass
import math

from .geometry import finite, width, flange_pose


class OperationCancelled(Exception):
    pass


@dataclass(frozen=True)
class Settings:
    tcp_length_at_6mm_mm: float = 160.0
    default_object_width_mm: float = 30.0
    default_opening_allowance_mm: float = 10.0
    default_grip_current_ma: float = 100.0
    release_extra_width_mm: float = 5.0
    grasp_overtravel_mm: float = 1.0
    home_joint_deg: tuple = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    joint_velocity_deg_s: float = 15.0
    joint_acceleration_deg_s2: float = 30.0
    linear_velocity_mm_s: float = 30.0
    linear_acceleration_mm_s2: float = 60.0
    angular_velocity_deg_s: float = 15.0
    angular_acceleration_deg_s2: float = 30.0
    mock_home_flange_pose: tuple = (400.0, 0.0, 450.0, 0.0, 180.0, 0.0)
    workspace_min_mm: tuple = (-900.0, -900.0, -300.0)
    workspace_max_mm: tuple = (900.0, 900.0, 1000.0)

    def validate(self):
        for name, value in self.__dict__.items():
            for number in value if isinstance(value, (tuple, list)) else [value]:
                finite(number, name)
        width(self.default_object_width_mm)
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
            self._step('home/move', .10, check, report)
            self.robot.check_tcp(check)
            self.robot.home(check)
        else:
            self.gripper.prepare(check)
        if operation in ('pick', 'pick_and_place'):
            self._pick(target, grasp, check, report, place_target)
        if operation in ('place', 'pick_and_place'):
            self._place(place_target if operation == 'pick_and_place' else target, check, report)
        self.stage = 'complete'
        report('complete', 1.)
        return self.robot.current_pose(check)

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

    def _pick(self, target, grasp, check, report, place_target=None):
        self._step('pick/prepare', .02, check, report)
        self.robot.check_tcp(check)
        destination, above = self._approach(target, grasp, check)
        if place_target is not None and self.target_pose(place_target, grasp)[2] > above[2] + .1:
            raise ValueError('place target flange height exceeds the current entry height')
        self._step('pick/horizontal', .12, check, report)
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
