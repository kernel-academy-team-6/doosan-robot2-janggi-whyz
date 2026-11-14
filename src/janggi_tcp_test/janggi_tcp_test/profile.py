"""Predict a zero-initial-velocity trapezoidal/triangular gripper trajectory.

Firmware Profile Velocity: 0.01 rev/min; Profile Acceleration: rev/min^2.
position_unit_deg is an explicit, calibratable mapping to the position register,
not an assertion that the motor rotor and finger-link angles are identical.
"""
from dataclasses import dataclass
import math

from janggi_pick_place2.geometry import finite, width, command_from_width, width_from_command, tcp_length, tcp_length_from_command


@dataclass(frozen=True)
class Profile:
    initial_u: float
    target_u: float
    velocity_raw: int = 50
    acceleration_raw: int = 100
    position_unit_deg: float = .088

    def __post_init__(self):
        for name, value in self.__dict__.items():
            finite(value, name)
        if not (0 <= self.initial_u <= 700 and 0 <= self.target_u <= 700):
            raise ValueError('position endpoints must lie in [0, 700]')
        if min(self.velocity_raw, self.acceleration_raw, self.position_unit_deg) <= 0:
            raise ValueError('profile velocity, acceleration and position unit must be positive')

    @property
    def distance(self):
        return abs(self.target_u-self.initial_u)

    @property
    def velocity(self):
        return self.velocity_raw*.01*360./60./self.position_unit_deg

    @property
    def acceleration(self):
        return self.acceleration_raw*360./3600./self.position_unit_deg

    @property
    def peak_velocity(self):
        return min(self.velocity, math.sqrt(self.distance*self.acceleration))

    @property
    def acceleration_time(self):
        return self.peak_velocity/self.acceleration

    @property
    def cruise_time(self):
        if self.distance <= self.velocity*self.velocity/self.acceleration:
            return 0.
        return max(0., self.distance/self.peak_velocity-self.acceleration_time)

    @property
    def duration(self):
        return 2*self.acceleration_time+self.cruise_time

    @property
    def kind(self):
        return 'trapezoidal' if self.cruise_time > 1e-9 else 'triangular'

    def position(self, t):
        t = finite(t, 'time')
        if t <= 0:
            return self.initial_u
        if t >= self.duration:
            return self.target_u
        a, ta, tc = self.acceleration, self.acceleration_time, self.cruise_time
        if t < ta:
            distance = .5*a*t*t
        elif t < ta+tc:
            distance = .5*a*ta*ta+self.peak_velocity*(t-ta)
        else:
            distance = self.distance-.5*a*(self.duration-t)**2
        sign = 1. if self.target_u >= self.initial_u else -1.
        return self.initial_u+sign*distance


def endpoints(initial_width_mm, target_width_mm):
    initial = command_from_width(width(initial_width_mm))
    target = command_from_width(width(target_width_mm))
    if initial == target:
        raise ValueError('both widths map to the same position command; choose distinct gaps')
    return initial, target


class CompensationPlan:
    def __init__(self, profile, initial_pose, reference_length_mm=132.6717):
        if len(initial_pose) != 6:
            raise ValueError('initial flange pose must have six values')
        self.pose = [finite(v, 'flange pose') for v in initial_pose]
        self.profile, self.length_reference = profile, reference_length_mm
        self.axis_z = math.cos(math.radians(self.pose[4]))
        if self.axis_z > -math.cos(math.radians(2.)):
            raise ValueError('flange Z must point down within two degrees; orient it before testing')
        self.initial_length = tcp_length_from_command(profile.initial_u, reference_length_mm)
        self.tip_z = self.pose[2]+self.axis_z*self.initial_length

    def sample(self, t):
        u = self.profile.position(t)
        w = width_from_command(u)
        length = tcp_length_from_command(u, self.length_reference)
        z = self.tip_z-self.axis_z*length
        return {'model_u': u, 'model_width_mm': w, 'model_length_mm': length,
                'desired_flange_z_mm': z, 'predicted_tip_z_mm': z+self.axis_z*length}

    def target_pose(self, t):
        pose = list(self.pose)
        pose[2] = self.sample(t)['desired_flange_z_mm']
        return pose

    def validate(self, max_duration, max_displacement, max_velocity, max_acceleration, min_z, max_z):
        if self.profile.duration > max_duration:
            raise ValueError('profile duration exceeds max_duration_sec')
        n = max(2, math.ceil(self.profile.duration/.01))
        dt = self.profile.duration/n
        if dt == 0:
            raise ValueError('zero-duration profile')
        zs = [self.sample(i*dt)['desired_flange_z_mm'] for i in range(n+1)]
        velocities = [(b-a)/dt for a,b in zip(zs,zs[1:])]
        # Include zero endpoint velocity when checking acceleration transitions.
        vs = [0., *velocities, 0.]
        accelerations = [(b-a)/dt for a,b in zip(vs,vs[1:])]
        if min(zs) < min_z or max(zs) > max_z:
            raise ValueError('compensation Z is outside configured flange height bounds')
        if max(abs(z-self.pose[2]) for z in zs) > max_displacement:
            raise ValueError('compensation exceeds max_z_displacement_mm')
        if max(map(abs, velocities)) > max_velocity or max(map(abs, accelerations)) > max_acceleration:
            raise ValueError('predicted Z speed/acceleration exceeds arm limits; reduce gripper profile')
        return {'peak_z_velocity_mm_s': max(map(abs,velocities)),
                'peak_z_acceleration_mm_s2': max(map(abs,accelerations)),
                'min_flange_z_mm': min(zs), 'max_flange_z_mm': max(zs)}
