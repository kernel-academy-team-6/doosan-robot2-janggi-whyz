"""User-supplied RH-P12-RN-A calibration. All lengths are millimetres."""
import math


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def width(value):
    value = finite(value, 'width_mm')
    if not 6.0 <= value <= 106.0:
        raise ValueError(f'width must be in [6, 106] mm, got {value}')
    return value


def command_from_width(width_mm):
    w = width(width_mm)
    angle_deg = math.degrees(math.acos((w - 8.2) / 114.0))
    return max(0, min(700, int(round((angle_deg - 30.331197) / 0.086670))))


def width_from_command(command):
    u = finite(command, 'position')
    if not 0 <= u <= 700:
        raise ValueError(f'position must be in [0, 700], got {u}')
    w = 8.2 + 114.0 * math.cos(math.radians(30.331197 + 0.086670 * u))
    # Quantization/calibration may put the endpoint just outside the domain.
    return max(6.0, min(106.0, w))


FLANGE_EXTENSION_MM = 160.0


def tcp_length(width_mm, reference_length_mm=FLANGE_EXTENSION_MM):
    """Exactly one fixed extension from joint-6 flange; jaw width changes no TCP."""
    width(width_mm)  # Keep input validation; width affects jaw command only.
    length = finite(reference_length_mm, 'flange_extension_mm')
    if not math.isclose(length, FLANGE_EXTENSION_MM, abs_tol=1e-9, rel_tol=0.):
        raise ValueError('This project requires a fixed 160 mm extension from the joint-6 flange')
    return FLANGE_EXTENSION_MM


def flange_pose(target, object_width_mm, reference_length_mm):
    if len(target) != 4:
        raise ValueError('target must contain [x, y, z, yaw]')
    x, y, z, yaw = [finite(v, 'target') for v in target]
    yaw = (yaw + 180.0) % 360.0 - 180.0
    # Rz(yaw) Ry(180): flange local +Z points down, parallel to base Z.
    return [x, y, z + tcp_length(object_width_mm, reference_length_mm), yaw, 180.0, 0.0]


def rotation_zyz(abc):
    a, b, c = map(math.radians, abc)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return ((ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb),
            (sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb),
            (-sb*cc, sb*sc, cb))


def orientation_error_deg(first, second):
    a, b = rotation_zyz(first), rotation_zyz(second)
    trace = sum(a[i][j] * b[i][j] for i in range(3) for j in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


# Cartesian segments begin only after the measured flange orientation is aligned.
CARTESIAN_ORIENTATION_TOLERANCE_DEG = 0.1


def joint6_candidates(target, current, lower=-270., upper=270.):
    """Equivalent endpoint angles, ordered by travel then distance from band centre."""
    target, current = finite(target, 'J6 target'), finite(current, 'J6 current')
    first = math.ceil((lower-target)/360.)
    last = math.floor((upper-target)/360.)
    centre = (lower+upper)/2.
    return sorted((target+360.*k for k in range(first, last+1)),
                  key=lambda angle: (round(abs(angle-current), 8), abs(angle-centre)))


def local_z_turn_deg(current, target):
    """Signed wrist rotation; reject a requested change of the flange Z axis."""
    a, b = rotation_zyz(current), rotation_zyz(target)
    axis_dot = sum(a[i][2]*b[i][2] for i in range(3))
    if math.degrees(math.acos(max(-1., min(1., axis_dot)))) > CARTESIAN_ORIENTATION_TOLERANCE_DEG:
        raise ValueError('J6-only alignment cannot change flange Z direction; orient the arm first')
    return math.degrees(math.atan2(sum(a[i][1]*b[i][0] for i in range(3)),
                                   sum(a[i][0]*b[i][0] for i in range(3))))
