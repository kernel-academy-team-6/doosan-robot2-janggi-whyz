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


def tcp_length(width_mm, reference_length_mm=160.0):
    w = width(width_mm)
    length = finite(reference_length_mm, 'tcp_length_at_6mm_mm')
    if length <= 0:
        raise ValueError('tcp_length_at_6mm_mm must be positive')
    result = length - math.sqrt(3249.0 - 1.21) + math.sqrt(3249.0 - (w / 2 - 4.1) ** 2)
    if result <= 0:
        raise ValueError('calculated TCP length must be positive')
    return result


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
