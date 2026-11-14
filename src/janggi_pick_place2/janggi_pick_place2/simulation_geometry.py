"""Gazebo-only stroke -> CAD joint angle, matching the shared measured W(c).

The ROS visualization publisher still encodes stroke as c/700 radians. These
values are synthetic targets, not real motor feedback. Never change serial c.
CAD inner flat pads: W(q)/2 = 4.1 + 49.3634*cos(q) - 28.5*sin(q) [mm].
"""
import math
from janggi_pick_place2.geometry import width_from_command, tcp_length_from_command

CAD_BASE_HALF_GAP_MM=8.0
CAD_PAD_INSET_MM=3.9
CAD_ARM_Y_MM=49.3634
CAD_ARM_Z_MM=28.5
CAD_RADIUS_MM=math.hypot(CAD_ARM_Y_MM,CAD_ARM_Z_MM)
CAD_PHASE_RAD=math.atan2(CAD_ARM_Z_MM,CAD_ARM_Y_MM)
FINGER_JOINTS=frozenset(('gripper_rh_r1','gripper_rh_r2','gripper_rh_l1','gripper_rh_l2'))


def cad_gap_mm(angle):
    return 2*(CAD_BASE_HALF_GAP_MM-CAD_PAD_INSET_MM+
              CAD_ARM_Y_MM*math.cos(angle)-CAD_ARM_Z_MM*math.sin(angle))


def angle_from_stroke(command):
    """Invert actual CAD gap, rather than directly reusing the real circle angle."""
    desired=width_from_command(command)
    cosine=(desired/2-(CAD_BASE_HALF_GAP_MM-CAD_PAD_INSET_MM))/CAD_RADIUS_MM
    if not -1.<=cosine<=1.:
        raise ValueError('Requested gripper width is outside Gazebo CAD range')
    angle=math.acos(cosine)-CAD_PHASE_RAD
    if not 0.<=angle<=1.101:
        raise ValueError('Calibrated Gazebo finger angle exceeds existing URDF limits')
    return angle


def angle_from_ros_target(position):
    """Decode unchanged c/700 visualization input and map only the Gazebo model."""
    if not math.isfinite(position):raise ValueError('Gripper target must be finite')
    return angle_from_stroke(max(0.,min(700.,position*700.)))


def cad_tip_length_mm(command):
    """Unadapted CAD base -> lowest inward pad edge, along flange local +Z."""
    angle = angle_from_stroke(command)
    return 88. + CAD_ARM_Y_MM*math.sin(angle) + CAD_ARM_Z_MM*math.cos(angle)


def calibrated_mount_mm():
    """Gazebo-only adapter matching measured flange-to-pad length at c=700.

    Width inversion already matches W(c). The CAD vertical arc differs slightly
    from H(c); this fixed adapter leaves <0.12 mm error across c=0..700.
    Never add this value to real PickPlace2 TCP calculations.
    """
    return tcp_length_from_command(700.) - cad_tip_length_mm(700.)
