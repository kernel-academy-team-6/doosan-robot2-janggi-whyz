"""Derive simulation-only board height and ideal-camera calibration; preserve hardware data."""
from pathlib import Path
import copy
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
HARDWARE_CALIBRATION = HERE.parent/'calibration/config/calibration.yaml'
SIM_CALIBRATION = HERE/'config/gazebo_calibration.yaml'


def simulation_data(original, config):
    data = copy.deepcopy(original)
    corners = np.array([c['base_xyz'] for c in data['corners']], float)
    normal = np.linalg.svd(corners-corners.mean(axis=0))[2][-1]
    if normal[2] < 0: normal = -normal
    board = config['board']
    # Remove measured roll/pitch while keeping the board's horizontal heading.
    table_z = float(board['tabletop_z_mm'])/1000.
    thickness = float(board['thickness_mm'])/1000.
    center = corners.mean(axis=0)
    tl,tr,br,bl = corners
    x = (tr-tl)+(br-bl)
    x -= normal*(normal@x)
    x /= np.linalg.norm(x)
    measured_rotation = np.column_stack([x,np.cross(normal,x),normal])
    yaw = float(np.arctan2(x[1],x[0]))
    cosine,sine = np.cos(yaw),np.sin(yaw)
    yaw_rotation = np.array([[cosine,-sine,0.],[sine,cosine,0.],[0.,0.,1.]])
    local = (corners-center)@measured_rotation
    local[:,2] = 0.  # Remove the small nonplanar fit residual too.
    target_center = center.copy()
    target_center[2] = table_z+thickness
    original_corners = corners.copy()
    corners = local@yaw_rotation.T+target_center
    normal = np.array([0.,0.,1.])
    T = np.array(data['T_base_camera']['matrix'],float)
    intr = data['camera']['color_intrinsics']
    intr['coeffs'] = [0.]*5
    intr['distortion_model'] = 'none'
    for entry, point, original_point in zip(data['corners'], corners, original_corners):
        entry['base_xyz'] = point.tolist()
        entry['robot_posx'][:3] = (np.array(entry['robot_posx'][:3])+(point-original_point)*1000.).tolist()
        optical = T[:3,:3].T@(point-T[:3,3])
        entry['pixel'] = [float(intr['fx']*optical[0]/optical[2]+intr['ppx']),
                          float(intr['fy']*optical[1]/optical[2]+intr['ppy'])]
    data['board_plane_base'] = dict(normal=normal.tolist(), d=float(-normal@corners.mean(axis=0)))
    data['method'] = 'Simulation-derived: measured grid/yaw, level board, configured table height, ideal RGB projection'
    data['simulation'] = dict(hardware_source=str(HARDWARE_CALIBRATION), board_center_z_shift_mm=float((target_center[2]-center[2])*1000.), board_yaw_deg=float(np.degrees(yaw)), board_orientation='yaw_only',
                              tabletop_z_mm=float(board['tabletop_z_mm']),
                              placement_rule='level board; entire bottom surface rests on horizontal table',
                              board_thickness_mm=float(board['thickness_mm']))
    # The original hardware fit residual does not describe this derived scene.
    data.pop('residual',None)
    return data


def ensure_simulation_calibration():
    data = simulation_data(yaml.safe_load(HARDWARE_CALIBRATION.read_text()),
                           yaml.safe_load((HERE/'config/scene.yaml').read_text()))
    content = yaml.safe_dump(data,allow_unicode=True,sort_keys=False)
    if not SIM_CALIBRATION.exists() or SIM_CALIBRATION.read_text()!=content:
        SIM_CALIBRATION.write_text(content)
    return SIM_CALIBRATION


if __name__ == '__main__': print(ensure_simulation_calibration())
