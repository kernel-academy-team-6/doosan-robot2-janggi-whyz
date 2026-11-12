"""Read-only numerical audit of recorded calibration; never contacts a robot/camera."""
from pathlib import Path
import copy
import json
import subprocess
import sys

import cv2
import numpy as np
import yaml

WS=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(WS/'src/janggi/calibration'),str(WS/'src/janggi'),
              str(WS/'src/janggi/robot_game'),str(WS/'janggi')]
import calib_math as cm
from action.board import BoardGeometry
from action.pick_place2 import load_pick_place_settings
from piece_detection.board_mapping import BoardMapper
from piece_detection.world import CameraToBase
from simulation_calibration import simulation_data
from session import RobotGame, RobotGameConfig


def camera(data):
    i=data['camera']['color_intrinsics']
    return CameraToBase([[i['fx'],0,i['ppx']],[0,i['fy'],i['ppy']],[0,0,1]],i['coeffs'],
                        data['T_base_camera']['matrix'],data['board_plane_base']['normal'],data['board_plane_base']['d'])


def grid_geometry(data):
    return BoardGeometry(np.array([c['base_xyz'] for c in data['corners']])*1000,
                         np.array(data['corners'][0]['robot_posx'][3:]),np.array(data['robot']['tcp_offset_mm']))


def normalized_pixels(data):
    i=data['camera']['color_intrinsics'];pixels=[c['pixel'] for c in data['corners']]
    try:
        import pyrealsense2 as rs
        intr=rs.intrinsics()
        for key in ('width','height','fx','fy','ppx','ppy'):setattr(intr,key,i[key])
        intr.model=getattr(rs.distortion,i['distortion_model']);intr.coeffs=i['coeffs']
        return np.array([rs.rs2_deproject_pixel_to_point(intr,p,1.)[:2] for p in pixels]),'RealSense SDK: same software deprojection as calibration UI; no device opened'
    except ImportError:
        return cv2.undistortPoints(np.array(pixels,float).reshape(-1,1,2),
                                  camera(data).K,np.array(i['coeffs'])).reshape(-1,2),'OpenCV runtime deprojection fallback'


def refit(points,normals,intr):
    r_cb,t_cb=cm.solve_pnp(normals,points);r,t=cm.invert(r_cb,t_cb)
    n,d=cm.fit_plane(points)
    reproj=np.linalg.norm((cm.project_norm(r_cb,t_cb,points)-normals)*[intr['fx'],intr['fy']],axis=1)
    distances=[np.linalg.norm(cm.ray_plane(t,r@np.r_[uv,1.],n,d)-p)*1000 for uv,p in zip(normals,points)]
    return dict(tilt_deg=float(np.degrees(np.arccos(n[2]))),camera_z_mm=float(t[2]*1000),
                reprojection_max_px=float(max(reproj)),board_rms_mm=float(np.sqrt(np.mean(np.array(distances)**2))))


def main():
    path=WS/'src/janggi/calibration/config/calibration.yaml'
    data=yaml.safe_load(path.read_text());points=np.array([c['base_xyz'] for c in data['corners']])*1000
    poses=np.array([c['robot_posx'] for c in data['corners']])
    recomputed=np.array([cm.tip_position_m(p,data['robot']['tcp_offset_mm'])*1000 for p in poses])
    z_component=recomputed[:,2]-poses[:,2]
    n,d=cm.fit_plane(points/1000);plane_residual=(points@n+d*1000)/n[2]
    original=yaml.safe_load(subprocess.run(['git','show','8d8ee75:src/janggi/calibration/config/calibration.yaml'],
                                         cwd=WS,capture_output=True,text=True,check=True).stdout)
    norm,method=normalized_pixels(data)
    scenarios={}
    for name,z in [('original',points[:,2]),('all_Z_divided_by_10',points[:,2]/10),
                   ('all_Z_plus_5mm',points[:,2]+5),('only_TL_Z_plus_5mm',points[:,2]+[5,0,0,0])]:
        changed=points.copy();changed[:,2]=z
        scenarios[name]=refit(changed/1000,norm,data['camera']['color_intrinsics'])
    cam=camera(data);geom=grid_geometry(data)
    pixels=BoardMapper([c['pixel'] for c in data['corners']]).grid_pixels().reshape(-1,2)
    base=np.array([cam.pixel_to_base(*pixel) for pixel in pixels])
    changed_plane=copy.deepcopy(data)
    changed_plane['board_plane_base']['d']-=float(cam.n[2])*.005
    changed=np.array([camera(changed_plane).pixel_to_base(*pixel) for pixel in pixels])
    translated=copy.deepcopy(changed_plane)
    translated['T_base_camera']['matrix'][2][3]+=.005
    shifted=np.array([camera(translated).pixel_to_base(*pixel) for pixel in pixels])
    # Current actual command pipeline, with only corners[].base_xyz Z edited.
    settings,_=load_pick_place_settings();command_changes={}
    centers={(x,y):tuple(uv) for y,row in enumerate(pixels.reshape(10,9,2)) for x,uv in enumerate(row)}
    def commands(config):
        game=RobotGame.__new__(RobotGame);game.geom=grid_geometry(config);game.cam=camera(config)
        game.picker=None;game.cfg=RobotGameConfig(command_settings=settings);game.log=lambda *_:None
        return np.array([game.grasp_from_camera(cell,centers,'cha')[0] for cell in centers])
    original_commands=commands(data)
    for name in ('all_Z_plus_5mm','all_Z_divided_by_10','only_TL_Z_plus_5mm'):
        edited=copy.deepcopy(data)
        for index,c in enumerate(edited['corners']):
            c['base_xyz'][2] = (points[index,2]/10 if name=='all_Z_divided_by_10' else
                                points[index,2]+(5 if name=='all_Z_plus_5mm' or index==0 else 0))/1000
        delta=commands(edited)-original_commands
        command_changes[name]=dict(max_actual_command_xy_shift_mm=float(np.max(np.linalg.norm(delta[:,:2],axis=1))),
                                    max_actual_command_z_shift_mm=float(np.max(np.abs(delta[:,2]))),
                                    camera_ray_plane_mapping_changed=False,
                                    note='Only corner Z fields edited. Stored camera T and plane unchanged; deliberately inconsistent file experiment, not a correction.')
    sim=simulation_data(data,yaml.safe_load((WS/'src/janggi/robot_game/config/scene.yaml').read_text()))
    result=dict(
        historical_commit='8d8ee75',corners_unchanged_since_initial_commit=data['corners']==original['corners'],
        robot_source=data['robot']['posx_source'],normalization=method,
        recorded_tip_z_span_mm=float(np.ptp(points[:,2])),recorded_flange_z_span_mm=float(np.ptp(poses[:,2])),
        offset_vertical_component_span_mm=float(np.ptp(z_component)),
        max_saved_tip_vs_formula_difference_mm=float(np.max(np.abs(points-recomputed))),
        plane_tilt_deg=float(np.degrees(np.arccos(n[2]))),plane_residual_z_mm=plane_residual.tolist(),
        plane_residual_z_span_mm=float(np.ptp(plane_residual)),
        tip_reference_command=data['robot']['gripper_position_cmd'],recorded_gripper_commands=[c['gripper_stroke'] for c in data['corners']],
        per_corner=[dict(name=c['name'],flange_z_mm=float(p[2]),offset_z_mm=float(off),
                         tip_z_mm=float(pt[2]),flange_z_minus_160_mm=float(p[2]-160))
                    for c,p,off,pt in zip(data['corners'],poses,z_component,points)],
        pnp_refit_experiments=scenarios,
        fixed_camera_plane_plus5mm=dict(max_xy_shift_mm=float(np.max(np.linalg.norm((changed-base)[:,:2],axis=1))),
                                       mean_xy_shift_mm=float(np.mean(np.linalg.norm((changed-base)[:,:2],axis=1)))),
        co_translated_camera_and_plane_plus5mm=dict(max_xy_shift_mm=float(np.max(np.linalg.norm((shifted-base)[:,:2],axis=1)))),
        current_runtime_partial_file_edits=command_changes,
        simulated_corner_z_mm=[float(c['base_xyz'][2]*1000) for c in sim['corners']],
        simulation_orientation=sim['simulation']['board_orientation'],
        original_calibration_modified=False,
        limitations='No historical contact video/independent surface measurement. Cannot uniquely identify why operator-recorded flange Z differed. PnP residual is same-point fit, not independent height validation.')
    output=Path(__file__).with_name('results.json')
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(output)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
