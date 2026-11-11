"""Calibrated board and D455f SDF geometry, independent of Gazebo transport.

The board surface and RGB optical pose come from the selected calibration.
The live Gazebo path derives a level, yaw-only board calibration first. Housing
offsets are nominal D455 geometry; outer size and thickness come from scene config.
The renderer is RGB-only; depth and point-cloud processing are disabled.
"""
from pathlib import Path
import math
from xml.sax.saxutils import escape

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_SCENE_CONFIG = HERE/'config/scene.yaml'
# Columns are Gazebo's forward/left/up axes expressed in ROS optical right/down/forward.
OPTICAL_FROM_GAZEBO = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])


def rotation_rpy(r):
    return np.array([math.atan2(r[2, 1], r[2, 2]),
                     -math.asin(np.clip(r[2, 0], -1., 1.)),
                     math.atan2(r[1, 0], r[0, 0])])


def pose_text(position, rotation):
    return ' '.join(f'{v:.12g}' for v in (*position, *rotation_rpy(rotation)))


def material(color):
    return f'<material><diffuse>{color}</diffuse><ambient>{color}</ambient></material>'


def box_visual(name, position, size, color, rotation=None):
    rotation = np.eye(3) if rotation is None else rotation
    dimensions = ' '.join(f'{v:.12g}' for v in size)
    return (f'<visual name="{name}"><pose>{pose_text(position, rotation)}</pose>'
            f'<geometry><box><size>{dimensions}</size></box></geometry>{material(color)}</visual>')


def cylinder_visual(name, position, radius, length, color):
    # Cylinder's local +Z points along camera-body +X.
    r = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    return (f'<visual name="{name}"><pose>{pose_text(position, r)}</pose>'
            f'<geometry><cylinder><radius>{radius:.12g}</radius><length>{length:.12g}</length>'
            f'</cylinder></geometry>{material(color)}</visual>')


class CalibratedLayout:
    def __init__(self, geometry, camera, intrinsics, config):
        self.geometry, self.camera, self.intrinsics, self.config = geometry, camera, intrinsics, config
        self.center = geometry.corners_mm.mean(axis=0)/1000.
        self.normal = geometry.normal()
        tl, tr, br, bl = geometry.corners_mm/1000.
        x = (tr-tl)+(br-bl)
        x -= self.normal*np.dot(self.normal, x)
        x /= np.linalg.norm(x)
        # Board row direction is -local Y; +local Z remains upward.
        self.rotation = np.column_stack([x, np.cross(self.normal, x), self.normal])
        self.outer_size_m = np.asarray(config['board']['outer_size_mm'], float)/1000.
        self.thickness_m = float(config['board']['thickness_mm'])/1000.
        if (self.outer_size_m.shape != (2,) or not np.isfinite(self.outer_size_m).all()
                or (self.outer_size_m <= 0).any() or not 0 < self.thickness_m < .1):
            raise ValueError('Invalid board presentation dimensions')
        local = np.array([self.to_local(p) for p in geometry.corners_mm/1000.])
        # The square is centered on the calibrated grid, without altering the grid.
        if (np.abs(local[:, :2]).max(axis=0) >= self.outer_size_m/2).any():
            raise ValueError('Calibrated black grid does not fit inside the configured board')

    @classmethod
    def from_world(cls, world, calibration, config_path=DEFAULT_SCENE_CONFIG):
        data = yaml.safe_load(Path(calibration).read_text())
        config = yaml.safe_load(Path(config_path).read_text())
        return cls(world.geom, world.cam, data['camera']['color_intrinsics'], config)

    def to_local(self, position_m):
        return self.rotation.T@(np.asarray(position_m)-self.center)

    def grid_point(self, x, y):
        return self.geometry.board_to_base(x, y)/1000.

    def grid_segments(self):
        pairs = [((0, y), (8, y)) for y in range(10)]
        pairs += [((x, 0), (x, 9)) for x in range(9)]
        pairs += [((3, 0), (5, 2)), ((5, 0), (3, 2)), ((3, 7), (5, 9)), ((5, 7), (3, 9))]
        return [(self.grid_point(*a), self.grid_point(*b)) for a, b in pairs]

    @property
    def camera_position_m(self):
        return self.camera.t_mm/1000.

    @property
    def camera_rotation(self):
        return self.camera.R@OPTICAL_FROM_GAZEBO

    def board_sdf(self):
        wood = box_visual('wood', [0., 0., -self.thickness_m/2],
                          [*self.outer_size_m, self.thickness_m], '0.66 0.48 0.32 1')
        lines = []
        for i, (a, b) in enumerate(self.grid_segments()):
            pa, pb = self.to_local(a), self.to_local(b)
            direction = pb-pa
            length = np.linalg.norm(direction)
            x = direction/length
            z = np.array([0., 0., 1.])-x*x[2]
            z /= np.linalg.norm(z)
            rotation = np.column_stack([x, np.cross(z, x), z])
            # Preserve the measured 3D endpoints; lift only for visible line thickness.
            mid = (pa+pb)/2 + np.array([0., 0., .0003])
            lines.append(box_visual(f'line{i}', mid, [length, .0012, .0004], '0.07 0.045 0.025 1', rotation))
        return (f'<sdf version="1.9"><model name="janggi_board"><static>true</static>'
                f'<pose>{pose_text(self.center, self.rotation)}</pose><link name="board_surface">'
                f'<collision name="board_contact"><pose>0 0 {-self.thickness_m/2} 0 0 0</pose>'
                f'<geometry><box><size>{self.outer_size_m[0]} {self.outer_size_m[1]} {self.thickness_m}</size>'
                f'</box></geometry></collision>{wood}{"".join(lines)}</link></model></sdf>')

    def camera_sdf(self):
        c = self.config['camera'];i = self.intrinsics
        width, height, depth = np.asarray(c['housing_size_mm'], float)/1000.
        # D455 nominal RGB optical origin is 59 mm right of left IR, which is
        # 47.5 mm left of the housing centre (official realsense-ros URDF).
        cy = .059-.0475
        front = .00465
        cx = front-depth/2
        visuals = [box_visual('housing', [cx, cy, 0.], [depth, width-height, height], '0.14 0.15 0.16 1')]
        for sign in (-1, 1):
            visuals.append(cylinder_visual(f'housing_end_{sign}', [cx, cy+sign*(width-height)/2, 0.],
                                            height/2, depth, '0.14 0.15 0.16 1'))
        visuals.append(box_visual('front_filter', [front+.0004, cy, 0.], [.0008, .11, .021], '0.025 0.03 0.035 1'))
        for name, y, radius in [('rgb_lens', 0., .0037), ('left_ir_lens', .059, .0045), ('right_ir_lens', -.036, .0045)]:
            visuals.append(cylinder_visual(name, [front+.001, y, 0.], radius, .0012, '0.08 0.13 0.19 1'))
        hfov = 2*math.atan(i['width']/(2*i['fx']))
        topic, optical = escape(c['topic']), escape(c['optical_frame'])
        lens = (f'<lens><type>gnomonical</type><scale_to_hfov>false</scale_to_hfov>'
                f'<intrinsics><fx>{i["fx"]}</fx><fy>{i["fy"]}</fy><cx>{i["ppx"]}</cx><cy>{i["ppy"]}</cy><s>0</s></intrinsics>'
                f'<projection><p_fx>{i["fx"]}</p_fx><p_fy>{i["fy"]}</p_fy><p_cx>{i["ppx"]}</p_cx><p_cy>{i["ppy"]}</p_cy><tx>0</tx><ty>0</ty></projection></lens>')
        return (f'<sdf version="1.9"><model name="intel_realsense_d455f"><static>true</static>'
                f'<pose>{pose_text(self.camera_position_m, self.camera_rotation)}</pose><link name="color_camera_frame">'
                f'{"".join(visuals)}<sensor name="d455f_rgb" type="camera">'
                f'<always_on>true</always_on><update_rate>{c["update_rate_hz"]}</update_rate><visualize>false</visualize>'
                f'<topic>{topic}/image</topic><camera>'
                f'<camera_info_topic>{topic}/camera_info</camera_info_topic><optical_frame_id>{optical}</optical_frame_id>'
                f'<horizontal_fov>{hfov:.12g}</horizontal_fov><image><width>{i["width"]}</width><height>{i["height"]}</height><format>R8G8B8</format></image>'
                f'<clip><near>{c["near_clip_m"]}</near><far>{c["far_clip_m"]}</far></clip>'
                f'{lens}</camera></sensor></link></model></sdf>')

    def summary(self):
        corners = self.geometry.corners_mm
        return dict(world_frame='base_link', board_center_m=self.center.tolist(),
                    board_rpy_deg=np.degrees(rotation_rpy(self.rotation)).tolist(),
                    board_outer_size_mm=(self.outer_size_m*1000).tolist(),
                    board_outer_size_source=self.config['board']['outer_size_source'],
                    board_thickness_mm=self.thickness_m*1000.,
                    horizontal_tabletop_z_mm=self.config['board']['tabletop_z_mm'],
                    grid_top_bottom_mm=[float(np.linalg.norm(corners[1]-corners[0])),float(np.linalg.norm(corners[2]-corners[3]))],
                    grid_left_right_mm=[float(np.linalg.norm(corners[3]-corners[0])),float(np.linalg.norm(corners[2]-corners[1]))],
                    camera_optical_position_m=self.camera_position_m.tolist(),
                    camera_sensor_rpy_deg=np.degrees(rotation_rpy(self.camera_rotation)).tolist(),
                    camera_topic=self.config['camera']['topic'],
                    distortion='ideal pinhole; recorded inverse-Brown distortion is not applied',
                    depth='disabled (RGB-only sensor)', point_cloud='disabled (RGB-only sensor)')
