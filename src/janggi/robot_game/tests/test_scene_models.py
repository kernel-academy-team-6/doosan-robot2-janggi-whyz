"""Calibrated projection, grid endpoints and board dimensions in Gazebo SDF."""
from pathlib import Path
from types import SimpleNamespace
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sim_app  # establishes the application's pure-Python paths
from action.board import BoardGeometry, DEFAULT_CALIBRATION
from piece_detection.world import CameraToBase
from scene_models import CalibratedLayout, OPTICAL_FROM_GAZEBO


@pytest.fixture
def layout():
    world = SimpleNamespace(geom=BoardGeometry.from_calibration(), cam=CameraToBase.from_calibration())
    return CalibratedLayout.from_world(world, DEFAULT_CALIBRATION)


def test_optical_pose_is_preserved_when_converting_to_gazebo_axes(layout):
    assert np.allclose(layout.camera_rotation @ OPTICAL_FROM_GAZEBO.T, layout.camera.R)
    assert np.allclose(layout.camera_rotation[:, 0], layout.camera.R[:, 2])
    assert np.allclose(layout.camera_position_m, [.61345529, .08859302, .57554227])
    # All calibrated grid corners must lie in front of the camera, in its image.
    for p in layout.geometry.corners_mm/1000.:
        optical = layout.camera.R.T @ (p-layout.camera_position_m)
        xforward = layout.camera_rotation.T @ (p-layout.camera_position_m)
        assert optical[2] > 0 and xforward[0] > 0
        np.testing.assert_allclose(optical, OPTICAL_FROM_GAZEBO @ xforward, atol=1e-12)
        uv = layout.camera.K @ optical
        uv = uv[:2]/uv[2]
        assert 0 < uv[0] < 1280 and 0 < uv[1] < 720


def test_board_retains_tilt_and_measured_black_grid_endpoints(layout):
    assert np.linalg.det(layout.rotation) == pytest.approx(1.)
    assert np.allclose(layout.rotation[:, 2], layout.geometry.normal())
    assert abs(layout.normal[0]) > .01  # approximately one degree, not forcibly horizontal
    segments = layout.grid_segments()
    assert len(segments) == 23
    np.testing.assert_allclose(segments[0][0]*1000, layout.geometry.corners_mm[0])
    np.testing.assert_allclose(segments[0][1]*1000, layout.geometry.corners_mm[1])
    np.testing.assert_allclose(segments[9][0]*1000, layout.geometry.corners_mm[3])
    np.testing.assert_allclose(segments[9][1]*1000, layout.geometry.corners_mm[2])
    np.testing.assert_allclose(layout.rotation.T@layout.rotation, np.eye(3), atol=1e-12)


def test_square_400mm_board_top_is_at_calibrated_surface(layout):
    model = ET.fromstring(layout.board_sdf()).find('model')
    visuals = model.find('link').findall('visual')
    wood = visuals[0]
    assert wood.findtext('geometry/box/size') == '0.4 0.4 0.01'
    position = np.fromstring(wood.findtext('pose'), sep=' ')[:3]
    assert position[2]+.01/2 == pytest.approx(0.)
    assert len(visuals) == 24
    assert layout.config['board']['outer_size_source'] == 'user_estimate_square_not_measured'


def test_grid_sdf_line_boxes_reconstruct_calibrated_segments(layout):
    # Parse poses independently of the builder's matrix-to-Euler implementation.
    def rotate(r,p,y):
        cx,sx,cy,sy,cz,sz=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
        return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]]) @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]) @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    visuals=ET.fromstring(layout.board_sdf()).find('model/link').findall('visual')[1:]
    for visual,(a,b) in zip(visuals,layout.grid_segments()):
        pose=np.fromstring(visual.findtext('pose'),sep=' ')
        length=float(visual.findtext('geometry/box/size').split()[0])
        direction=rotate(*pose[3:])[:,0]
        for sign,expected in [(-1,a),(1,b)]:
            local=pose[:3]+sign*direction*length/2-np.array([0,0,.0003])
            np.testing.assert_allclose(layout.center+layout.rotation@local,expected,atol=1e-10)


def test_rgb_only_camera_preserves_existing_image_topic_and_calibration(layout):
    sensor = ET.fromstring(layout.camera_sdf()).find('model/link/sensor')
    assert sensor.get('type') == 'camera'
    assert sensor.findtext('topic') == '/janggi_sim/d455f/image'
    assert sensor.findtext('visualize') == 'false'
    assert sensor.findtext('always_on') == 'true'
    camera = sensor.find('camera')
    assert camera.findtext('image/width') == '1280'
    assert camera.findtext('image/height') == '720'
    i=layout.intrinsics
    for key,value in [('fx',i['fx']),('fy',i['fy']),('cx',i['ppx']),('cy',i['ppy'])]:
        assert float(camera.findtext('lens/intrinsics/'+key)) == value
    assert camera.findtext('optical_frame_id') == 'sim_d455f_color_optical_frame'
    assert camera.find('depth_camera') is None
    assert camera.findtext('camera_info_topic') == '/janggi_sim/d455f/camera_info'
    assert camera.find('distortion') is None  # inverse Brown is not a forward Brown model


def test_impossible_board_size_is_rejected(layout):
    import copy
    config=copy.deepcopy(layout.config);config['board']['outer_size_mm']=[200,200]
    with pytest.raises(ValueError,match='does not fit'):
        CalibratedLayout(layout.geometry,layout.camera,layout.intrinsics,config)


def test_unused_shadow_rendering_is_disabled():
    from scene_models import HERE
    world=ET.parse(HERE/'worlds/janggi_camera.sdf').find('world')
    assert world.findtext('scene/shadows')=='false'
    assert world.findtext('light/cast_shadows')=='false'
