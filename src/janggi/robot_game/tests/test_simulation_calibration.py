from pathlib import Path
import copy
import sys
import numpy as np
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation_calibration import simulation_data,HARDWARE_CALIBRATION,HERE


def test_height_matches_30mm_table_and_10mm_board_without_changing_hardware_calibration():
    hardware=yaml.safe_load(HARDWARE_CALIBRATION.read_text());before=copy.deepcopy(hardware)
    config=yaml.safe_load((HERE/'config/scene.yaml').read_text())
    data=simulation_data(hardware,config)
    assert hardware==before
    n=np.array(data['board_plane_base']['normal']);d=data['board_plane_base']['d']
    # The lowest outer wood corner touches the horizontal table.
    from action.board import BoardGeometry
    from types import SimpleNamespace
    from scene_models import CalibratedLayout
    from piece_detection.world import CameraToBase
    from simulation_calibration import ensure_simulation_calibration
    path=ensure_simulation_calibration()
    layout=CalibratedLayout.from_world(SimpleNamespace(geom=BoardGeometry.from_calibration(path),cam=CameraToBase.from_calibration(path)),path)
    bottom=[layout.center+layout.rotation@np.array([a*.2,b*.2,-.010]) for a in (-1,1) for b in (-1,1)]
    np.testing.assert_allclose([p[2] for p in bottom],-.030,atol=1e-12)
    np.testing.assert_allclose(layout.normal,[0,0,1],atol=1e-12)
    from scene_models import rotation_rpy
    np.testing.assert_allclose(rotation_rpy(layout.rotation)[:2],[0,0],atol=1e-12)
    assert np.degrees(rotation_rpy(layout.rotation)[2])==pytest.approx(-90.0004824729,abs=1e-8)
    original=np.array([c['base_xyz'] for c in hardware['corners']])
    updated=np.array([c['base_xyz'] for c in data['corners']])
    np.testing.assert_allclose(updated.mean(axis=0)[:2],original.mean(axis=0)[:2],atol=1e-12)
    np.testing.assert_allclose(updated[:,2],-.020,atol=1e-12)
    for a,b in [(0,1),(1,2),(2,3),(3,0)]:
        assert np.linalg.norm(updated[b]-updated[a])==pytest.approx(np.linalg.norm(original[b]-original[a]),abs=1e-7)
    assert data['T_base_camera']==hardware['T_base_camera']
    assert data['camera']['color_intrinsics']['coeffs']==[0.]*5
    i=data['camera']['color_intrinsics'];T=np.array(data['T_base_camera']['matrix'])
    for corner in data['corners']:
        p=T[:3,:3].T@(np.array(corner['base_xyz'])-T[:3,3])
        expected=[i['fx']*p[0]/p[2]+i['ppx'],i['fy']*p[1]/p[2]+i['ppy']]
        np.testing.assert_allclose(corner['pixel'],expected)


def test_simulation_preserves_original_calibration_reference_separately_from_pick_offset():
    from action.board import BoardGeometry
    from simulation_calibration import ensure_simulation_calibration
    path=ensure_simulation_calibration();data=yaml.safe_load(path.read_text())
    hardware=yaml.safe_load(HARDWARE_CALIBRATION.read_text())
    geometry=BoardGeometry.from_calibration(path)
    np.testing.assert_array_equal(geometry.tcp_offset_mm,[0.,0.,222.])
    for original,derived in zip(hardware['corners'],data['corners']):
        delta=(np.array(derived['base_xyz'])-np.array(original['base_xyz']))*1000.
        np.testing.assert_allclose(derived['robot_posx'][:3],np.array(original['robot_posx'][:3])+delta)
