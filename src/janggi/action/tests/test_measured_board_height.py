"""Independent physical board-contact height map, never camera calibration Z."""
from dataclasses import replace
from pathlib import Path
import sys
import hashlib
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from action.board import BoardGeometry,DEFAULT_CALIBRATION
from action.board_height import BoardHeightMap,bilinear_weights
from janggi_pick_place2.geometry import tcp_length
from test_pick_place2 import picker


def test_measured_corner_contacts_and_support_reference():
    bed=BoardHeightMap.from_file()
    expected=[-16.8108,-16.8869,-22.9544,-22.7325]
    for xy,z in zip(bed.xy_mm,expected):
        assert bed.surface_z(*xy)==pytest.approx(z,abs=1e-9)
        assert bed.height_above_support(*xy)==pytest.approx(z+30.,abs=1e-9)
    assert bed.probe_tool_length_mm==160.3681


def test_bilinear_mean_interior_and_measured_xy_inversion():
    bed=BoardHeightMap.from_file();xy=bed.xy_mm.mean(axis=0)
    assert bed.surface_z(*xy)==pytest.approx(np.mean([-16.8108,-16.8869,-22.9544,-22.7325]))
    for u,v in [(0,.5),(.3,.7),(1,.25)]:
        weights=np.array([(1-u)*(1-v),u*(1-v),u*v,(1-u)*v])
        xy=weights@bed.xy_mm
        assert bilinear_weights(xy,bed.xy_mm)==pytest.approx(weights,abs=1e-9)
        assert bed.surface_z(*xy)==pytest.approx(weights@bed.surface_corner_z_mm)
    assert bed.surface_z(500,100)==pytest.approx(-19.2505978,abs=1e-6)


def test_external_base_coordinate_is_clamped_unless_extrapolation_is_requested():
    bed=BoardHeightMap.from_file()
    assert min(bed.surface_corner_z_mm)<=bed.surface_z(100,100)<=max(bed.surface_corner_z_mm)
    assert bed.surface_z(100,100,extrapolate=True)<min(bed.surface_corner_z_mm)
    with pytest.raises(ValueError):bed.surface_z(float('nan'),100)
    with pytest.raises(ValueError):bilinear_weights([0,0],np.zeros((4,2)))


def test_all_piece_groups_pick_and_place_two_mm_above_local_surface_without_altering_calibration():
    before=hashlib.sha256(DEFAULT_CALIBRATION.read_bytes()).hexdigest()
    for kind,width in [('soldier',20.),('cha',26.),('king',32.)]:
        p,events,_,_=picker();p.mover.board=BoardGeometry.from_calibration()
        board=p.mover.board;bed=board.height_map;p.set_piece_kind(kind)
        source=board.board_to_base(0,0)
        p.pick_abs(source)
        descent=[e[1] for e in events if e[0]=='move'][-2]
        assert descent[2]==pytest.approx(-16.8108+2+tcp_length(width))
        assert p.grasp_height_for_kind(kind)==2.
        p.place(8,9)
        descent=[e[1] for e in events if e[0]=='move'][-2]
        assert descent[2]==pytest.approx(-22.9544+2+tcp_length(width))
        assert board.corners_mm[:,2]==pytest.approx([-76.49,-76.662,-82.539,-82.738])
        # Further manual corrections are optional and additive above the new map.
        manual=replace(p.settings,board_z_compensation_enabled=True,board_z_offset_mm=.5,
                       board_z_corner_offsets_mm=(1,0,0,0))
        assert board.command_surface_z(source,manual)==pytest.approx(-16.8108+1.5)
    assert hashlib.sha256(DEFAULT_CALIBRATION.read_bytes()).hexdigest()==before


@pytest.mark.parametrize('kind,width', [('soldier',20.),('cha',26.),('king',32.)])
def test_gazebo_pick_and_place_ignore_physical_height_map_and_manual_offsets(kind, width):
    p,events,_,_=picker()
    p.backend='virtual'
    p.mover.board=BoardGeometry.from_calibration()
    p.set_motion_settings(replace(p.settings, board_z_compensation_enabled=True,
                                 board_z_offset_mm=4., board_z_corner_offsets_mm=(1.,2.,3.,4.)))
    p.set_piece_kind(kind)
    source=p.mover.board.board_to_base(0,0)
    assert p.mover.board.command_surface_z(source,p.settings,virtual=True)==-20.
    p.pick_abs(source)
    descent=[e[1] for e in events if e[0]=='move'][-2]
    assert descent[2]==pytest.approx(-18.+tcp_length(width))
    p.place(8,9)
    descent=[e[1] for e in events if e[0]=='move'][-2]
    assert descent[2]==pytest.approx(-18.+tcp_length(width))
