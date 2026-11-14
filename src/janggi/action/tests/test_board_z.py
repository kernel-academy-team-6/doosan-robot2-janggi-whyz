from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[2]), str(Path(__file__).resolve().parents[4]/'janggi')]
from action.board import BoardGeometry
from action.pick_place2 import load_pick_place_settings
from test_pick_place2 import picker
from janggi_pick_place2.geometry import tcp_length


def test_historical_calibration_z_is_never_applied_and_only_explicit_manual_offsets_change_height():
    g = BoardGeometry.from_calibration(height_map_path=None)
    settings, _ = load_pick_place_settings()
    original = g.corners_mm.copy()
    measured = original[:,2]-original[:,2].min()
    assert measured == pytest.approx([6.248,6.076,.199,0.])
    assert not settings.board_z_compensation_enabled
    for index,point in enumerate(original):
        assert g.command_surface_z(point, settings) == -20
        assert g.command_surface_z(point, replace(settings,board_z_compensation_enabled=True)) == -20
        # Absolute calibration Z or piece height must not leak into bed lookup.
        assert g.command_surface_z([*point[:2],999.], settings) == -20
    center = g.board_to_base(4,4.5)
    assert g.command_surface_z(center, settings) == -20
    manual = replace(settings, board_z_compensation_enabled=True,
                     board_z_offset_mm=.5, board_z_corner_offsets_mm=(0,1,2,1))
    assert g.command_surface_z(center, manual) == pytest.approx(-18.5)
    assert g.command_surface_z(center, replace(manual,board_z_compensation_enabled=False)) == -20
    np.testing.assert_array_equal(g.corners_mm, original)


def test_pick_and_place_use_destination_height_while_discard_keeps_separate_datum():
    p,events,_,_ = picker(compensate=True)
    p.set_motion_settings(replace(p.settings,board_z_corner_offsets_mm=(1,0,0,0)))
    p.set_piece_kind('soldier')
    point = p.mover.board.board_to_base(0,0)
    p.pick_abs(point)
    descended = [e[1] for e in events if e[0]=='move'][-2]
    assert descended[2] == pytest.approx(-20+1+2+tcp_length(20.))
    assert descended[:2] == list(point[:2])
    p.place(0,9)
    assert [e[1][2] for e in events if e[0]=='move'][-2] == pytest.approx(-18+tcp_length(20.))
    p.pick_abs(point)
    before=len(events);p.discard()
    assert all(e[1][2]==450. for e in events[before:] if e[0]=='move')


def test_no_motion_camera_preview_and_real_command_use_same_height_map():
    from session import RobotGame
    p,_,_,_ = picker(compensate=True)
    game = RobotGame.__new__(RobotGame)
    game.picker = None; game.geom = p.mover.board; game.log = lambda *_:None
    game.cfg = SimpleNamespace(command_settings=p.settings,piece_height_mm=5.,max_offset_mm=18.)
    game.cam=SimpleNamespace(pixel_to_base=lambda *args,**kwargs:np.array([500.,100.,-10.]))
    point, source = game.grasp_from_camera((0,0), {(0,0):(123,456)}, 'king')
    assert point[2] == -18.
    assert source == 'camera'
    np.testing.assert_allclose(point, p.command_target_for_kind(point, 'king'))
