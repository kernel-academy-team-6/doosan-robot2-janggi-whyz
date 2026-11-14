"""Never snap a pick's camera centre XY to the grid, even far from a crossing."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2])]
from session import RobotGame
from action.board import BoardGeometry
from action.pick_place2 import load_pick_place_settings


def game():
    g=RobotGame.__new__(RobotGame);g.geom=BoardGeometry.from_calibration();g.picker=None
    g.cfg=SimpleNamespace(command_settings=load_pick_place_settings()[0],max_offset_mm=1.,piece_height_mm=5.)
    g.log=lambda *_:None
    # Deliberately nowhere near the requested (0,0) crossing: old code snapped.
    g.cam=SimpleNamespace(pixel_to_base=lambda *args,height_mm:np.array([500.123,100.456,-10.]))
    return g


def test_detected_center_xy_is_preserved_and_only_board_height_sets_z():
    g=game()
    point,source=g.grasp_from_camera((0,0),{(0,0):(123,456)},'king')
    assert source=='camera'
    assert point[:2]==pytest.approx([500.123,100.456],abs=1e-12)
    assert point[2]==pytest.approx(g.geom.command_surface_z(point,g.cfg.command_settings)+2)


def test_missing_center_fails_instead_of_returning_grid_coordinates():
    with pytest.raises(RuntimeError,match='격자점으로 대체하지 않습니다'):
        game().grasp_from_camera((0,0),{},'soldier')


@pytest.mark.parametrize('value',[[float('nan'),0,0],[0,0],None])
def test_invalid_center_conversion_is_not_replaced_with_grid(value):
    g=game();g.cam.pixel_to_base=lambda *args,**kwargs:value
    with pytest.raises(RuntimeError,match='변환하지 못했습니다'):
        g.grasp_from_camera((0,0),{(0,0):(1,2)},'soldier')
