"""board.py 단위 테스트. 실행: python3 -m pytest src/janggi/action/tests"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from action.board import BoardGeometry, zyz_to_matrix  # noqa: E402

# 실제 보정과 비슷한 판: x(0→8)는 base -Y, y(0→9)는 base -X, 약간 기울어짐
CORNERS = np.array([[643.0, 237.0, -76.5], [643.0, -119.5, -76.7], [288.4, -119.5, -82.5], [288.4, 239.6, -82.7]])


def geom(abc=(5.2, 178.7, -34.8), offset=(0, 0, 222.0)):
    return BoardGeometry(CORNERS.copy(), np.array(abc), np.array(offset))


def test_corners_exact():
    g = geom()
    for (x, y), c in zip([(0, 0), (8, 0), (8, 9), (0, 9)], CORNERS):
        assert np.allclose(g.board_to_base(x, y), c)


def test_center_and_edges():
    g = geom()
    assert np.allclose(g.board_to_base(4, 4.5), CORNERS.mean(axis=0))
    mid_top = g.board_to_base(4, 0)
    assert np.allclose(mid_top, (CORNERS[0] + CORNERS[1]) / 2)
    w, h = g.spacing_mm()
    assert 44 < w < 45 and 39 < h < 40


def test_range_check():
    g = geom()
    for bad in [(-1, 0), (9, 0), (0, 10), (8.01, 9)]:
        with pytest.raises(ValueError):
            g.board_to_base(*bad)


def test_flange_pose_roundtrip():
    g = geom()
    tip = g.tip_target(3, 7, 30.0)
    pose = g.flange_pose(tip)
    assert np.allclose(g.tip_from_posx(pose), tip)
    assert pose[2] > tip[2] + 200  # 툴이 아래를 보므로 플랜지가 팁보다 ~222 mm 위


def test_height_along_normal():
    g = geom()
    n = g.normal()
    assert n[2] > 0.99
    assert np.isclose(np.dot(g.tip_target(2, 2, 30) - g.board_to_base(2, 2), n), 30)


def test_load_real_calibration_if_present():
    p = Path(__file__).resolve().parents[2] / "calibration" / "config" / "calibration.yaml"
    if not p.exists():
        pytest.skip("calibration.yaml 없음")
    g = BoardGeometry.from_calibration(p)
    assert g.corners_mm.shape == (4, 3)
    assert np.allclose(zyz_to_matrix(0, 180, 0) @ [0, 0, 1], [0, 0, -1])


def test_base_to_board_inverse_and_off_board():
    g = geom()
    for x, y in [(0, 0), (3.3, 7.2), (8, 9), (-1.5, 4.0), (9.5, 0)]:
        p = g.board_to_base(x, y, check=False) + g.normal() * 12.0
        assert np.allclose(g.base_to_board(p), (x, y), atol=1e-3)   # 네 모서리가 완전한 평면은 아니다
        assert np.isclose(g.height_above(p), 12.0, atol=0.3)
    with pytest.raises(ValueError):
        g.board_to_base(-1.5, 0)
