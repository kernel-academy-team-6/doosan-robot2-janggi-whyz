"""board_mapping 단위 테스트 (torch 불필요). 실행: /usr/bin/python3 -m pytest janggi/piece_detection/tests"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from piece_detection.board_mapping import BoardMapper, BoardStabilizer  # noqa: E402
from piece_detection.detector import Detection  # noqa: E402

# 실제 보정과 비슷한 모서리 픽셀 (카메라 영상에서 판이 90° 돌아 있음)
CORNERS = [(675, 220), (679, 572), (333, 573), (328, 229)]


def det(name, u, v, conf=0.9):
    side, kind = name.split()
    return Detection(0, name, side, kind, conf, (u - 18, v - 18, u + 18, v + 18))


def test_corners_and_center_roundtrip():
    m = BoardMapper(CORNERS)
    for (x, y), px in zip([(0, 0), (8, 0), (8, 9), (0, 9)], CORNERS):
        assert np.allclose(m.board_to_pixel(x, y), px, atol=1e-6)
        assert np.allclose(m.pixel_to_board(*px, on_piece=False), (x, y), atol=1e-6)
    assert m.grid_pixels().shape == (10, 9, 2)


def test_assign_nearest_threshold_and_conflict():
    m = BoardMapper(CORNERS)
    u, v = m.board_to_pixel(3, 4)
    a = det("blue horse", u + 3, v - 2, 0.8)
    b = det("red cha", u, v, 0.95)          # 같은 칸, 신뢰도 높음 → 남음
    far = det("red po", *m.board_to_pixel(5.5, 5.5))   # 두 교차점 사이 → 할당 안 함
    board = m.assign([a, b, far])
    assert board == {(3, 4): ("red", "cha")}
    assert a.board is None and b.board == (3, 4) and far.board is None


def test_parallax_correction_pulls_toward_principal():
    m = BoardMapper(CORNERS, principal_px=(640, 360), camera_dist_mm=650, piece_height_mm=13)
    true = m.board_to_pixel(0, 0)
    seen = (640, 360) + (true - (640, 360)) * 650 / (650 - 13)   # 말 윗면은 바깥쪽으로 밀려 보인다
    assert np.allclose(m.pixel_to_board(*seen), (0, 0), atol=1e-6)
    assert not np.allclose(BoardMapper(CORNERS).pixel_to_board(*seen), (0, 0), atol=0.05)


def test_stabilizer_majority():
    s = BoardStabilizer(window=5, min_ratio=0.6)
    p = ("blue", "king")
    for k in range(5):
        st = s.update({(4, 8): p} if k != 2 else {})      # 한 프레임 놓쳐도 유지
    assert st == {(4, 8): p}
    for _ in range(3):
        st = s.update({(4, 8): p, (0, 0): ("red", "cha")})
    assert st[(0, 0)] == ("red", "cha")
    s.update({}); s.update({}); st = s.update({})
    assert st == {}
