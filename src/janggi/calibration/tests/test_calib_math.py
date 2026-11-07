"""calib_math 단위 테스트.  실행: .venv/bin/python -m pytest tests  (또는 python3 tests/test_calib_math.py)"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calib_math as cm  # noqa: E402


def random_rotation(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def board_corners_cam():
    # 카메라 아래 0.8 m, 약간 기울어진 40x45 cm 판의 네 모서리
    return np.array([[-0.20, -0.22, 0.80], [0.20, -0.22, 0.82], [0.20, 0.23, 0.84], [-0.20, 0.23, 0.82]])


def test_fit_rigid_exact_coplanar():
    rng = np.random.default_rng(0)
    for _ in range(20):
        r = random_rotation(rng)
        t = rng.normal(size=3)
        cam = board_corners_cam()
        base = cm.transform(r, t, cam)
        r2, t2 = cm.fit_rigid(cam, base)
        assert np.allclose(r2, r, atol=1e-9)
        assert np.allclose(t2, t, atol=1e-9)
        assert np.isclose(np.linalg.det(r2), 1.0)


def test_fit_rigid_noise():
    rng = np.random.default_rng(1)
    r, t = random_rotation(rng), np.array([0.5, -0.1, 0.9])
    cam = board_corners_cam()
    base = cm.transform(r, t, cam) + rng.normal(scale=0.001, size=(4, 3))
    r2, t2 = cm.fit_rigid(cam, base)
    err = np.linalg.norm(cm.transform(r2, t2, cam) - base, axis=1)
    assert err.max() < 0.004


def test_fit_rigid_collinear_raises():
    cam = np.array([[0, 0, 1.0], [0.1, 0, 1.0], [0.2, 0, 1.0], [0.3, 0, 1.0]])
    try:
        cm.fit_rigid(cam, cam)
    except ValueError:
        return
    raise AssertionError("collinear points must raise")


def test_zyz_and_tip_offset():
    # 팁이 아래(base -z)를 보는 자세: A=0, B=180, C=0 → 툴 z축 = base -z
    r = cm.zyz_to_matrix(0, 180, 0)
    assert np.allclose(r @ [0, 0, 1], [0, 0, -1])
    tip = cm.tip_position_m([400, 100, 300, 0, 180, 0], (0, 0, 150))
    assert np.allclose(tip, [0.4, 0.1, 0.15])
    # A, C는 z축 회전
    assert np.allclose(cm.zyz_to_matrix(90, 0, 0) @ [1, 0, 0], [0, 1, 0])


def test_quaternion_roundtrip():
    rng = np.random.default_rng(2)
    for _ in range(50):
        r = random_rotation(rng)
        x, y, z, w = cm.matrix_to_quat_xyzw(r)
        r2 = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        assert np.allclose(r, r2, atol=1e-9)


def test_robust_depth():
    a = np.full((7, 7), 800, np.uint16)
    a[0, 0] = 0  # 무효
    a[1, 1] = 3000  # 이상치
    b = np.full((7, 7), 802, np.uint16)
    z, s, n = cm.robust_depth([a, b, np.zeros((7, 7), np.uint16)])
    assert n == 2 and 800 <= z <= 802 and s <= 1.0
    assert cm.robust_depth([np.zeros((3, 3))]) == (0.0, 0.0, 0)



def _camera_looking_down(rng):
    """base 위 0.75 m에서 판을 내려다보는 카메라 (약간 기울임). T_cam_base 반환."""
    tilt = cm.zyz_to_matrix(rng.uniform(-180, 180), 180 + rng.uniform(-12, 12), rng.uniform(-30, 30))
    r_bc = tilt  # camera axes in base
    c = np.array([0.45, 0.0, 0.80]) + rng.normal(scale=0.03, size=3)
    return cm.invert(r_bc, c)


def _board_base():
    # base 좌표계 판 네 모서리 (40x45 cm, 높이 3 cm, 살짝 회전)
    return np.array([[0.65, 0.20, 0.03], [0.65, -0.20, 0.03], [0.20, -0.20, 0.031], [0.20, 0.20, 0.029]])


def test_pnp_exact_and_noisy():
    rng = np.random.default_rng(3)
    for _ in range(20):
        r_cb, t_cb = _camera_looking_down(rng)
        base = _board_base()
        xy = cm.project_norm(r_cb, t_cb, base)
        r, t = cm.solve_pnp(xy, base)
        assert np.allclose(r, r_cb, atol=1e-6) and np.allclose(t, t_cb, atol=1e-6)
        # 0.5 px 노이즈 (fx≈646) + 로봇 점 0.5 mm 노이즈 → 판 위 오차 수 mm 이내
        xy_n = xy + rng.normal(scale=0.5 / 646, size=xy.shape)
        r2, t2 = cm.solve_pnp(xy_n, base + rng.normal(scale=0.0005, size=base.shape))
        r_bc, c = cm.invert(r2, t2)
        n, d = cm.fit_plane(base)
        center_px = cm.project_norm(r_cb, t_cb, base.mean(axis=0)[None])[0]
        hit = cm.ray_plane(c, r_bc @ np.array([*center_px, 1.0]), n, d)
        assert np.linalg.norm(hit - base.mean(axis=0)) < 0.004


def test_ray_plane_and_fit_plane():
    n, d = cm.fit_plane(np.array([[0, 0, 1.0], [1, 0, 1], [0, 1, 1], [1, 1, 1]]))
    assert np.allclose(n, [0, 0, 1]) and np.isclose(d, -1)
    assert np.allclose(cm.ray_plane([0, 0, 3], [0.1, 0, -1], n, d), [0.2, 0, 1])
    assert cm.ray_plane([0, 0, 3], [0, 0, 1], n, d) is None


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
