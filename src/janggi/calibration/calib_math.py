"""카메라-로봇 보정에 쓰는 순수 수학 함수. ROS/카메라 의존 없음 (pytest로 검증).

좌표 규약
- 카메라 점: RealSense color 광학 좌표계 (x 오른쪽, y 아래, z 앞), 단위 m
- 로봇 점: Doosan base 좌표계 (DR_BASE), posx는 [x, y, z (mm), A, B, C (deg, ZYZ)]
- 결과 T_base_camera: p_base = R @ p_cam + t (단위 m)
"""

from __future__ import annotations

import numpy as np


def zyz_to_matrix(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """Doosan posx의 (A, B, C) ZYZ 오일러각 → 3x3 회전행렬. R = Rz(A) Ry(B) Rz(C)."""
    a, b, c = np.radians([a_deg, b_deg, c_deg])

    def rz(t):
        return np.array([[np.cos(t), -np.sin(t), 0.0], [np.sin(t), np.cos(t), 0.0], [0.0, 0.0, 1.0]])

    def ry(t):
        return np.array([[np.cos(t), 0.0, np.sin(t)], [0.0, 1.0, 0.0], [-np.sin(t), 0.0, np.cos(t)]])

    return rz(a) @ ry(b) @ rz(c)


def tip_position_m(posx, tcp_offset_mm=(0.0, 0.0, 0.0)) -> np.ndarray:
    """posx(현재 TCP 자세)와 TCP 좌표계 기준 추가 오프셋으로 그리퍼 팁 위치(base, m)를 구한다.

    컨트롤러 TCP가 이미 그리퍼 팁으로 설정되어 있으면 오프셋은 0이다.
    """
    p = np.asarray(posx[:3], dtype=float)
    r = zyz_to_matrix(*posx[3:6])
    return (p + r @ np.asarray(tcp_offset_mm, dtype=float)) / 1000.0


def fit_rigid(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """dst ≈ R @ src + t 를 최소제곱으로 푸는 강체 변환 (Kabsch / Umeyama, 스케일 없음).

    점 3개 이상, 한 직선 위에 있지 않아야 한다. 평면 위 점(장기판 네 모서리)도 해가 유일하다.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or len(src) < 3:
        raise ValueError("src, dst는 같은 (N>=3, 3) 배열이어야 한다")
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - cs, dst - cd
    if np.linalg.matrix_rank(a, tol=1e-6) < 2:
        raise ValueError("카메라 점들이 한 직선 위에 있다 (또는 겹친다)")
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    t = cd - r @ cs
    return r, t


def transform(r: np.ndarray, t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return np.asarray(pts, dtype=float) @ np.asarray(r).T + np.asarray(t)


def to_homogeneous(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def matrix_to_quat_xyzw(r: np.ndarray) -> np.ndarray:
    """3x3 회전행렬 → 단위 쿼터니언 [x, y, z, w] (ROS 순서), w >= 0."""
    r = np.asarray(r, dtype=float)
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def robust_depth(patches: list[np.ndarray]) -> tuple[float, float, int]:
    """여러 프레임의 depth 패치(같은 픽셀 주변, 0=무효)에서 대표 depth를 구한다.

    프레임마다 유효 픽셀의 중앙값을 구하고, 다시 프레임 간 중앙값을 쓴다.
    반환: (depth, 프레임 간 표준편차, 유효 프레임 수). 단위는 입력과 같다.
    """
    per_frame = []
    for p in patches:
        v = np.asarray(p).ravel()
        v = v[v > 0]
        if v.size:
            per_frame.append(float(np.median(v)))
    if not per_frame:
        return 0.0, 0.0, 0
    arr = np.asarray(per_frame)
    return float(np.median(arr)), float(arr.std()), len(arr)


def pairwise_distances(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)


# --------------------------------------------------------------------------- PnP (depth 없이)

def fit_plane(pts: np.ndarray) -> tuple[np.ndarray, float]:
    """최소제곱 평면 n·x + d = 0 (|n| = 1). 법선 방향은 +z 쪽으로 맞춘다."""
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    n = np.linalg.svd(pts - c)[2][-1]
    if n[2] < 0:
        n = -n
    return n, float(-n @ c)


def ray_plane(origin, direction, n, d) -> np.ndarray | None:
    """origin + s·direction 과 평면 n·x + d = 0 의 교점. 평행하거나 뒤쪽이면 None."""
    o, v = np.asarray(origin, float), np.asarray(direction, float)
    den = n @ v
    if abs(den) < 1e-12:
        return None
    s = -(n @ o + d) / den
    return None if s <= 0 else o + s * v


def solve_pnp(norm_xy: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """정규화 이미지 좌표(왜곡 제거된 x/z, y/z)와 base 3D 점으로 T_cam_base를 구한다.

    반환 (R, t): p_cam = R @ p_base + t. 4점 이상, 평면 위 점이면 IPPE로 시작해 LM으로 다듬는다.
    """
    import cv2

    obj = np.asarray(base_pts, dtype=np.float64).reshape(-1, 1, 3)
    img = np.asarray(norm_xy, dtype=np.float64).reshape(-1, 1, 2)
    if len(obj) < 4:
        raise ValueError("PnP에는 점이 4개 이상 필요하다")
    k, dist = np.eye(3), np.zeros(5)
    # IPPE는 물체 좌표계의 z=0 평면을 가정하므로, 판 평면 좌표계로 옮겨서 푼다.
    c = obj.reshape(-1, 3).mean(axis=0)
    u, s, vt = np.linalg.svd(obj.reshape(-1, 3) - c)
    if s[1] < 1e-6 * max(s[0], 1e-12):
        raise ValueError("로봇 점들이 한 직선 위에 있다")
    rp = vt  # 행: 판 평면 축 (x, y, 법선)
    if np.linalg.det(rp) < 0:
        rp[2] = -rp[2]
    local = (obj.reshape(-1, 3) - c) @ rp.T
    planar = s[2] < 0.02 * s[1]
    best = None
    if planar:
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(local.reshape(-1, 1, 3), img, k, dist,
                                                    flags=cv2.SOLVEPNP_IPPE)
        if ok:
            for rv, tv in zip(rvecs, tvecs):
                rv, tv = cv2.solvePnPRefineLM(local.reshape(-1, 1, 3), img, k, dist, rv.copy(), tv.copy())
                proj = cv2.projectPoints(local.reshape(-1, 1, 3), rv, tv, k, dist)[0]
                e = float(np.linalg.norm(proj - img, axis=2).mean())
                r_l = cv2.Rodrigues(rv)[0]
                if (r_l @ local.T + tv).T[:, 2].min() > 0 and (best is None or e < best[0]):
                    best = (e, rv, tv)
    if best is None:
        ok, rv, tv = cv2.solvePnP(local.reshape(-1, 1, 3), img, k, dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            raise ValueError("PnP 실패")
        rv, tv = cv2.solvePnPRefineLM(local.reshape(-1, 1, 3), img, k, dist, rv, tv)
        best = (0.0, rv, tv)
    r_l = cv2.Rodrigues(best[1])[0]
    t_l = best[2].reshape(3)
    # p_cam = R_l (Rp (p_base - c)) + t_l
    r = r_l @ rp
    t = t_l - r @ c
    return r, t


def invert(r: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return r.T, -r.T @ t


def project_norm(r_cb, t_cb, base_pts) -> np.ndarray:
    """base 점 → 정규화 이미지 좌표 (T_cam_base 사용)."""
    pc = np.asarray(base_pts, float) @ np.asarray(r_cb).T + t_cb
    return pc[:, :2] / pc[:, 2:3]


def affine_2d(src_xy: np.ndarray, dst_xyz: np.ndarray):
    """3점 이상 (정규화 이미지 → base) 아핀 근사. 4번째 점 위치 미리보기용."""
    a = np.hstack([np.asarray(src_xy, float), np.ones((len(src_xy), 1))])
    m, *_ = np.linalg.lstsq(a, np.asarray(dst_xyz, float), rcond=None)
    return lambda xy: np.hstack([np.asarray(xy, float), [1.0]]) @ m
