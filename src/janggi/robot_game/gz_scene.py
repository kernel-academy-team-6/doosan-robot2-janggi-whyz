"""Physical board, dynamic pieces and torque-driven robot in calibrated base coordinates.

Entity poses are set only at initial spawn. Physics owns all subsequent motion.
"""

from __future__ import annotations

import re
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

try:
    import google.protobuf  # noqa: F401  (gz.msgs에 필요)
    import gz.transport13  # noqa: F401
except ImportError:   # janggi/.venv에서: 시스템 패키지(gz, protobuf)를 맨 뒤에 붙인다 (venv의 numpy 등이 우선)
    import sys
    sys.path.append("/usr/lib/python3/dist-packages")

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from action.board import DEFAULT_CALIBRATION
from scene_models import CalibratedLayout, DEFAULT_SCENE_CONFIG
from piece_visuals import piece_visuals, piece_physics, piece_height_m
from collision_masks import ENVIRONMENT_CATEGORY, ENVIRONMENT_COLLIDE, contact_xml


def rot_to_quat(r):
    """회전행렬 → 쿼터니언 (x, y, z, w). 대각 성분이 가장 큰 축으로 나눠 어느 회전에서도 안정하다."""
    r = np.asarray(r, float)
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(1.0 + r[i, i] - r[j, j] - r[k, k], 1e-12)) * 2
    q = np.zeros(4)
    q[i], q[j], q[k] = 0.25 * s, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def rpy_to_rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rot(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def origin_T(el):
    t = np.eye(4)
    if el is not None:
        t[:3, 3] = [float(v) for v in el.get("xyz", "0 0 0").split()]
        t[:3, :3] = rpy_to_rot(*[float(v) for v in el.get("rpy", "0 0 0").split()])
    return t


def rot_to_rpy(r):
    return (np.arctan2(r[2, 1], r[2, 2]), -np.arcsin(np.clip(r[2, 0], -1, 1)), np.arctan2(r[1, 0], r[0, 0]))


class GzScene:
    def __init__(self, world, gz_world: str = "empty", calibration=DEFAULT_CALIBRATION,
                 scene_config=DEFAULT_SCENE_CONFIG):
        self.w, self.gz_world = world, gz_world
        self.node = Node()
        self.layout = CalibratedLayout.from_world(world, calibration, scene_config)
        self.n, self.R, self.center = self.layout.normal, self.layout.rotation, self.layout.center
        self._existing_models = set()
        self._last = {}
        self._stop = threading.Event()

    # ------------------------------------------------------------------ gz 서비스
    def _svc(self, name, req, req_t, rep_t=Boolean, timeout=2000):
        ok, rep = self.node.request(f"/world/{self.gz_world}/{name}", req, req_t, rep_t, timeout)
        return ok and getattr(rep, "data", True)

    def _refresh_existing_models(self):
        ok, scene = self.node.request(f"/world/{self.gz_world}/scene/info", Empty(), Empty, Scene, 3000)
        if not ok:
            raise RuntimeError(f"Gazebo world '{self.gz_world}' is not ready (scene/info unavailable)")
        self._existing_models = {model.name for model in scene.model}

    def _create(self, sdf: str):
        req = EntityFactory()
        req.sdf = sdf
        req.allow_renaming = False
        name = ET.fromstring(sdf).find('model').get('name')
        if not self._svc("create/blocking", req, EntityFactory, timeout=30000):
            raise RuntimeError(f"Gazebo model creation failed: {name}")
        self._existing_models.add(name)
        return True

    def _remove(self, name: str):
        # Avoid asking Gazebo to remove absent entities on the first startup.
        if name not in self._existing_models:
            return True
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        result = self._svc("remove/blocking", req, Entity)
        if result:
            self._existing_models.discard(name)
        return result

    def set_camera(self, eye, target):
        """Gazebo GUI 카메라를 eye(m)에서 target을 보게 둔다 (GUI 카메라는 +x로 본다). GUI가 없으면 무시."""
        from gz.msgs10.gui_camera_pb2 import GUICamera

        eye, target = np.asarray(eye, float), np.asarray(target, float)
        x = target - eye
        x /= np.linalg.norm(x)
        y = np.cross([0, 0, 1.0], x)
        y /= np.linalg.norm(y)
        q = rot_to_quat(np.stack([x, y, np.cross(x, y)], axis=1))
        req = GUICamera()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = map(float, eye)
        req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = map(float, q)
        ok, reply = self.node.request("/gui/move_to/pose", req, GUICamera, Boolean, 1000)
        return ok and reply.data

    def wait_for_gui(self, timeout=60.):
        deadline = time.monotonic()+timeout
        ready_checks = 0
        while time.monotonic() < deadline:
            services = self.node.service_list()
            pending_initial_state = any(re.search(r'/[0-9]+/state_async$', name) for name in services)
            if '/gui/move_to/pose' in services and not pending_initial_state:
                ready_checks += 1
                if ready_checks >= 3:
                    return
            else:
                ready_checks = 0
            time.sleep(.2)
        raise RuntimeError('Gazebo GUI initial state is not ready; scene creation has not started')

    def _pose_sdf(self, p, r):
        return " ".join(f"{v:.5f}" for v in (*p, *rot_to_rpy(r)))

    # ------------------------------------------------------------------ 장면 만들기
    def build(self):
        self._refresh_existing_models()
        for name in ["ground_plane", "janggi_table", "janggi_board", "intel_realsense_d455f"] + [f"piece_{i}" for i in range(40)]:
            self._remove(name)
        for sdf in self.board_model_sdfs():
            self._create(sdf)
        self._last = {}
        self._view_ready = self.focus_board()
        import json
        print('[gz] calibrated layout: '+json.dumps(self.layout.summary(), ensure_ascii=False), flush=True)

    def board_model_sdfs(self):
        """Generate board/table/piece visuals without changing the running world."""
        R, c = self.R, self.center
        models = []
        # The calibrated surface is the TOP of the board, not its bottom.
        tc = c+R[:,0]*-.02
        tc[2] = self.layout.config['board']['tabletop_z_mm']/1000.-.01
        models.append(f"""<sdf version="1.9"><model name="janggi_table"><static>true</static>
          <pose>{self._pose_sdf(tc, np.eye(3))}</pose><link name="l"><visual name="v">
          <geometry><box><size>0.75 0.9 0.02</size></box></geometry>
          <material><diffuse>0.45 0.45 0.48 1</diffuse><ambient>0.45 0.45 0.48 1</ambient></material>
          </visual><collision name="table_contact"><geometry><box><size>0.75 0.9 0.02</size></box></geometry>
          <surface>{contact_xml(ENVIRONMENT_CATEGORY, ENVIRONMENT_COLLIDE)}</surface></collision></link></model></sdf>""")
        models.append(self.layout.board_sdf())
        for i, p in enumerate(self.w.pieces):
            height=piece_height_m(p['kind'])
            models.append(f'<sdf version="1.11"><model name="piece_{i}"><static>false</static>'
                         f'<pose>{self._pose_sdf(self._piece_pos(p), R)}</pose><link name="l">'
                         f'{piece_physics(p["kind"], height, **self.layout.config.get("pieces", {}))}'
                         f'{piece_visuals(p["side"], p["kind"], height, p.get("texture_key"))}</link></model></sdf>')
        return models

    def refresh_board_models(self):
        raise RuntimeError('Recreating moving physics bodies is disabled; restart the scene instead')

    def focus_board(self):
        """Move the GUI viewpoint only; never recreate or move scene bodies."""
        c = self.center
        return self.set_camera(c + np.array([.60, -.80, .80]), c + np.array([-.12, 0., .08]))

    def _piece_pos(self, p, tip_mm=None):
        """Initial spawn pose only. Subsequent motion belongs to physics."""
        return p["pos"] / 1000 + self.n * piece_height_m(p['kind']) / 2

    def sync(self, tip_mm=None):
        raise RuntimeError('Direct piece pose updates are disabled in physics mode')

    def start(self, tip_fn=None, hz: float = 200.0, robot_ns: str | None = "dsr01"):
        if tip_fn is not None:
            raise RuntimeError('Kinematic attachment callbacks are disabled')
        self.build()
        self.robot = None
        if robot_ns:
            from physics_robot import PhysicsRobot
            self.robot = PhysicsRobot(self, robot_ns)
            self.robot.build()
        # Load rendering sensors last: first-time render-engine initialization
        # must not hold up the board/robot entity creation requests.
        self._create(self.layout.camera_sdf())

        def loop():
            k = 0
            while not self._stop.is_set():
                tick = time.monotonic()
                try:
                    # GUI loading may lag behind server-side model creation.
                    if not self._view_ready and k < 5*hz and k % max(1,round(hz)) == 0:
                        self._view_ready = self.focus_board()
                    if self.robot:
                        self.robot.sync()
                except Exception as e:  # noqa: BLE001  (report target publication failures)
                    print(f"[gz] 동기화 오류: {e}")
                k += 1
                self._stop.wait(max(0., 1.0 / hz-(time.monotonic()-tick)))

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if getattr(self, "_thread", None):
            self._thread.join(timeout=2.0)
        if getattr(self, "robot", None) is not None:
            self.robot.close()


# --------------------------------------------------------------------------- 로봇 미러

def _resolve_uri(fn: str) -> str:
    m = re.match(r"package://([^/]+)/(.*)", fn)
    if m:
        from ament_index_python.packages import get_package_share_directory
        return "file://" + get_package_share_directory(m.group(1)) + "/" + m.group(2)
    return fn if "://" in fn else "file://" + fn


class GzRobotMirror:
    """ROS feedback reader and diagnostic FK; pose-driven rendering is disabled."""

    def __init__(self, scene: GzScene, ns: str = "dsr01", prefix: str = "rb_"):
        self.scene, self.ns, self.prefix = scene, ns, prefix
        self.q: dict[str, float] = {}
        self._last = {}

    def _ros(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("janggi_gz_mirror")
        self.urdf = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, f"/{self.ns}/robot_description", lambda m: setattr(self, "urdf", m.data), qos)
        self.node.create_subscription(JointState, f"/{self.ns}/joint_states", self._on_js, 10)
        self.executor = ex = SingleThreadedExecutor()
        ex.add_node(self.node)

        def spin():
            try:
                ex.spin()
            except Exception:  # noqa: BLE001  (종료 때 컨텍스트가 먼저 닫혀도 조용히 끝낸다)
                pass
        threading.Thread(target=spin, daemon=True).start()
        t0 = time.time()
        while self.urdf is None and time.time() - t0 < 10:
            time.sleep(0.05)
        if self.urdf is None:
            raise RuntimeError(f"/{self.ns}/robot_description을 받지 못했다 (로봇 bringup 확인)")

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass

    def _on_js(self, m):
        for n, v in zip(m.name, m.position):
            self.q[n] = v

    def build(self):
        raise RuntimeError('Static robot mirror is disabled; use PhysicsRobot')

    def fk(self) -> dict[str, np.ndarray]:
        T = {self.root: np.eye(4)}
        pending = list(self.joints)
        while pending:
            rest = []
            for j in pending:
                if j["parent"] not in T:
                    rest.append(j)
                    continue
                q = 0.0
                if j["type"] in ("revolute", "continuous", "prismatic"):
                    if j["mimic"]:
                        src, mul, off = j["mimic"]
                        q = self.q.get(src, 0.0) * mul + off
                    else:
                        q = self.q.get(j["name"], 0.0)
                m = np.eye(4)
                if j["type"] == "prismatic":
                    m[:3, 3] = np.asarray(j["axis"]) * q
                elif j["type"] != "fixed":
                    m[:3, :3] = axis_rot(j["axis"], q)
                T[j["child"]] = T[j["parent"]] @ j["T"] @ m
            if len(rest) == len(pending):
                break
            pending = rest
        # DR_BASE calibration is based on base_link, even when the URDF root
        # has a non-identity fixed transform to that link.
        if 'base_link' not in T:
            raise RuntimeError('Robot URDF does not contain the calibrated base_link frame')
        base_inverse = np.linalg.inv(T['base_link'])
        return {name: base_inverse @ pose for name, pose in T.items()}

    def sync(self):
        raise RuntimeError('Direct robot pose updates are disabled; use joint forces')
