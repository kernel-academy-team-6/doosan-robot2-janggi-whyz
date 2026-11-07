#!/usr/bin/env python3
"""RealSense D455 실시간 3D 포인트클라우드 시각화.

pyrealsense2로 color/depth를 받아 depth를 decimation·후처리한 뒤 3D 점으로 바꾸고,
각 점을 color 이미지에 투영해 색을 입혀 Open3D 창에 실시간으로 그린다.
카메라 수신·필터는 별도 프로세스에서 돌려 렌더링과 GIL 경합을 피한다.

키 조작 (Open3D 창에 포커스가 있을 때):
    Q / ESC  종료
    P        일시정지 / 재개
    F        depth 후처리 필터 on/off
    S        현재 프레임 저장 (PLY + color PNG + depth PNG)
    R        시점 초기화
    마우스    좌클릭 드래그=회전, 휠=줌, Shift/Ctrl+드래그=이동
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import queue
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# RealSense 광학 좌표계(x 오른쪽, y 아래, z 앞)를 Open3D 보기 좋은 좌표계(y 위, z 뒤)로 뒤집는다.
FLIP = np.array([1.0, -1.0, -1.0], dtype=np.float32)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RealSense D455 실시간 3D 시각화")
    p.add_argument("--width", type=int, default=848)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--min-dist", type=float, default=0.2, help="표시할 최소 거리 [m]")
    p.add_argument("--max-dist", type=float, default=3.0, help="표시할 최대 거리 [m]")
    p.add_argument("--decimate", type=int, default=2, choices=range(1, 9), metavar="{1..8}",
                   help="depth 해상도 축소 배율. 1=전체(무거움), 2=가로세로 1/2 (기본)")
    p.add_argument("--voxel", type=float, default=0.0, help="voxel 다운샘플 크기 [m], 0이면 끔")
    p.add_argument("--no-filter", action="store_true", help="depth 후처리 필터 끄고 시작")
    p.add_argument("--show-2d", action="store_true", help="color/depth 2D 이미지 창도 띄움 (OpenCV)")
    p.add_argument("--serial", type=str, default=None, help="여러 대 연결 시 사용할 카메라 시리얼")
    p.add_argument("--save-dir", type=Path, default=Path(__file__).resolve().parent / "captures")
    p.add_argument("--point-size", type=float, default=2.0)
    return p.parse_args()


# --------------------------------------------------------------------------- 캡처 프로세스

class DepthFilters:
    """RealSense 권장 순서의 depth 후처리 체인."""

    def __init__(self, rs, min_dist: float, max_dist: float, decimate: int):
        self.decimation = None
        if decimate > 1:
            self.decimation = rs.decimation_filter()
            self.decimation.set_option(rs.option.filter_magnitude, decimate)
        self.threshold = rs.threshold_filter(min_dist, max_dist)
        self.to_disparity = rs.disparity_transform(True)
        self.spatial = rs.spatial_filter()
        # 기본값(2회 반복)은 프레임당 ~9ms라 30FPS를 못 맞춘다. 1회로도 노이즈 감소는 충분하다.
        self.spatial.set_option(rs.option.filter_magnitude, 1)
        self.temporal = rs.temporal_filter()
        self.from_disparity = rs.disparity_transform(False)
        self.hole_filling = rs.hole_filling_filter(1)  # 1 = farthest from around

    def process(self, depth, enabled: bool):
        if self.decimation is not None:
            depth = self.decimation.process(depth)
        depth = self.threshold.process(depth)
        if enabled:
            depth = self.to_disparity.process(depth)
            depth = self.spatial.process(depth)
            depth = self.temporal.process(depth)
            depth = self.from_disparity.process(depth)
            depth = self.hole_filling.process(depth)
        return depth


def capture_main(args: argparse.Namespace, out_q: mp.Queue, info_q: mp.Queue,
                 stop: mp.Event, paused: mp.Event, filter_on: mp.Event):
    """카메라를 열고 (points f32, colors u8, color_img, depth_img)를 out_q로 보낸다."""
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        info_q.put({"error": str(e)})
        return

    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()
    if depth_sensor.supports(rs.option.visual_preset):
        try:
            depth_sensor.set_option(rs.option.visual_preset, 3)  # High Accuracy
        except RuntimeError:
            pass
    info_q.put({
        "name": dev.get_info(rs.camera_info.name),
        "serial": dev.get_info(rs.camera_info.serial_number),
        "depth_scale": depth_sensor.get_depth_scale(),
    })

    filters = DepthFilters(rs, args.min_dist, args.max_dist, args.decimate)
    pc = rs.pointcloud()
    out_q.cancel_join_thread()  # 종료 시 큐에 남은 프레임 때문에 멈추지 않도록

    try:
        for _ in range(15):  # 자동 노출 안정화
            pipeline.wait_for_frames()
        while not stop.is_set():
            if paused.is_set():
                time.sleep(0.02)
                continue
            try:
                frames = pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError as e:
                print(f"\n[경고] 프레임 수신 실패: {e}", flush=True)
                continue
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue
            depth = filters.process(depth, filter_on.is_set()).as_depth_frame()

            color_img = np.asanyarray(color.get_data()).copy()
            depth_img = np.asanyarray(depth.get_data()).copy()
            # depth 점을 color 카메라에 투영한 텍스처 좌표(0~1)로 색을 가져온다.
            pc.map_to(color)
            points = pc.calculate(depth)
            verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
            tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
            ch, cw = color_img.shape[:2]
            valid = (verts[:, 2] > 0) & (tex[:, 0] >= 0) & (tex[:, 0] < 1) & (tex[:, 1] >= 0) & (tex[:, 1] < 1)
            idx = np.flatnonzero(valid)
            pts = np.take(verts, idx, axis=0)
            pts *= FLIP
            t = np.take(tex, idx, axis=0)
            cols = color_img[(t[:, 1] * ch).astype(np.int32), (t[:, 0] * cw).astype(np.int32)]

            # 렌더가 밀리면 오래된 프레임은 버리고 최신 것만 유지
            try:
                out_q.put_nowait((pts, cols, color_img, depth_img))
            except queue.Full:
                try:
                    out_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    out_q.put_nowait((pts, cols, color_img, depth_img))
                except queue.Full:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()


# --------------------------------------------------------------------------- 렌더 (메인 프로세스)

def depth_colormap(depth_img: np.ndarray, depth_scale: float, max_dist: float):
    import cv2

    d = np.clip(depth_img * (depth_scale / max_dist), 0, 1)
    cm = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_JET)
    cm[depth_img == 0] = 0
    return cm


def save_frame(save_dir: Path, pcd, color_img, depth_img):
    import cv2
    import open3d as o3d

    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # 저장은 원래 카메라 광학 좌표계(y 아래, z 앞, 단위 m)로 되돌려서 한다.
    out = o3d.geometry.PointCloud(pcd)
    out.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * FLIP)
    o3d.io.write_point_cloud(str(save_dir / f"{stamp}_cloud.ply"), out)
    cv2.imwrite(str(save_dir / f"{stamp}_color.png"), color_img[:, :, ::-1])
    # 16bit, 단위 = depth_scale. decimation·필터 적용 후 depth 해상도이며 color와 정렬되어 있지 않다.
    cv2.imwrite(str(save_dir / f"{stamp}_depth.png"), depth_img)
    print(f"\n[저장] {save_dir}/{stamp}_*", flush=True)


def main():
    args = parse_args()
    ctx = mp.get_context("spawn")
    out_q = ctx.Queue(maxsize=2)
    info_q = ctx.Queue()
    stop, paused, filter_on = ctx.Event(), ctx.Event(), ctx.Event()
    if not args.no_filter:
        filter_on.set()

    proc = ctx.Process(target=capture_main, args=(args, out_q, info_q, stop, paused, filter_on),
                       daemon=True)
    proc.start()
    try:
        info = info_q.get(timeout=15)
    except queue.Empty:
        info = {"error": "카메라 응답 없음 (15초 타임아웃)"}
    if "error" in info:
        stop.set()
        proc.join(timeout=3)
        sys.exit(f"[오류] RealSense 시작 실패: {info['error']}\n"
                 f"  - 카메라 연결(USB 3) 확인, 다른 프로그램(realsense-viewer, ROS 드라이버)이 쓰는지 확인\n"
                 f"  - 해상도/FPS 조합 확인 (--width/--height/--fps)")
    print(f"[연결] {info['name']} (S/N {info['serial']}), depth scale {info['depth_scale']:.6f} m/unit, "
          f"{args.width}x{args.height}@{args.fps}, decimate {args.decimate}")
    print("키 조작" + __doc__.split("키 조작", 1)[1], flush=True)

    import open3d as o3d

    state = {"run": True, "save": False, "reset": False}

    def toggle(ev):
        ev.clear() if ev.is_set() else ev.set()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"RealSense 3D - {info['name']}", width=1280, height=800)
    callbacks = {
        ord("Q"): lambda: state.update(run=False),
        256: lambda: state.update(run=False),  # ESC
        ord("P"): lambda: toggle(paused),
        ord("F"): lambda: toggle(filter_on),
        ord("S"): lambda: state.update(save=True),
        ord("R"): lambda: state.update(reset=True),
    }
    for k, fn in callbacks.items():
        vis.register_key_callback(k, lambda v, fn=fn: (fn(), False)[1])

    opt = vis.get_render_option()
    opt.point_size = args.point_size
    opt.background_color = np.array([0.08, 0.08, 0.1])

    pcd = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)  # 카메라 원점
    first = True
    last = None
    fps_t, fps_n = time.time(), 0

    try:
        while state["run"]:
            data = None
            try:
                data = out_q.get(timeout=0.005)
            except queue.Empty:
                pass

            if data is not None:
                last = data
                pts, cols, color_img, depth_img = data
                pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
                pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64) * (1.0 / 255.0))
                if args.voxel > 0:
                    down = pcd.voxel_down_sample(args.voxel)
                    pcd.points, pcd.colors = down.points, down.colors
                if first:
                    vis.add_geometry(pcd)
                    vis.add_geometry(axes)
                    first = False
                    state["reset"] = True
                else:
                    vis.update_geometry(pcd)

                fps_n += 1
                now = time.time()
                if now - fps_t >= 1.0:
                    fps = fps_n / (now - fps_t)
                    fps_t, fps_n = now, 0
                    print(f"\r{fps:5.1f} FPS | {len(pcd.points):7d} pts | "
                          f"filter {'ON ' if filter_on.is_set() else 'OFF'}   ", end="", flush=True)

                if args.show_2d:
                    import cv2
                    cm = depth_colormap(depth_img, info["depth_scale"], args.max_dist)
                    cm = cv2.resize(cm, (color_img.shape[1], color_img.shape[0]),
                                    interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("color | depth", np.hstack([color_img[:, :, ::-1], cm]))

            if args.show_2d:
                import cv2
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    state["run"] = False

            if state["reset"]:
                ctr = vis.get_view_control()
                ctr.set_front([0, 0, 1])
                ctr.set_up([0, 1, 0])
                ctr.set_lookat([0, 0, -1.0])
                ctr.set_zoom(0.5)
                state["reset"] = False

            if state["save"]:
                state["save"] = False
                if last is not None:
                    save_frame(args.save_dir, pcd, last[2], last[3])

            if not vis.poll_events():
                break
            vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        print()
        stop.set()
        proc.join(timeout=3)
        if proc.is_alive():
            proc.terminate()
        vis.destroy_window()
        if args.show_2d:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
