#!/usr/bin/env python3
"""실제 대국 UI(realtime_ui/app.py --robot)를 카메라 없이 시뮬레이터로 돌린다.

카메라 인식 스레드 대신 sim_check.SimWorld(가상 판·카메라·그리퍼)를 쓰고, 로봇은 Doosan 에뮬레이터로 움직인다.
사람 수는 자동으로 둔다(무작위 합법 수): '대국 시작' → (사람 수 → '착수 완료') × N 을 UI 버튼과 같은 함수로 누른다.

    ./run_sim_app.sh --turns 3
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "janggi" / "realtime_ui"))
from sim_check import START_FEN, SimWorld  # noqa: E402  (session이 sys.path를 맞춘다)

import app as ui  # noqa: E402
from action import BoardGeometry  # noqa: E402
from action.board import DEFAULT_CALIBRATION  # noqa: E402
from engine import fen_to_board  # noqa: E402
from piece_detection import BoardMapper, BoardStabilizer, CameraToBase  # noqa: E402

SIDE_BGR = {"blue": (255, 110, 30), "red": (40, 40, 230)}


class SimWorker:
    """app.Worker 대신: 가상 세계를 그린 프레임과 가짜 검출을 준다."""

    def __init__(self, world: SimWorld, mapper: BoardMapper, args):
        self.world, self.mapper, self.args = world, mapper, args
        self.stab = BoardStabilizer(window=args.window)
        self.use_roi, self.fps, self.error = False, 30.0, None
        self.last_frame_t = time.time()

    def render(self):
        img = np.full((self.args.height, self.args.width, 3), (120, 160, 200), np.uint8)
        g = self.mapper.grid_pixels()
        for y in range(g.shape[0]):
            cv2.line(img, tuple(map(int, g[y, 0])), tuple(map(int, g[y, -1])), (40, 60, 90), 1, cv2.LINE_AA)
        for x in range(g.shape[1]):
            cv2.line(img, tuple(map(int, g[0, x])), tuple(map(int, g[-1, x])), (40, 60, 90), 1, cv2.LINE_AA)
        n = self.world.geom.normal()
        for p in list(self.world.pieces):
            if p["state"] == "board":
                u, v = self.world.cam.base_to_pixel(p["pos"] + n * self.world.piece_h)
                cv2.circle(img, (int(u), int(v)), 16, (210, 235, 245), -1, cv2.LINE_AA)
                cv2.circle(img, (int(u), int(v)), 16, SIDE_BGR[p["side"]], 2, cv2.LINE_AA)
        return img

    def get(self):
        dets = self.world.detections()
        raw = self.mapper.assign(dets, self.args.max_dist)
        self.last_frame_t = time.time()
        return self.render(), dets, raw, self.stab.update(raw)

    def fresh_state(self, timeout=10.0):
        return self.world.fresh_state()

    def stop(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--turns", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--robot-vel", type=float, default=300.0)
    ap.add_argument("--no-motion", action="store_true")
    ap.add_argument("--gazebo", action="store_true", help="장기판·말을 Gazebo에 띄워 보여준다 (sim_gazebo.launch.py)")
    ap.add_argument("--screenshot", type=Path, help="마지막 화면을 이 경로에 PNG로 저장")
    a = ap.parse_args()
    sys.argv = [sys.argv[0], "--robot", "--movetime", "300", "--robot-vel", str(a.robot_vel), "--source", "sim"] + \
        (["--no-motion"] if a.no_motion else [])
    args = ui.parse_args()

    rng = np.random.default_rng(a.seed)
    random.seed(a.seed)
    geom = BoardGeometry.from_calibration(DEFAULT_CALIBRATION)
    mapper = BoardMapper.from_calibration(DEFAULT_CALIBRATION, piece_height_mm=args.piece_height)
    board0, _ = fen_to_board(START_FEN, "red")
    world = SimWorld(board0, geom, CameraToBase.from_calibration(DEFAULT_CALIBRATION), mapper, rng)
    app = ui.App(args, SimWorker(world, mapper, args), mapper)

    results, step = [], {"turn": 0, "phase": "start", "t": time.time()}
    log = []

    def finish():
        log.extend(app.log.get(0, "end"))
        if a.screenshot:
            from PIL import ImageGrab
            r = app.root
            r.update()
            x, y = r.winfo_rootx(), r.winfo_rooty()
            ImageGrab.grab((x, y, x + r.winfo_width(), y + r.winfo_height())).save(a.screenshot)
        app.quit()

    def drive():
        if app.robot_busy:
            return app.root.after(200, drive)
        if step["phase"] == "start":
            app.robot_start()
            step["phase"] = "started"
        elif step["phase"] in ("started", "robot_done"):
            if step["phase"] == "robot_done":
                plan = app.rg.last_plan
                if a.no_motion and plan and plan.src is not None and not plan.is_pass:
                    world.human_move(plan.src, plan.dst)   # 로봇이 안 움직이므로 세계에 직접 반영
                ok = world.board() == app.game.board
                results.append(ok)
                print(f"[턴 {step['turn']}] {app.robot_phase} | 실제 판 == 게임 판: {ok}", flush=True)
            if app.rg is None or app.rg.game is None:
                print("[실패] 대국이 시작되지 않았다:", app.robot_phase)
                results.append(False)
                return finish()
            if step["turn"] >= a.turns:
                return app.root.after(800, finish)
            g = app.game
            moves = [m for m in app.rg.engine.perft_moves(g.board, g.top_side, g.turn) if m[0] != m[1]]
            mv = random.choice(moves)
            step["turn"] += 1
            print(f"[턴 {step['turn']}] 사람 {mv[0]}→{mv[1]} 두고 '착수 완료'", flush=True)
            world.human_move(*mv)
            app.robot_human_done()
            step["phase"] = "robot_done"
        app.root.after(300, drive)

    if not a.no_motion:
        world_grip = world.on_grip
        orig = ui.App._ensure_robot

        def ensure(self):   # 가상 그리퍼를 PiecePicker에 연결
            new = self.rg is None
            orig(self)
            if new and self.rg.picker is not None:
                self.rg.picker.on_grip = world_grip
                self.rg.picker.mover.grip_wait_s = 0.2
                if a.gazebo:
                    from gz_scene import GzScene
                    GzScene(world).start(tip_fn=self.rg.picker.mover.current_tip)
        ui.App._ensure_robot = ensure
    app.root.after(1000, drive)
    app.run()
    print("\n기보:\n  " + "\n  ".join(log))
    print("\n집기 오차: " + ", ".join(f"{n} {d:.1f}mm(교차점 {gd:.1f})" for n, d, gd in world.grasp_log))
    ok = results and all(results) and not any(line.startswith("!") for line in log)
    print("모두 통과" if ok else "실패")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
