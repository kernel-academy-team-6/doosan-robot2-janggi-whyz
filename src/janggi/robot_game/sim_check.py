#!/usr/bin/env python3
"""사람 ↔ 로봇 대국 파이프라인을 시뮬레이터(Doosan 에뮬레이터)에서 확인한다. 카메라·실제 말 없이 돈다.

가상 세계:
    - 말은 base 좌표(mm)로 판 위에 놓여 있다. 손으로 놓은 것처럼 교차점에서 몇 mm씩 비켜 놓는다.
    - 가상 카메라: 말 윗면 중심을 보정 결과(T_base_camera, 내부 파라미터)로 픽셀에 투영 + 잡음 → 가짜 YOLO 박스
      → 실제 앱과 같은 BoardMapper.assign / BoardStabilizer로 판 상태를 만든다.
    - 그리퍼: 닫힐 때 팁(에뮬레이터 get_current_posx로 읽은 실제 위치) 근처의 말을 붙잡고, 열릴 때 그 자리에 놓는다.
사람 수는 무작위 합법 수(또는 --human engine). 로봇 수는 session.RobotGame이 엔진 → move_abs/move로 실행한다.

턴마다 확인:
    - 집을 때 팁과 실제 말 중심의 수평 거리 (카메라 위치로 집으므로 교차점보다 가까워야 한다)
    - 로봇이 둔 뒤 가상 세계의 판 == 게임 판 (놓은 말이 목표 교차점에 있음)
    - 잡은 말이 판 바깥(보정 사각형 밖)에 있음

    ./run_sim_check.sh                  # 에뮬레이터: ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual ...
    ./run_sim_check.sh --turns 6 --seed 3
    ./run_sim_check.sh --no-robot       # 로봇 없이 엔진·좌표까지만
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from session import RobotGame, RobotGameConfig, TurnError, median_centers, piece_name  # noqa: E402  (sys.path 설정 포함)

from action import BoardGeometry, BoardMover, PickConfig, PiecePicker  # noqa: E402
from action.board import DEFAULT_CALIBRATION  # noqa: E402
from engine import JanggiEngine, fen_to_board  # noqa: E402
from piece_detection import BoardMapper, BoardStabilizer, CameraToBase  # noqa: E402
from piece_detection.detector import Detection  # noqa: E402
from piece_detection.labels import CLASS_NAMES  # noqa: E402
from rules import Game  # noqa: E402

START_FEN = "rnba1abnr/4k4/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/4K4/RNBA1ABNR w - - 0 1"
MODEL_KIND = {"soldier": "solder"}


class SimWorld:
    """가상 장기판 + 가상 카메라 + 가상 그리퍼."""

    def __init__(self, board, geom: BoardGeometry, cam: CameraToBase, mapper: BoardMapper, rng,
                 place_noise_mm=5.0, pixel_noise=1.0, piece_h=5.0, window=8):
        self.geom, self.cam, self.mapper, self.rng = geom, cam, mapper, rng
        self.place_noise, self.pixel_noise, self.piece_h, self.window = place_noise_mm, pixel_noise, piece_h, window
        self.pieces = []   # dict(side, kind, pos(mm, 판 평면 위 말 바닥 중심), state 'board'|'held'|'off')
        for cell, (side, kind) in board.items():
            self.pieces.append(dict(side=side, kind=kind, pos=self._placed(cell), state="board"))
        self.held = None
        self.human_captured = 0
        self.grasp_log = []   # (말, 팁-말 수평 거리 mm, 교차점-말 수평 거리 mm)

    def _placed(self, cell):
        """손으로 놓은 것처럼 교차점에서 비켜 놓은 위치."""
        off = self.rng.normal(0, self.place_noise / 1.5, 2).clip(-self.place_noise * 1.6, self.place_noise * 1.6)
        return self.geom.board_to_base(cell[0] + off[0] / 44.5, cell[1] + off[1] / 39.4, check=False)

    def _horiz(self, a, b):
        n = self.geom.normal()
        d = np.asarray(a, float) - np.asarray(b, float)
        return float(np.linalg.norm(d - n * (n @ d)))

    def cell_of(self, p):
        q = self.geom.base_to_board(p)
        c = (int(round(q[0])), int(round(q[1])))
        return c, float(np.hypot(*(q - c)))

    def board(self):
        out = {}
        for p in self.pieces:
            if p["state"] == "board":
                c, _ = self.cell_of(p["pos"])
                if c in out:
                    raise AssertionError(f"한 교차점에 말 두 개: {c}")
                out[c] = (p["side"], p["kind"])
        return out

    # ------------------------------------------------------------------ 사람
    def human_move(self, src, dst):
        for p in self.pieces:
            if p["state"] == "board" and self.cell_of(p["pos"])[0] == dst:
                p["state"] = "off"    # 사람이 잡은 말은 판 밖(로봇 버리는 쪽의 반대편)으로 치운다
                col, row = divmod(self.human_captured, 10)
                p["pos"] = self.geom.board_to_base(9.5 + col, row, check=False)
                self.human_captured += 1
        for p in self.pieces:
            if p["state"] == "board" and self.cell_of(p["pos"])[0] == src:
                p["pos"] = self._placed(dst)
                return
        raise AssertionError(f"사람이 옮길 말이 {src}에 없다")

    # ------------------------------------------------------------------ 그리퍼 (PiecePicker.on_grip)
    def on_grip(self, cmd, tip):
        if cmd >= 600:   # 닫힘
            best = None
            for p in self.pieces:
                if p["state"] != "board":
                    continue
                d = self._horiz(tip, p["pos"])
                h = self.geom.height_above(tip)
                if d < 15.0 and -1.0 <= h <= self.piece_h + 1.0 and (best is None or d < best[0]):
                    best = (d, p)
            if best is None:
                raise AssertionError(f"그리퍼가 닫혔는데 잡힌 말이 없다 (팁 {np.round(tip, 1)})")
            d, p = best
            c, _ = self.cell_of(p["pos"])
            grid_d = self._horiz(self.geom.board_to_base(*c, check=False), p["pos"])
            self.grasp_log.append((piece_name((p["side"], p["kind"])), d, grid_d))
            p["state"], self.held = "held", p
        elif self.held is not None:   # 열림: 지금 팁 자리에 내려놓는다
            p = self.held
            p["pos"] = tip - self.geom.normal() * self.geom.height_above(tip)
            q = self.geom.base_to_board(p["pos"])
            inside = -0.5 <= q[0] <= 8.5 and -0.5 <= q[1] <= 9.5
            p["state"] = "board" if inside else "off"
            self.held = None

    # ------------------------------------------------------------------ 가상 카메라 (vision 인터페이스)
    def detections(self):
        dets = []
        n = self.geom.normal()
        for p in self.pieces:
            if p["state"] != "board":
                continue
            u, v = self.cam.base_to_pixel(p["pos"] + n * self.piece_h) + self.rng.normal(0, self.pixel_noise, 2)
            r = 17 + self.rng.normal(0, 1.0)
            name = f"{p['side']} {MODEL_KIND.get(p['kind'], p['kind'])}"
            dets.append(Detection(CLASS_NAMES.index(name), name, p["side"], p["kind"],
                                  float(0.8 + 0.15 * self.rng.random()), (u - r, v - r, u + r, v + r)))
        return dets

    def fresh_state(self):
        stab = BoardStabilizer(window=self.window)
        hist = deque(maxlen=self.window)
        for _ in range(self.window):
            dets = self.detections()
            stab.update(self.mapper.assign(dets))
            hist.append({d.board: d.center for d in dets if d.board is not None})
        board = stab.state()
        return board, median_centers(board, hist)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--turns", type=int, default=4, help="최소 턴 수 (잡는 수가 한 번 나올 때까지 --max-turns까지 더 둔다)")
    ap.add_argument("--max-turns", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--human", choices=("random", "engine"), default="random")
    ap.add_argument("--fen", default=START_FEN, help="시작 판 (초=대문자가 아래). 우리 판에는 --top-side 방향으로 놓는다")
    ap.add_argument("--top-side", choices=("red", "blue"), default="red", help="우리 판(카메라 기준) 위쪽 진영")
    ap.add_argument("--movetime", type=int, default=300, help="엔진 생각 시간 [ms]")
    ap.add_argument("--vel", type=float, default=300.0, help="로봇 직선 속도 [mm/s]")
    ap.add_argument("--place-noise", type=float, default=5.0, help="말을 교차점에서 비켜 놓는 정도 [mm]")
    ap.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    ap.add_argument("--no-robot", action="store_true", help="로봇 없이 엔진·좌표까지만")
    ap.add_argument("--gazebo", action="store_true", help="장기판·말을 Gazebo에 띄워 보여준다 (sim_gazebo.launch.py)")
    args = ap.parse_args()

    if args.gazebo and args.calibration == DEFAULT_CALIBRATION:
        from simulation_calibration import ensure_simulation_calibration
        args.calibration = ensure_simulation_calibration()
    rng = np.random.default_rng(args.seed)
    random.seed(args.seed)
    geom = BoardGeometry.from_calibration(args.calibration)
    cam = CameraToBase.from_calibration(args.calibration)
    mapper = BoardMapper.from_calibration(args.calibration, piece_height_mm=5.0)
    board0, _ = fen_to_board(args.fen, args.top_side)
    world = SimWorld(board0, geom, cam, mapper, rng, place_noise_mm=args.place_noise)

    engine = JanggiEngine()
    human_engine = JanggiEngine(skill_level=0) if args.human == "engine" else None
    mover = picker = None
    if not args.no_robot:
        mover = BoardMover(calibration=args.calibration, vel=(args.vel, 60.0), acc=(args.vel * 2, 120.0))
        mover.grip_wait_s = 0.2
        picker = PiecePicker(mover, PickConfig(), on_grip=world.on_grip)
    scene = None
    if args.gazebo:
        from gz_scene import GzScene
        scene = GzScene(world, calibration=args.calibration)
        scene.start(tip_fn=mover.current_tip if mover else None)
    cfg = RobotGameConfig(human_side="blue", movetime_ms=args.movetime)
    rg = RobotGame(engine, cam, geom, picker, cfg, log=lambda m: print("  " + m, flush=True))

    failures = []

    def check(cond, msg):
        print(f"    [{'OK' if cond else 'FAIL'}] {msg}", flush=True)
        if not cond:
            failures.append(msg)

    try:
        t0 = time.time()
        board, _ = world.fresh_state()
        check(board == board0, f"처음 인식한 판 == 실제 판 ({len(board)}개)")
        rg.start(board)
        captures = 0
        for turn in range(1, args.max_turns + 1):
            g: Game = rg.game
            # 1. 사람이 둔다 (실제 판의 말을 옮김)
            if human_engine:
                r = human_engine.best_move(g.board, g.top_side, g.turn, 50)
                mv = (r.src, r.dst)
            else:
                moves = engine.perft_moves(g.board, g.top_side, g.turn)
                moves = [m for m in moves if m[0] != m[1]] or moves
                mv = random.choice(moves)
            print(f"\n=== 턴 {turn}: 사람 {mv[0]}→{mv[1]} (실제 판에서 옮김) → '착수 완료'", flush=True)
            world.human_move(*mv)
            # 2~5. 착수 완료
            n_grasp = len(world.grasp_log)
            ts = time.time()
            try:
                plan = rg.human_done(world)
            except TurnError as e:
                check(False, f"사람 수 읽기 실패: {e}")
                break
            dt = time.time() - ts
            if plan.uci is None:
                print("  대국 끝")
                break
            print(f"    턴 소요 {dt:.1f}s")
            if picker is not None:
                for name, d, grid_d in world.grasp_log[n_grasp:]:
                    check(d < 3.0, f"{name} 집기: 팁-말 중심 {d:.1f} mm (교차점으로 집었다면 {grid_d:.1f} mm)")
                wb = world.board()
                check(wb == g.board, "로봇이 둔 뒤 실제 판 == 게임 판")
                if plan.dst is not None and not plan.is_pass:
                    p = next(p for p in world.pieces if p["state"] == "board"
                             and world.cell_of(p["pos"])[0] == plan.dst)
                    _, off = world.cell_of(p["pos"])
                    check(off < 0.05, f"놓은 말 {plan.dst} 교차점 오차 {off * 44:.1f} mm")
                if plan.captured:
                    captures += 1
                    q = geom.base_to_board(picker.discard_spot(picker.discarded - 1))
                    off = [p for p in world.pieces if p["state"] == "off"]
                    check(not (0 <= q[0] <= 8 and 0 <= q[1] <= 9),
                          f"잡은 말 {piece_name(plan.captured)} → 판 밖 ({q[0]:.1f}, {q[1]:.1f}) 칸 좌표")
                    check(len(off) >= 1, f"판 밖 말 {len(off)}개")
                tip = mover.current_tip()
                check(geom.height_above(tip) > 100, f"관찰 자세 복귀 (판 위 {geom.height_above(tip):.0f} mm)")
            elif not plan.is_pass:
                captures += bool(plan.captured)
                world.human_move(plan.src, plan.dst)   # 로봇 없이: 결과만 세계에 반영
            if turn >= args.turns and captures:
                break
        print(f"\n총 {time.time() - t0:.0f}s, 로봇이 잡은 수 {captures}번, 집기 {len(world.grasp_log)}번")
        if not captures:
            print("  (잡는 수가 나오지 않았다. --max-turns를 늘리거나 --seed를 바꾼다)")
    finally:
        if scene:
            time.sleep(0.5)
            scene.stop()
        engine.close()
        if human_engine:
            human_engine.close()
        if mover:
            mover.close()
    if failures:
        print(f"\n실패 {len(failures)}건")
        sys.exit(1)
    print("\n모두 통과")


if __name__ == "__main__":
    main()
