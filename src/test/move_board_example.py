#!/usr/bin/env python3
"""장기판 좌표 이동 예제: (0,0) → (1,1) → … → (8,8) 대각선으로 이동한 뒤 마지막에 (8,9).

    ./run_move_example.sh                 # 각 교차점 판 표면(높이 0)까지, 사이 이동은 30 mm 높이로
    ./run_move_example.sh --height 10     # 교차점 위 10 mm에서 멈춤
    ./run_move_example.sh --dry-run       # 로봇 없이 목표 좌표만 출력

로봇 드라이버(bringup)가 떠 있어야 한다. 비상정지 버튼을 손 닿는 곳에 둔다.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "janggi"))
from action import BoardGeometry, BoardMover  # noqa: E402
from action.board import DEFAULT_CALIBRATION  # noqa: E402

PATH = [(i, i) for i in range(9)] + [(8, 9)]


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--height", type=float, default=0.0, help="도착 높이: 교차점 위 팁 높이 [mm] (0 = 판 표면)")
    p.add_argument("--travel", type=float, default=30.0, help="교차점 사이 이동 높이 [mm]")
    p.add_argument("--vel", type=float, default=100.0, help="직선 속도 [mm/s]")
    p.add_argument("--pause", type=float, default=0.5, help="각 점에서 멈추는 시간 [s]")
    p.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    p.add_argument("--dry-run", action="store_true", help="로봇을 움직이지 않고 목표만 출력")
    args = p.parse_args()

    board = BoardGeometry.from_calibration(args.calibration)
    w, h = board.spacing_mm()
    print(f"칸 간격: 가로 {w:.1f} mm, 세로 {h:.1f} mm / 도착 높이 {args.height:g} mm, 이동 높이 {args.travel:g} mm")

    if args.dry_run:
        for x, y in PATH:
            t = board.tip_target(x, y, args.height)
            print(f"  ({x},{y}) → 팁 base ({t[0]:7.1f}, {t[1]:7.1f}, {t[2]:6.1f}) mm")
        return

    with BoardMover(calibration=args.calibration, height_mm=args.height, travel_mm=args.travel,
                    vel=(args.vel, 30.0),
                    acc=(args.vel * 2, 60.0)) as mover:
        try:
            for x, y in PATH:
                target = board.tip_target(x, y, args.height)
                tip = mover.move(x, y)
                err = tip - target
                print(f"  ({x},{y}) 도착  팁 ({tip[0]:7.1f}, {tip[1]:7.1f}, {tip[2]:6.1f}) mm  "
                      f"목표 대비 |{(err ** 2).sum() ** 0.5:.2f}| mm", flush=True)
                time.sleep(args.pause)
        except KeyboardInterrupt:
            mover.stop()
            print("\n중단: 로봇 정지 명령을 보냈다.")


if __name__ == "__main__":
    main()
