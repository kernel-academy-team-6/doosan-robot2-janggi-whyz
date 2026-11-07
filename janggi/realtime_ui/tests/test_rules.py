"""rules.py 테스트. 실행: /usr/bin/python3 -m pytest janggi/realtime_ui/tests"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rules import Game, Position, detect_top_side, infer_move  # noqa: E402

R, B = "red", "blue"


def pos(pieces, top=R):
    return Position(dict(pieces), top)


def mv(p, c):
    return sorted(p.legal_moves(c))


def kings():  # 서로 보지 않게 궁 배치 (top=red)
    return {(4, 1): (R, "king"), (3, 8): (B, "king")}


def test_horse_and_block():
    b = {**kings(), (1, 9): (B, "horse")}
    assert mv(pos(b), (1, 9)) == [(0, 7), (2, 7)]
    assert sorted(pos(b).pseudo_moves((1, 9))) == [(0, 7), (2, 7)]  # (3,8)은 자기 궁
    b[(1, 8)] = (B, "soldier")  # 위쪽 직진 칸 막힘
    assert sorted(pos(b).pseudo_moves((1, 9))) == []


def test_sang_two_blocks():
    b = {**kings(), (2, 9): (B, "sang")}
    assert sorted(pos(b).pseudo_moves((2, 9))) == [(0, 6), (4, 6), (5, 7)]
    b2 = dict(b); b2[(3, 7)] = (R, "soldier")   # (4,6) 가는 길 두 번째 칸 막힘
    assert (4, 6) not in pos(b2).pseudo_moves((2, 9))


def test_cha_slides_and_palace_diagonal():
    b = {**kings(), (0, 9): (B, "cha"), (0, 5): (R, "soldier")}
    m = pos(b).pseudo_moves((0, 9))
    assert (0, 5) in m and (0, 4) not in m and (1, 9) in m and (8, 9) in m
    b = {(4, 1): (R, "king"), (5, 9): (B, "king"), (3, 7): (B, "cha")}
    m = pos(b).pseudo_moves((3, 7))
    assert (4, 8) in m and (5, 9) not in m       # 대각선, 자기 궁에서 멈춤


def test_po_needs_screen_and_cannot_touch_po():
    b = {**kings(), (1, 7): (B, "po")}
    assert pos(b).pseudo_moves((1, 7)) == []           # 넘을 말 없음
    b[(1, 5)] = (R, "soldier")
    m = pos(b).pseudo_moves((1, 7))
    assert (1, 4) in m and (1, 0) in m and (1, 5) not in m
    b[(1, 2)] = (R, "po")
    m = pos(b).pseudo_moves((1, 7))
    assert (1, 2) not in m and (1, 1) not in m          # 포는 포를 잡지 못함
    b2 = {**kings(), (1, 7): (B, "po"), (1, 5): (R, "po"), (1, 3): (R, "cha")}
    assert all(c[0] != 1 or c[1] > 5 for c in pos(b2).pseudo_moves((1, 7)))  # 포를 넘지 못함


def test_po_palace_diagonal():
    b = {(4, 1): (R, "king"), (5, 9): (B, "king"), (3, 7): (B, "po"), (4, 8): (B, "sa")}
    assert (5, 9) not in pos(b).pseudo_moves((3, 7))   # 자기 궁 (잡기 불가)
    b = {(3, 1): (R, "king"), (5, 9): (B, "king"), (3, 2): (B, "po"), (4, 1): (R, "sa"), (5, 0): (R, "cha")}
    assert (5, 0) in pos(b).pseudo_moves((3, 2))


def test_king_and_sa_stay_in_palace():
    b = {(4, 1): (R, "king"), (3, 8): (B, "king"), (3, 0): (R, "sa")}
    assert sorted(pos(b).pseudo_moves((4, 1))) == sorted([(3, 1), (5, 1), (4, 0), (4, 2), (5, 0), (3, 2), (5, 2)])
    assert sorted(pos(b).pseudo_moves((3, 0))) == [(3, 1), (4, 0)]   # (4,1)은 자기 궁


def test_soldier_forward_side_and_enemy_palace():
    b = {**kings(), (0, 6): (B, "soldier")}
    assert sorted(pos(b).pseudo_moves((0, 6))) == [(0, 5), (1, 6)]   # blue는 아래쪽 → 앞은 y-1
    b = {(5, 0): (R, "king"), (3, 8): (B, "king"), (3, 2): (B, "soldier")}
    assert (4, 1) in pos(b).pseudo_moves((3, 2))                     # 상대 궁성 대각선 전진
    b = {(5, 0): (R, "king"), (3, 8): (B, "king"), (6, 3): (R, "soldier")}
    assert sorted(pos(b).pseudo_moves((6, 3))) == [(5, 3), (6, 4), (7, 3)]


def test_cannot_leave_king_in_check():
    b = {(4, 1): (R, "king"), (4, 8): (B, "king"), (4, 5): (B, "sang"), (4, 3): (R, "cha")}
    # 붉은 차가 (4,3)에서 빠지면? 상관없음. 파란 상이 비키면 붉은 차가 파란 궁을 본다
    assert pos(b).legal_moves((4, 5)) == []
    g = Game(dict(b), top_side=R, turn=B)
    assert g.status() == ""


def test_game_move_undo_and_turns():
    b = {**kings(), (0, 9): (B, "cha"), (0, 3): (R, "soldier")}
    g = Game(dict(b), top_side=R, turn=B)
    assert g.legal_moves((0, 3)) == []        # 한 차례가 아님
    rec = g.move((0, 9), (0, 3))
    assert rec.captured == (R, "soldier") and g.turn == R
    g.undo()
    assert g.board == b and g.turn == B


def test_detect_top_side_and_infer_move():
    assert detect_top_side({(4, 1): (B, "king")}) == B
    assert detect_top_side({(4, 8): (B, "king")}) == R
    before = {(0, 9): (B, "cha"), (0, 3): (R, "soldier")}
    after = {(0, 3): (B, "cha")}
    assert infer_move(before, after, B) == ((0, 9), (0, 3))
    assert infer_move(before, {(0, 9): (B, "cha")}, B) is None
