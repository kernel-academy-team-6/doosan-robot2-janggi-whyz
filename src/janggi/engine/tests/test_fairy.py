"""FEN 변환·좌표 단위 테스트 (엔진 바이너리 불필요). 실행: /usr/bin/python3 -m pytest src/janggi/engine/tests"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from engine.fairy import DEFAULT_ENGINE, JanggiEngine, board_to_fen, fen_to_board, parse_uci, square_name  # noqa: E402

START = "rnba1abnr/4k4/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/4K4/RNBA1ABNR w - - 0 1"


@pytest.mark.parametrize("top", ["red", "blue"])
def test_fen_roundtrip(top):
    board, turn = fen_to_board(START, top)
    assert turn == "blue" and len(board) == 32
    assert board_to_fen(board, top, "blue") == START


def test_orientation():
    red_top, _ = fen_to_board(START, "red")
    blue_top, _ = fen_to_board(START, "blue")
    assert red_top[(4, 1)] == ("red", "king") and red_top[(4, 8)] == ("blue", "king")
    assert blue_top[(4, 1)] == ("blue", "king") and blue_top[(4, 8)] == ("red", "king")
    assert square_name((0, 9), "red") == "a1" and square_name((0, 9), "blue") == "i10"


def test_parse_uci():
    assert parse_uci("a10a9", "red") == ((0, 0), (0, 1))
    assert parse_uci("a10a9", "blue") == ((8, 9), (8, 8))
    assert parse_uci("e9e9", "red") == ((4, 1), (4, 1))   # 한수쉼
    with pytest.raises(ValueError):
        parse_uci("j1a1", "red")


@pytest.mark.skipif(not DEFAULT_ENGINE.exists(), reason="엔진 바이너리 없음 (get_engine.sh)")
def test_engine_move_is_legal():
    board, _ = fen_to_board(START, "red")
    with JanggiEngine() as e:
        legal = e.perft_moves(board, "red", "blue")
        assert len(legal) == 32
        r = e.best_move(board, "red", "blue", movetime_ms=100)
        assert (r.src, r.dst) in legal and board[r.src][0] == "blue"
