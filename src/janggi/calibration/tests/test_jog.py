"""jog.JogController 단위 테스트."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jog import JogController  # noqa: E402


def make(speed=3.0):
    sent = []
    return JogController(lambda v, s: sent.append((v, s)), speed), sent


def test_press_release_sends_start_and_stop_once():
    j, sent = make()
    j.press("X+")
    j.press("X+")  # 중복 누름은 무시
    assert sent == [([1.0, 0, 0, 0, 0, 0], 3.0)]
    assert j.moving
    j.release("X+")
    assert sent[-1] == ([0.0] * 6, 0.0) and len(sent) == 2
    assert not j.moving
    j.release("X+")
    assert len(sent) == 2


def test_combined_and_opposite_directions():
    j, sent = make()
    j.press("X+")
    j.press("Z-")
    assert sent[-1] == ([1.0, 0, -1.0, 0, 0, 0], 3.0)
    j.press("X-")  # X 상쇄 → Z-만 남음
    assert sent[-1] == ([0.0, 0, -1.0, 0, 0, 0], 3.0)
    j.release("Z-")  # 모두 상쇄 → 정지
    assert sent[-1] == ([0.0] * 6, 0.0)


def test_speed_change_while_moving_and_idle():
    j, sent = make()
    j.set_speed(10)  # 정지 중에는 아무것도 안 보냄
    assert sent == []
    j.press("Y-")
    j.set_speed(1)
    assert sent[-1] == ([0, -1.0, 0, 0, 0, 0], 1.0)


def test_force_stop_resends():
    j, sent = make()
    j.stop()
    assert sent == []  # 이미 정지, 불필요한 명령 없음
    j.stop(force=True)
    assert sent == [([0.0] * 6, 0.0)]
    j.press("Z+")
    j.stop(force=True)
    assert sent[-1] == ([0.0] * 6, 0.0) and not j.held
