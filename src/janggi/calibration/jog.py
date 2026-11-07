"""누르고 있는 동안 움직이는 조그 상태 관리. ROS/GUI 의존 없음.

눌린 방향들의 합으로 multi_jog 단위 벡터를 만들고, 벡터나 속도가 바뀔 때만 send를 호출한다.
send(vec6, speed_percent)에서 speed 0 또는 영벡터는 정지를 뜻한다.
"""

from __future__ import annotations

from typing import Callable

# 방향 이름 → (축 인덱스, 부호). 축: 0=X, 1=Y, 2=Z (base 좌표계)
DIRECTIONS = {
    "X+": (0, 1.0), "X-": (0, -1.0),
    "Y+": (1, 1.0), "Y-": (1, -1.0),
    "Z+": (2, 1.0), "Z-": (2, -1.0),
}


class JogController:
    def __init__(self, send: Callable[[list[float], float], None], speed: float):
        self._send = send
        self.speed = float(speed)
        self.held: set[str] = set()
        self._last: tuple[tuple[float, ...], float] | None = (tuple([0.0] * 6), 0.0)  # 정지 상태로 시작

    def vector(self) -> list[float]:
        v = [0.0] * 6
        for d in self.held:
            axis, sign = DIRECTIONS[d]
            v[axis] += sign  # 반대 방향을 함께 누르면 상쇄
        return v

    @property
    def moving(self) -> bool:
        return self._last is not None and self._last[1] > 0

    def press(self, d: str):
        if d in DIRECTIONS and d not in self.held:
            self.held.add(d)
            self._update()

    def release(self, d: str):
        if d in self.held:
            self.held.discard(d)
            self._update()

    def set_speed(self, speed: float):
        self.speed = float(speed)
        self._update()

    def stop(self, force: bool = False):
        """모든 방향을 놓고 정지. force면 이미 정지 상태여도 정지 명령을 다시 보낸다."""
        self.held.clear()
        if force:
            self._last = None
        self._update()

    def _update(self):
        v = self.vector()
        cmd = (tuple(v), self.speed) if any(v) else (tuple([0.0] * 6), 0.0)
        if cmd == self._last:
            return
        if cmd[1] == 0 and self._last is not None and self._last[1] == 0:
            return
        self._last = cmd
        self._send(list(cmd[0]), cmd[1])
