"""장기 규칙 (이동 가능한 칸 계산). GUI·카메라 의존 없음.

좌표: x 0~8, y 0~9 (piece_detection / src/janggi/action과 같음). 판 위쪽(y 0~2) 궁성을 쓰는 진영이 top이다.
진영: 'blue' = 초, 'red' = 한. 초가 먼저 둔다.

구현한 규칙
- 궁·사: 자기 궁성 안에서 선을 따라 한 칸 (궁성 대각선 포함)
- 차: 직선으로 막힐 때까지, 궁성 안에서는 대각선도
- 포: 말 하나(포 제외)를 넘어서 이동/잡기, 포는 넘거나 잡을 수 없음, 궁성 대각선도
- 마: 한 칸 직진 후 한 칸 대각선 (직진 칸이 막히면 못 감)
- 상: 한 칸 직진 후 두 칸 대각선 (지나는 칸이 막히면 못 감)
- 졸/병: 앞·옆 한 칸, 상대 궁성 안에서는 앞쪽 대각선도
- 자기 궁이 잡히는 수(장군을 방치하는 수)는 둘 수 없음
빅장·점수 계산·반복수 제한은 다루지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field

COLS, ROWS = 9, 10
Cell = tuple[int, int]
Piece = tuple[str, str]   # (side, kind)
Board = dict[Cell, Piece]
ORTHO = ((1, 0), (-1, 0), (0, 1), (0, -1))

PALACE_CENTERS = {"top": (4, 1), "bottom": (4, 8)}


def other(side: str) -> str:
    return "red" if side == "blue" else "blue"


def on_board(c: Cell) -> bool:
    return 0 <= c[0] < COLS and 0 <= c[1] < ROWS


def palace_of(c: Cell) -> str | None:
    x, y = c
    if 3 <= x <= 5:
        if 0 <= y <= 2:
            return "top"
        if 7 <= y <= 9:
            return "bottom"
    return None


def palace_diag_steps(c: Cell) -> list[Cell]:
    """궁성 대각선으로 연결된 이웃 칸 (모서리 ↔ 중심)."""
    p = palace_of(c)
    if p is None:
        return []
    cx, cy = PALACE_CENTERS[p]
    if c == (cx, cy):
        return [(cx + dx, cy + dy) for dx in (-1, 1) for dy in (-1, 1)]
    if abs(c[0] - cx) == 1 and abs(c[1] - cy) == 1:
        return [(cx, cy)]
    return []


def palace_diag_lines(c: Cell) -> list[list[Cell]]:
    """c에서 궁성 대각선을 따라 뻗는 경로들 (가까운 칸부터). 차·포용."""
    p = palace_of(c)
    if p is None:
        return []
    cx, cy = PALACE_CENTERS[p]
    lines = []
    for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        path, cur = [], c
        while True:
            nxt = (cur[0] + dx, cur[1] + dy)
            # 대각선 위 점은 중심과 그 네 모서리뿐이다
            if palace_of(nxt) != p or abs(nxt[0] - cx) != abs(nxt[1] - cy):
                break
            if abs(cur[0] - cx) != abs(cur[1] - cy):
                break
            path.append(nxt)
            cur = nxt
        if path:
            lines.append(path)
    return lines


@dataclass
class Position:
    board: Board
    top_side: str = "red"   # 위쪽 궁성을 쓰는 진영

    def home_palace(self, side: str) -> str:
        return "top" if side == self.top_side else "bottom"

    def forward(self, side: str) -> int:
        return 1 if side == self.top_side else -1

    # ------------------------------------------------------------------ 의사 합법 수
    def pseudo_moves(self, src: Cell) -> list[Cell]:
        b = self.board
        side, kind = b[src]
        x, y = src
        out: list[Cell] = []

        def free_or_enemy(c):
            return on_board(c) and (c not in b or b[c][0] != side)

        if kind in ("king", "sa"):
            home = self.home_palace(side)
            for dx, dy in ORTHO:
                c = (x + dx, y + dy)
                if palace_of(c) == home and free_or_enemy(c):
                    out.append(c)
            out += [c for c in palace_diag_steps(src) if free_or_enemy(c)]

        elif kind == "soldier":
            f = self.forward(side)
            for c in ((x, y + f), (x - 1, y), (x + 1, y)):
                if free_or_enemy(c):
                    out.append(c)
            if palace_of(src) and palace_of(src) != self.home_palace(side):
                out += [c for c in palace_diag_steps(src) if c[1] - y == f and free_or_enemy(c)]

        elif kind == "horse":
            for dx, dy in ORTHO:
                if (x + dx, y + dy) in b or not on_board((x + dx, y + dy)):
                    continue
                for s in (-1, 1):
                    c = (x + 2 * dx + s * dy, y + 2 * dy + s * dx)
                    if free_or_enemy(c):
                        out.append(c)

        elif kind == "sang":
            for dx, dy in ORTHO:
                p1 = (x + dx, y + dy)
                if p1 in b or not on_board(p1):
                    continue
                for s in (-1, 1):
                    p2 = (x + 2 * dx + s * dy, y + 2 * dy + s * dx)
                    if p2 in b or not on_board(p2):
                        continue
                    c = (x + 3 * dx + 2 * s * dy, y + 3 * dy + 2 * s * dx)
                    if free_or_enemy(c):
                        out.append(c)

        elif kind == "cha":
            rays = [[(x + dx * k, y + dy * k) for k in range(1, 10)] for dx, dy in ORTHO]
            rays += palace_diag_lines(src)
            for ray in rays:
                for c in ray:
                    if not on_board(c):
                        break
                    if c in b:
                        if b[c][0] != side:
                            out.append(c)
                        break
                    out.append(c)

        elif kind == "po":
            rays = [[(x + dx * k, y + dy * k) for k in range(1, 10)] for dx, dy in ORTHO]
            rays += palace_diag_lines(src)
            for ray in rays:
                screen = False
                for c in ray:
                    if not on_board(c):
                        break
                    if not screen:
                        if c in b:
                            if b[c][1] == "po":   # 포는 포를 넘지 못한다
                                break
                            screen = True
                        continue
                    if c in b:
                        if b[c][0] != side and b[c][1] != "po":   # 포는 포를 잡지 못한다
                            out.append(c)
                        break
                    out.append(c)
        return out

    # ------------------------------------------------------------------ 합법 수
    def king_cell(self, side: str) -> Cell | None:
        for c, (s, k) in self.board.items():
            if s == side and k == "king":
                return c
        return None

    def in_check(self, side: str) -> bool:
        k = self.king_cell(side)
        if k is None:
            return False
        return any(k in self.pseudo_moves(c) for c, (s, _) in list(self.board.items()) if s != side)

    def legal_moves(self, src: Cell) -> list[Cell]:
        if src not in self.board:
            return []
        side = self.board[src][0]
        out = []
        for dst in self.pseudo_moves(src):
            nb = dict(self.board)
            nb[dst] = nb.pop(src)
            if not Position(nb, self.top_side).in_check(side):
                out.append(dst)
        return out

    def has_any_move(self, side: str) -> bool:
        return any(self.legal_moves(c) for c, (s, _) in list(self.board.items()) if s == side)


def detect_top_side(board: Board, default: str = "red") -> str:
    """궁 위치로 위쪽 진영을 정한다. 궁이 안 보이면 default."""
    for (x, y), (side, kind) in board.items():
        if kind == "king":
            if y <= 2:
                return side
            if y >= 7:
                return other(side)
    return default


@dataclass
class MoveRecord:
    src: Cell
    dst: Cell
    piece: Piece
    captured: Piece | None
    side: str
    passed: bool = False


@dataclass
class Game:
    board: Board
    top_side: str = "red"
    turn: str = "blue"
    history: list[MoveRecord] = field(default_factory=list)

    @property
    def position(self) -> Position:
        return Position(self.board, self.top_side)

    def legal_moves(self, src: Cell) -> list[Cell]:
        if src not in self.board or self.board[src][0] != self.turn:
            return []
        return self.position.legal_moves(src)

    def move(self, src: Cell, dst: Cell) -> MoveRecord:
        if dst not in self.legal_moves(src):
            raise ValueError(f"둘 수 없는 수: {src} → {dst}")
        rec = MoveRecord(src, dst, self.board[src], self.board.get(dst), self.turn)
        self.board[dst] = self.board.pop(src)
        self.history.append(rec)
        self.turn = other(self.turn)
        return rec

    def pass_turn(self) -> MoveRecord:
        rec = MoveRecord((-1, -1), (-1, -1), ("", ""), None, self.turn, passed=True)
        self.history.append(rec)
        self.turn = other(self.turn)
        return rec

    def undo(self) -> MoveRecord | None:
        if not self.history:
            return None
        rec = self.history.pop()
        if not rec.passed:
            self.board[rec.src] = self.board.pop(rec.dst)
            if rec.captured:
                self.board[rec.dst] = rec.captured
        self.turn = rec.side
        return rec

    def status(self) -> str:
        pos = self.position
        if pos.king_cell(self.turn) is None:
            return "궁 없음"
        if pos.in_check(self.turn):
            return "외통" if not pos.has_any_move(self.turn) else "장군"
        return ""


def infer_move(before: Board, after: Board, side: str) -> tuple[Cell, Cell] | None:
    """두 판 상태의 차이로 side가 둔 한 수를 찾는다 (사람이 실제 판에서 둔 수 읽기).

    정상 수: side의 말 하나가 사라지고(src), 같은 말이 다른 칸에 나타남(dst, 빈칸이었거나 상대 말 자리).
    그 외 변화가 있으면 None.
    """
    changed = {c for c in set(before) | set(after) if before.get(c) != after.get(c)}
    src = [c for c in changed if before.get(c, (None,))[0] == side and c not in after]
    dst = [c for c in changed if after.get(c, (None,))[0] == side and before.get(c) != after.get(c)]
    if len(src) == 1 and len(dst) == 1 and len(changed) == 2 and before[src[0]] == after[dst[0]]:
        return src[0], dst[0]
    return None
