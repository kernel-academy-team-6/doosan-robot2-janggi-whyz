"""Fairy-Stockfish(장기 지원 Stockfish) UCI 래퍼 + 장기판 상태 ↔ FEN 변환.

    eng = JanggiEngine()                                   # bin/fairy-stockfish (get_engine.sh로 받는다)
    res = eng.best_move(board, top_side="red", turn="red", movetime_ms=1000)
    res.src, res.dst                                       # 우리 판 좌표 (x 0~8, y 0~9)

판 좌표는 piece_detection / action과 같다: (0,0) 좌상단 ~ (8,9) 우하단 (카메라·로봇 보정 기준).
엔진(FEN)은 초(blue, 대문자, 먼저 둠)가 아래(rank 1), 한(red, 소문자)이 위(rank 10)다.
우리 판에서 초가 위쪽(top_side == 'blue')이면 180° 돌려서 넘기고, 받은 수도 되돌린다.
"""

from __future__ import annotations

import queue
import re
import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path

COLS, ROWS = 9, 10
DEFAULT_ENGINE = Path(__file__).resolve().parent / "bin" / "fairy-stockfish"
LETTER = {"king": "k", "cha": "r", "horse": "n", "sang": "b", "sa": "a", "po": "c", "soldier": "p"}
KIND_OF_LETTER = {v: k for k, v in LETTER.items()}
FEN_SIDE = {"blue": "w", "red": "b"}

Cell = tuple[int, int]
Board = dict[Cell, tuple[str, str]]


# --------------------------------------------------------------------------- 좌표 / FEN

def _flip(top_side: str) -> bool:
    return top_side == "blue"


def to_engine_cell(c: Cell, top_side: str) -> Cell:
    """우리 판 좌표 → 엔진 기준 좌표 (한이 위쪽). 180° 회전은 자기 역함수다."""
    return (COLS - 1 - c[0], ROWS - 1 - c[1]) if _flip(top_side) else c


def square_name(c: Cell, top_side: str) -> str:
    """우리 판 좌표 → UCI 칸 이름 (a1 ~ i10)."""
    x, y = to_engine_cell(c, top_side)
    return f"{chr(ord('a') + x)}{ROWS - y}"


def parse_square(s: str, top_side: str) -> Cell:
    x = ord(s[0]) - ord("a")
    y = ROWS - int(s[1:])
    if not (0 <= x < COLS and 0 <= y < ROWS):
        raise ValueError(f"칸 이름이 이상하다: {s}")
    return to_engine_cell((x, y), top_side)


def board_to_fen(board: Board, top_side: str, turn: str) -> str:
    grid = [[None] * COLS for _ in range(ROWS)]
    for c, (side, kind) in board.items():
        x, y = to_engine_cell(c, top_side)
        ch = LETTER[kind]
        grid[y][x] = ch.upper() if side == "blue" else ch
    rows = []
    for y in range(ROWS):
        out, empty = "", 0
        for x in range(COLS):
            if grid[y][x] is None:
                empty += 1
            else:
                if empty:
                    out += str(empty)
                    empty = 0
                out += grid[y][x]
        rows.append(out + (str(empty) if empty else ""))
    return f"{'/'.join(rows)} {FEN_SIDE[turn]} - - 0 1"


def fen_to_board(fen: str, top_side: str = "red") -> tuple[Board, str]:
    rows = fen.split()[0].split("/")
    board: Board = {}
    for y, row in enumerate(rows):
        x = 0
        for tok in re.findall(r"\d+|[a-zA-Z]", row):
            if tok.isdigit():
                x += int(tok)
                continue
            side = "blue" if tok.isupper() else "red"
            board[to_engine_cell((x, y), top_side)] = (side, KIND_OF_LETTER[tok.lower()])
            x += 1
    turn = "blue" if fen.split()[1] == "w" else "red"
    return board, turn


def parse_uci(move: str, top_side: str) -> tuple[Cell, Cell]:
    m = re.fullmatch(r"([a-i]\d{1,2})([a-i]\d{1,2})", move)
    if not m:
        raise ValueError(f"UCI 수 형식이 아니다: {move}")
    return parse_square(m.group(1), top_side), parse_square(m.group(2), top_side)


# --------------------------------------------------------------------------- 엔진

@dataclass
class EngineResult:
    uci: str | None              # 'e7e6', 패스는 출발=도착 (예: 'e9e9'), 둘 수 없으면 None
    src: Cell | None = None
    dst: Cell | None = None
    score: str = ""              # 'cp 35' / 'mate 3' (둘 차례 기준)
    pv: list[str] = field(default_factory=list)

    @property
    def is_pass(self) -> bool:
        return self.src is not None and self.src == self.dst

    @property
    def no_move(self) -> bool:
        return self.uci is None


class JanggiEngine:
    def __init__(self, path: str | Path = DEFAULT_ENGINE, variant: str = "janggi", threads: int = 2,
                 hash_mb: int = 128, skill_level: int | None = None, options: dict | None = None):
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"엔진 바이너리가 없다: {path} (src/janggi/engine/get_engine.sh로 받는다)")
        self.proc = subprocess.Popen([str(path)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
        self._lines: queue.Queue[str | None] = queue.Queue()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        self._lock = threading.Lock()
        self._send("uci")
        self._wait_for("uciok", 10.0)
        opts = {"UCI_Variant": variant, "Threads": threads, "Hash": hash_mb}
        if skill_level is not None:
            opts["Skill Level"] = skill_level
        opts.update(options or {})
        for k, v in opts.items():
            self._send(f"setoption name {k} value {v}")
        self._send("isready")
        self._wait_for("readyok", 10.0)

    def _read(self):
        for line in self.proc.stdout:
            self._lines.put(line.rstrip("\n"))
        self._lines.put(None)

    def _send(self, cmd: str):
        self.proc.stdin.write(cmd + "\n")
        self.proc.stdin.flush()

    def _wait_for(self, prefix: str, timeout: float) -> list[str]:
        out = []
        while True:
            try:
                line = self._lines.get(timeout=timeout)
            except queue.Empty:
                raise TimeoutError(f"엔진 응답 없음 ('{prefix}' 대기)") from None
            if line is None:
                raise RuntimeError("엔진이 종료됐다: " + " | ".join(out[-5:]))
            out.append(line)
            if line.startswith(prefix):
                return out

    def best_move(self, board: Board, top_side: str, turn: str, movetime_ms: int = 1000,
                  depth: int | None = None) -> EngineResult:
        fen = board_to_fen(board, top_side, turn)
        with self._lock:
            self._send(f"position fen {fen}")
            self._send(f"go depth {depth}" if depth else f"go movetime {int(movetime_ms)}")
            lines = self._wait_for("bestmove", movetime_ms / 1000.0 + 30.0)
        best = lines[-1].split()[1] if len(lines[-1].split()) > 1 else "(none)"
        score, pv = "", []
        for line in reversed(lines):
            if line.startswith("info") and " pv " in line:
                m = re.search(r"score (cp -?\d+|mate -?\d+)", line)
                score = m.group(1) if m else ""
                pv = line.split(" pv ", 1)[1].split()
                break
        if best in ("(none)", "0000"):
            return EngineResult(None, score=score, pv=pv)
        src, dst = parse_uci(best, top_side)
        return EngineResult(best, src, dst, score, pv)

    def perft_moves(self, board: Board, top_side: str, turn: str) -> list[tuple[Cell, Cell]]:
        """엔진 규칙으로 둘 수 있는 모든 수 (사람이 둔 수 검증용)."""
        with self._lock:
            self._send(f"position fen {board_to_fen(board, top_side, turn)}")
            self._send("go perft 1")
            lines = self._wait_for("Nodes searched", 10.0)
        out = []
        for line in lines:
            m = re.match(r"([a-i]\d{1,2}[a-i]\d{1,2}): \d+", line)
            if m:
                out.append(parse_uci(m.group(1), top_side))
        return out

    def close(self):
        if self.proc.poll() is None:
            try:
                self._send("quit")
                self.proc.wait(timeout=2.0)
            except (OSError, subprocess.TimeoutExpired):
                self.proc.kill()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
