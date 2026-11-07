"""장기판 좌표 기반 로봇 동작. 사용: from action import move"""

from .board import COLS, MAX_X, MAX_Y, ROWS, BoardGeometry
from .mover import BoardMover, move, shutdown

__all__ = ["BoardGeometry", "BoardMover", "move", "shutdown", "COLS", "ROWS", "MAX_X", "MAX_Y"]
