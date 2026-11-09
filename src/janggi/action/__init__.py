"""장기판 좌표 / base 좌표 기반 로봇 동작. 사용: from action import move, move_abs"""

from .board import COLS, MAX_X, MAX_Y, ROWS, BoardGeometry
from .mover import GRIPPER_CLOSE, GRIPPER_OPEN, BoardMover, move, move_abs, shutdown
from .pick_place import PickConfig, PiecePicker

__all__ = ["BoardGeometry", "BoardMover", "PickConfig", "PiecePicker", "move", "move_abs", "shutdown",
           "COLS", "ROWS", "MAX_X", "MAX_Y", "GRIPPER_OPEN", "GRIPPER_CLOSE"]
