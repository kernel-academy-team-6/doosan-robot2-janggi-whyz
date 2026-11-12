from collections import deque
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import App, Worker
from rules import Game
from session import RobotGame, RobotGameConfig, RobotPlan
from piece_detection.detector import Detection


def var(value):
    return SimpleNamespace(get=lambda:value,set=lambda v:None)


def test_current_snapshot_ignores_old_generation_stability_and_missing_revision():
    worker=Worker.__new__(Worker); worker.lock=threading.Lock()
    known={(0,6):('blue','soldier')}
    visible={**known,(1,1):('unknown','piece')}
    detection=Detection(0,'blue soldier','blue','soldier',.9,(10,20,30,40),board=(0,6))
    worker.latest=(np.zeros((2,2,3)),[detection],{},visible)
    worker._stable_gate=False; worker.tracking_status='배치 보류'; worker.game_tracking_generation=-100
    worker.error=None
    board,centers=worker.fresh_state(timeout=0,new_game=False)
    assert board==known and centers=={(0,6):(20.,30.)}
    assert visible[(1,1)]==('unknown','piece')
    board.clear(); assert worker.latest[3]


def test_arbitrary_observed_changes_replace_board_even_when_old_turn_was_robot():
    rg=RobotGame.__new__(RobotGame); rg.cfg=RobotGameConfig(); rg.robot_side='red'; rg.log=lambda *_:None
    rg.game=Game({(0,0):('red','cha')},top_side='red',turn='red')
    observed={(0,6):('blue','soldier'),(4,1):('red','king'),(4,8):('blue','king')}
    rg.apply_human_move(observed)
    assert rg.game.board==observed and rg.game.turn=='red'
    assert rg.game.board is not observed


def test_next_turn_always_reads_current_board_then_executes_without_result_matching_gate():
    observed={(0,6):('blue','soldier'),(4,1):('red','king'),(4,8):('blue','king')}
    calls=[]
    rg=RobotGame.__new__(RobotGame); rg.cfg=RobotGameConfig(); rg.robot_side='red'; rg.log=lambda *_:None
    rg.game=Game({(8,9):('red','cha')},top_side='red',turn='red')
    def turn(centers,stop_check):
        assert rg.game.board==observed and rg.game.turn=='red'
        calls.append(('execute',centers))
        return RobotPlan('e9d9',(4,1),(3,1),('red','king'))
    rg.robot_turn=turn
    ui=App.__new__(App); ui.rg=rg; ui.game=rg.game; ui.mode=var('live')
    ui.args=SimpleNamespace(no_motion=False); ui.robot_stop=False
    ui.worker=SimpleNamespace(fresh_state=lambda:(dict(observed),{'current':'centers'}),robot_motion=False)
    ui._ensure_robot=lambda:None
    ui._robot_job=lambda phase,job:job()
    ui.robot_human_done()
    assert calls==[('execute',{'current':'centers'})]
    assert ui.last_move==((4,1),(3,1))


def test_clicks_show_both_sides_moves_and_never_edit_or_command_anything():
    ui=App.__new__(App); ui.mode=var('game'); ui.robot_busy=False; ui.rg=object()
    ui.game=Game({(0,0):('red','cha'),(0,6):('blue','soldier')},top_side='red',turn='blue')
    ui.stable=dict(ui.game.board); ui.sel=None; ui.drag_xy=None
    original=dict(ui.game.board)
    ui.on_press((0,0),None)
    assert ui.sel==(0,0) and (1,0) in ui.selection_moves(ui.game.board)
    ui.on_drag(SimpleNamespace(x=1,y=1)); ui.on_release((1,0),None)
    assert ui.game.board==original and ui.drag_xy is None
    ui.on_press((1,0),None)
    assert ui.sel is None and ui.game.board==original
    ui.show_diff=var(True); ui.stable.pop((0,6))
    assert ui.diff_cells()=={(0,6)}


def test_start_can_update_game_without_implicit_home_or_recognition_gate():
    rg=RobotGame.__new__(RobotGame); rg.cfg=RobotGameConfig(); rg.robot_side='red'
    rg.log=lambda *_:None
    rg.go_observe=lambda:(_ for _ in ()).throw(AssertionError('unexpected home'))
    assert rg.start({},observe=False).board=={}
