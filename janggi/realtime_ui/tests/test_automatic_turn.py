"""Human move/capture + uninterrupted clear-frame countdown; no robot hardware."""
from pathlib import Path
from types import SimpleNamespace
from collections import deque
import sys
import threading
import pytest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automatic_turn import AutomaticTurn
from app import App, Worker
from rules import Game

BASE={(0,6):('blue','soldier'),(0,5):('red','soldier'),(4,1):('red','king'),(4,8):('blue','king')}


def moved(capture=False):
    board=dict(BASE);piece=board.pop((0,6));board[(0,5) if capture else (1,6)]=piece
    return board


def update(m,board,now,*,clear=True,epoch=0,stamp=None):
    return m.update(board,'blue',now=now,stamp=now if stamp is None else stamp,clear=clear,epoch=epoch)


def armed():
    m=AutomaticTurn();update(m,BASE,.1);update(m,BASE,.5)
    assert m.baseline==BASE
    return m


@pytest.mark.parametrize('capture',[False,True])
def test_move_and_capture_fire_once_after_three_seconds(capture):
    m=armed();board=moved(capture)
    assert update(m,board,1.).remaining==3.
    assert not update(m,board,3.99).fire
    result=update(m,board,4.)
    assert result.fire and result.move==((0,6),(0,5) if capture else (1,6))
    assert not update(m,board,10.).fire


def test_obstruction_resets_timer_but_keeps_pre_move_baseline():
    m=armed();board=moved();update(m,board,1.)
    assert update(m,board,3.8,clear=False,epoch=1).remaining is None
    assert m.baseline==BASE
    assert update(m,board,4.,epoch=1).remaining==3.
    assert not update(m,board,6.99,epoch=1).fire
    assert update(m,board,7.,epoch=1).fire


def test_brief_obstruction_between_ui_polls_also_resets_timer():
    m=armed();board=moved();update(m,board,1.)
    # Current frame is already clear, but the worker saw a blocked frame in between.
    assert update(m,board,3.8,epoch=2).remaining==3.
    assert not update(m,board,4.,epoch=2).fire
    assert update(m,board,6.8,epoch=2).fire


@pytest.mark.parametrize('change', ['unchanged','lifted_only','enemy_move','classification','extra_change'])
def test_no_countdown_without_one_complete_human_move(change):
    m=armed();board=dict(BASE)
    if change=='lifted_only':board.pop((0,6))
    elif change=='enemy_move':board[(1,5)]=board.pop((0,5))
    elif change=='classification':board[(0,6)]=('blue','sa')
    elif change=='extra_change':board=moved();board.pop((4,1))
    for at in (1.,5.,10.):
        result=update(m,board,at)
        assert not result.fire and result.remaining is None


def test_cancelled_move_and_changed_destination_restart_full_countdown():
    m=armed();board=moved();update(m,board,1.)
    assert update(m,BASE,3.).remaining is None
    assert update(m,board,4.).remaining==3.
    different=dict(BASE);different[(2,6)]=different.pop((0,6))
    assert update(m,different,6.9).remaining==3.
    assert not update(m,different,7.).fire
    assert update(m,different,9.9).fire


def test_frozen_video_does_not_complete_countdown_and_recovery_restarts_it():
    m=armed();board=moved();update(m,board,1.)
    assert not update(m,board,4.,stamp=1.).fire
    assert update(m,board,4.1).remaining==3.
    assert update(m,board,7.1).fire


def test_reset_rearms_from_actual_post_robot_board_not_predicted_game():
    m=armed();update(m,moved(),1.);assert update(m,moved(),4.).fire
    m.reset()
    # Robot may fail to move: establish the actual board, not its intended target.
    update(m,moved(),5.);update(m,moved(),5.5)
    assert m.baseline==moved()
    assert update(m,moved(),10.).remaining is None


def make_ui(monkeypatch):
    ui=App.__new__(App);clock=[.1];observed=[dict(BASE)];epoch=[0];calls=[]
    monkeypatch.setattr('app.time.monotonic',lambda:clock[0])
    ui.auto_move_enabled=SimpleNamespace(get=lambda:True)
    ui.auto_turn=AutomaticTurn();ui._auto_resume_after=0.
    ui.robot_busy=False;ui.robot_stop=False;ui.args=SimpleNamespace(human_side='blue')
    ui.rg=SimpleNamespace(game=Game(dict(BASE),top_side='red',turn='blue'))
    ui.lbl_auto_move=SimpleNamespace(config=lambda **kw:None);ui._msgs=deque()
    ui.worker=SimpleNamespace(automatic_state=lambda:dict(board=dict(observed[0]),centers={'fresh':clock[0]},stamp=clock[0],clear=True,epoch=epoch[0]))
    ui.robot_human_done=lambda **kw:calls.append(kw)
    return ui,clock,observed,calls


def test_ui_uses_single_latest_snapshot_and_never_repeats_trigger(monkeypatch):
    ui,clock,board,calls=make_ui(monkeypatch)
    ui._tick_auto_move();clock[0]=.5;ui._tick_auto_move()
    board[0]=moved(True);clock[0]=1.;ui._tick_auto_move()
    clock[0]=3.9;ui._tick_auto_move();assert not calls
    clock[0]=4.;ui._tick_auto_move()
    assert calls==[{'snapshot':(moved(True),{'fresh':4.})}]
    clock[0]=8.;ui._tick_auto_move();assert len(calls)==1


def test_ui_does_not_start_while_busy_stopped_or_before_new_post_home_frame(monkeypatch):
    ui,clock,board,calls=make_ui(monkeypatch)
    ui.robot_busy=True;ui._tick_auto_move();assert ui.auto_turn.baseline is None
    ui.robot_busy=False;ui._auto_resume_after=1.;ui._tick_auto_move()
    assert ui.auto_turn.baseline is None
    ui._auto_resume_after=0.;ui.robot_stop=True;ui._tick_auto_move()
    assert not calls and ui.auto_turn.baseline is None


def test_atomic_worker_snapshot_copies_board_centers_and_obstruction_epoch():
    worker=Worker.__new__(Worker);worker.lock=threading.Lock()
    unknown={(8,8):('unknown','piece')}
    worker.latest=(np.zeros((2,2,3)),[SimpleNamespace(board=(0,6),center=(20,30))],{},dict(BASE)|unknown)
    worker.view_status={'state':'stable'};worker._auto_frame_stamp=10.;worker._auto_obstruction_epoch=3
    state=worker.automatic_state()
    assert state['board']==BASE and state['centers']=={(0,6):(20,30)}
    assert state['stamp']==10. and state['clear'] and state['epoch']==3
    state['board'].clear();assert worker.latest[3]


def test_stop_button_disables_automatic_mode_without_starting_motion(monkeypatch):
    ui,clock,board,calls=make_ui(monkeypatch)
    enabled=[True]
    ui.auto_move_enabled=SimpleNamespace(get=lambda:enabled[0],set=lambda value:enabled.__setitem__(0,value))
    ui._mover=None
    update(ui.auto_turn,BASE,.1);update(ui.auto_turn,BASE,.5);update(ui.auto_turn,moved(),1.)
    clock[0]=3.5;ui.robot_halt()
    assert ui.robot_stop and not enabled[0]
    assert ui.auto_turn.baseline is None and not calls


def test_robot_job_rebases_only_from_frames_after_home_return(monkeypatch):
    ui,clock,board,calls=make_ui(monkeypatch)
    ui._human_phase=lambda:'사람 차례'
    ui.auto_turn=armed();ui.worker.robot_motion=True
    class ImmediateThread:
        def __init__(self,target,**kwargs):self.target=target
        def start(self):self.target()
    monkeypatch.setattr('app.threading.Thread',ImmediateThread)
    clock[0]=5.
    ui._robot_job('테스트',lambda:None)
    assert not ui.robot_busy and not ui.worker.robot_motion
    assert ui.auto_turn.baseline is None and ui._auto_resume_after==5.
    # The last frame captured before job completion must not become the next baseline.
    clock[0]=4.9;ui._tick_auto_move();assert ui.auto_turn.baseline is None


def test_recent_actual_home_snapshot_seeds_next_human_turn_before_fast_move(monkeypatch):
    ui,clock,board,calls=make_ui(monkeypatch)
    ui._human_phase=lambda:'사람 차례'
    actual=moved(True)
    ui.worker.automatic_home_baseline=lambda since:dict(actual)
    class ImmediateThread:
        def __init__(self,target,**kwargs):self.target=target
        def start(self):self.target()
    monkeypatch.setattr('app.threading.Thread',ImmediateThread)
    clock[0]=5.;ui._robot_job('로봇 착수',lambda:None)
    assert ui.auto_turn.baseline==actual
    fast=dict(actual);fast[(1,5)]=fast.pop((0,5))
    board[0]=fast;clock[0]=5.1;ui._tick_auto_move()
    assert not calls
    clock[0]=8.1;ui._tick_auto_move()
    assert calls[0]['snapshot'][0]==fast


def test_home_baseline_rejects_old_views_and_uses_measured_cells():
    worker=Worker.__new__(Worker);worker.lock=threading.Lock()
    worker._auto_home_view={'stamp':5.,'objects':[
        {'cell':(0,6),'piece':('blue','soldier')},
        {'cell':(3,3),'piece':('unknown','piece')},
        {'cell':None,'piece':('red','king')}]}
    assert worker.automatic_home_baseline(4.)=={(0,6):('blue','soldier')}
    assert worker.automatic_home_baseline(5.) is None
