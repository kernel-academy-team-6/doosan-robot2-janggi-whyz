"""Physical victory ending; fake actuators only, never connects to a robot."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2])]
from session import RobotGame, RobotGameConfig, RobotPlan
from rules import Game
from engine import JanggiEngine, DEFAULT_ENGINE

DIRECT={(4,1):('red','king'),(4,8):('blue','king'),(4,6):('red','cha')}
MATE={(4,1):('red','king'),(4,8):('blue','king'),(3,9):('red','cha'),
      (5,9):('red','cha'),(4,6):('red','soldier')}


def game(board=DIRECT,turn='red'):
    events=[]
    def unexpected(*args,**kwargs):raise AssertionError('unexpected engine search')
    engine=SimpleNamespace(best_move=unexpected,perft_moves=unexpected)
    picker=SimpleNamespace(cfg=SimpleNamespace(grasp_height_mm=2.),
        set_piece_kind=lambda kind:events.append(('kind',kind)),
        pick_abs=lambda pt:events.append(('pick',tuple(pt))),
        discard=lambda:events.append(('grave',)),place=lambda *dst:events.append(('place',dst)))
    rg=RobotGame(engine,None,None,picker,RobotGameConfig(),log=lambda text:events.append(('log',text)))
    rg.game=Game(dict(board),top_side='red',turn=turn)
    rg.go_observe=lambda:events.append(('home',))
    rg.grasp_from_camera=lambda cell,centers,kind:(np.array([*cell,2.]),'camera')
    return rg,events


def test_available_king_capture_precedes_engine_and_winner_is_after_home():
    rg,events=game()
    def home():
        assert rg.winner is None
        events.append(('home',))
    rg.go_observe=home
    plan=rg.robot_turn({})
    assert plan.src==(4,6) and plan.dst==(4,8) and plan.captured==('blue','king')
    operations=[e for e in events if e[0]!='log']
    assert operations==[('kind','king'),('pick',(4.,8.,2.)),('grave',),
                        ('kind','cha'),('pick',(4.,6.,2.)),('place',(4,8)),('home',)]
    assert rg.winner=='red' and not rg.victory_pending and rg.last_execution_completed
    assert rg.game.board[(4,8)]==('red','cha')
    old=list(events)
    assert rg.robot_turn({}).uci is None
    rg.human_done(SimpleNamespace(fresh_state=lambda:pytest.fail('read after victory')))
    assert events==old
    rg.start(DIRECT,observe=False)
    assert rg.winner is None and not rg.victory_pending


@pytest.mark.parametrize('board',[
    {**DIRECT,(4,7):('red','horse')},  # blocked rook, blocker cannot capture king
    {**DIRECT,(0,1):('blue','cha')},  # own king in check; capture does not resolve it
    {(4,8):('blue','king'),(4,6):('red','cha')}, # missing own king
    {**DIRECT,(3,8):('blue','king')}, # ambiguous duplicate king
])
def test_capture_requires_clear_legal_path_and_one_king_per_side(board):
    rg,_=game(board)
    assert rg._winning_king_capture() is None


def test_interrupted_grave_does_not_announce_win_or_move_attacker():
    rg,events=game()
    def fail():raise RuntimeError('release failed')
    rg.picker.discard=fail
    with pytest.raises(RuntimeError,match='release failed'):rg.robot_turn({})
    assert rg.winner is None and rg.victory_pending
    assert rg.game.board==DIRECT
    assert not any(e[0]=='place' for e in events)


def test_confirmed_mate_collects_king_after_normal_turn_then_announces_win():
    rg,events=game(MATE,turn='blue')
    plan=RobotPlan('e5e4',(4,5),(4,6),('red','soldier'))
    rg.plan=lambda centers:plan
    rg.execute=lambda *args:events.append(('normal_turn',)) or True
    rg.engine.perft_moves=lambda *args:[]
    rg.robot_turn({})
    ops=[e for e in events if e[0]!='log']
    assert ops==[('normal_turn',),('home',),('kind','king'),('pick',(4.,8.,2.)),('grave',),('home',)]
    assert rg.winner=='red' and (4,8) not in rg.game.board


@pytest.mark.parametrize('mate,replies',[(False,[]),(True,[((4,8),(4,7))])])
def test_check_alone_or_engine_disagreement_does_not_remove_king(mate,replies):
    b=MATE if mate else {(4,1):('red','king'),(4,8):('blue','king'),(3,8):('red','cha')}
    rg,events=game(b,turn='blue')
    rg.engine.perft_moves=lambda *args:replies
    assert not rg._finish_checkmate({},lambda:False)
    assert (4,8) in rg.game.board and rg.winner is None
    assert not any(e[0] in ('grave','pick') for e in events)


@pytest.mark.skipif(not DEFAULT_ENGINE.exists(),reason='engine not installed')
def test_mate_fixture_is_also_terminal_in_real_engine():
    assert Game(MATE,top_side='red',turn='blue').status()=='외통'
    with JanggiEngine() as engine:
        assert engine.perft_moves(MATE,'red','blue')==[]
