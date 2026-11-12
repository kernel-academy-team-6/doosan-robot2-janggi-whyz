from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2])]
from manipulation import ManipulationOptions, SeenPiece, choose_yaw, locate, move_evidence, ManipulationCoordinator


def piece(x=0.,y=0.,pid='own',label=('blue','soldier')):
    return SeenPiece(pid,(0,0),label,np.array([x,y,0.]),12.)


def test_fingers_choose_free_direction_instead_of_opening_into_neighbor():
    options=ManipulationOptions()
    yaw,clearance=choose_yaw([0,0,0],[piece(40.,0.,'other')],20.,30.,0.,options)
    assert abs(yaw)>=45 and clearance>0
    assert choose_yaw([0,0,0],[],20.,30.,37.,options)[0]==37.


def test_target_identity_follows_displacement_and_ambiguous_identical_pieces_are_rejected():
    own=piece(25,5); other=piece(-24,0,'other')
    assert locate([own,other],'own',own.piece,np.zeros(3),45) is own
    with pytest.raises(RuntimeError): locate([own,other],None,own.piece,np.zeros(3),45)
    with pytest.raises(RuntimeError): locate([piece(100,0)],'own',own.piece,np.zeros(3),45)


@pytest.mark.parametrize('state,scene',[
    ('placed',[piece(50)]),('unchanged',[piece()]),('missing',[]),
    ('uncertain',[piece(25)]),('uncertain',[piece(50,pid='other',label=('unknown','piece'))]),
])
def test_visual_evidence_requires_three_consistent_frames_and_never_calls_absence_verified_holding(state,scene):
    assert move_evidence([scene]*3,'own',('blue','soldier'),np.zeros(3),np.array([50,0,0]),12)==state
    if state!='uncertain':
        assert move_evidence([scene,[piece(25)],scene],'own',('blue','soldier'),np.zeros(3),np.array([50,0,0]),12)=='uncertain'


def test_same_kind_pieces_elsewhere_do_not_prevent_missing_state_but_target_seen_elsewhere_does():
    source=np.zeros(3); destination=np.array([50,0,0])
    assert move_evidence([[piece(150,pid='another')]]*3,'own',('blue','soldier'),source,destination,12)=='missing'
    assert move_evidence([[piece(100)]]*3,'own',('blue','soldier'),source,destination,12)=='uncertain'


class FakePicker:
    def __init__(self,first): self.first=first;self.pos=0.;self.events=[];self.attempts=0
    def set_piece_kind(self,kind): pass
    def set_operation_yaw(self,yaw): self.events.append(('yaw',yaw))
    def pick_abs(self,target,refresh=None):
        if refresh is not None: target=refresh()
        self.events.append(('pick',float(target[0])));self.pos=None
    def place(self,*dst):
        self.attempts+=1;self.events.append(('place',dst))
        self.pos=(0. if self.first=='unchanged' else None) if self.attempts==1 else 50.
        if self.first=='placed':self.pos=50.
    def retry_place(self,*dst):self.events.append(('retry_place',dst));self.pos=50.


@pytest.mark.parametrize('first,expected_picks,expected_retry_place',[
    ('placed',1,0),('unchanged',2,0),('missing',1,1),
])
def test_bounded_retry_runs_full_move_or_release_only(first,expected_picks,expected_retry_place):
    p=FakePicker(first);logs=[]
    game=SimpleNamespace(picker=p,log=logs.append,
                         geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])),
                         go_observe=lambda:p.events.append(('home',)))
    coordinator=ManipulationCoordinator(game,None,ManipulationOptions(adaptive_grasp_enabled=False,
                                                                      refresh_before_pick=False),lambda:None)
    coordinator.scenes=lambda:[[] if p.pos is None else [piece(p.pos)]]*3
    plan=SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None)
    coordinator.execute(plan)
    assert sum(e[0]=='pick' for e in p.events)==expected_picks
    assert sum(e[0]=='retry_place' for e in p.events)==expected_retry_place
    assert game._execution_observed


def test_retry_toggle_off_does_not_repeat_failed_move():
    p=FakePicker('unchanged')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
                      geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(adaptive_grasp_enabled=False,
                                                        refresh_before_pick=False,retry_move_enabled=False),lambda:None)
    c.scenes=lambda:[[piece()] for _ in range(3)]
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert [e[0] for e in p.events].count('pick')==1


def test_repeated_failure_is_not_an_infinite_retry():
    p=FakePicker('unchanged')
    def place(*dst):p.events.append(('place',dst));p.pos=0.
    p.place=place
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
                      geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(adaptive_grasp_enabled=False,refresh_before_pick=False),lambda:None)
    c.scenes=lambda:[[piece()] for _ in range(3)]
    with pytest.raises(RuntimeError,match='한 번 재시도'):
        c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert sum(e[0]=='pick' for e in p.events)==2


def test_fresh_pick_callback_updates_target_immediately_before_descent():
    p=FakePicker('placed')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None)
    c=ManipulationCoordinator(g,None,ManipulationOptions(adaptive_grasp_enabled=False),lambda:None)
    scenes=iter([[[piece(5)]]*3,[[piece(20)]]*3])
    c.scenes=lambda:next(scenes)
    token,target=c.pick(('blue','soldier'),np.zeros(3),'own')
    assert token=='own' and target[0]==20 and ('pick',20.) in p.events


def test_capture_displacing_own_piece_is_reobserved_before_own_pick():
    positions={'own':0.,'enemy':50.}; events=[]
    class Picker:
        def set_piece_kind(self,kind):pass
        def set_operation_yaw(self,yaw):pass
        def pick_abs(self,target,refresh=None):
            token='enemy' if 'enemy' in positions else 'own'
            if token=='own': positions['own']+=10.  # Bumped again during approach.
            updated=refresh() if refresh else target
            assert updated[0]==positions[token]
            events.append((token,float(updated[0])));positions.pop(token)
        def discard(self):positions['own']=20.
        def place(self,*dst):positions['own']=50.
    game=SimpleNamespace(picker=Picker(),log=lambda *_:None,go_observe=lambda:None,
                         geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    coordinator=ManipulationCoordinator(game,None,ManipulationOptions(adaptive_grasp_enabled=False,
                                                                     retry_move_enabled=False),lambda:None)
    coordinator.scenes=lambda:[[piece(x,pid=pid,label=('red','horse') if pid=='enemy' else ('blue','soldier'))
                                for pid,x in positions.items()] for _ in range(3)]
    coordinator.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),
                                         captured=('red','horse'),capture_mm=np.array([50.,0.,0.])))
    assert events==[('enemy',50.),('own',30.)]
