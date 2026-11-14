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
    p=FakePicker(first);logs=[];notices=[]
    game=SimpleNamespace(picker=p,log=logs.append,
                         on_manipulation_retry=notices.append,
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
    assert notices==({'placed':[], 'unchanged':['move'], 'missing':['place']}[first])


def test_retry_toggle_off_does_not_repeat_failed_move():
    p=FakePicker('unchanged')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
                      geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(adaptive_grasp_enabled=False,
                                                        refresh_before_pick=False,retry_move_enabled=False),lambda:None)
    c.scenes=lambda:[[piece()] for _ in range(3)]
    g.on_manipulation_retry=lambda kind:pytest.fail('disabled retry announced')
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
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert g._execution_observed
    assert sum(e[0]=='pick' for e in p.events)==2


def test_pick_uses_home_snapshot_without_descent_refresh_callback():
    p=FakePicker('placed')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None)
    c=ManipulationCoordinator(g,None,ManipulationOptions(adaptive_grasp_enabled=False),lambda:None)
    scenes=iter([[[piece(5)]]*3,[[piece(20)]]*3])
    c.scenes=lambda:next(scenes)
    token,target=c.pick(('blue','soldier'),np.zeros(3),'own')
    assert token=='own' and target[0]==5 and ('pick',5.) in p.events


def test_capture_displacing_own_piece_is_reobserved_before_own_pick():
    positions={'own':0.,'enemy':50.}; events=[]
    class Picker:
        def set_piece_kind(self,kind):pass
        def set_operation_yaw(self,yaw):pass
        def pick_abs(self,target,refresh=None):
            token='enemy' if 'enemy' in positions else 'own'
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
    assert events==[('enemy',50.),('own',20.)]


def test_removed_adaptive_rotation_cannot_be_reenabled_by_old_settings(monkeypatch):
    import manipulation
    monkeypatch.setattr(manipulation,'choose_yaw',lambda *args:pytest.fail('removed neighbor rotation must not run'))
    picker=FakePicker('placed')
    game=SimpleNamespace(picker=picker,log=lambda *_:None,
                         go_observe=lambda:pytest.fail('orientation reset must not home'))
    options=ManipulationOptions(adaptive_grasp_enabled=True)
    coordinator=ManipulationCoordinator(game,None,options,lambda:None)
    coordinator.orient([piece(1)],np.zeros(3),'king','own')
    assert picker.events==[('yaw',None)]
    loaded=ManipulationOptions.from_parameters(dict(adaptive_grasp_enabled=True,
        refresh_before_pick=True,retry_move_enabled=True))
    assert not loaded.adaptive_grasp_enabled
    assert loaded.refresh_before_pick and loaded.retry_move_enabled


def test_uncertain_visual_state_or_occupied_target_does_not_latch_or_block_next_commands():
    p=FakePicker('placed')
    p.mark_holding_unknown=lambda:pytest.fail('visual uncertainty must not latch holding state')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
        geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    # An unrelated piece at the destination used to block place outright.
    c.scenes=lambda:[[piece(50,pid='other',label=('red','horse'))]]*3
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert any(e[0]=='place' for e in p.events)
    assert g._execution_observed


def test_all_robot_snapshots_happen_at_home_not_between_pick_and_place():
    p=FakePicker('placed');at_home=[False];order=[]
    original_pick=p.pick_abs;original_place=p.place
    def pick(target,refresh=None):
        assert refresh is None
        assert at_home[0]
        order.append('pick');at_home[0]=False
        original_pick(target,refresh)
    def place(*dst):
        assert not at_home[0]
        order.append('place');original_place(*dst)
    def home():at_home[0]=True;order.append('home')
    p.pick_abs=pick;p.place=place
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=home,
        geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(),lambda:None)
    def snapshot():
        assert at_home[0], 'camera target query while wrist occludes board'
        order.append('snapshot')
        return [[piece(p.pos)]]*3
    c.scenes=snapshot
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert order==['home','snapshot','pick','place','home','snapshot']


@pytest.mark.parametrize('neighbors,expected,counts',[
    ([piece(40,pid='r'),piece(-40,pid='l')],90.,(2,0)),
    ([piece(0,40,pid='u'),piece(0,-40,pid='d')],0.,(0,2)),
    ([piece(40,pid='r'),piece(-40,pid='l'),piece(0,40,pid='u')],90.,(2,1)),
    ([],0.,(0,0)),
    ([piece(40,pid='r'),piece(0,40,pid='u')],0.,(1,1)),
    ([piece(400,pid='far')],0.,(0,0)),
])
def test_binary_direction_uses_fewer_nearby_pieces_and_defaults_to_flat(neighbors,expected,counts):
    from manipulation import choose_orthogonal_direction
    offset,actual=choose_orthogonal_direction(np.zeros(3),neighbors,20.,30.,0.,ManipulationOptions())
    assert offset==expected and actual==counts


def test_binary_direction_tracks_home_axis_and_never_blocks_a_crowded_scene():
    from manipulation import choose_orthogonal_direction
    offset,_=choose_orthogonal_direction(np.zeros(3),[piece(0,40,pid='up')],20.,30.,90.,ManipulationOptions())
    assert offset==90.
    offset,counts=choose_orthogonal_direction(np.zeros(3),[piece(8,8),piece(-8,-8,pid='other')],20.,30.,0.,ManipulationOptions())
    assert offset in (0.,90.) and min(counts)>0  # choose even when both are crowded


@pytest.mark.parametrize('clears', [False, True])
def test_king_capture_retries_then_requires_removal_before_attacker_moves(clears):
    p=FakePicker('placed');p.enemy=True;p.drops=0;notices=[]
    def discard():
        p.drops+=1;p.events.append(('grave',))
        if clears and p.drops==2:p.enemy=False
    p.discard=discard
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
                      geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    g.on_manipulation_retry=notices.append
    c=ManipulationCoordinator(g,None,ManipulationOptions(retry_move_enabled=True),lambda:None)
    def scenes():
        scene=([piece(0)] if p.drops<2 else [piece(0)])
        if p.enemy:scene.append(piece(50,pid='enemy',label=('red','king')))
        # After the attacker's place call, show it at the destination.
        if p.attempts:scene=[piece(50)]
        return [scene]*3
    c.scenes=scenes
    plan=SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),
                         captured=('red','king'),capture_mm=np.array([50.,0.,0.]))
    completed=c.execute(plan)
    assert p.drops==2 and completed is clears
    assert any(e[0]=='place' for e in p.events) is clears
    assert notices==['capture']


def test_failed_retry_notice_does_not_stop_robot_retry():
    p=FakePicker('unchanged')
    def fail(kind):raise RuntimeError('audio unavailable')
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
                      on_manipulation_retry=fail,
                      geom=SimpleNamespace(board_to_base=lambda *args:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    c.scenes=lambda:[[] if p.pos is None else [piece(p.pos)]]*3
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert sum(e[0]=='pick' for e in p.events)==2


def test_piece_seen_off_board_with_empty_source_and_destination_can_retry_place():
    held=piece(150);held.cell=None
    assert move_evidence([[held]]*3,'own',held.piece,np.zeros(3),np.array([50.,0.,0.]),12)=='missing'
    held.cell=(2,2)
    assert move_evidence([[held]]*3,'own',held.piece,np.zeros(3),np.array([50.,0.,0.]),12)=='uncertain'


def test_full_move_retry_can_be_followed_by_separate_place_only_retry():
    p=FakePicker('unchanged');notices=[]
    original=p.place
    def place(*dst):
        original(*dst)
        if p.attempts==2:p.pos=None  # Second pick succeeded, second release failed.
    p.place=place
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
        on_manipulation_retry=notices.append,geom=SimpleNamespace(board_to_base=lambda *dst:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    c.scenes=lambda:[[] if p.pos is None else [piece(p.pos)]]*3
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert notices==['move','place']
    assert len([e for e in p.events if e[0]=='pick'])==2
    assert len([e for e in p.events if e[0]=='retry_place'])==1 and p.pos==50.


def test_uncertain_placement_is_reobserved_before_release_retry():
    p=FakePicker('missing');notices=[];calls=[]
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
        on_manipulation_retry=notices.append,geom=SimpleNamespace(board_to_base=lambda *dst:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    def scenes():
        calls.append(True)
        if len(calls)==1:return [[piece()]]*3
        if len(calls)==2:return [[piece(50,pid='other',label=('unknown','piece'))]]*3
        return [[] if p.pos is None else [piece(p.pos)]]*3
    c.scenes=scenes
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert notices==['vision','place'] and p.pos==50.
    assert len([e for e in p.events if e[0]=='pick'])==1


@pytest.mark.parametrize('already_placed',[False,True])
def test_release_command_exception_enters_visual_place_recovery_without_picking_again(already_placed):
    from janggi_pick_place2.executor import PlaceReleaseFailed
    p=FakePicker('missing');notices=[]
    def place(*dst):
        p.events.append(('place',dst))
        if already_placed:p.pos=50.
        raise PlaceReleaseFailed('release failed after retreat')
    p.place=place
    p.mark_place_observed=lambda:p.events.append(('observed',))
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
        on_manipulation_retry=notices.append,geom=SimpleNamespace(board_to_base=lambda *dst:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    c.scenes=lambda:[[] if p.pos is None else [piece(p.pos)]]*3
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert notices==([] if already_placed else ['place']) and p.pos==50.
    assert len([e for e in p.events if e[0]=='pick'])==1
    assert len([e for e in p.events if e[0]=='observed'])==1


def test_repeated_place_failure_is_bounded_and_motion_errors_are_not_retried():
    from janggi_pick_place2.executor import PlaceReleaseFailed
    p=FakePicker('missing');notices=[]
    def retry(*dst):p.events.append(('retry_place',dst));raise PlaceReleaseFailed('still failed')
    p.retry_place=retry
    g=SimpleNamespace(picker=p,log=lambda *_:None,go_observe=lambda:None,
        on_manipulation_retry=notices.append,geom=SimpleNamespace(board_to_base=lambda *dst:np.array([50.,0.,0.])))
    c=ManipulationCoordinator(g,None,ManipulationOptions(refresh_before_pick=False),lambda:None)
    c.scenes=lambda:[[] if p.pos is None else [piece(p.pos)]]*3
    c.execute(SimpleNamespace(piece=('blue','soldier'),pick_mm=np.zeros(3),dst=(1,0),captured=None))
    assert notices==['place'] and len([e for e in p.events if e[0]=='retry_place'])==1
    def motion_error():raise RuntimeError('arm path failed')
    with pytest.raises(RuntimeError,match='arm path failed'):c.attempt_place(motion_error)
    c.o=replace(c.o,retry_move_enabled=False)
    with pytest.raises(PlaceReleaseFailed):c.attempt_place(lambda:retry(1,0))
