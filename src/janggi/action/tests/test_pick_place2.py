"""Game adapter uses the actual pick_place2 executor and measured geometry."""
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[4]/'janggi')]
from action.board import BoardGeometry
from action.pick_place2 import BoardPickPlace2, PIECE_WIDTHS_MM
from janggi_pick_place2.geometry import tcp_length, command_from_width
from janggi_pick_place2.executor import OperationCancelled


class Arm:
    def __init__(self, events):
        self.events=events;self.pose=[400.,0.,450.,0.,180.,0.];self.stopped=False
    def current_pose(self, check):check();return self.pose.copy()
    def check_tcp(self, check):check();self.events.append(('tcp',))
    def move(self, pose, check):check();self.pose=list(pose);self.events.append(('move',list(pose)))
    def home(self,check,**kwargs):check();self.events.append(('home',kwargs))
    def stop(self):self.stopped=True


class Hand:
    def __init__(self, events):self.events=events;self.fail=False
    def prepare(self, check):check()
    def open_to(self, width, current, check):check();self.events.append(('open',width,command_from_width(width)))
    def grasp(self, grasp, check):
        check();self.events.append(('grasp',grasp.close_width_mm,command_from_width(grasp.close_width_mm)))
        if self.fail:raise RuntimeError('command failed')
        return {'grip_verified':False}


def picker(stop=lambda:False, *, compensate=False):
    events=[];arm=Arm(events);hand=Hand(events)
    mover=SimpleNamespace(board=BoardGeometry.from_calibration(),vel=(30.,30.),acc=(60.,60.))
    p=BoardPickPlace2(mover,robot=arm,gripper=hand,stop_check=stop)
    from dataclasses import replace
    p.set_motion_settings(replace(p.settings, board_z_compensation_enabled=compensate))
    return p,events,arm,hand


def test_motion_update_invalidates_cached_durations_and_preserves_held_piece():
    from dataclasses import replace
    p, events, arm, _ = picker()
    p.set_piece_kind('king'); p.pick_abs([400.,0.,-20.])
    held = p.runner.held
    arm._locked_paths = ['old timed path']
    arm._line_paths = ['old MoveL path']
    updated = replace(p.settings, linear_velocity_mm_s=40., angular_velocity_deg_s=20.,
                      joint_velocity_deg_s=20., joint_acceleration_deg_s2=40.)
    count = len(events)
    p.set_motion_settings(updated)
    assert p.settings is p.runner.settings is arm.settings is updated
    assert arm._locked_paths == [] and p.runner.held is held
    assert arm._line_paths == []
    assert p.held_grasp_height_mm == 3. and len(events) == count
    assert p.mover.vel == (40.,20.)


def test_refresh_after_horizontal_movement_retargets_descent_and_maintains_selected_yaw():
    p,events,_,_=picker()
    def refresh():
        assert any(e[0]=='move' for e in events)
        p.set_operation_yaw(45.)
        return [420.,15.,-20.]
    result=p.pick_abs([400.,0.,-20.],refresh=refresh)
    moves=[e[1] for e in events if e[0]=='move']
    assert moves[-2][:3]==[420.,15.,142.]
    assert moves[-2][3:]==[45.,180.,0.]
    assert list(result[:2])==[420.,15.] and p.runner.refresh_pick_target is None


def test_release_only_retry_never_grasps_again_and_requires_matching_completed_place():
    p,events,_,_=picker();p.set_piece_kind('soldier')
    with pytest.raises(RuntimeError):p.retry_place(4,4)
    p.pick_abs([400.,0.,-20.]);p.place(4,4)
    before=len([e for e in events if e[0]=='grasp'])
    p.retry_place(4,4)
    assert len([e for e in events if e[0]=='grasp'])==before
    assert p.runner.held is None and p.held_grasp_height_mm is None
    with pytest.raises(RuntimeError):p.retry_place(3,4)


def test_pick_sequence_gap_dependent_tcp_and_original_entry_height():
    p,events,arm,_=picker();p.set_piece_kind('soldier')
    target=p.mover.board.tip_target(0,6,0)
    p.pick_abs(target)
    moves=[e[1] for e in events if e[0]=='move']
    assert len(moves)==3
    w=PIECE_WIDTHS_MM['soldier']
    destination=[*target[:2],142.,0.,180.,0.]
    np.testing.assert_allclose(moves[1],destination)
    assert moves[0][2]==moves[2][2]==450.
    assert [e[0] for e in events]==['tcp','move','open','move','grasp','move']
    assert next(e[1] for e in events if e[0]=='open')==pytest.approx(w+10)
    assert next(e[1] for e in events if e[0]=='grasp')==pytest.approx(w)
    assert p.runner.holding_state=='holding_unverified'
    assert p.settings.linear_velocity_mm_s==30.


def test_place_retains_picked_width_and_discard_only_counts_completed_release():
    p,events,arm,_=picker();p.set_piece_kind('king')
    p.pick_abs(p.mover.board.tip_target(1,1,0))
    p.set_piece_kind('soldier')  # already-held object's width must remain king width
    p.place(4,4)
    assert [e for e in events if e[0]=='open'][-1][1]==pytest.approx(PIECE_WIDTHS_MM['king']+10)
    assert [e[1][2] for e in events if e[0]=='move'][-2]==143.
    assert p.runner.holding_state=='empty_unverified'
    assert arm.pose[2]==450.
    p.pick_abs(p.mover.board.tip_target(2,2,0))
    p.discard()
    assert p.discarded==1 and p.runner.held is None
    assert p.mover.board.base_to_board(p.discard_spot())[0]<0


def test_cancel_and_grasp_failure_stop_without_release():
    p,events,arm,hand=picker(stop=lambda:True)
    with pytest.raises(OperationCancelled):p.pick_abs([400,0,-20])
    assert arm.stopped and not events
    p,events,arm,hand=picker();hand.fail=True
    with pytest.raises(RuntimeError,match='command failed'):p.pick_abs([400,0,-20])
    assert arm.stopped and p.runner.holding_state=='unknown'
    assert len([e for e in events if e[0]=='open'])==1
    with pytest.raises(RuntimeError,match='unknown'):p.pick_abs([400,0,-20])


def test_target_above_entry_height_never_starts_moving():
    p,events,arm,_=picker();arm.pose[2]=50
    with pytest.raises(ValueError,match='entry height'):p.pick_abs([400,0,100])
    assert arm.stopped and not [e for e in events if e[0]=='move']


def test_game_applies_capture_and_moving_piece_widths_in_order():
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'robot_game'))
    from session import RobotGame, RobotPlan
    calls=[]
    class Picker:
        def set_piece_kind(self,kind):calls.append(('kind',kind))
        def pick_abs(self,point):calls.append(('pick',tuple(point)))
        def discard(self):calls.append(('discard',))
        def place(self,x,y):calls.append(('place',x,y))
    game=RobotGame.__new__(RobotGame)
    game.picker=Picker();game.log=lambda message:None;game.has_moves=lambda side:True
    game.game=SimpleNamespace(turn='blue',move=lambda *args,**kwargs:calls.append(('game_move',)))
    plan=RobotPlan('a6a5',(0,6),(0,5),('blue','soldier'),('red','horse'))
    plan.capture_mm=np.array([300,100,-20]);plan.pick_mm=np.array([340,100,-20])
    game.execute(plan)
    assert calls==[('kind','horse'),('pick',(300,100,-20)),('discard',),
                   ('kind','soldier'),('pick',(340,100,-20)),('place',0,5),('game_move',)]


def test_fixed_flange_extension_is_independent_of_piece_width():
    for kind,z in [('soldier',142.),('cha',142.),('king',143.)]:
        p,events,_,_=picker();p.set_piece_kind(kind)
        np.testing.assert_array_equal(p.mover.board.tcp_offset_mm,[0.,0.,222.])
        p.pick_abs([400,0,-20])
        descended=[event[1] for event in events if event[0]=='move'][1]
        assert descended[2]==z  # commanded piece Z + 160mm ONCE; not calibration 222mm


def test_ui_does_not_infer_yaw_from_calibration_and_preserves_explicit_yaw():
    p,_,_,_=picker()
    assert p.mover.board.tool_abc[0]-p.mover.board.tool_abc[2]==pytest.approx(40.0203)
    assert p.yaw is None
    events=[];arm=Arm(events);hand=Hand(events)
    from dataclasses import replace
    # Explicit yaw belongs to the opt-out Cartesian mode.
    explicit=BoardPickPlace2(p.mover,robot=arm,gripper=hand)
    explicit.settings=replace(explicit.settings,joint6_lock_enabled=False)
    explicit.runner.settings=explicit.settings
    explicit.yaw=12.
    explicit.pick_abs([406.5872,-119.455,-20])
    assert [e[1][3:] for e in events if e[0]=='move']==[[12.,180.,0.]]*4


def test_ui_and_direct_executor_produce_same_moves_for_same_explicit_inputs(tmp_path):
    from janggi_pick_place2.executor import PickPlaceExecutor
    p,ui_events,_,_=picker()
    # Entry height and explicit object width are controlled for this comparison.
    p.object_width_mm=27.
    source=np.array([406.5872,-119.455,-20.])
    p.pick_abs(source)
    p.place(7,6)
    direct_events=[];arm=Arm(direct_events);hand=Hand(direct_events)
    direct=PickPlaceExecutor(arm,hand,p.settings)
    target=p.mover.board.tip_target(7,6,0)
    source=p.command_target_for_kind(source,'cha')
    target=p.command_target_for_kind(target,'cha')
    direct.run('pick_and_place',[*source,0.],direct.resolve(27.,10.,100.),lambda:None,
               lambda *args:None,[*target,0.])
    # Separate actions check the TCP twice; compare all physical move/gap commands.
    assert [e for e in ui_events if e[0]!='tcp']==[e for e in direct_events if e[0]!='tcp']


def test_motion_trace_preserves_actual_flange_command_values(tmp_path):
    import json
    p,_,arm,_=picker();p.trace_path=tmp_path/'robot_commands.jsonl'
    p.record_motion('move_line',dict(flange_pose=[406.5872,-119.455,140.,0.,180.,0.],
                                   velocity=[120.,60.],ref=0))
    row=json.loads(p.trace_path.read_text())
    assert row['flange_pose']==[406.5872,-119.455,140.,0.,180.,0.]
    assert row['ref']==0


def test_observation_movement_uses_pick_place2_home_not_board_mover():
    p,events,_,_=picker()
    joints=[-90.,8.5645,113.1065,0.,58.3289,90.]
    p.go_observe(joints,30.,60.)
    assert events==[('tcp',),('home',dict(joints=joints,velocity=30.,acceleration=60.))]


@pytest.mark.parametrize('kind,z,closed,opened,thickness',[
    ('sa',-18.,20.,30.,6.),('soldier',-18.,20.,30.,6.),
    ('cha',-18.,26.,36.,7.),('horse',-18.,26.,36.,7.),
    ('sang',-18.,26.,36.,7.),('po',-18.,26.,36.,7.),('king',-17.,32.,42.,8.),
])
def test_measured_piece_profile_pick_and_place(kind,z,closed,opened,thickness):
    p,events,_,_=picker();p.set_piece_kind(kind)
    point=p.mover.board.tip_target(2,3,0)
    assert point[2] < -70  # original calibration stays untouched
    p.pick_abs(point)
    assert next(e[1] for e in events if e[0]=='grasp')==closed
    assert next(e[1] for e in events if e[0]=='open')==opened
    pick_moves=[e[1] for e in events if e[0]=='move']
    assert pick_moves[1][2]==z+160
    assert pick_moves[1][:2]==list(point[:2])
    assert p.piece_height_for_kind(kind)==thickness
    p.place(4,5)
    assert [e[1][2] for e in events if e[0]=='move'][-2]==z+160
    assert [e[1] for e in events if e[0]=='open'][-1]==opened


def test_default_keeps_home_gripper_attitude_without_zero_yaw_alignment():
    p,events,arm,_=picker()
    assert not p.settings.joint6_lock_enabled
    arm.pose[3:]=[90.,180.,0.]
    p.set_piece_kind('cha');p.pick_abs([400,0,-20]);p.place(4,5)
    moves=[e[1] for e in events if e[0]=='move']
    assert len(moves)==6
    assert all(pose[3:]==[90.,180.,0.] for pose in moves)


@pytest.mark.parametrize('kind,height,z',[('soldier',6.,-18.),('cha',7.,-18.),('king',8.,-17.)])
def test_camera_plan_uses_piece_top_height_and_command_board_z(kind,height,z):
    from session import RobotGame
    p,_,_,_=picker();game=RobotGame.__new__(RobotGame)
    game.picker=p;game.geom=p.mover.board;game.log=lambda *args:None
    game.cfg=SimpleNamespace(piece_height_mm=5.,max_offset_mm=18.)
    called=[]
    def pixel(*args,height_mm):
        called.append(height_mm)
        return game.geom.tip_target(2,3,height_mm)
    game.cam=SimpleNamespace(pixel_to_base=pixel)
    point,source=game.grasp_from_camera((2,3),{(2,3):(100,100)},kind)
    assert source=='camera' and point[2]==z and called==[height]


@pytest.mark.parametrize('kind,z',[('soldier',-18.),('cha',-18.),('king',-17.)])
def test_no_motion_game_preview_uses_same_command_profile(kind,z):
    from session import RobotGame
    p,_,_,_=picker();game=RobotGame.__new__(RobotGame)
    game.picker=None;game.geom=p.mover.board;game.log=lambda *args:None
    game.cfg=SimpleNamespace(command_settings=p.settings,piece_height_mm=5.,max_offset_mm=18.)
    target,source=game.grasp_from_camera((2,3),{},kind)
    assert source=='grid' and target[2]==z
