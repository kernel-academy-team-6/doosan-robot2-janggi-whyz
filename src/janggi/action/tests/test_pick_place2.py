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
    mover=SimpleNamespace(board=BoardGeometry.from_calibration(height_map_path=None),vel=(30.,30.),acc=(60.,60.))
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
    assert p.held_grasp_height_mm == 2. and len(events) == count
    assert p.mover.vel == (40.,20.)


def test_refresh_after_horizontal_movement_retargets_descent_and_maintains_selected_yaw():
    p,events,_,_=picker()
    def refresh():
        assert any(e[0]=='move' for e in events)
        p.set_operation_yaw(45.)
        return [420.,15.,-20.]
    result=p.pick_abs([400.,0.,-20.],refresh=refresh)
    moves=[e[1] for e in events if e[0]=='move']
    assert moves[-2][:3]==pytest.approx([420.,15.,-18+tcp_length(26.)])
    assert moves[-2][3:]==[45.,180.,0.]
    assert list(result[:2])==[420.,15.] and p.runner.refresh_pick_target is None


def test_release_only_retry_never_grasps_again_and_does_not_require_verified_holding():
    p,events,_,_=picker();p.set_piece_kind('soldier')
    p.retry_place(4,4)
    p.pick_abs([400.,0.,-20.]);p.place(4,4)
    before=len([e for e in events if e[0]=='grasp'])
    p.retry_place(4,4)
    assert len([e for e in events if e[0]=='grasp'])==before
    assert p.runner.held is None and p.held_grasp_height_mm is None
    p.retry_place(3,4)


def test_pick_sequence_gap_dependent_tcp_and_original_entry_height():
    p,events,arm,_=picker();p.set_piece_kind('soldier')
    target=p.mover.board.tip_target(0,6,0)
    p.pick_abs(target)
    moves=[e[1] for e in events if e[0]=='move']
    assert len(moves)==3
    w=PIECE_WIDTHS_MM['soldier']
    destination=[*target[:2],-18+tcp_length(20.),0.,180.,0.]
    np.testing.assert_allclose(moves[-2],destination)
    assert moves[0][2]==moves[-1][2]==450.
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
    assert [e[1][2] for e in events if e[0]=='move'][-2]==pytest.approx(-18+tcp_length(32.))
    assert p.runner.holding_state=='empty_unverified'
    assert arm.pose[2]==450.
    p.pick_abs(p.mover.board.tip_target(2,2,0))
    p.discard()
    assert p.discarded==1 and p.runner.held is None
    assert p.discard_spot()[:2]==pytest.approx([465.6875,308.258])


def test_cancel_and_grasp_failure_stop_without_release():
    p,events,arm,hand=picker(stop=lambda:True)
    with pytest.raises(OperationCancelled):p.pick_abs([400,0,-20])
    assert arm.stopped and not events
    p,events,arm,hand=picker();hand.fail=True
    with pytest.raises(RuntimeError,match='command failed'):p.pick_abs([400,0,-20])
    assert arm.stopped and p.runner.holding_state=='unknown'
    assert len([e for e in events if e[0]=='open'])==1
    hand.fail=False
    p.pick_abs([400,0,-20])
    assert p.runner.holding_state=='holding_unverified'


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


def test_baseline_plus_height_is_applied_once_and_ignores_calibration_222():
    for kind,z in [('soldier',142.),('cha',142.),('king',143.)]:
        p,events,_,_=picker();p.set_piece_kind(kind)
        np.testing.assert_array_equal(p.mover.board.tcp_offset_mm,[0.,0.,222.])
        p.pick_abs([400,0,-20])
        descended=[event[1] for event in events if event[0]=='move'][-2]
        assert descended[2]==pytest.approx(-20+p.grasp_height_for_kind(kind)+tcp_length(p.object_width_mm))
        assert p.settings.tcp_length_at_6mm_mm==132.6717


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
    direct.orientation_abc=list(p.runner.orientation_abc)
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
    ('sang',-18.,26.,36.,7.),('po',-18.,26.,36.,7.),('king',-18.,32.,42.,8.),
])
def test_measured_piece_profile_pick_and_place(kind,z,closed,opened,thickness):
    p,events,_,_=picker();p.set_piece_kind(kind)
    point=p.mover.board.tip_target(2,3,0)
    assert point[2] < -70  # original calibration stays untouched
    p.pick_abs(point)
    assert next(e[1] for e in events if e[0]=='grasp')==closed
    assert next(e[1] for e in events if e[0]=='open')==opened
    pick_moves=[e[1] for e in events if e[0]=='move']
    assert pick_moves[-2][2]==pytest.approx(z+tcp_length(closed))
    assert pick_moves[-2][:2]==list(point[:2])
    assert p.piece_height_for_kind(kind)==thickness
    p.place(4,5)
    assert [e[1][2] for e in events if e[0]=='move'][-2]==pytest.approx(z+tcp_length(closed))
    assert [e[1] for e in events if e[0]=='open'][-1]==opened


def test_default_keeps_flat_home_gripper_attitude():
    p,events,arm,_=picker()
    assert not p.settings.joint6_lock_enabled
    arm.pose[3:]=[90.,180.,0.]
    p.set_piece_kind('cha');p.pick_abs([400,0,-20]);p.place(4,5)
    moves=[e[1] for e in events if e[0]=='move']
    assert len(moves)==6
    assert p.settings.grasp_yaw_offset_deg==0.
    assert all(pose[3:]==[90.,180.,0.] for pose in moves)


@pytest.mark.parametrize('kind,height,z',[('soldier',6.,-18.),('cha',7.,-18.),('king',8.,-18.)])
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


@pytest.mark.parametrize('kind,z',[('soldier',-18.),('cha',-18.),('king',-18.)])
def test_no_motion_game_preview_uses_same_command_profile(kind,z):
    from session import RobotGame
    p,_,_,_=picker();game=RobotGame.__new__(RobotGame)
    game.picker=None;game.geom=p.mover.board;game.log=lambda *args:None
    game.cfg=SimpleNamespace(command_settings=p.settings,piece_height_mm=5.,max_offset_mm=18.)
    game.cam=SimpleNamespace(pixel_to_base=lambda *args,**kwargs:np.array([500.,100.,-10.]))
    target,source=game.grasp_from_camera((2,3),{(2,3):(123,456)},kind)
    assert source=='camera' and target[2]==z
    assert target[:2]==pytest.approx([500.,100.])


def test_captured_piece_drops_at_midpoint_plus_base_y_without_board_descent_or_home_motion():
    p,events,arm,hand=picker()
    p.set_piece_kind('king')
    p.pick_abs([400,0,-20])
    # Home FK is queried; querying must not move the robot to home.
    arm.home_flange_pose=lambda check:[400.,0.,450.,0.,180.,0.]
    arm.pose[2]=350.
    before=len(events)
    point=p.discard()
    commands=[e[1] for e in events[before:] if e[0]=='move']
    assert point==pytest.approx([465.6875,308.258,450.])
    assert commands[0][:3]==[400.,0.,450.]  # vertical lift first
    assert commands[-1][:3]==pytest.approx(point)
    assert all(m[2]==450. for m in commands)
    assert not any(e[0] in ('home','grasp') for e in events[before:])
    assert [e for e in events[before:] if e[0]=='open']==[('open',42.,command_from_width(42.))]*2
    assert p.runner.held is None and p.discarded==1
    assert p.discard_spot(100)==pytest.approx(point)  # no per-piece drop-location stepping


@pytest.mark.parametrize('cancel', [False, True])
def test_grave_second_release_failure_does_not_mark_piece_discarded(cancel):
    p,events,arm,hand=picker()
    p.set_piece_kind('king');p.pick_abs([400.,0.,-18.])
    arm.home_flange_pose=lambda check:arm.pose.copy()
    held=p.runner.held;calls=[]
    def open_to(width,current,check):
        calls.append((width,arm.pose.copy()))
        if len(calls)==2:
            raise OperationCancelled() if cancel else RuntimeError('second release failed')
    hand.open_to=open_to
    with pytest.raises((OperationCancelled,RuntimeError)):
        p.discard()
    assert len(calls)==2 and calls[0]==calls[1]
    assert p.runner.held is held and p.discarded==0
    assert p.runner.holding_state=='unknown'


def test_orthogonal_direction_is_not_accumulated_and_preserves_flange_axis():
    from janggi_pick_place2.geometry import rotation_zyz
    p,events,arm,_=picker();arm.pose[3:]=[90.,179.9,0.]
    p.set_operation_yaw(None)
    assert p.runner.orientation_abc==[90.,179.9,0.]
    p.set_orthogonal_grasp(90.)
    assert p.runner.orientation_abc==[90.,179.9,-90.]
    before=np.asarray(rotation_zyz(p.home_orientation_abc))
    after=np.asarray(rotation_zyz(p.runner.orientation_abc))
    np.testing.assert_allclose(before[:,2],after[:,2],atol=1e-12)
    arm.pose[3:]=[90.,179.9,-90.]
    p.set_orthogonal_grasp(90.)
    assert p.runner.orientation_abc==[90.,179.9,-90.]
    p.set_orthogonal_grasp(0.)
    assert p.runner.orientation_abc==[90.,179.9,0.]
    arm.pose[3:]=[90.,179.9,0.]
    p.go_observe(p.settings.home_joint_deg,30.,60.)
    assert p.operation_yaw is None and p.operation_orientation_abc is None
    p.set_operation_yaw(None)
    assert p.runner.orientation_abc==[90.,179.9,0.]


def test_home_scene_left_right_neighbors_select_90_and_work_motion_keeps_selected_attitude():
    from manipulation import ManipulationCoordinator,ManipulationOptions,SeenPiece
    p,events,arm,_=picker()
    g=SimpleNamespace(picker=p,log=lambda *_:None)
    c=ManipulationCoordinator(g,None,ManipulationOptions(),lambda:None)
    point=np.array([400.,0.,-18.])
    scene=[SeenPiece('own',(0,0),('blue','soldier'),point,12.),
           SeenPiece('left',(0,1),('red','horse'),point+[-40.,0.,0.],12.),
           SeenPiece('right',(0,2),('red','horse'),point+[40.,0.,0.],12.)]
    p.set_piece_kind('soldier')
    c.orient(scene,point,'soldier','own')
    assert p.runner.orientation_abc==[0.,180.,-90.]
    p.pick_abs(point)
    assert all(e[1][3:]==[0.,180.,-90.] for e in events if e[0]=='move')
    c.orient([],point,'soldier','own')
    assert p.runner.orientation_abc==[0.,180.,0.]  # Empty scene restores flat, no accumulated turn.


def test_stop_after_first_grave_open_prevents_second_command():
    stopped=[False]
    p,events,arm,hand=picker(stop=lambda:stopped[0])
    p.set_piece_kind('king');p.pick_abs([400.,0.,-18.])
    arm.home_flange_pose=lambda check:arm.pose.copy()
    calls=[]
    def open_to(width,current,check):
        calls.append(arm.pose.copy());stopped[0]=True
    hand.open_to=open_to
    with pytest.raises(OperationCancelled):p.discard()
    assert len(calls)==1 and p.discarded==0
    assert p.runner.held is not None


def test_failed_place_release_retreats_and_preserves_measured_grasp_for_release_only_retry():
    from janggi_pick_place2.executor import PlaceReleaseFailed
    p,events,arm,hand=picker();p.set_piece_kind('king');p.pick_abs([400.,0.,-18.])
    original=hand.open_to
    def fail(*args):raise TimeoutError('release command timeout')
    hand.open_to=fail
    with pytest.raises(PlaceReleaseFailed):p.place(4,4)
    assert p._last_place[0].width_mm==32. and p._last_place[1]==2.
    assert not arm.stopped and p.runner.held is not None
    assert arm.pose[2]>0.  # Retreat succeeded before recoverable error surfaced.
    before=sum(e[0]=='grasp' for e in events)
    hand.open_to=original;p.retry_place(4,4)
    assert sum(e[0]=='grasp' for e in events)==before
    assert p.runner.held is None and p.held_grasp_height_mm is None


def test_place_retreat_failure_after_release_error_is_not_marked_recoverable():
    from janggi_pick_place2.executor import PlaceReleaseFailed
    p,events,arm,hand=picker();p.pick_abs([400.,0.,-18.])
    def fail(*args):
        def failed_retreat(*args):raise RuntimeError('retreat motion failed')
        arm.move=failed_retreat
        raise TimeoutError('release timeout')
    hand.open_to=fail
    with pytest.raises(RuntimeError,match='retreat motion failed') as caught:p.place(4,4)
    assert not isinstance(caught.value,PlaceReleaseFailed) and arm.stopped
