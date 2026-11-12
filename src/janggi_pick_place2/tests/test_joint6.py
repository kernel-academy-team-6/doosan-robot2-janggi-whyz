"""Wrist winding must be bounded before sending a motion, including path interiors."""
from types import SimpleNamespace
import pytest

from janggi_pick_place2.geometry import joint6_candidates, local_z_turn_deg
from janggi_pick_place2.executor import Settings
from janggi_pick_place2.robot_client import RobotClient


def arm():
    robot = RobotClient.__new__(RobotClient)
    robot.settings = Settings(joint6_lock_enabled=False)
    robot.stop = lambda: None
    return robot


@pytest.mark.parametrize('target,current,expected', [
    (362.5, 0., 2.5), (270., 90., -90.), (-270., -90., 90.),
    (450., 10., 90.), (-450., -10., -90.), (180., 170., 180.),
])
def test_equivalent_angle_chooses_bounded_shortest_turn(target, current, expected):
    assert joint6_candidates(target, current)[0] == expected


def test_local_z_alignment_detects_axis_tilt():
    assert abs(local_z_turn_deg([180.,180.,0.], [0.,180.,0.])) == pytest.approx(180.)
    with pytest.raises(ValueError, match='Z direction'):
        local_z_turn_deg([0.,175.,0.], [0.,180.,0.])


def test_path_unwraps_continuously_instead_of_hiding_limit_with_modulo():
    robot = arm()
    robot.io = SimpleNamespace(call=lambda kind,path,req,check:
                              SimpleNamespace(conv_posj=[0.,0.,90.,0.,90.,-80.]))
    # -80 degrees is the wrapped endpoint of a continuous 260 -> 280 turn.
    with pytest.raises(ValueError, match='280.000'):
        robot._preview_line([0,0,200,0,180,0], [10,0,200,0,180,0],
                            [0,0,90,0,90,260], 0, lambda:None)


def test_midpoint_limit_is_checked_even_with_valid_endpoint():
    robot = arm()
    robot.io = SimpleNamespace(call=lambda kind,path,req,check:
                              SimpleNamespace(conv_posj=[0.,0.,90.,0.,90.,280. if req.pos[0]==10 else 260.]))
    with pytest.raises(ValueError, match='280.000'):
        robot._preview_line([0,0,200,0,180,0], [20,0,200,0,180,0],
                            [0,0,90,0,90,260], 0, lambda:None)


def test_alignment_selects_reverse_half_turn_at_same_flange_position():
    robot = arm(); sent=[]
    pose=[0.,0.,200.,180.,180.,0.]
    robot.current_pose=lambda check:list(pose)
    robot.current_joints=lambda check:[-90.,8.,113.,0.,59.,90.]
    robot._solution=lambda check:0
    def joint_move(target, check, velocity, acceleration, purpose):
        sent.append(target)
        pose[3:]=[0.,180.,0.]
    robot._joint_move=joint_move
    robot.align_for_path([0.,0.,200.,0.,180.,0.], [], lambda:None)
    assert sent==[[-90.,8.,113.,0.,59.,-90.]]


def test_no_whole_turn_rewind_while_holding():
    robot=arm()
    robot.current_pose=lambda check:[0.,0.,200.,0.,180.,0.]
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,260.]
    robot._solution=lambda check:0
    robot.io=SimpleNamespace(call=lambda kind,path,req,check:
                             SimpleNamespace(conv_posj=[0.,0.,90.,0.,90.,280.]))
    robot._joint_move=lambda *args:pytest.fail('must not move')
    with pytest.raises(ValueError, match='No continuous J6 path'):
        robot.align_for_path([0.,0.,200.,0.,180.,0.], [[10.,0.,200.,0.,180.,0.]],
                             lambda:None, allow_rewind=False)


def test_empty_clearance_rewind_is_planned_before_any_motion():
    robot=arm(); sent=[]
    robot.current_pose=lambda check:[0.,0.,200.,0.,180.,0.]
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,260.]
    robot._solution=lambda check:0
    robot.io=SimpleNamespace(call=lambda kind,path,req,check:
                             SimpleNamespace(conv_posj=[0.,0.,90.,0.,90.,280.]))
    robot._joint_move=lambda target,*args:sent.append(target)
    robot.align_for_path([0.,0.,200.,0.,180.,0.], [[10.,0.,200.,0.,180.,0.]],
                         lambda:None, allow_rewind=True)
    assert sent[0][5] == -100.


def test_out_of_band_start_stops_instead_of_automatic_recovery():
    robot=arm(); stopped=[]
    robot.stop=lambda:stopped.append(True)
    robot.current_pose=lambda check:[0.,0.,200.,0.,180.,0.]
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,362.5]
    robot.io=SimpleNamespace(call=lambda *args:pytest.fail('must not send requests'))
    with pytest.raises(RuntimeError, match='no automatic recovery'):
        robot.move([10.,0.,200.,0.,180.,0.], lambda:None)
    assert stopped==[True]


def test_joint6_settings_reject_invalid_band():
    with pytest.raises(ValueError, match='J6 bounds'):
        Settings(joint6_max_deg=361.).validate()


def test_planner_keeps_margin_before_soft_limit():
    robot=arm()
    robot.io=SimpleNamespace(call=lambda kind,path,req,check:
                             SimpleNamespace(conv_posj=[0.,0.,90.,0.,90.,268.]))
    with pytest.raises(ValueError, match='planning band'):
        robot._preview_line([0,0,200,0,180,0], [10,0,200,0,180,0],
                            [0,0,90,0,90,260], 0, lambda:None)


def test_runtime_guard_stops_at_out_of_band_feedback_before_motion_check():
    robot=arm(); stopped=[]
    robot.motion_timeout=10.
    robot.current_joints=lambda check:[0.,0.,90.,0.,90.,270.1]
    robot.stop=lambda:stopped.append(True)
    robot.io=SimpleNamespace(call=lambda *args:pytest.fail('must stop before more requests'))
    with pytest.raises(RuntimeError, match='outside software band'):
        robot._wait(lambda:True, lambda:None)
    assert stopped==[True]


@pytest.mark.parametrize('entry_height,expected', [(450.,True), (185.,False)])
def test_executor_allows_full_rewind_only_empty_with_clearance(entry_height, expected):
    from janggi_pick_place2.robot_client import MockRobot
    from janggi_pick_place2.executor import PickPlaceExecutor
    robot=MockRobot(Settings()); robot.last_pose[2]=entry_height
    permissions=[]
    def align(pose, waypoints, check, *, allow_rewind):
        permissions.append(allow_rewind)
        robot.last_pose=list(pose)
    robot.align_for_path=align
    hand=SimpleNamespace(prepare=lambda check:None, open_to=lambda *args:None,
                         grasp=lambda *args:{'verified':False})
    executor=PickPlaceExecutor(robot,hand,Settings())
    grasp=executor.resolve()
    executor.run('pick_and_place',[400.,0.,0.,0.],grasp,lambda:None,lambda *args:None,
                 place_target=[410.,0.,0.,0.])
    assert permissions==[expected,False]
