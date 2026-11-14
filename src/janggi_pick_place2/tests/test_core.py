import math

import pytest

from janggi_pick_place2.geometry import (command_from_width, width_from_command,
                                        tcp_length, flange_pose, orientation_error_deg)
from janggi_pick_place2.executor import PickPlaceExecutor, Settings, OperationCancelled


class Arm:
    def __init__(self):
        self.pose = [400., 0., 450., 0., 180., 0.]
        self.events = []

    def current_pose(self, check):
        check()
        return list(self.pose)

    def check_tcp(self, check):
        check()

    def home(self, check):
        check()
        self.events.append(('home',))
        self.pose = [400., 0., 450., 0., 180., 0.]

    def move(self, pose, check):
        check()
        self.events.append(('move', list(pose)))
        self.pose = list(pose)


class Hand:
    def __init__(self):
        self.events = []
        self.fail_grasp = False
        self.prepare_calls = 0

    def prepare(self, check):
        check()
        self.prepare_calls += 1

    def open_to(self, width, current, check):
        check()
        self.events.append(('open', width, current))

    def grasp(self, grasp, check):
        check()
        self.events.append(('grasp', grasp))
        if self.fail_grasp:
            raise TimeoutError('contact not confirmed')
        return {'width_mm': grasp.width_mm, 'current_ma': grasp.current_ma}


@pytest.fixture
def execution():
    return PickPlaceExecutor(Arm(), Hand(), Settings())


@pytest.mark.parametrize('w', [6., 8.2, 10., 30., 60., 96., 106.])
def test_calibrated_width_roundtrip(w):
    u = command_from_width(w)
    assert isinstance(u, int) and 0 <= u <= 700
    assert width_from_command(u) == pytest.approx(w, abs=.31 if w == 6. else .101)


@pytest.mark.parametrize('w', [5.99, 106.01, float('nan'), float('inf')])
def test_invalid_width_rejected(w):
    with pytest.raises(ValueError):
        command_from_width(w)
    with pytest.raises(ValueError):
        tcp_length(w)


def test_length_adds_circle_height_to_fixed_baseline():
    from janggi_pick_place2.geometry import height_from_command
    assert tcp_length(30.) == pytest.approx(132.6717+height_from_command(command_from_width(30.)))
    assert tcp_length(40.) < tcp_length(30.)
    with pytest.raises(ValueError, match='132.6717'):
        tcp_length(30., 222.)
    assert flange_pose([300., 100., -80., 90.], 30., 132.6717) == pytest.approx(
        [300., 100., -80+tcp_length(30.), 90., 180., 0.])


def test_down_axis_singular_euler_equivalence():
    assert orientation_error_deg([0., 180., 0.], [30., 180., 30.]) == pytest.approx(0., abs=1e-5)


def test_default_and_zero_allowance(execution):
    g = execution.resolve()
    assert (g.width_mm, g.open_width_mm, g.close_width_mm, g.release_width_mm, g.current_ma) == (26., 36., 26., 36., 100)
    assert execution.resolve(opening_allowance_mm=0.).open_width_mm == 26.
    with pytest.raises(ValueError):
        execution.resolve(grip_current_ma=0.)
    with pytest.raises(ValueError):
        execution.resolve(opening_allowance_mm=-2.)


def test_clearances_are_validated_without_clipping(execution):
    with pytest.raises(ValueError):
        execution.resolve(object_width_mm=100.)  # 110 mm opening is impossible
    with pytest.raises(ValueError):
        execution.resolve(object_width_mm=100., opening_allowance_mm=0.)  # release also needs +10
    assert execution.resolve(object_width_mm=96., opening_allowance_mm=0.).release_width_mm == 106.


def test_pick_then_place_height_and_state(execution):
    g = execution.resolve()
    execution.robot.pose = [250., -20., 375., 30., 180., 30.]
    pick, place = [300., 100., -80., 20.], [350., -100., -75., -30.]
    check, report = lambda: None, lambda *_: None
    execution.run('pick', pick, g, check, report)
    assert execution.holding_state == 'holding_unverified'
    moves = [event[1] for event in execution.robot.events if event[0] == 'move']
    assert [p[2] for p in moves] == pytest.approx([375., 375., -80.+tcp_length(26.), 375.])
    assert moves[0][:3] == [250., -20., 375.]  # fixed-XYZ alignment before XY
    assert moves[1][:2] == moves[2][:2] == moves[3][:2] == pick[:2]
    assert execution.gripper.events[0] == ('open', 36., 100)
    with pytest.raises(RuntimeError):
        execution.preflight('pick', pick, g)
    execution.run('place', place, None, check, report)
    assert execution.holding_state == 'empty_unverified' and execution.held is None
    assert execution.gripper.events[-1] == ('open', 36., 100)
    assert sum(e[0] == 'home' for e in execution.robot.events) == 0
    moves = [event[1] for event in execution.robot.events if event[0] == 'move']
    assert [p[2] for p in moves[-3:]] == pytest.approx([375., -75.+tcp_length(26.), 375.])


def test_place_requires_a_pick(execution):
    with pytest.raises(RuntimeError):
        execution.run('place', [300., 0., 0., 0.], None, lambda: None, lambda *_: None)
    assert execution.robot.events == []


def test_contact_failure_does_not_release_or_retreat(execution):
    execution.gripper.fail_grasp = True
    with pytest.raises(TimeoutError):
        execution.run('pick', [300., 0., 0., 0.], execution.resolve(), lambda: None, lambda *_: None)
    assert execution.holding_state == 'unknown'
    assert [e[0] for e in execution.gripper.events] == ['open', 'grasp']
    assert len([e for e in execution.robot.events if e[0] == 'move']) == 2


def test_cancel_after_grasp_retains_object(execution):
    def report(stage, _):
        if stage == 'pick/retreat':
            raise OperationCancelled()
    with pytest.raises(OperationCancelled):
        execution.run('pick', [300., 0., 0., 0.], execution.resolve(), lambda: None, report)
    assert execution.holding_state == 'holding_unverified'
    assert len([e for e in execution.gripper.events if e[0] == 'open']) == 1


def test_invalid_combined_destination_caught_before_grasp(execution):
    with pytest.raises(ValueError):
        execution.run('pick_and_place', [300., 0., 0., 0.], execution.resolve(),
                      lambda: None, lambda *_: None, [350., 0., 400., 0.])
    assert execution.gripper.events == []


def test_home_is_separate_and_does_not_touch_gripper(execution):
    execution.robot.pose = [250., -30., 270., 0., 90., 0.]
    result = execution.run('home', None, None, lambda: None, lambda *_: None)
    assert result == [400., 0., 450., 0., 180., 0.]
    assert execution.robot.events == [('home',)]
    assert execution.gripper.prepare_calls == 0
    assert execution.gripper.events == []
    assert execution.holding_state == 'empty_unverified'


def test_home_preserves_an_existing_grasp(execution):
    grasp = execution.resolve()
    execution.held, execution.holding_state = grasp, 'holding_unverified'
    execution.run('home', None, None, lambda: None, lambda *_: None)
    assert execution.held is grasp and execution.holding_state == 'holding_unverified'
    assert execution.gripper.prepare_calls == 0
    assert execution.gripper.events == []


def test_combined_uses_current_height_without_home(execution):
    execution.robot.pose = [200., 50., 310., 0., 180., 0.]
    execution.run('pick_and_place', [300., 0., 0., 0.], execution.resolve(),
                  lambda: None, lambda *_: None, [350., 0., 0., 90.])
    assert all(e[0] != 'home' for e in execution.robot.events)
    moves = [e[1] for e in execution.robot.events if e[0] == 'move']
    assert [p[2] for p in moves] == pytest.approx([
        310., tcp_length(26.), 310., 310., 310., tcp_length(26.), 310.])
    assert execution.holding_state == 'empty_unverified'


def test_tilted_pick_rejected_without_implicit_home(execution):
    execution.robot.pose[4] = 90.
    with pytest.raises(ValueError, match='flange Z pointing down'):
        execution.run('pick', [300., 0., 0., 0.], execution.resolve(),
                      lambda: None, lambda *_: None)
    assert execution.robot.events == []
    assert execution.gripper.events == []


def test_axis_alignment_precedes_xy_and_all_translations_hold_downward_orientation(execution):
    from janggi_pick_place2.geometry import rotation_zyz
    initial=[250.,-20.,375.,15.,179.,-10.]
    execution.robot.pose=initial.copy()
    stages=[]
    execution.run('pick',[300.,100.,-20.,25.],execution.resolve(),lambda:None,
                  lambda stage,progress:stages.append(stage))
    moves=[event[1] for event in execution.robot.events if event[0]=='move']
    assert moves[0][:3]==initial[:3]  # alignment never translates the flange
    assert stages.index('pick/align')<stages.index('pick/horizontal')
    for pose in moves:
        matrix=rotation_zyz(pose[3:])
        assert [row[2] for row in matrix]==pytest.approx([0.,0.,-1.],abs=1e-12)
    assert moves[1][2]==moves[0][2]  # XY segment keeps Z fixed
    assert moves[2][:2]==moves[1][:2]  # descend keeps XY fixed
    assert all(pose[3:]==[25.,180.,0.] for pose in moves)


def test_failed_alignment_blocks_xy_and_gripper_commands(execution):
    execution.robot.pose=[250.,-20.,375.,0.,179.,0.]
    def fails_to_align(pose,check):
        execution.robot.events.append(('move',list(pose)))
        execution.robot.pose=[*pose[:3],pose[3],179.,pose[5]]
    execution.robot.move=fails_to_align
    with pytest.raises(RuntimeError,match='no XY translation'):
        execution.run('pick',[300.,100.,-20.,0.],execution.resolve(),lambda:None,lambda *args:None)
    moves=[e[1] for e in execution.robot.events if e[0]=='move']
    assert len(moves)==1 and moves[0][:3]==[250.,-20.,375.]
    assert execution.gripper.events==[]


def test_equivalent_zyz_angles_do_not_trigger_spurious_alignment(execution):
    execution.robot.pose=[250.,-20.,375.,30.,180.,30.]
    stages=[]
    execution.run('pick',[300.,100.,-20.,0.],execution.resolve(),lambda:None,
                  lambda stage,progress:stages.append(stage))
    assert 'pick/align' not in stages
    assert len([e for e in execution.robot.events if e[0]=='move'])==3


def test_preclose_descent_accounts_for_future_jaw_extension_without_lowering_tip_into_board(execution):
    from janggi_pick_place2.geometry import height_from_command
    grasp=execution.resolve(20.,10.,100.)
    execution.run('pick',[400.,50.,-18.,0.],grasp,lambda:None,lambda *_:None)
    descent=[e[1] for e in execution.robot.events if e[0]=='move'][-2]
    opened=command_from_width(grasp.open_width_mm);closed=command_from_width(grasp.close_width_mm)
    # During jaw closure the tip approaches the requested height from ABOVE.
    for i in range(101):
        c=opened+(closed-opened)*i/100
        tip=descent[2]-132.6717-height_from_command(c)
        assert tip>=-18.-1e-9
    assert descent[2]-132.6717-height_from_command(closed)==pytest.approx(-18.)
    assert execution.settings.tcp_length_at_6mm_mm==132.6717
    before=len(execution.robot.events)
    execution.run('place',[450.,100.,-18.,0.],None,lambda:None,lambda *_:None)
    placed=[e[1] for e in execution.robot.events[before:] if e[0]=='move'][-2]
    assert placed[2]==pytest.approx(descent[2])
    assert execution.holding_state=='empty_unverified'


def test_extreme_closing_range_includes_interior_height_maximum(execution):
    from janggi_pick_place2.geometry import height_from_command
    grasp=execution.resolve(6.,10.,100.)
    # H(c) peaks before c=700; endpoint-only checks would miss this small maximum.
    destination,_=execution._approach([400.,50.,0.,0.],grasp,lambda:None)
    opened=command_from_width(grasp.open_width_mm)
    for i in range(101):
        c=opened+(700-opened)*i/100
        assert destination[2]-132.6717-height_from_command(c)>=-1e-9
