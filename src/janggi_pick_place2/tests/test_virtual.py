"""Virtual mode must animate gripper without touching serial/hardware control."""
from types import SimpleNamespace
import json

import pytest

from janggi_pick_place2.executor import Grasp, OperationCancelled
from janggi_pick_place2.geometry import command_from_width
from janggi_pick_place2.gripper import VirtualGripperClient
from janggi_pick_place2.robot_client import RobotClient, VirtualRobotClient


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, kind, topic, depth):
        publisher = Publisher()
        self.publishers[topic] = publisher
        return publisher

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))

    def create_client(self, *args, **kwargs):
        raise AssertionError('virtual gripper must not create any serial clients')


def test_virtual_gripper_shows_contact_and_release_without_serial():
    node = Node()
    gripper = VirtualGripperClient(node, '/dsr02', 0.)
    check = lambda: None
    gripper.prepare(check)
    gripper.open_to(40., 100, check)
    sample = gripper.grasp(Grasp(30., 100, 40., 29., 35.), check)
    assert sample['source'] == 'virtual_command_only' and not sample['feedback_received']
    assert not sample['grip_verified'] and gripper.last_sample is None
    gripper.open_to(35., 100, check)
    gripper.close_session()
    assert [m.data for m in node.publishers['/dsr02/gripper/stroke'].messages] == [
        command_from_width(w) for w in (40., 29., 35.)]
    state = json.loads(node.publishers['~/gripper/state'].messages[-1].data)
    assert state['commanded_width_mm'] == 35. and state['source'] == 'virtual_command_only'


def test_virtual_gripper_cancel_does_not_publish_another_open():
    node = Node()
    gripper = VirtualGripperClient(node, '/dsr01', .1)
    def cancelled():
        raise OperationCancelled()
    with pytest.raises(OperationCancelled):
        gripper.open_to(40., 100, cancelled)
    assert node.publishers['/dsr01/gripper/stroke'].messages == []


def test_virtual_arm_refuses_real_controller_before_motion(monkeypatch):
    robot = object.__new__(VirtualRobotClient)
    robot._virtual_confirmed = False
    robot.io = SimpleNamespace(call=lambda *args: SimpleNamespace(robot_system=0))
    monkeypatch.setattr(RobotClient, 'check_tcp', lambda *args: pytest.fail('unexpected motion preparation'))
    with pytest.raises(RuntimeError, match='ROBOT_SYSTEM_VIRTUAL'):
        robot.check_tcp(lambda: None)
    assert robot.stop() is None


def test_virtual_arm_accepts_virtual_system_and_checks_tcp(monkeypatch):
    robot = object.__new__(VirtualRobotClient)
    robot._virtual_confirmed = False
    robot.io = SimpleNamespace(call=lambda *args: SimpleNamespace(robot_system=1))
    called = []
    monkeypatch.setattr(RobotClient, 'check_tcp', lambda *args: called.append('tcp'))
    monkeypatch.setattr(RobotClient, 'stop', lambda *args: called.append('stop'))
    robot.check_tcp(lambda: None)
    robot.stop()
    assert called == ['tcp', 'stop']


@pytest.mark.parametrize('first_position', [0, 700])
def test_virtual_full_travel_waits_for_the_servo_and_preserves_cancel(monkeypatch, first_position):
    import janggi_pick_place2.gripper as module
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    gripper=VirtualGripperClient(Node(),'/isolated',.4)
    result=gripper.set_position(first_position,lambda:None)
    assert result['travel_wait_sec']==pytest.approx(1.95)
    assert clock[0]>=1.95
    assert not result['grip_verified']  # a time allowance is not grasp sensing
    result=gripper.set_position(40 if first_position==0 else 660,lambda:None)
    assert result['travel_wait_sec']==pytest.approx(.4)


def test_virtual_cancelled_travel_does_not_assume_commanded_opening(monkeypatch):
    import janggi_pick_place2.gripper as module
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    gripper=VirtualGripperClient(Node(),'/isolated',.4)
    gripper.set_position(0,lambda:None)
    cancel_at=clock[0]+.1
    def cancelled_during_wait():
        if clock[0]>=cancel_at:
            raise OperationCancelled()
    with pytest.raises(OperationCancelled):
        gripper.set_position(700,cancelled_during_wait)
    assert not gripper.last_command['wait_completed']
    # Retrying the same goal must allow a full travel, not just the 0.4 s minimum.
    result=gripper.set_position(700,lambda:None)
    assert result['travel_wait_sec']==pytest.approx(1.95)
