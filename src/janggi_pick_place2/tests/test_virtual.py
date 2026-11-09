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
