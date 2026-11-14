"""Use exactly the existing Int32 command route, without serial clients."""
from types import SimpleNamespace
import json

import pytest

from janggi_pick_place2.executor import OperationCancelled
from janggi_pick_place2.geometry import command_from_width
from janggi_pick_place2.gripper import GripperClient


class Publisher:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


class Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, _kind, topic, _depth):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))

    def create_client(self, *_args, **_kwargs):
        raise AssertionError('no gripper service clients permitted')


@pytest.fixture
def hand():
    hand = GripperClient(Node(), '/dsr01', {
        'service_timeout_sec': .001, 'gripper_command_wait_sec': 5.,
    })
    hand.delays = []
    def sleep(seconds, check):
        check()
        hand.delays.append(seconds)
    hand._sleep = sleep
    return hand


def test_exact_existing_topic_for_open_close_release_without_serial(hand):
    hand.prepare(lambda: None)
    assert not hand.position_cmd.messages  # No startup gripper movement.
    hand.open_to(40., 100, lambda: None)
    state = hand.grasp(SimpleNamespace(close_width_mm=29., current_ma=100), lambda: None)
    hand.open_to(35., 100, lambda: None)
    hand.close_session()
    assert set(hand.node.publishers) == {'/dsr01/gripper/position_cmd', '~/gripper/state'}
    assert [msg.data for msg in hand.position_cmd.messages] == [
        command_from_width(w) for w in (40., 29., 35.)]
    assert hand.delays == [5., 5., 5.]
    assert state['wait_completed'] and not state['feedback_received']
    assert not state['grip_verified'] and not state['current_limit_command_sent']
    assert state['requested_current_ma'] == 100
    assert hand.last_sample is None
    published = json.loads(hand.node.publishers['~/gripper/state'].messages[-1].data)
    assert published['commanded_width_mm'] == 35.
    assert 'width_mm' not in published and 'current_ma' not in published


def test_missing_subscriber_fails_before_sending(hand):
    hand.position_cmd.subscribers = 0
    with pytest.raises(TimeoutError, match='no subscriber'):
        hand.prepare(lambda: None)
    assert not hand.position_cmd.messages


def test_lost_subscriber_is_checked_for_every_command(hand):
    hand.prepare(lambda: None)
    hand.position_cmd.subscribers = 0
    with pytest.raises(TimeoutError, match='no subscriber'):
        hand.open_to(40., 100, lambda: None)
    assert not hand.position_cmd.messages


def test_cancel_during_wait_does_not_send_release(hand):
    def cancelled(_seconds, _check):
        raise OperationCancelled('cancel during wait')
    hand._sleep = cancelled
    with pytest.raises(OperationCancelled):
        hand.grasp(SimpleNamespace(close_width_mm=29., current_ma=100), lambda: None)
    assert not hand.last_command['wait_completed']
    hand.close_session()
    assert [msg.data for msg in hand.position_cmd.messages] == [command_from_width(29.)]


def test_cancel_before_publish_sends_nothing(hand):
    def cancelled():
        raise OperationCancelled()
    with pytest.raises(OperationCancelled):
        hand.open_to(40., 100, cancelled)
    assert not hand.position_cmd.messages


def test_workspace_defaults_progress_without_any_result_response():
    from pathlib import Path
    import yaml
    params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/pick_place.yaml').read_text())['/janggi_pick_place2']['ros__parameters']
    assert params['gripper_wait_for_result'] is False
    assert params['gripper_command_wait_sec']==2.
    # Node intentionally has no create_subscription/count_publishers result API.
    h=GripperClient(Node(),'/dsr01',params)
    h._wait_result=lambda *args:pytest.fail('must not wait for bridge response')
    waits=[];h._sleep=lambda seconds,check:waits.append(seconds)
    state=h.open_to(30.,100,lambda:None)
    assert h.topic=='/dsr01/gripper/position_cmd'
    assert waits==[2.] and state['wait_completed']
    assert not state['feedback_received'] and not state['grip_verified']


def test_identical_grave_release_commands_are_published_twice_with_wait():
    # Use actual configuration path; no response subscription or serial probe.
    hand=GripperClient(Node(),'/dsr01',{'gripper_command_wait_sec':2.,'service_timeout_sec':.01})
    delays=[];hand._sleep=lambda seconds,check:delays.append(seconds)
    hand.open_to(42.,100,lambda:None)
    hand.open_to(42.,100,lambda:None)
    assert [m.data for m in hand.position_cmd.messages]==[command_from_width(42.)]*2
    assert delays==[2.,2.]
