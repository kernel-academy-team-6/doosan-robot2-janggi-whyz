"""The real adapter delegates to position_cmd and never opens serial clients."""
from types import SimpleNamespace
import time

import pytest

from janggi_tcp_test.transport import TestGripper
from janggi_pick_place2.geometry import command_from_width


class Publisher:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


class Node:
    def create_publisher(self, _kind, topic, _depth):
        self.topic = topic
        self.publisher = Publisher()
        return self.publisher

    def create_client(self, *_args, **_kwargs):
        raise AssertionError('no gripper service clients permitted')


@pytest.fixture
def hand():
    return TestGripper(Node(), '/dsr01', {
        'service_timeout_sec': .001, 'gripper_command_wait_sec': .03,
    })


def test_initialize_waits_after_single_initial_position_command(hand):
    start = time.monotonic()
    u = hand.initialize(40., lambda: None)
    assert time.monotonic()-start >= .03
    assert u == command_from_width(40.)
    assert hand.topic == '/dsr01/gripper/position_cmd'
    assert [msg.data for msg in hand.publisher.messages] == [u]


def test_transition_dispatch_timestamp_does_not_wait_for_gripper_feedback(hand):
    called = []
    before = time.monotonic()
    hand.begin(500, called.append, lambda: None)
    after = time.monotonic()
    assert len(called) == 1 and before <= called[0] <= after
    assert [msg.data for msg in hand.publisher.messages] == [500]
    hand.close_session()
    assert len(hand.publisher.messages) == 1  # No hold/open on cleanup.


def test_missing_receiver_fails_without_publishing(hand):
    hand.publisher.subscribers = 0
    with pytest.raises(TimeoutError, match='no subscriber'):
        hand.initialize(40., lambda: None)
    assert not hand.publisher.messages


def test_cancellation_during_initial_wait_does_not_send_a_hold_or_release(hand):
    checks = []
    def cancelled():
        checks.append(True)
        if hand.publisher.messages:
            raise RuntimeError('cancelled')
    with pytest.raises(RuntimeError, match='cancelled'):
        hand.initialize(40., cancelled)
    hand.close_session()
    assert [msg.data for msg in hand.publisher.messages] == [command_from_width(40.)]


@pytest.mark.parametrize('u', [-1, 701, 30.5])
def test_invalid_position_rejected_before_publish(hand, u):
    with pytest.raises(ValueError):
        hand.begin(u, lambda _stamp: None, lambda: None)
    assert not hand.publisher.messages
