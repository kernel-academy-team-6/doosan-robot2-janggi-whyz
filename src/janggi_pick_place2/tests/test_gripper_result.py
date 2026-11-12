import json
from types import SimpleNamespace
import time

import pytest
from janggi_pick_place2.gripper import GripperClient


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
    def get_subscription_count(self):return 1


class Node:
    def create_publisher(self,*args):return Publisher()
    def create_subscription(self,kind,topic,callback,depth):self.callback=callback;return object()
    def count_publishers(self,topic):return 1
    def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1))


def hand():
    return GripperClient(Node(),'/dsr01',dict(service_timeout_sec=.1,gripper_command_wait_sec=1.,
                                              gripper_wait_for_result=True,gripper_result_timeout_sec=.04))


def test_ack_wait_ignores_old_or_other_commands_and_does_not_claim_physical_feedback():
    h=hand();issued=time.time_ns()
    for position,stamp in [(50,issued-1),(99,issued+1)]:
        h._result(SimpleNamespace(data=json.dumps(dict(position=position,success=True,request_id=stamp))))
    with pytest.raises(TimeoutError):h._wait_result(50,issued,lambda:None)
    h._result(SimpleNamespace(data=json.dumps(dict(position=50,success=True,request_id=issued))))
    h._wait_result(50,issued,lambda:None)


def test_negative_ack_and_cancel_stop_before_following_arm_action():
    h=hand();issued=time.time_ns()
    h._result(SimpleNamespace(data=json.dumps(dict(position=50,success=False,request_id=issued))))
    with pytest.raises(RuntimeError):h._wait_result(50,issued,lambda:None)
    def cancelled():raise RuntimeError('cancelled')
    with pytest.raises(RuntimeError,match='cancelled'):h._wait_result(50,issued,cancelled)


def test_real_command_waits_for_matching_result_instead_of_one_second_guess():
    h=hand()
    def publish(message):
        h._result(SimpleNamespace(data=json.dumps(dict(position=message.data[1],success=True,request_id=message.data[0]))))
    h.position_cmd.publish=publish
    h._sleep=lambda *args:pytest.fail('acknowledged commands must not use fixed wait')
    result=h.open_to(30.,100,lambda:None)
    assert result['bridge_acknowledged'] and result['wait_completed']
    assert not result['grip_verified'] and not result['feedback_received']


@pytest.mark.parametrize('command',[700,650,0])
def test_raw_command_does_not_round_trip_through_width_formula(command):
    h=hand();sent=[]
    def publish(message):
        sent.append(message.data[1])
        h._result(SimpleNamespace(data=json.dumps(dict(position=message.data[1],success=True,request_id=message.data[0]))))
    h.position_cmd.publish=publish
    result=h.set_position(command,lambda:None)
    assert sent==[command] and result['commanded_position']==command
    assert result['width_source']=='legacy_formula_estimate'
