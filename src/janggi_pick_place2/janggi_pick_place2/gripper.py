"""Delegate gripper serial I/O to its bridge; optionally correlate command results."""
import json
import time
import threading
from collections import deque

from std_msgs.msg import Int32, Int64MultiArray, String
from .geometry import command_from_width, width_from_command


def raw_position(value):
    if isinstance(value,bool) or not isinstance(value,int) or not 0<=value<=700:
        raise ValueError('그리퍼 명령은 0~700 정수여야 합니다.')
    return value


def command_state(width_mm, current_ma, source):
    return {
        'source': source, 'feedback_received': False, 'grip_verified': False,
        'commanded_position': command_from_width(width_mm),
        'commanded_width_mm': width_mm, 'requested_current_ma': int(current_ma),
        'current_limit_command_sent': False, 'current_limit_verified': False,
    }


class GripperClient:
    def __init__(self, node, prefix, parameters):
        self.node, self.p = node, parameters
        self.last_sample = None
        self.last_command = None
        self.wait_for_result = parameters.get('gripper_wait_for_result',False)
        self.topic = prefix+('/gripper/position_request' if self.wait_for_result else '/gripper/position_cmd')
        self.position_cmd = node.create_publisher(Int64MultiArray if self.wait_for_result else Int32,self.topic,10)
        self.state = node.create_publisher(String, '~/gripper/state', 10)
        self.result_topic = prefix+'/gripper/command_result'
        self._result_lock = threading.Lock()
        self._results = deque(maxlen=12)
        self.result_subscription = (node.create_subscription(String,self.result_topic,self._result,10)
                                    if self.wait_for_result else None)

    def _result(self, message):
        try:
            result=json.loads(message.data)
            if (not isinstance(result.get('position'),int) or not isinstance(result.get('success'),bool)
                    or not isinstance(result.get('request_id'),int)):
                return
        except (ValueError,TypeError,AttributeError): return
        with self._result_lock: self._results.append(result)

    def _wait_result(self, position, issued_ns, check):
        deadline=time.monotonic()+self.p.get('gripper_result_timeout_sec',20.)
        while True:
            check()
            with self._result_lock:
                results=[r for r in self._results if r['position']==position and r['request_id']==issued_ns]
            if results:
                if not results[-1]['success']:
                    raise RuntimeError('그리퍼 노드가 명령 처리 실패를 반환했습니다.')
                return
            if time.monotonic()>=deadline:
                raise TimeoutError('그리퍼 명령 처리 응답이 없습니다. gripper_service_node를 다시 빌드/실행하세요.')
            time.sleep(.02)

    def _sleep(self, seconds, check):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.02, max(0., deadline-time.monotonic())))
        check()

    def prepare(self, check):
        deadline = time.monotonic()+self.p['service_timeout_sec']
        while self.position_cmd.get_subscription_count() == 0:
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'no subscriber on {self.topic}; start existing gripper bringup first')
            time.sleep(.02)
        check()

        if self.wait_for_result:
            deadline=time.monotonic()+self.p['service_timeout_sec']
            while self.node.count_publishers(self.result_topic)==0:
                check()
                if time.monotonic()>=deadline:
                    raise TimeoutError('command_result 퍼블리셔가 없습니다. 갱신된 그리퍼 bringup을 실행하세요.')
                time.sleep(.02)

    def _command(self, width_mm, current_ma, check, *, position_override=None):
        position = command_from_width(width_mm) if position_override is None else raw_position(position_override)
        self.prepare(check)
        self.last_command = command_state(width_mm, current_ma, 'position_request' if self.wait_for_result else 'position_cmd')
        self.last_command['commanded_position'] = position
        self.last_command['stamp_ns'] = self.node.get_clock().now().nanoseconds
        self.last_command['wait_completed'] = False
        # The existing gripper_service_node performs torque enable + goal write.
        # It owns the serial port and stroke animation. We publish exactly once.
        observer = getattr(self, 'command_observer', None)
        if observer:
            observer('gripper_command', dict(topic=self.topic, position=position,
                                            width_mm=width_mm, wait_sec=self.p['gripper_command_wait_sec'],
                                            completion='bridge_result' if self.wait_for_result else 'timed_wait'))
        issued_ns=time.time_ns()
        self.position_cmd.publish(Int64MultiArray(data=[issued_ns,position]) if self.wait_for_result else Int32(data=position))
        self.state.publish(String(data=json.dumps(self.last_command)))
        if self.wait_for_result:
            self._wait_result(position,issued_ns,check)
            self.last_command['bridge_acknowledged']=True
        else:
            self._sleep(self.p['gripper_command_wait_sec'], check)
        self.last_command['wait_completed'] = True
        self.state.publish(String(data=json.dumps(self.last_command)))
        return dict(self.last_command)

    def open_to(self, width_mm, current_ma, check):
        return self._command(width_mm, current_ma, check)

    def grasp(self, grasp, check):
        return self._command(grasp.close_width_mm, grasp.current_ma, check)

    def set_position(self, position, check):
        position = raw_position(position)
        result = self._command(width_from_command(position),100,check,position_override=position)
        # Width here is only an estimate from the old model, not a measurement.
        result['width_source'] = 'legacy_formula_estimate'
        return result

    def close_session(self):
        # Existing gripper_service_node owns its serial lifecycle.
        pass


class MockGripper:
    def __init__(self):
        self.calls = []
        self.last_sample = None
        self.last_command = None

    def prepare(self, check):
        check()

    def open_to(self, width_mm, current_ma, check):
        check()
        self.calls.append(('open', width_mm, command_from_width(width_mm), current_ma))
        self.last_command = command_state(width_mm, current_ma, 'mock_command_only')
        self.last_command['wait_completed'] = True
        return dict(self.last_command)

    def grasp(self, grasp, check):
        check()
        self.calls.append(('grasp', grasp.close_width_mm,
                           command_from_width(grasp.close_width_mm), grasp.current_ma))
        self.last_command = command_state(grasp.close_width_mm, grasp.current_ma, 'mock_command_only')
        self.last_command['wait_completed'] = True
        return dict(self.last_command)

    def close_session(self):
        pass

    def set_position(self, position, check):
        position = raw_position(position);check()
        self.calls.append(('raw_position',position))
        self.last_command = command_state(width_from_command(position),100,'mock_raw_position')
        self.last_command.update(commanded_position=position,wait_completed=True,width_source='legacy_formula_estimate')
        return dict(self.last_command)


class VirtualGripperClient(MockGripper):
    """Commanded-gap animation without claiming physical contact."""
    def __init__(self, node, prefix, delay_sec):
        super().__init__()
        self.node, self.delay_sec = node, delay_sec
        self.topic = prefix+'/gripper/stroke'
        self.stroke = node.create_publisher(Int32, prefix+'/gripper/stroke', 10)
        self.state = node.create_publisher(String, '~/gripper/state', 10)

    def _show(self, check):
        check()
        self.last_command['source'] = 'virtual_command_only'
        self.last_command['stamp_ns'] = self.node.get_clock().now().nanoseconds
        self.last_command['wait_completed'] = False
        observer = getattr(self, 'command_observer', None)
        if observer:
            observer('gripper_command', dict(topic=self.topic,
                                            position=self.last_command['commanded_position'],
                                            width_mm=self.last_command['commanded_width_mm'],
                                            wait_sec=self.delay_sec))
        self.stroke.publish(Int32(data=self.last_command['commanded_position']))
        self.state.publish(String(data=json.dumps(self.last_command)))
        deadline = time.monotonic()+self.delay_sec
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.02, max(0., deadline-time.monotonic())))
        check()
        self.last_command['wait_completed'] = True
        self.state.publish(String(data=json.dumps(self.last_command)))
        return dict(self.last_command)

    def set_position(self, position, check):
        super().set_position(position,check)
        self._show(check)
        return dict(self.last_command)

    def open_to(self, width_mm, current_ma, check):
        check()
        super().open_to(width_mm, current_ma, check)
        return self._show(check)

    def grasp(self, grasp, check):
        check()
        super().grasp(grasp, check)
        return self._show(check)
