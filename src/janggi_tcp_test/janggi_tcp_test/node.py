"""Independent, slow model-based gripper/Z synchronization experiment."""
from pathlib import Path
import json
import math
import signal
import threading
import time
import uuid

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.validate_namespace import validate_namespace
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from dsr_msgs2.msg import ServolStream
from dsr_msgs2.srv import GetRobotSystem

from janggi_pick_place2.executor import Settings
from janggi_pick_place2.geometry import width_from_command, tcp_length, orientation_error_deg
from janggi_pick_place2.robot_client import RobotClient, VirtualRobotClient, ServiceCaller
from janggi_tcp_test.action import TestTcpHeight
from .profile import Profile, CompensationPlan, endpoints
from .report import save_report
from .transport import TestGripper


DEFAULTS = {
    'backend':'mock', 'robot_namespace':'dsr01', 'expected_flange_tcp_name':'',
    'tcp_length_at_6mm_mm':132.6717, 'position_unit_deg':.088,
    'gripper_profile_velocity':50, 'gripper_profile_acceleration':100,
    'gripper_command_wait_sec':5.,
    'stream_rate_hz':20., 'servo_time_sec':.1, 'arm_lookahead_sec':.1,
    'gripper_start_delay_sec':.3, 'arm_velocity_mm_s':5., 'arm_acceleration_mm_s2':10.,
    'max_z_displacement_mm':30., 'min_flange_z_mm':-200., 'max_flange_z_mm':1000.,
    'max_duration_sec':120., 'max_stream_gap_sec':.25, 'feedback_interval_sec':.5,
    'max_feedback_age_sec':3., 'max_estimated_tip_error_mm':3.,
    'final_settle_sec':1., 'completion_timeout_sec':8.,
    'mock_initial_flange_pose':[400.,0.,450.,0.,180.,0.],
    'report_directory':'/tmp/janggi_tcp_test',
    'service_timeout_sec':5., 'motion_timeout_sec':60.,

}


class Cancelled(Exception):
    pass


class TcpTestNode(Node):
    def __init__(self):
        super().__init__('janggi_tcp_test')
        self.p = {}
        for name,default in DEFAULTS.items():
            self.declare_parameter(name,default,ParameterDescriptor(read_only=True))
            self.p[name] = self.get_parameter(name).value
        p = self.p
        if p['backend'] not in ('mock','virtual','real'):
            raise ValueError('backend must be mock, virtual, or real')
        for name,value in p.items():
            if isinstance(value,(int,float)) and not math.isfinite(value):
                raise ValueError(name+' must be finite')
        for name in ('position_unit_deg','tcp_length_at_6mm_mm','gripper_profile_velocity',
                     'gripper_profile_acceleration','stream_rate_hz','servo_time_sec',
                     'arm_velocity_mm_s','arm_acceleration_mm_s2','max_z_displacement_mm',
                     'max_duration_sec','max_stream_gap_sec','feedback_interval_sec',
                     'max_feedback_age_sec','max_estimated_tip_error_mm','completion_timeout_sec',
                     'service_timeout_sec','motion_timeout_sec','gripper_command_wait_sec'):
            if p[name] <= 0:
                raise ValueError(name+' must be positive')
        for name in ('arm_lookahead_sec','gripper_start_delay_sec','final_settle_sec'):
            if p[name] < 0:
                raise ValueError(name+' must be nonnegative')
        if (len(p['mock_initial_flange_pose']) != 6
                or not all(math.isfinite(v) for v in p['mock_initial_flange_pose'])):
            raise ValueError('mock_initial_flange_pose must contain six finite values')
        if not p['report_directory'].strip():
            raise ValueError('report_directory must not be empty')
        if p['max_flange_z_mm'] <= p['min_flange_z_mm'] or p['stream_rate_hz'] > 100:
            raise ValueError('invalid Z bounds or stream rate (max 100 Hz)')
        if p['max_stream_gap_sec'] <= 1./p['stream_rate_hz']:
            raise ValueError('max_stream_gap_sec must exceed the stream period')
        self.prefix = '/'+p['robot_namespace'].strip('/') if p['robot_namespace'].strip('/') else ''
        validate_namespace(self.prefix or '/')
        self.lock = threading.Lock()
        self.busy = False
        self.cancelled = threading.Event()
        self.shutting_down = threading.Event()
        self.stop_worker = threading.Event()
        self.shared_lock = threading.Lock()
        self.robot = self.gripper = self.servo = self.stroke = None
        self.motion_permitted = False
        self.summary = {'backend':p['backend'],'stage':'idle','last_report':'',
                        'gripper_transport':'position_cmd' if p['backend']=='real' else p['backend'],
                        'gripper_feedback_received':False,'profile_settings_sent':False}
        self.guard_io = ServiceCaller(self,'',p['service_timeout_sec'])
        if p['backend'] != 'mock':
            settings = Settings(tcp_length_at_6mm_mm=p['tcp_length_at_6mm_mm'],
                                linear_velocity_mm_s=p['arm_velocity_mm_s'],
                                linear_acceleration_mm_s2=p['arm_acceleration_mm_s2'])
            cls = VirtualRobotClient if p['backend']=='virtual' else RobotClient
            self.robot = cls(self,self.prefix,settings,p['expected_flange_tcp_name'],
                             p['service_timeout_sec'],p['motion_timeout_sec'])
            self.servo = self.create_publisher(ServolStream,self.prefix+'/servol_stream',1)
            if p['backend']=='virtual':
                self.stroke = self.create_publisher(Int32,self.prefix+'/gripper/stroke',10)
            else:
                self.gripper = TestGripper(self,self.prefix,p)
        self.telemetry = self.create_publisher(String,'~/profile',10)
        group = ReentrantCallbackGroup()
        self.server = ActionServer(self,TestTcpHeight,'~/test',self.execute,
                                   goal_callback=self.accept,cancel_callback=self.cancel,
                                   callback_group=group)
        self.status_service = self.create_service(Trigger,'~/status',self.status,callback_group=group)
        self.get_logger().info(f'TCP height test ready: backend={p["backend"]}; no startup motion. '
                               'Two inputs: initial_width_mm, target_width_mm. No home or XY moves.')
        if p['backend']=='real':
            self.get_logger().warn('Gripper uses existing position_cmd only. Speed/acceleration are '
                                   'MODEL assumptions; no current/profile settings or gripper reads are sent. '
                                   'Cancellation stops the arm, but a published gripper goal may continue.')

    def check(self):
        if self.cancelled.is_set() or self.shutting_down.is_set() or not self.context.ok():
            raise Cancelled('test cancelled; arm stop requested; published gripper command may continue')

    def io_check(self):
        # Allow an in-flight arm-pose query to finish before the worker exits.
        if not self.context.ok():
            raise Cancelled('ROS context shut down')

    def make_profile(self,u0,u1):
        return Profile(u0,u1,self.p['gripper_profile_velocity'],
                       self.p['gripper_profile_acceleration'],self.p['position_unit_deg'])

    def accept(self,request):
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            try:
                u0,u1 = endpoints(request.initial_width_mm,request.target_width_mm)
                if self.make_profile(u0,u1).duration > self.p['max_duration_sec']:
                    raise ValueError('profile duration exceeds max_duration_sec')
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return GoalResponse.REJECT
            self.busy = True
            self.motion_permitted = False
            self.cancelled.clear()
            self.stop_worker.clear()
        return GoalResponse.ACCEPT

    def cancel(self,_goal):
        self.cancelled.set()
        self.stop_arm()
        return CancelResponse.ACCEPT

    def stop_arm(self):
        if self.robot is not None and self.motion_permitted:
            return self.robot.stop()
        return None

    def status(self,_request,response):
        response.success = True
        response.message = json.dumps(dict(self.summary,busy=self.busy),ensure_ascii=False)
        return response

    def stage(self,name):
        self.summary['stage'] = name
        self.get_logger().info(name)

    def ensure_idle(self):
        for name in ('janggi_pick_place','janggi_pick_place2'):
            path = '/'+name+'/status'
            client = self.guard_io.client(Trigger,path)
            if client.service_is_ready():
                state = json.loads(self.guard_io.call(Trigger,path,Trigger.Request(),self.check).message)
                if state.get('busy') or state.get('holding_state','empty') not in ('empty','empty_unverified'):
                    raise RuntimeError(name+' is busy or holding an object; test requires empty gripper')

    def prepare_arm(self):
        p = self.p
        if p['backend']=='mock':
            return list(p['mock_initial_flange_pose'])
        system = self.robot.io.call(GetRobotSystem,'/system/get_robot_system',
                                    GetRobotSystem.Request(),self.check)
        expected = 1 if p['backend']=='virtual' else 0
        if system.robot_system != expected:
            raise RuntimeError(f'backend={p["backend"]} requires robot_system={expected}; no motion sent')
        self.motion_permitted = True
        self.robot.check_tcp(self.check)
        deadline = time.monotonic()+p['service_timeout_sec']
        while self.servo.get_subscription_count()==0:
            self.check()
            if time.monotonic()>deadline:
                raise TimeoutError('no /servol_stream subscriber; controller not ready')
            time.sleep(.02)
        return self.robot.current_pose(self.check)

    def validate_plan(self,plan):
        p = self.p
        return plan.validate(p['max_duration_sec'],p['max_z_displacement_mm'],
                             p['arm_velocity_mm_s'],p['arm_acceleration_mm_s2'],
                             p['min_flange_z_mm'],p['max_flange_z_mm'])

    def send_pose(self,pose):
        if self.servo is None:
            return
        msg = ServolStream()
        msg.pos = list(pose)
        msg.vel = [self.p['arm_velocity_mm_s'],5.]
        msg.acc = [self.p['arm_acceleration_mm_s2'],10.]
        msg.time = self.p['servo_time_sec']
        self.servo.publish(msg)

    def feedback_row(self,plan,dispatch):
        begin = time.monotonic()
        pose = self.robot.current_pose(self.io_check)
        now = time.monotonic()
        t = now-dispatch-self.p['gripper_start_delay_sec']
        u = plan.profile.position(t)
        source = ('gripper_model_and_arm_pose' if self.p['backend']=='real' else
                  'virtual_gripper_model_and_arm_pose')
        w = width_from_command(u)
        tip = pose[2]+math.cos(math.radians(pose[4]))*tcp_length(w,self.p['tcp_length_at_6mm_mm'])
        if math.dist(pose[:2],plan.pose[:2]) > 1. or orientation_error_deg(pose[3:],plan.pose[3:]) > 1.:
            raise RuntimeError('unexpected XY/orientation change during Z-only test')
        row = {'kind':'feedback','elapsed_sec':t,'model_u':u,'model_width_mm':w,
               'measured_flange_z_mm':pose[2],'estimated_tip_z_mm':tip,
               'tip_error_mm':tip-plan.tip_z,'measurement_span_sec':now-begin,'source':source,
               'gripper_feedback_received':False,'arm_pose_received':True}
        return row,now

    def worker(self,plan,dispatch,shared,feedback_rows):
        try:
            while not self.stop_worker.is_set():
                row,stamp = self.feedback_row(plan,dispatch)
                with self.shared_lock:
                    feedback_rows.append(row)
                    shared.update(last_feedback=row,last_feedback_at=stamp,
                                  target_reached=row['elapsed_sec']>=plan.profile.duration)
                if abs(row['tip_error_mm']) > self.p['max_estimated_tip_error_mm']:
                    raise RuntimeError('model-and-arm estimated tip height error exceeds configured limit')
                self.stop_worker.wait(self.p['feedback_interval_sec'])
        except Exception as exc:
            with self.shared_lock:
                shared['error'] = str(exc)

    def execute(self,goal):
        p = self.p
        result = TestTcpHeight.Result()
        result.max_estimated_tip_error_mm = float('nan')
        plan_rows,feedback_rows,shared = [],[],{}
        worker = None
        plan = None
        directory = Path(p['report_directory']).expanduser()/('run_'+uuid.uuid4().hex[:12])
        metadata = {'backend':p['backend'],'initial_width_mm':goal.request.initial_width_mm,
                    'target_width_mm':goal.request.target_width_mm,'parameters':p,
                    'gripper_transport':'position_cmd' if p['backend']=='real' else p['backend'],
                    'gripper_feedback_received':False,'profile_settings_sent':False,
                    'note':'Predicted gripper gap only; arm pose is read. Physical tip height is unverified.'}
        try:
            self.stage('checking')
            self.check()
            self.ensure_idle()
            u0,u1 = endpoints(goal.request.initial_width_mm,goal.request.target_width_mm)
            pose = self.prepare_arm()
            self.validate_plan(CompensationPlan(self.make_profile(u0,u1),pose,p['tcp_length_at_6mm_mm']))
            self.stage('initializing_gap_arm_fixed')
            if self.gripper is not None:
                u0 = self.gripper.initialize(goal.request.initial_width_mm,self.check)
                pose = self.robot.current_pose(self.check)
            elif self.stroke is not None:
                self.stroke.publish(Int32(data=int(u0)))
                deadline = time.monotonic()+.5
                while time.monotonic()<deadline:
                    self.check()
                    time.sleep(.02)
            plan = CompensationPlan(self.make_profile(u0,u1),pose,p['tcp_length_at_6mm_mm'])
            limits = self.validate_plan(plan)
            result.model_duration_sec = plan.profile.duration
            metadata.update(initial_u=u0,target_u=u1,initial_flange_pose=pose,
                            initial_tip_z_mm=plan.tip_z,model_duration_sec=plan.profile.duration,
                            profile_kind=plan.profile.kind,limits=limits)
            self.stage('synchronized_transition')
            if p['backend']=='mock':
                shared['dispatch'] = time.monotonic()
            else:
                self.send_pose(plan.pose)
                if self.gripper is not None:
                    self.gripper.begin(int(u1),lambda stamp: shared.update(dispatch=stamp),self.check)
                else:
                    shared['dispatch'] = time.monotonic()
                worker = threading.Thread(target=self.worker,
                                          args=(plan,shared['dispatch'],shared,feedback_rows),daemon=True)
                worker.start()
            dispatch = shared['dispatch']
            period = 1./p['stream_rate_hz']
            last_tick,next_tick,last_feedback = time.monotonic(),time.monotonic(),-1.
            while True:
                self.check()
                now = time.monotonic()
                if now-last_tick > p['max_stream_gap_sec']:
                    raise RuntimeError('servo publishing deadline missed')
                last_tick = now
                elapsed = now-dispatch-p['gripper_start_delay_sec']
                sample = plan.sample(elapsed)
                command = plan.target_pose(elapsed+p['arm_lookahead_sec'])
                self.send_pose(command)
                if self.stroke is not None:
                    self.stroke.publish(Int32(data=int(round(sample['model_u']))))
                row = dict(sample,kind='model',elapsed_sec=elapsed,command_flange_z_mm=command[2],source='predicted')
                plan_rows.append(row)
                if p['backend']=='mock':
                    simulated = {'kind':'feedback','elapsed_sec':elapsed,'model_u':sample['model_u'],
                                 'model_width_mm':sample['model_width_mm'],
                                 'measured_flange_z_mm':sample['desired_flange_z_mm'],
                                 'estimated_tip_z_mm':plan.tip_z,'tip_error_mm':0.,
                                 'measurement_span_sec':0.,'source':'mock_model_only',
                                 'gripper_feedback_received':False,'arm_pose_received':False}
                    feedback_rows.append(simulated)
                    shared.update(last_feedback=simulated,last_feedback_at=now,
                                  target_reached=elapsed>=plan.profile.duration)
                with self.shared_lock:
                    latest = dict(shared.get('last_feedback',{}))
                    age = max(0.,now-shared.get('last_feedback_at',dispatch))
                    error = shared.get('error')
                    reached = shared.get('target_reached',False)
                if error:
                    raise RuntimeError(error)
                if age > p['max_feedback_age_sec']:
                    raise TimeoutError('feedback is stale; stopping Z stream')
                if elapsed > plan.profile.duration+p['completion_timeout_sec']:
                    raise TimeoutError('model duration elapsed but final arm target was not confirmed')
                if now-last_feedback >= .2:
                    feedback = TestTcpHeight.Feedback()
                    feedback.stage = self.summary['stage']
                    feedback.elapsed_sec = max(0.,elapsed)
                    feedback.progress = min(1.,max(0.,elapsed/max(plan.profile.duration,1e-6)))
                    feedback.predicted_width_mm = sample['model_width_mm']
                    feedback.command_flange_z_mm = command[2]
                    feedback.estimated_tip_z_mm = latest.get('estimated_tip_z_mm',float('nan'))
                    feedback.feedback_age_sec = age
                    goal.publish_feedback(feedback)
                    self.telemetry.publish(String(data=json.dumps(dict(row,latest_feedback=latest))))
                    last_feedback = now
                if elapsed >= plan.profile.duration+p['final_settle_sec'] and reached and latest:
                    if abs(latest['measured_flange_z_mm']-plan.target_pose(plan.profile.duration)[2]) <= .5:
                        break
                next_tick += period
                if next_tick < time.monotonic():
                    next_tick = time.monotonic()
                self.cancelled.wait(max(0.,next_tick-time.monotonic()))
            result.success = True
            result.message = ('Model-only simulation completed; no physical height verified'
                              if p['backend']=='mock' else
                              'Command/profile sequence completed; gripper gap and physical tip height unverified; '
                              'inspect external measurements')
        except Cancelled as exc:
            result.message = str(exc)
        except Exception as exc:
            result.message = str(exc)
            self.get_logger().error(result.message)
        finally:
            self.stop_arm()
            self.stop_worker.set()
            if worker is not None:
                worker.join(timeout=20.)
            worker_alive = worker is not None and worker.is_alive()
            if worker_alive:
                result.success = False
                result.message += '; arm-pose worker still active: do not start another test'
            if shared.get('error'):
                result.success = False
                result.message += '; feedback worker: '+shared['error']
            errors = [abs(row['tip_error_mm']) for row in feedback_rows]
            if errors:
                result.max_estimated_tip_error_mm = max(errors)
            result.feedback_samples = len(feedback_rows)
            result.feedback_source = {'mock':'mock_model_only','virtual':'virtual_gripper_model_and_arm_pose',
                                      'real':'gripper_model_and_arm_pose'}[p['backend']]
            metadata.update(success=result.success,message=result.message,
                            feedback_source=result.feedback_source,feedback_samples=result.feedback_samples,
                            max_estimated_tip_error_mm=max(errors) if errors else None)
            try:
                save_report(directory,plan_rows,feedback_rows,metadata)
                result.report_directory = str(directory)
            except Exception as exc:
                result.success = False
                result.message += f'; report save failed: {exc}'
            self.summary.update(stage='complete' if result.success else 'stopped',
                                last_report=result.report_directory,message=result.message)
            self.get_logger().info(f'{result.message}; report: {result.report_directory}')
            with self.lock:
                self.busy = worker_alive  # refuse subsequent goals if pose query is still running
        if result.success:
            goal.succeed()
        elif goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        return result


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    node = executor = None
    try:
        node = TcpTestNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        # Bounded waits let Python SIGINT run even when the DDS graph is idle.
        while node.context.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutting_down.set()
            node.cancelled.set()
            if node.busy:
                node.stop_arm()
            deadline = time.monotonic()+25.
            while executor is not None and node.busy and node.context.ok() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.1)
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
