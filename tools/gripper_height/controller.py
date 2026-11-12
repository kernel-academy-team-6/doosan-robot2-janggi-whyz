"""Operator-triggered raw flange motions. No camera/board/TCP-height correction."""
from dataclasses import replace
import math
import sys
import threading
from pathlib import Path

from .model import number

WS = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(WS/'src/janggi'), str(WS/'src/janggi_pick_place2')]


class MockController:
    def __init__(self, **kwargs):
        self.pose=[400.,0.,250.,0.,180.,0.]; self.command=None
        self.stopped=threading.Event()
    def check(self):
        if self.stopped.is_set():raise RuntimeError('사용자가 정지했습니다.')
    def read(self):self.check();return list(self.pose)
    def move_xy(self,x,y):self.check();self.pose[:2]=[number(x,'X'),number(y,'Y')];self.pose[3:]=[0.,180.,0.];return self.read()
    def jog(self,delta):self.check();self.pose[2]+=number(delta,'Z 이동량');return self.read()
    def set_gripper(self,command):
        self.check()
        if not isinstance(command,int) or not 0<=command<=700:raise ValueError('명령은 0~700 정수')
        self.command=command
    def home(self):self.check();self.pose=[400.,0.,250.,0.,180.,0.];return self.read()
    def stop(self):self.stopped.set()
    def close(self):pass


class RosController:
    def __init__(self, *, backend='real', namespace='dsr01', velocity=10., acceleration=20.,
                 angular_velocity=10., angular_acceleration=20.):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from dsr_msgs2.srv import GetRobotSystem
        from action.pick_place2 import load_pick_place_settings
        from janggi_pick_place2.runtime import create_clients
        if backend not in ('real','virtual'):raise ValueError('ROS backend must be real or virtual')
        if not namespace.strip('/'):raise ValueError('로봇 네임스페이스를 입력하세요.')
        self.stopped=threading.Event();self._rclpy=rclpy
        settings,parameters=load_pick_place_settings()
        self.settings=replace(settings,joint6_lock_enabled=False,
            linear_velocity_mm_s=number(velocity,'속도'),linear_acceleration_mm_s2=number(acceleration,'가속도'),
            angular_velocity_deg_s=number(angular_velocity,'회전 속도'),
            angular_acceleration_deg_s2=number(angular_acceleration,'회전 가속도'),
            joint_velocity_deg_s=number(angular_velocity,'홈 속도'),
            joint_acceleration_deg_s2=number(angular_acceleration,'홈 가속도')).validate()
        self.context=Context();rclpy.init(context=self.context,signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('gripper_height_calibration',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.arm=None;self.hand=None
        try:
            self.arm,self.hand=create_clients(self.node,'/'+namespace.strip('/'),self.settings,parameters,backend)
            status=self.arm.io.call(GetRobotSystem,'/system/get_robot_system',GetRobotSystem.Request(),self.check)
            if status.robot_system!=(0 if backend=='real' else 1):
                raise RuntimeError('선택한 backend와 연결된 로봇의 real/virtual 모드가 다릅니다.')
            self.arm.check_tcp(self.check);self.hand.prepare(self.check)
            self.read()
        except BaseException:
            self.close();raise

    def check(self):
        if self.stopped.is_set():raise RuntimeError('사용자가 정지했습니다.')

    def read(self):
        self.check();return self.arm.current_pose(self.check)

    def _workspace(self,pose):
        if len(pose)!=6 or not all(math.isfinite(v) for v in pose):raise ValueError('플랜지 좌표가 잘못되었습니다.')
        for value,lo,hi in zip(pose[:3],self.settings.workspace_min_mm,self.settings.workspace_max_mm):
            if not lo<=value<=hi:raise ValueError('목표가 설정된 작업 영역 밖입니다.')

    def move_xy(self,x,y):
        from dsr_msgs2.srv import MoveLine
        from janggi_pick_place2.geometry import orientation_error_deg
        self.arm.check_tcp(self.check)
        current=self.read()
        target=[number(x,'X'),number(y,'Y'),current[2],0.,180.,0.]
        self._workspace(target)
        aligned=[*current[:3],0.,180.,0.]
        if orientation_error_deg(current[3:],aligned[3:])>.1:
            # Rotation only, at current flange XYZ. No 160/222mm offsets here.
            solution=self.arm._solution(self.check)
            joints=self.arm._inverse(aligned,solution,self.check)
            self.arm._check_joint6(joints[5])
            request=MoveLine.Request();request.pos=aligned
            request.vel=[self.settings.linear_velocity_mm_s,self.settings.angular_velocity_deg_s]
            request.acc=[self.settings.linear_acceleration_mm_s2,self.settings.angular_acceleration_deg_s2]
            request.time=0.;request.ref=0;request.mode=0;request.sync_type=1
            self.arm.io.call(MoveLine,'/motion/move_line',request,self.check)
            def reached():
                actual=self.read()
                return math.dist(actual[:3],aligned[:3])<=.5 and orientation_error_deg(actual[3:],aligned[3:])<=.1
            self.arm._wait(reached,self.check)
        # Main MoveL checks the continuous J6 winding for the translational path.
        if math.dist(self.read()[:2],target[:2])>.01:self.arm.move(target,self.check)
        return self.read()

    def jog(self,delta):
        from janggi_pick_place2.geometry import orientation_error_deg
        self.arm.check_tcp(self.check)
        pose=self.read()
        if orientation_error_deg(pose[3:],[0.,180.,0.])>.1:
            raise RuntimeError('먼저 XY 이동 버튼으로 XZ 개폐 자세를 맞추세요.')
        pose[2]+=number(delta,'Z 이동량');self._workspace(pose)
        self.arm.move(pose,self.check)
        return self.read()

    def set_gripper(self,command):self.hand.set_position(command,self.check)
    def home(self):self.arm.home(self.check);return self.read()
    def stop(self):
        self.stopped.set()
        if self.arm is not None:self.arm.stop()
    def close(self):
        if self.hand is not None:self.hand.close_session()
        self.executor.shutdown(timeout_sec=2.)
        if self.thread is not threading.current_thread():self.thread.join(timeout=2.)
        self.node.destroy_node()
        if self.context.ok():self.context.shutdown()
