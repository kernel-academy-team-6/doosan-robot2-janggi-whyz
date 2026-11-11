"""Live Gazebo RGB subscriber. No generated frames or detection metadata."""
import threading
from collections import deque
from types import SimpleNamespace
import time
import numpy as np


def image_to_bgr(message):
    channels = {'rgb8':3, 'bgr8':3, 'rgba8':4, 'bgra8':4, 'mono8':1}
    if message.encoding not in channels:
        raise ValueError(f'지원하지 않는 ROS 영상 형식: {message.encoding}')
    count = channels[message.encoding]
    row_bytes = message.width*count
    if message.step < row_bytes or len(message.data) < message.step*message.height:
        raise ValueError('ROS Image의 step/data 길이가 올바르지 않습니다')
    rows = np.frombuffer(message.data,dtype=np.uint8,count=message.step*message.height).reshape(message.height,message.step)
    image = rows[:,:row_bytes].reshape(message.height,message.width,count)
    if message.encoding=='mono8':return np.repeat(image,3,axis=2).copy()
    if message.encoding.startswith('rgb'):return image[:,:,:3][:,:,::-1].copy()
    return image[:,:,:3].copy()


def gazebo_image_to_bgr(message):
    enum=message.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type
    name=enum.values_by_number[message.pixel_format_type].name
    encodings={'RGB_INT8':'rgb8','RGBA_INT8':'rgba8','BGR_INT8':'bgr8','BGRA_INT8':'bgra8','L_INT8':'mono8'}
    if name not in encodings:raise ValueError(f'지원하지 않는 Gazebo 영상 형식: {name}')
    return image_to_bgr(SimpleNamespace(width=message.width,height=message.height,step=message.step,
                                       data=message.data,encoding=encodings[name]))


def ros_image_qos():
    # Large uncompressed frames lose complete samples on this Best Effort path.
    # Keep only the freshest frame while allowing DDS to retransmit fragments.
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)


class GazeboSource:
    def __init__(self, topic='/janggi_sim/d455f/color/image_raw', timeout=15., namespace='dsr01', transport='gazebo', native_topic='/janggi_sim/d455f/image'):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        if transport not in ('gazebo','ros'):raise ValueError('Unsupported Gazebo image transport')
        self.transport=transport;self.native_node=None
        self.context=Context();rclpy.init(context=self.context)
        self.node=rclpy.create_node('janggi_gazebo_rgb',context=self.context)
        self._string_type=String
        self.condition=threading.Condition();self.frame=None;self.sequence=0;self.consumed=0;self.skipped_frames=0;self.receive_times=deque(maxlen=120);self.decode_ms=deque(maxlen=120)
        self.stopped=False;self.error=None;self.source_frame=None;self.source_time=None;self.robot_description=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String,f"/{namespace.strip(chr(47))}/robot_description",self._description,qos)
        self.original_fps=30.;self.last_received=0.
        self.topic=native_topic if transport=='gazebo' else topic
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        try:
            if transport=='ros':
                self.node.create_subscription(Image,topic,self._receive,ros_image_qos())
            else:
                try:
                    from gz.transport13 import Node
                    from gz.msgs10.image_pb2 import Image as GzImage
                except ImportError:
                    import sys
                    sys.path.append('/usr/lib/python3/dist-packages')
                    from gz.transport13 import Node
                    from gz.msgs10.image_pb2 import Image as GzImage
                self.native_node=Node()
                if not self.native_node.subscribe(GzImage,native_topic,self._receive_native):
                    raise RuntimeError(f'Gazebo 원본 토픽 구독 실패: {native_topic}')
            with self.condition:
                ready=self.condition.wait_for(lambda:self.frame is not None or self.error,timeout)
            if not ready or self.error:
                raise RuntimeError(self.error or f'{self.topic} 영상을 받지 못했습니다. Gazebo 카메라와 선택한 수신 경로를 확인하세요.')
        except BaseException:
            self.close()
            raise

    def _accept_frame(self,frame,stamp,started):
        with self.condition:
            if self.stopped:return
            self.frame=frame;self.sequence+=1;self.last_received=time.monotonic()
            self.receive_times.append(self.last_received);self.decode_ms.append((self.last_received-started)*1000.)
            self.stamp=stamp;self.condition.notify_all()

    def _receive_native(self,message):
        started=time.monotonic()
        try:
            frame=gazebo_image_to_bgr(message)
            self._accept_frame(frame,message.header.stamp.sec+message.header.stamp.nsec/1e9,started)
        except (ValueError,KeyError) as exc:
            with self.condition:self.error=str(exc);self.condition.notify_all()

    def _receive(self, message):
        started=time.monotonic()
        try:
            frame=image_to_bgr(message)
            self._accept_frame(frame,message.header.stamp.sec+message.header.stamp.nanosec/1e9,started)
        except ValueError as exc:
            with self.condition:self.error=str(exc);self.condition.notify_all()

    def statistics(self):
        with self.condition:
            stamps=list(self.receive_times)
            rate=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0.
            return dict(transport=self.transport,topic=self.topic,ros_image_qos='reliable/keep_last_1' if self.transport=='ros' else None,received_frames=self.sequence,
                        input_fps=rate,skipped_latest_frames=self.skipped_frames,
                        conversion_ms_mean=sum(self.decode_ms)/len(self.decode_ms) if self.decode_ms else None)

    def read(self):
        with self.condition:
            self.condition.wait_for(lambda:self.sequence>self.consumed or self.stopped or self.error,1.)
            if self.error:raise RuntimeError(self.error)
            if self.stopped:return None
            if self.sequence==self.consumed:
                if time.monotonic()-self.last_received>5.:
                    raise RuntimeError('Gazebo 카메라 영상이 5초 이상 중단됐습니다 (일시정지/브리지 확인)')
                return None
            self.skipped_frames+=max(0,self.sequence-self.consumed-1)
            self.consumed=self.sequence;self.source_frame=self.sequence;self.source_time=self.stamp
            return self.frame.copy()

    def _description(self,message):
        with self.condition:
            self.robot_description=message.data;self.condition.notify_all()

    def require_virtual_driver(self):
        with self.condition:
            self.condition.wait_for(lambda:self.robot_description is not None,5.)
            require_virtual_description(self.robot_description)

    def move_piece(self,source,target):
        raise RuntimeError('물리 모드에서는 말 좌표를 직접 변경하지 않습니다. 접촉·힘으로 이동하세요.')

    def interrupt(self):
        with self.condition:self.stopped=True;self.condition.notify_all()

    def close(self):
        if getattr(self,'_closed',False):return
        self._closed=True;self.interrupt()
        if self.native_node is not None:self.native_node.unsubscribe(self.topic)
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.);self.node.destroy_node();self.context.shutdown()


def require_virtual_description(description):
    import xml.etree.ElementTree as ET
    if not description:raise RuntimeError('가상 드라이버의 robot_description을 받지 못했습니다')
    try:
        root=ET.fromstring(description)
        params={p.get('name'):(p.text or '').strip() for p in root.findall('./ros2_control/hardware/param')}
    except ET.ParseError as exc:raise RuntimeError('드라이버 URDF를 읽을 수 없습니다') from exc
    if params.get('mode')!='virtual' or params.get('host') not in ('127.0.0.1','localhost','::1'):
        raise RuntimeError('Gazebo 입력의 로봇 대국은 localhost의 mode:=virtual 드라이버에만 연결합니다')
