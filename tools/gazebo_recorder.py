#!/usr/bin/env python3
"""Record native Gazebo RGB, at fixed playback FPS using camera simulation timestamps."""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import shutil
import sys
import threading
from types import SimpleNamespace
import time

import cv2

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'janggi/realtime_ui'))
from ros_source import image_to_bgr, ros_image_qos


class CameraTimeline:
    """Hold the preceding frame only when the sensor timestamp has missing slots."""
    def __init__(self,fps,duration):
        if not all(math.isfinite(x) and x>0 for x in (fps,duration)):
            raise ValueError('FPS와 녹화 길이는 유한한 양수여야 합니다')
        self.fps=fps;self.target=round(fps*duration);self.index=0
        self.first_stamp=None;self.previous=None;self.previous_stamp=None;self.previous_sequence=None

    def push(self,stamp,sequence,frame):
        if self.first_stamp is None:
            self.first_stamp=stamp
        if self.previous_stamp is not None:
            if stamp<self.previous_stamp:raise RuntimeError('Gazebo 카메라 시계가 되돌아갔습니다. 현재 파일로 녹화를 종료합니다.')
            if stamp-self.previous_stamp>2.:raise RuntimeError('카메라 타임스탬프가 2초 넘게 건너뛰었습니다. 녹화를 종료합니다.')
        until=min(self.target,int(math.floor((stamp-self.first_stamp)*self.fps+1e-6)))
        while self.index<until and self.previous is not None:
            yield self.index,self.previous_stamp,self.previous_sequence,self.previous
            self.index+=1
        self.previous=frame;self.previous_stamp=stamp;self.previous_sequence=sequence


def record(args):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    context=Context();rclpy.init(context=context)
    node=rclpy.create_node('janggi_gazebo_recorder',context=context)
    frames=queue.Queue(maxsize=16);stats={'received_frames':0,'queue_dropped_frames':0};info={};errors=[]
    def receive(message):
        try:
            frame=image_to_bgr(message)
            stats['received_frames']+=1
            item=(message.header.stamp.sec+message.header.stamp.nanosec/1e9,stats['received_frames'],frame)
            try:frames.put_nowait(item)
            except queue.Full:
                try:frames.get_nowait()
                except queue.Empty:pass
                stats['queue_dropped_frames']+=1;frames.put_nowait(item)
        except Exception as exc:errors.append(str(exc))
    def camera_info(message):
        info.update(width=message.width,height=message.height,frame_id=message.header.frame_id,
                    distortion_model=message.distortion_model,k=list(message.k),d=list(message.d),p=list(message.p))
    native_node=None
    if args.transport=='gazebo':
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
        native_node=Node()
        def receive_native(message):
            try:
                enum=message.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type
                name=enum.values_by_number[message.pixel_format_type].name
                encoding={'RGB_INT8':'rgb8','RGBA_INT8':'rgba8','BGR_INT8':'bgr8','BGRA_INT8':'bgra8','L_INT8':'mono8'}[name]
                stamp=message.header.stamp
                receive(SimpleNamespace(width=message.width,height=message.height,step=message.step,data=message.data,encoding=encoding,
                        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp.sec,nanosec=stamp.nsec))))
            except Exception as exc:errors.append(str(exc))
        native_info_topic=args.native_topic.rsplit('/',1)[0]+'/camera_info'
        def receive_native_info(message):
            frame_id=next((entry.value[0] for entry in message.header.data if entry.key=='frame_id' and entry.value),'')
            info.update(width=message.width,height=message.height,frame_id=frame_id,distortion_model='plumb_bob',
                        k=list(message.intrinsics.k),d=list(message.distortion.k),p=list(message.projection.p))
        native_node.subscribe(GzCameraInfo,native_info_topic,receive_native_info)
        if not native_node.subscribe(GzImage,args.native_topic,receive_native):
            raise RuntimeError('Gazebo 원본 토픽 구독에 실패했습니다')
    else:
        node.create_subscription(Image,args.topic,receive,ros_image_qos())
    if args.transport=='ros':
        node.create_subscription(CameraInfo,'/janggi_sim/d455f/color/camera_info',camera_info,qos_profile_sensor_data)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    topic=args.native_topic if args.transport=='gazebo' else args.topic
    timeline=CameraTimeline(args.fps,args.duration)
    folder=None;video=None;timestamps=None;written=0;duplicates=0;used=set();last_sequence=None
    started=None;failure=None
    try:
        print(f'[녹화] 카메라 대기: {topic}',flush=True)
        try:item=frames.get(timeout=15.)
        except queue.Empty:raise RuntimeError('Gazebo RGB 영상이 없습니다. sim_gazebo.launch.py와 선택한 수신 경로를 확인하세요.')
        if errors:raise RuntimeError(errors[0])
        frame=item[2];height,width=frame.shape[:2]
        folder=args.output_root/f'gazebo_{datetime.now().astimezone():%Y%m%d_%H%M%S_%f}'
        folder.mkdir(parents=True,exist_ok=False)
        video=cv2.VideoWriter(str(folder/'color.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,(width,height))
        if not video.isOpened():raise RuntimeError('MP4 인코더를 열 수 없습니다')
        timestamps=(folder/'frames.csv').open('w',newline='')
        rows=csv.writer(timestamps);rows.writerow(['video_frame','video_time_s','camera_stamp_s','source_sequence','duplicate'])
        started=time.monotonic();last_progress=started
        (folder/'metadata.json').write_text(json.dumps(dict(status='recording',topic=topic,transport=args.transport,fps=args.fps,requested_sim_duration_s=args.duration),indent=2)+'\n')
        print(f'[녹화] 시작: {folder}/color.mp4 ({width}x{height}, {args.fps:g} FPS, 카메라 시간 {args.duration:g}초)',flush=True)
        while written<timeline.target:
            if errors:raise RuntimeError(errors[0])
            for index,stamp,sequence,image in timeline.push(*item):
                if image.shape[:2]!=(height,width):raise RuntimeError('녹화 중 카메라 해상도가 변경됐습니다')
                duplicate=sequence==last_sequence
                video.write(image);rows.writerow([index,f'{index/args.fps:.9f}',f'{stamp:.9f}',sequence,int(duplicate)])
                written+=1;duplicates+=int(duplicate);used.add(sequence);last_sequence=sequence
            if time.monotonic()-last_progress>=5.:
                print(f'[녹화] {written/args.fps:.1f}/{args.duration:g}초 · {written}프레임 · 중복 {duplicates}',flush=True)
                timestamps.flush();last_progress=time.monotonic()
            if written>=timeline.target:break
            try:item=frames.get(timeout=10.)
            except queue.Empty:raise RuntimeError('카메라 영상이 10초간 중단됐습니다. 현재 파일로 녹화를 종료합니다.')
    except KeyboardInterrupt:
        failure='사용자가 녹화를 중단했습니다'
    except Exception as exc:
        failure=str(exc)
    finally:
        if video is not None:video.release()
        if timestamps is not None:timestamps.close()
        if native_node is not None:
            native_node.unsubscribe(args.native_topic);native_node.unsubscribe(native_info_topic)
        executor.shutdown(timeout_sec=2.);thread.join(timeout=2.);node.destroy_node();context.shutdown()
        if folder is not None:
            calibration=ROOT/'src/janggi/robot_game/config/gazebo_calibration.yaml'
            if calibration.is_file():shutil.copy2(calibration,folder/'calibration.yaml')
            metadata=dict(status='complete' if failure is None and written>=timeline.target else 'interrupted',
                error=failure,topic=topic,transport=args.transport,fps=args.fps,requested_sim_duration_s=args.duration,
                video_frames=written,video_duration_s=written/args.fps,wall_duration_s=time.monotonic()-started if started else 0.,
                unique_source_frames_used=len(used),duplicate_frames=duplicates,**stats,camera_info=info,
                note='Original rendered RGB, no detection overlay. Timeline uses camera simulation timestamps. Missing slots hold the preceding frame and are marked in frames.csv.')
            (folder/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
            print('[녹화] 저장: '+str(folder/'color.mp4'),flush=True)
            print(json.dumps(metadata,ensure_ascii=False),flush=True)
    if failure:raise RuntimeError(failure)
    return folder


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=60.,help='카메라 시뮬레이션 시간 기준 녹화 길이(초)')
    parser.add_argument('--fps',type=float,default=30.)
    parser.add_argument('--transport',choices=('gazebo','ros'),default='gazebo',help='기본: Gazebo 원본 직접 구독 (ROS 브리지 우회)')
    parser.add_argument('--native-topic',default='/janggi_sim/d455f/image')
    parser.add_argument('--topic',default='/janggi_sim/d455f/color/image_raw')
    parser.add_argument('--output-root',type=Path,default=ROOT/'tools/captures')
    args=parser.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (args.fps,args.duration)):parser.error('FPS와 녹화 길이는 양수여야 합니다')
    try:record(args)
    except RuntimeError as exc:parser.exit(1,f'[녹화 오류] {exc}\n')


if __name__=='__main__':main()
