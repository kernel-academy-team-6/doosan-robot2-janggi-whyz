#!/usr/bin/env python3
"""Physical scene with dynamic pieces and force-driven robot joints."""
import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE),str(HERE.parent),str(HERE.parents[2]/'janggi')]
from simulation_calibration import ensure_simulation_calibration
from action.board import BoardGeometry
from piece_detection.world import CameraToBase
from engine import fen_to_board
from gz_scene import GzScene
from piece_visuals import piece_height_m
from gz.msgs10.pose_v_pb2 import Pose_V

START_FEN='rnba1abnr/4k4/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/4K4/RNBA1ABNR w - - 0 1'


class RenderedWorld:
    """Scene bodies only. Recognition never reads this object."""
    def __init__(self,calibration):
        self.geom=BoardGeometry.from_calibration(calibration)
        self.cam=CameraToBase.from_calibration(calibration)
        board,_=fen_to_board(START_FEN,'red')
        self.pieces=[dict(side=s,kind=k,pos=self.geom.board_to_base(*cell),state='board')
                     for cell,(s,k) in board.items()]
        from collections import Counter
        instances=Counter()
        for piece in self.pieces:
            label=f"{piece['side']}_{piece['kind']}"
            piece['texture_key']=f'{label}_{instances[label]}'
            instances[label]+=1
        self.lock=threading.RLock()

    def update_poses(self,message):
        # Read-only observation of actual Gazebo physics state.
        with self.lock:
            for pose in message.pose:
                if not pose.name.startswith('piece_'):continue
                try:index=int(pose.name[6:]);piece=self.pieces[index]
                except (ValueError,IndexError):continue
                center=np.array([pose.position.x,pose.position.y,pose.position.z])*1000.
                piece['pos']=center-self.geom.normal()*(piece_height_m(piece['kind'])*500.)

    def grip(self, command):
        raise RuntimeError('Virtual attachment is disabled; only physical contact can move pieces')

    def move_piece(self, source, target):
        raise RuntimeError('Direct piece movement is disabled; use physical forces/contact')


def focus_existing_board(refresh=False):
    calibration=ensure_simulation_calibration()
    scene=GzScene(RenderedWorld(calibration),calibration=calibration)
    try:
        scene._refresh_existing_models()
        names=scene._existing_models
        print(f"[scene] board={'janggi_board' in names}, table={'janggi_table' in names}, pieces={sum(name.startswith('piece_') for name in names)}",flush=True)
        if 'janggi_board' not in names:
            raise RuntimeError('현재 Gazebo 서버에 janggi_board가 없습니다. 장면 생성 프로세스의 오류 로그를 확인하세요.')
        if refresh:
            count=scene.refresh_board_models()
            print(f'[scene] {count}개 시각 모델을 현재 위치 그대로 다시 등록했습니다.',flush=True)
        if not scene.focus_board():
            raise RuntimeError('GUI 시점 이동 서비스가 응답하지 않습니다. Gazebo GUI가 열려 있는지 확인하세요.')
        print('[scene] 장기판으로 시점을 이동했습니다. 판·말·로봇 위치는 바꾸지 않았습니다.',flush=True)
    finally:
        scene.stop()


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    
    # Piece motion is owned exclusively by Gazebo physics.
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ns',default='dsr01')
    parser.add_argument('--wait-for-gui',action='store_true',help='GUI 초기 상태 수신 완료 후 모델 생성')
    parser.add_argument('--refresh-board',action='store_true',help='사용 불가: 물리 모드에서 실행 중 물체 재생성 금지')
    parser.add_argument('--focus-board',action='store_true',help='장면을 다시 만들지 않고 GUI 시점만 장기판으로 이동')
    parser.add_argument('--move-piece',nargs=2,metavar=('X,Y','X,Y'))
    args=parser.parse_args()
    if args.focus_board or args.refresh_board:
        focus_existing_board(refresh=args.refresh_board)
        return
    rclpy.init();node=rclpy.create_node('janggi_rendered_scene')
    scene=None;executor=None
    try:
        if args.move_piece:
            raise RuntimeError('Direct piece movement is disabled; use physical forces/contact')
        calibration=ensure_simulation_calibration();world=RenderedWorld(calibration)
        scene=GzScene(world,calibration=calibration)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            ok,_=scene.node.request('/world/empty/scene/info',Empty(),Empty,Scene,200)
            if ok:break
            time.sleep(.2)
        else:raise RuntimeError('Gazebo world did not become ready')
        if args.wait_for_gui:
            node.get_logger().info('Waiting for Gazebo GUI initial state before creating scene bodies')
            scene.wait_for_gui()
        scene.start(robot_ns=args.ns)
        scene.node.subscribe(Pose_V,'/world/empty/pose/info',world.update_poses)
        executor=SingleThreadedExecutor();executor.add_node(node)
        node.get_logger().info('Rendered scene ready. No automatic turns. Use realtime_ui/run.sh --source gazebo --model-mode hybrid --robot')
        executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if scene:scene.stop()
        if executor:executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
