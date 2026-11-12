#!/usr/bin/env python3
"""장기말 실시간 인식 + 게임 UI.

왼쪽: 카메라 영상 위 인식 결과 (박스·말 이름·신뢰도·장기판 격자·할당된 교차점)
오른쪽: 장기판
    실시간 모드 — 인식된 판 상태(최근 프레임 다수결로 안정화)를 그대로 보여준다
    게임 모드   — 인식된 판에서 시작해 말을 클릭(또는 드래그)해서 둔다. 둘 수 있는 칸 표시,
                  차례·장군·외통 판정, 되돌리기, 한수쉼, 실제 판에서 사람이 둔 수 읽기

    ./run.sh                          # RealSense
    ./run.sh --source 0               # 일반 웹캠 / 동영상 파일 / 이미지 파일 경로
    ./run.sh --conf 0.4 --piece-height 15
    ./run.sh --robot                  # 사람 ↔ 로봇 대국 (엔진 + 로봇 드라이버 필요, ../robot_game/README.md)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))          # janggi/ → piece_detection 패키지
sys.path.insert(0, str(HERE))
SRC_JANGGI = HERE.parents[1] / "src" / "janggi"
sys.path.insert(0, str(SRC_JANGGI / "robot_game"))   # session (로봇 대국)
sys.path.insert(0, str(SRC_JANGGI))                  # action (로봇 동작), engine (Fairy-Stockfish)

from piece_detection import BoardMapper, BoardStabilizer, piece_char  # noqa: E402
from piece_detection.board_mapping import DEFAULT_CALIBRATION  # noqa: E402
from piece_detection.detector import DEFAULT_MODEL, count_unique_detections, deduplicate_detections  # noqa: E402
from rules import COLS, ROWS, Game, detect_top_side, infer_move, other  # noqa: E402
from diagnostics import ObstructionMonitor, BoxSizeEstimator, box_label
from classification_async import LatestClassification
from piece_state import PieceTracker, StateStore, size_prior_metadata
from piece_motion import PieceMotionMonitor, draw_piece_regions
from count_log import CountLog  # noqa: E402
from board_region import BoardRecognitionRegion
from video_log import BoardRenderer, EvaluationVideos  # noqa: E402

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_FAMILY = "Noto Sans CJK KR"
SIDE_RGB = {"blue": (30, 110, 255), "red": (230, 40, 40)}
SIDE_HEX = {"blue": "#1e5fd9", "red": "#d62828"}
SIDE_KO = {"blue": "초", "red": "한"}
FULL_SET = {"king": 1, "cha": 2, "po": 2, "horse": 2, "sang": 2, "sa": 2, "soldier": 5}


def parse_args():
    p = argparse.ArgumentParser(description="장기말 실시간 인식 + 게임 UI")
    p.add_argument("--source", default="realsense", help="realsense | gazebo (원본 RGB 직접 수신) | 웹캠 번호 | 동영상/이미지 파일 경로")
    p.add_argument("--gazebo-transport", choices=("gazebo", "ros"), default="gazebo",
                   help="Gazebo 영상 수신 경로: 기본은 원본 직접 구독; ros는 비교/호환용")
    p.add_argument("--playback-fps", type=float, default=0.0,
                   help="동영상 재생 FPS (기본/0: 속도 제한 없음). 카메라·사진에는 적용 안 함")
    model_options = p.add_mutually_exclusive_group()
    model_options.add_argument("--model-mode", "-model-mode", choices=("piece", "full", "hybrid", "cnn"), default=None,
                               help="piece: 위치 전용, full: 기존 14종, hybrid/cnn: 위치 YOLO + 별도 분류 (생략 시 weights.pt)")
    model_options.add_argument("--model", type=Path, default=None, help="모델 파일 경로 직접 지정")
    p.add_argument("--classifier", type=Path, default=HERE.parents[1]/"models/piece_classifier_pieces2_224/best.pt",
                   help="hybrid/cnn 모드의 분류 가중치")
    p.add_argument("--classifier-backend", choices=("mobilenet", "ultralytics"), default="mobilenet")
    p.add_argument("--classifier-conf", type=float, default=.5, help="종류 분류 확률 기준; 미달이면 위치만 표시")
    p.add_argument("--classification-interval", type=float, default=0., help="정상 객체 CNN 갱신의 추가 최소 간격(초), 기본 0")
    p.add_argument("--unknown-classification-interval", type=float, default=0.,
                   help="알 객체 CNN 재분류 최소 간격(초), 기본 0: 매 처리 프레임 요청")
    p.add_argument("--classification-frames", type=int, default=4, help="정상 객체 CNN 갱신 프레임 간격; 기본 4")
    p.add_argument("--tracking-low-conf", type=float, default=.1, help="hybrid 기존 ID 추적 보조 검출 하한; 새 객체/종류 갱신에는 사용 안 함")
    p.add_argument("--history-alpha", type=float, default=.2, help="현재 관측 비중; 기본 과거 80%% + 현재 20%%")
    p.add_argument("--size-strength", type=float, default=.5, help="바닥값 없는 크기 가우시안 강도 (폭 1.2mm); 0이면 사용 안 함")
    p.add_argument("--state-dir", type=Path, default=HERE/"states", help="말별 상태·이벤트 세션의 상위 저장 폴더")
    p.add_argument("--classifier-batch", type=int, default=32)
    p.add_argument("--classifier-size", type=int, default=224, help="Ultralytics 분류 입력 크기; MobileNet은 체크포인트 설정 사용")
    p.add_argument("--device", default=None, help="cpu 또는 cuda:0; 생략 시 자동 선택")
    p.add_argument("--box-label", choices=("confidence", "size-px", "size-mm", "geometric-px", "geometric-mm"), default="confidence",
                   help="박스 표시: 신뢰도 또는 가로x세로 크기")
    p.add_argument("--show-piece-motion", action="store_true", help="말 박스 기반 국소 방해/움직임 영역 표시(실험)")
    p.add_argument("--local-conf-drop", type=float, default=.15, help="말 센서 신뢰도 하락 기준")
    p.add_argument("--local-size-change", type=float, default=.25, help="말 센서 기하평균 크기 변화 비율")
    p.add_argument("--local-move-ratio", type=float, default=.2, help="최근 0.3초의 이동 거리 / 박스 크기 기준")
    p.add_argument("--obstruction-motion", type=float, default=.03, help="방해 의심 움직임 면적 비율 (기본 0.03)")
    p.add_argument("--obstruction-window", type=float, default=2., help="검출 수 진동 확인 시간(초)")
    p.add_argument("--obstruction-settle", type=float, default=.8, help="안정 표시 전 대기 시간(초)")
    p.add_argument("--conf", type=float, default=0.6, help="검출 신뢰도 임계값 (기본 0.6, UI 슬라이더로 바꿀 수 있다)")
    p.add_argument("--full-frame", action="store_true", help="판 영역만 잘라 인식하지 않고 전체 프레임을 인식")
    p.add_argument("--roi-margin", type=int, default=30, help="판 사각형 경계 바깥 인식 여유 [원본 영상 px, 기본 30]")
    p.add_argument("--calibration", type=Path, default=None,
                   help="보정 파일 직접 지정; 생략 시 녹화 폴더의 보정/Gazebo 보정/실물 기본값 자동 선택")
    p.add_argument("--piece-height", type=float, default=5.0, help="말 높이 [mm] (시차 보정)")
    p.add_argument("--max-dist", type=float, default=0.45, help="교차점 할당 최대 거리 [칸]")
    p.add_argument("--window", type=int, default=8, help="판 상태 안정화 프레임 수")
    p.add_argument("--log-counts", action="store_true", help="인식 개수·conf 로그와 오버레이/인식판 영상을 evaluations/에 저장")
    p.add_argument("--expected-pieces", type=int, default=32, help="개수 비교 기준 (기본 32; 실제 말 개수가 고정된 구간에 사용)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    g = p.add_argument_group("로봇 대국 (--robot)")
    g.add_argument("--robot", action="store_true", help="사람 ↔ 로봇 대국 패널을 켠다")
    g.add_argument("--human-side", choices=("blue", "red"), default="blue", help="사람 진영 (먼저 둔다)")
    g.add_argument("--movetime", type=int, default=1500, help="엔진 생각 시간 [ms]")
    g.add_argument("--robot-vel", type=float, default=None, help="로봇 직선 속도 [mm/s]; 생략 시 pick_place2 YAML 설정")
    g.add_argument("--ns", default="dsr01", help="로봇 네임스페이스")
    g.add_argument("--pick-place-config", type=Path, default=SRC_JANGGI.parent/"janggi_pick_place2/config/pick_place.yaml",
                   help="janggi_pick_place2 집기·TCP·그리퍼 설정 파일")
    g.add_argument("--pick-yaw", type=float, default=None, help="집기·놓기 yaw 강제 지정 [deg]; 생략하면 홈의 그리퍼 방향 유지")
    g.add_argument("--no-motion", action="store_true", help="로봇을 움직이지 않고 엔진 수와 집을 좌표만 표시")
    g.add_argument('--adaptive-grasp', action=argparse.BooleanOptionalAction, default=None,
                   help='주변 말과의 손가락 여유를 보고 집기/놓기 방향 선택 (기본 YAML)')
    g.add_argument('--refresh-before-pick', action=argparse.BooleanOptionalAction, default=None,
                   help='집기 직전 새 영상으로 목표 위치 재확인 (기본 YAML)')
    g.add_argument('--auto-retry', action=argparse.BooleanOptionalAction, default=None,
                   help='착수 관측 실패 시 한 번만 이동/놓기 재시도 (기본 YAML)')
    args = p.parse_args()
    from source_calibration import resolve_calibration, is_simulation_calibration
    try:
        args.calibration, args.calibration_origin = resolve_calibration(args.source,args.calibration,DEFAULT_CALIBRATION)
        args.simulation_calibration = is_simulation_calibration(args.calibration)
    except (ValueError,OSError) as exc:
        p.error(str(exc))
    if (args.source == 'gazebo' or args.simulation_calibration) and args.model_mode is None and args.model is None:
        args.model_mode = 'hybrid'
    if args.model is None:
        model_names = {"piece": "weights_only_piece.pt", "full": "weights_14class.pt",
                       "hybrid": "weights_only_piece.pt", "cnn": "weights_only_piece.pt"}
        args.model = DEFAULT_MODEL.with_name(model_names[args.model_mode]) if args.model_mode else DEFAULT_MODEL
    if args.playback_fps is not None and (not np.isfinite(args.playback_fps) or args.playback_fps < 0):
        p.error("--playback-fps는 0 이상의 유한한 숫자여야 한다")
    if not 0 <= args.classifier_conf <= 1 or args.classifier_batch < 1 or not 32 <= args.classifier_size <= 1024:
        p.error("분류 신뢰도는 0~1, 배치는 양수, 입력 크기는 32~1024여야 한다")
    if (not all(np.isfinite(v) for v in (args.obstruction_motion, args.obstruction_window, args.obstruction_settle))
            or not 0 < args.obstruction_motion <= 1 or args.obstruction_window <= 0 or args.obstruction_settle < 0):
        p.error("방해 감지 비율은 0~1, 관측 창은 양수, 안정 대기는 0 이상이어야 한다")
    if (not np.isfinite(args.unknown_classification_interval) or args.unknown_classification_interval < 0):
        p.error("미분류 객체의 분류 간격은 0 이상인 유한한 숫자여야 한다")
    if (not np.isfinite(args.classification_interval) or args.classification_interval < 0
            or not 0 < args.history_alpha <= 1 or not np.isfinite(args.size_strength) or args.size_strength < 0):
        p.error("분류 간격은 0 이상, 현재 관측 비중은 0~1, 크기 가중치는 0 이상이어야 한다")
    if (not all(np.isfinite(v) for v in (args.local_conf_drop, args.local_size_change, args.local_move_ratio))
            or not 0 < args.local_conf_drop <= 1 or args.local_size_change <= 0 or args.local_move_ratio <= 0):
        p.error("말 센서 신뢰도 하락 기준은 0~1, 크기/이동 변화 비율은 양수여야 한다")
    if args.pick_yaw is not None and not np.isfinite(args.pick_yaw):
        p.error("--pick-yaw는 유한한 각도여야 한다")
    if args.robot_vel is not None and (not np.isfinite(args.robot_vel) or args.robot_vel <= 0):
        p.error("--robot-vel은 양수인 유한한 숫자여야 한다")
    if args.roi_margin < 0:
        p.error("--roi-margin은 0 이상의 정수여야 한다")
    if args.classification_frames < 1:
        p.error("분류 프레임 간격은 양수여야 한다")
    if not np.isfinite(args.tracking_low_conf) or not 0 <= args.tracking_low_conf <= 1:
        p.error("추적 보조 검출 하한은 0~1이어야 한다")
    if args.expected_pieces <= 0:
        p.error("--expected-pieces는 양수여야 한다")
    return args


# --------------------------------------------------------------------------- 영상 입력

class RealSenseSource:
    def __init__(self, w, h, fps=30):
        import pyrealsense2 as rs

        self.rs, self.size = rs, (w, h, fps)
        from realsense_exposure import ExposureControl
        self.exposure_control = ExposureControl(rs.option)
        self.pipe = None
        self._open()

    def _open(self):
        rs = self.rs
        w, h, fps = self.size
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)   # 인식에는 color만 쓴다
        profile = pipe.start(cfg)
        self.pipe = pipe
        try:
            self.exposure_control.bind(profile.get_device().first_color_sensor())
        except (RuntimeError, ValueError) as exc:
            self.exposure_control.unbind(str(exc))

    def read(self):
        try:
            if self.pipe is None:
                self._open()
            self.exposure_control.process_pending()
            f = self.pipe.wait_for_frames(1000).get_color_frame()
            return np.asanyarray(f.get_data()).copy() if f else None
        except RuntimeError:
            # USB가 잠깐 끊기면 파이프라인을 다시 연다
            self.exposure_control.unbind('카메라 재연결 대기 중')
            try:
                if self.pipe is not None:
                    self.pipe.stop()
            except RuntimeError:
                pass
            self.pipe = None
            time.sleep(0.5)
            return None

    def close(self):
        self.exposure_control.unbind()
        try:
            if self.pipe is not None:
                self.pipe.stop()
        except RuntimeError:
            pass


class CvSource:
    """웹캠 번호, 동영상 파일, 이미지 파일."""

    def __init__(self, src: str, w, h, playback_fps: float = 0.0):
        self.image = None
        self.source_frame = self.source_time = None
        self.source_loop = 0
        self.is_video = not src.isdigit()
        self.original_fps = 0.0
        self._interrupted = threading.Event()
        self.frame_interval = 0.0
        self.next_frame_at = None
        if not src.isdigit() and Path(src).suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"):
            self.image = cv2.imread(src)
            if self.image is None:
                raise RuntimeError(f"이미지를 열 수 없다: {src}")
            return
        self.cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
        if src.isdigit():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            raise RuntimeError(f"영상을 열 수 없다: {src}")
        if not src.isdigit():
            self.original_fps = self.cap.get(cv2.CAP_PROP_FPS)
            fps = playback_fps
            self.frame_interval = 1.0 / fps if fps > 0 else 0.0
            print(f"[영상] 재생 속도: {fps:g} FPS" if fps else "[영상] 재생 속도: 제한 없음")

    def read(self):
        if self.image is not None:
            time.sleep(0.03)
            return self.image.copy()
        if self.frame_interval:
            if self.next_frame_at is not None:
                delay = self.next_frame_at - time.monotonic()
                if delay > 0 and self._interrupted.wait(delay):
                    return None
            # 느린 추론 뒤에 밀린 프레임을 몰아서 재생하지 않는다.
            self.next_frame_at = time.monotonic() + self.frame_interval
        ok, f = self.cap.read()
        if not ok:  # 동영상 끝 → 처음부터
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.next_frame_at = None
            self.source_loop += 1
            return None
        if self.is_video:
            self.source_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
            self.source_time = (self.source_frame / self.original_fps
                                if np.isfinite(self.original_fps) and self.original_fps > 0 else None)
        return f

    def interrupt(self):
        self._interrupted.set()

    def close(self):
        if self.image is None:
            self.cap.release()


# --------------------------------------------------------------------------- 인식 스레드

class Worker:
    """영상 읽기 → 검출 → 교차점 할당 → 안정화를 백그라운드에서 반복한다."""

    def __init__(self, source, detector, mapper, args):
        self.source, self.detector, self.mapper, self.args = source, detector, mapper, args
        self.stab = BoardStabilizer(window=args.window)
        self.center_hist = deque(maxlen=args.window)   # 프레임별 {칸: 박스 중심 픽셀}
        self.frames = 0
        self.lock = threading.Lock()
        self.latest = None   # (frame, dets, raw_board, stable_board)
        self.fps = 0.0
        self.last_frame_t = 0.0
        self.running = True
        self.error = None
        self._performance_samples=deque(maxlen=120)
        self._performance_directory=None
        self._last_performance_write=float('-inf')
        self.use_roi = mapper is not None and not args.full_frame
        self.count_log = None
        self.video_log = None
        self.video_zoom = mapper is not None
        self.video_detail = False
        self.box_label_mode = getattr(args, 'box_label', 'confidence')
        self.obstruction = ObstructionMonitor(getattr(args, 'obstruction_window', 2.),
                                             getattr(args, 'obstruction_settle', .8),
                                             getattr(args, 'obstruction_motion', .03))
        self.obstruction_reset = threading.Event()
        self.show_piece_motion = getattr(args, 'show_piece_motion', False)
        self.piece_motion = PieceMotionMonitor(getattr(args, 'local_conf_drop', .15),
                                              getattr(args, 'local_size_change', .25),
                                              getattr(args, 'local_move_ratio', .2))
        self.local_regions = []
        self._piece_motion_enabled = self.show_piece_motion
        self.view_status = dict(state='settling', reason='영상 입력 대기 중', motion=0.)
        self.size_estimator = None
        if mapper is not None and args.calibration:
            try:
                self.size_estimator = BoxSizeEstimator.from_calibration(args.calibration, getattr(args, 'piece_height', 5.))
            except (KeyError, ValueError, OSError) as exc:
                print(f"[크기 표시] mm 추정 사용 불가: {exc}")
        self.monitor_context = None
        self.board_polygon = None
        if mapper is not None:
            grid = mapper.grid_pixels()
            self.board_polygon = np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
        self.board_region = BoardRecognitionRegion(self.board_polygon) if self.board_polygon is not None else None
        self.classifier_worker = None
        self.tracker = None
        self.state_store = None
        self.tracking_status = '프레임별 인식'
        self.last_classification = None
        self.visual_counts = (0, 0, 0)
        self.game_turn = None
        self.game_tracking_generation = None
        self.robot_motion = False
        self.expected_robot_move = None
        self._stable_gate = False
        self._visible_cells = None
        self._robot_views = deque(maxlen=12)
        if hasattr(detector, 'detect_positions'):
            def validate_move(board, src, dst):
                from rules import Position
                known = {c: p for c, p in board.items() if p[0] != 'unknown'}
                try:
                    return dst in Position(known, detect_top_side(known)).legal_moves(src)
                except (KeyError, ValueError):
                    return False
            self.tracker = PieceTracker(detector.classification_conf, getattr(args, 'history_alpha', .2),
                                        getattr(args, 'size_strength', .5), validate_move)
            self.tracker.pieces = self.piece_motion.pieces
            from datetime import datetime
            directory = getattr(args, 'state_dir', HERE/'states')/datetime.now().strftime('session_%Y%m%d_%H%M%S_%f')
            self.state_store = StateStore(directory)
            self._performance_directory=directory
            self.classifier_worker = LatestClassification(detector.classify_observations,
                                                          getattr(args, 'classification_interval', 0.),
                                                          urgent_interval=getattr(args, 'unknown_classification_interval', 0.))
            import json, hashlib
            metadata = dict(source=args.source, video_transport=getattr(source, 'transport', None), inference_device=str(detector.device), calibration_origin=getattr(args, 'calibration_origin', 'explicit'), detector=str(args.model),
                            detector_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(), classifier=str(detector.classifier.path),
                            classifier_sha256=hashlib.sha256(detector.classifier.path.read_bytes()).hexdigest(),
                            classes=detector.classifier.classes, calibration=str(args.calibration),
                            piece_height_mm=getattr(args, 'piece_height', 5.),
                            classification_frames=getattr(args, 'classification_frames', 4),
                            classification_interval=getattr(args, 'classification_interval', 0.),
                            unknown_classification_interval=getattr(args, 'unknown_classification_interval', 0.),
                            tracking_low_conf=getattr(args, 'tracking_low_conf', .1),
                            tracking_policy='bounded velocity prediction; global high/low assignment',
                            initial_state='fresh; no automatic state restore', object_model='unified-detection-v2',
                            missing_retention='session lifetime; immediate spatial attachment, semantic verification; MISS overlay 2 seconds',
                            semantic_gate='regular=healthy objects; unknown=latest preview, threshold 0.5, rate limited; game=global stability')
            metadata.update(size_strength=getattr(args, 'size_strength', .5), **size_prior_metadata())
            if args.calibration and Path(args.calibration).is_file():
                import shutil
                shutil.copyfile(args.calibration, directory/'calibration.yaml')
            (directory/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
            print(f"[말 상태] 저장: {directory}")
        if args.log_counts:
            import hashlib
            self.count_log = CountLog(HERE / "evaluations", args.expected_pieces, dict(
                source=args.source, model=str(args.model.resolve()),
                model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                calibration=str(args.calibration) if mapper else None,
                playback_fps=args.playback_fps, max_dist=args.max_dist, window=args.window,
                deduplication_iou=0.5, model_iou=detector.iou, imgsz=detector.imgsz))
            classifier = getattr(detector, "classifier", None)
            if classifier is not None:
                self.count_log.metadata.update(classifier=str(classifier.path.resolve()),
                    classifier_sha256=hashlib.sha256(classifier.path.read_bytes()).hexdigest(),
                    classifier_classes=classifier.classes, classifier_size=classifier.size,
                    classifier_backend=type(classifier).__name__,
                    classifier_threshold=detector.classification_conf,
                    confidence_semantics="conf=YOLO detection probability; classification_conf=history/size-fused probability when tracking",
                    state_directory=str(self.state_store.directory) if self.state_store else None,
                    classification_frames=getattr(args, "classification_frames", 4),
                    classification_interval=getattr(args, "classification_interval", 0.),
                    unknown_classification_interval=getattr(args, "unknown_classification_interval", 0.),
                    size_strength=getattr(args, 'size_strength', .5), **size_prior_metadata())
                self.classification_log = (self.count_log.path.parent/"classifications.jsonl").open("w", encoding="utf-8")
            else:
                self.classification_log = None
            print(f"[개수 기록] 저장 시작: {self.count_log.path}")
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        n, t0 = 0, time.time()
        try:
            if self.count_log:
                fps = getattr(self.source, "original_fps", 0)
                fps = fps if np.isfinite(fps) and fps > 0 else 30.0
                if (getattr(self.source, "is_video", False) and getattr(self.source, "image", None) is None
                        and self.args.playback_fps > 0):
                    fps = self.args.playback_fps
                self.video_log = EvaluationVideos(self.count_log.path, fps)
                video_glyphs = GlyphCache(FONT_PATH, 16)
                board_renderer = BoardRenderer(FONT_PATH)
            while self.running:
                frame_started=time.perf_counter()
                frame = self.source.read()
                received_at=time.perf_counter()
                if frame is None:
                    continue
                roi = None
                region = self.board_region if self.use_roi else None
                margin = getattr(self.args, 'roi_margin', 30)
                detector_frame = frame
                if region is not None:
                    detector_frame, roi = region.prepare(frame, margin)
                positions_only = self.tracker is not None
                detect = self.detector.detect_positions if positions_only else self.detector.detect
                if roi is not None and (roi[2] <= roi[0] or roi[3] <= roi[1]):
                    dets, tracking_candidates = [], []
                elif positions_only and hasattr(self.detector, 'detect_tracking_positions'):
                    tracking_candidates = self.detector.detect_tracking_positions(
                        detector_frame, conf=self.args.conf, roi=roi, low_conf=getattr(self.args, 'tracking_low_conf', .1))
                    if region is not None:
                        tracking_candidates = region.filter(tracking_candidates, margin)
                    dets = [d for d in tracking_candidates if d.conf >= self.args.conf]
                else:
                    dets = detect(detector_frame, conf=self.args.conf, roi=roi)
                    tracking_candidates = None
                if region is not None:
                    dets = region.filter(dets, margin)
                detected_at=time.perf_counter()
                count_dets = list(dets)
                raw = self.mapper.assign(dets, self.args.max_dist) if self.mapper else {}
                unique_dets = (dets if positions_only and getattr(self.detector, 'positions_deduplicated', False)
                               else deduplicate_detections(dets))
                context = (getattr(self.source, 'source_loop', None), self.args.conf, self.use_roi,
                           margin if region is not None else None)
                if self.obstruction_reset.is_set() or context != self.monitor_context:
                    self.obstruction.reset()
                    self.obstruction_reset.clear()
                    with self.lock:
                        if context != self.monitor_context and self.monitor_context is not None:
                            if self.tracker is not None:
                                self.classifier_worker.invalidate()
                                self.tracker.reset('영상 반복 또는 인식 설정 변경')
                                self.expected_robot_move = None
                            self.piece_motion.reset()
                        else:
                            self.piece_motion.reset_baselines()
                    self.monitor_context = context
                view_status = self.obstruction.update(frame, len(unique_dets), len(raw) if self.mapper else None,
                                                      time.monotonic(), self.board_polygon)
                enabled = self.show_piece_motion or self.tracker is not None
                with self.lock:
                    if enabled != self._piece_motion_enabled:
                        if self.tracker is None:
                            self.piece_motion.reset()
                        self._piece_motion_enabled = enabled
                    # 통합 객체는 표시 체크박스와 독립적으로 유지한다.
                    local_regions = self.piece_motion.update(
                        tracking_candidates if tracking_candidates is not None else unique_dets,
                        time.monotonic(), confidence_threshold=self.args.conf) if enabled else []
                    if tracking_candidates is not None:
                        # 약한 관측은 위치 추적/화면 표시만 수행하고 게임판/기존 검출 통계에는 넣지 않는다.
                        dets.extend(d for d in tracking_candidates if d.conf < self.args.conf and d.piece_id is not None)
                if self.tracker is None:
                    # 분류기가 없는 표시 모드도 수명 이벤트 큐가 계속 쌓이지 않도록 비운다.
                    self.piece_motion.drain_lifecycle()
                tracked_board = None
                if self.tracker is not None:
                    with self.lock:
                        tracked_board = self._update_tracks(frame, dets, raw, view_status)
                tracked_at=time.perf_counter()
                if getattr(self, "classification_log", None) is not None:
                    import json
                    self.classification_log.write(json.dumps(dict(inference_frame=self.frames, frame=getattr(self.source, "source_frame", None),
                        source_time=getattr(self.source, "source_time", None), loop=getattr(self.source, "source_loop", None),
                        detections=[dict(box=d.box, label=d.name, detection_conf=d.conf,
                                         classification_conf=d.classification_conf, classification_preview=d.classification_preview,
                                         piece_id=d.piece_id, tracking_only=d.tracking_only,
                                         geometric_size_mm=d.geometric_size_mm) for d in dets],
                        piece_motion_regions=local_regions if enabled else None))+"\n")
                with self.lock:
                    stable = tracked_board if tracked_board is not None else self.stab.update(raw) if self.mapper else {}
                    self.center_hist.append({d.board: d.center for d in dets if d.board is not None})
                    self.latest = (frame, dets, raw, stable)
                    self._robot_views.append(dict(stamp=frame_started, objects=[
                        dict(piece_id=d.piece_id, cell=d.board, piece=(d.side,d.kind), pixel=d.center,
                             confidence=d.conf) for d in dets]))
                    self.view_status = view_status
                    self.local_regions = local_regions
                    self.frames += 1
                self.last_frame_t = time.time()
                if self.count_log:
                    crop = board_crop_box(self.mapper, frame.shape[1], frame.shape[0]) if self.video_zoom and self.mapper else None
                    overlay = draw_overlay(frame, dets, self.mapper, video_glyphs, crop, self.video_detail,
                                           self.box_label_mode, self.size_estimator, local_regions if self.show_piece_motion else None)
                    self.video_log.write(overlay, board_renderer.render(stable))
                    self.count_log.record(len(count_dets), len(unique_dets),
                                          len(raw) if self.mapper else None, len(stable) if self.mapper else None,
                                          self.args.conf, roi is not None,
                                          getattr(self.source, "source_frame", None),
                                          getattr(self.source, "source_time", None),
                                          getattr(self.source, "source_loop", None),
                                          confidences=[d.conf for d in count_dets],
                                          deduplicated_confidences=[d.conf for d in unique_dets])
                finished_at=time.perf_counter()
                self._performance_samples.append([(received_at-frame_started)*1000.,(detected_at-received_at)*1000.,
                    (tracked_at-detected_at)*1000.,(finished_at-tracked_at)*1000.,(finished_at-frame_started)*1000.])
                if self._performance_directory is not None and finished_at-self._last_performance_write>=5.:
                    import json
                    means=np.mean(np.asarray(self._performance_samples),axis=0)
                    performance=dict(wall_time=time.time(),processed_frames=self.frames,processing_fps=self.fps,
                        inference_device=str(self.detector.device),samples=len(self._performance_samples),
                        stages_ms=dict(zip(('input_wait','position_yolo','tracking_fusion_state','output_logging','frame_total'),map(float,means))),
                        source=self.source.statistics() if hasattr(self.source,'statistics') else dict(kind=type(self.source).__name__),
                        cnn=self.classifier_worker.statistics() if self.classifier_worker is not None else None)
                    temporary=self._performance_directory/'performance.tmp'
                    temporary.write_text(json.dumps(performance,ensure_ascii=False,indent=2)+'\n')
                    temporary.replace(self._performance_directory/'performance.json')
                    self._last_performance_write=finished_at
                n += 1
                if time.time() - t0 >= 1.0:
                    self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
        except Exception as e:  # noqa: BLE001
            self.error = repr(e)
        finally:
            if self.classifier_worker is not None:
                self.classifier_worker.close()
            if self.state_store is not None:
                self.state_store.close(self.tracker, time.monotonic())
            if getattr(self, "classification_log", None) is not None:
                self.classification_log.close()
            if self.video_log:
                try:
                    self.video_log.close()
                except Exception as e:  # noqa: BLE001
                    self.error = self.error or repr(e)
                self.count_log.metadata["videos"] = self.video_log.info()
            if self.count_log:
                self.count_log.close(error=self.error)

    def _update_tracks(self, frame, dets, raw, view_status):
        now = time.monotonic()
        self._classification_frame = getattr(self, "_classification_frame", -1) + 1
        self._visible_cells = frozenset(raw)
        self._stable_gate = view_status['state'] == 'stable' and not self.robot_motion
        if self.classifier_worker.error:
            raise RuntimeError('비동기 분류 오류: '+self.classifier_worker.error)
        for piece in self.tracker.pieces.values():
            if piece.reference_reset_evidence is not None:
                self.tracker.event('size_reference_rebuilt', piece_id=piece.id, previous=piece.reference_reset_evidence,
                                   reference_px=float(np.median(piece.sensor_sizes)) if piece.sensor_sizes else None)
                piece.reference_reset_evidence = None
        events, links = self.piece_motion.drain_lifecycle()
        updated = set()
        for link in links:
            pid, provisional = link['piece_id'], link['provisional_id']
            self.tracker.aliases[provisional] = pid
            if link['observation'] is not None:
                self.tracker.update_visual(pid, dict(link['observation'], piece_id=pid),
                                           link['stamp'] or dict(observed_at=now))
                updated.add(pid)
        for event in events:
            self.tracker.event(event['event'], **{k: v for k, v in event.items() if k != 'event'})
        result = self.classifier_worker.poll()
        if result is not None:
            for pid, observation in result['observations'].items():
                piece = self.tracker.pieces.get(pid)
                epoch = result['metadata'].get('epochs', {}).get(pid)
                urgent = pid in result['metadata'].get('urgent_ids', [])
                if piece is None or not piece.visible or piece.status in ('captured', 'superseded'):
                    continue
                if not urgent:
                    piece.last_regular_attempt = now
                if not observation.get('probabilities'):
                    continue
                if not urgent and (not piece.eligible or piece.visual_epoch != epoch):
                    continue
                if urgent and now-result['metadata']['observed_at'] > .5:
                    continue
                box = observation['box']
                center = np.array([(box[0]+box[2])/2, (box[1]+box[3])/2])
                actual_center = np.array([(piece.box[0]+piece.box[2])/2, (piece.box[1]+piece.box[3])/2])
                tolerance = .75 if urgent else .25
                if np.linalg.norm(center-actual_center) > max(4., tolerance*max(box[2]-box[0], box[3]-box[1])):
                    continue
                if urgent:
                    self.tracker.update_preview(pid, observation, result['metadata'])
                    # 방해 중에는 즉시 표시만 한다. 정상 결과만 융합/게임 증거로 사용한다.
                    if (not piece.eligible or piece.visual_epoch != epoch
                            or max(observation['probabilities'].values()) < .5):
                        continue
                width, height = box[2]-box[0], box[3]-box[1]
                observation.update(width_px=width, height_px=height, geometric_size_px=float(np.sqrt(width*height)))
                size = (self.size_estimator.measure(tuple(box), tuple(result['metadata']['frame_size']))
                        if self.size_estimator is not None else None)
                observation['size_mm'] = float(np.sqrt(size[0]*size[1])) if size is not None else None
                observation['width_mm'], observation['height_mm'] = size if size is not None else (None, None)
                self.tracker.update_visual(pid, observation, result['metadata'])
                updated.add(pid)
            if updated:
                self.last_classification = now
        # 게임은 전역 안정 뒤에만 완전한 판을 확인한다. 의미 분류는 위에서 객체별로 이미 갱신한다.
        board_observations = {}
        complete = bool(raw) and self._stable_gate
        for detection in dets:
            if detection.tracking_only:
                complete = False
                continue
            if detection.board is None:
                continue
            pid = self.tracker.resolve_id(detection.piece_id)
            piece = self.tracker.pieces.get(pid)
            if (piece is None or not piece.game_eligible or piece.last_raw is None
                    or now-piece.last_classified > max(1., 3*getattr(self.args, 'classification_interval', .5))):
                complete = False
                continue
            board_observations[detection.board] = dict(piece.last_raw, piece_id=pid)
        if complete and updated and len(board_observations) == len(raw):
            self.tracker.turn = self.game_turn
            self.tracker.expected_move = self.expected_robot_move
            if self.tracker.apply(board_observations, result['metadata'] if result is not None else dict(observed_at=now), update_semantics=False):
                self.expected_robot_move = self.tracker.expected_move
        elif not complete:
            self.tracker.pause()
        eligible = []
        epochs = {}
        urgent_ids, regular_ids = [], []
        interval = getattr(self.args, 'classification_interval', .5)
        for detection in dets:
            pid = self.tracker.resolve_id(detection.piece_id)
            piece = self.tracker.pieces.get(pid)
            if piece is None or not piece.visible:
                continue
            detection.piece_id = pid
            detection.geometric_size_mm = piece.size_mm
            detection.classification_conf = piece.display_confidence
            detection.classification_preview = piece.label is None and piece.preview_label is not None
            if piece.display_label is not None:
                detection.name = piece.display_label
                detection.side, detection.kind = piece.display_label.split('_', 1)
            urgent = piece.display_label is None
            frame_interval = getattr(self.args, 'classification_frames', 0)
            frame_due = not frame_interval or self._classification_frame % frame_interval == 0
            due = (piece.eligible and frame_due
                   and now-max(piece.last_classified, piece.last_regular_attempt) >= interval)
            if urgent or due:
                eligible.append(detection)
                epochs[pid] = piece.visual_epoch
                (urgent_ids if urgent else regular_ids).append(pid)
        if eligible:
            metadata = dict(frame=getattr(self.source, 'source_frame', None), source_time=getattr(self.source, 'source_time', None),
                            loop=getattr(self.source, 'source_loop', None), frame_size=[frame.shape[1], frame.shape[0]],
                            observed_at=now, wall_time=time.time(), epochs=epochs, urgent_ids=urgent_ids, regular_ids=regular_ids,
                            processing_frame=self._classification_frame)
            self.classifier_worker.submit(frame, eligible, metadata)
        elif hasattr(self.classifier_worker, 'cancel_pending'):
            self.classifier_worker.cancel_pending()
        self.tracking_status = self.tracker.status
        visible = [p for p in self.tracker.pieces.values() if p.visible]
        self.visual_counts = (len(visible), sum(p.display_label is not None for p in visible), sum(not p.eligible for p in visible))
        self.state_store.save(self.tracker, now)
        # 실시간 보드는 현재 보이는 객체를 투영한다. 확정 게임판이나 숨은 위치를 복제해 그리지 않는다.
        return {d.board: (d.side, d.kind) for d in dets if d.board is not None}

    def expect_robot_move(self, src, dst):
        with self.lock:
            self.expected_robot_move = (tuple(src), tuple(dst))
            self.robot_motion = True
            if self.tracker is not None:
                self.tracker.event('robot_planned', src=list(src), dst=list(dst), wall_time=time.time())

    def get(self):
        with self.lock:
            return self.latest

    def get_piece_regions(self, frame):
        with self.lock:
            # UI가 읽은 프레임과 영역 결과가 다르면 다음 프레임에 표시한다.
            return list(self.local_regions) if self.latest is not None and self.latest[0] is frame else []

    def tracking_ready(self, new_game=False):
        with self.lock:
            if self.tracker is None:
                return True
            return (self._stable_gate and self.tracking_status == '확정 배치 갱신됨'
                    and self._visible_cells == frozenset(self.tracker.occupancy)
                    and (new_game or self.game_tracking_generation is None or self.game_tracking_generation == self.tracker.generation))

    def mark_game_generation(self):
        with self.lock:
            self.game_tracking_generation = self.tracker.generation if self.tracker is not None else None

    def get_obstruction(self):
        with self.lock:
            return dict(self.view_status)

    def fresh_state(self, timeout: float = 10.0, new_game=False):
        """Snapshot the currently displayed observations; never wait for a stable generation."""
        with self.lock:
            if self.error:
                raise RuntimeError('인식 오류: '+self.error)
            if self.latest is None:
                return {}, {}
            _, detections, _, visible = self.latest
            board = {cell:piece for cell,piece in visible.items()
                     if piece[0] in SIDE_KO and piece[1] in FULL_SET}
            centers = {d.board:d.center for d in detections if d.board in board}
            return board, centers

    def robot_snapshot(self, *, samples=3, timeout=3., check=lambda:None):
        """Only frames captured after this request; preserve IDs and off-grid centers."""
        started=time.monotonic()
        # frame_started uses perf_counter, which shares the monotonic clock on Linux.
        barrier=time.perf_counter()
        while True:
            check()
            with self.lock:
                if self.error:
                    raise RuntimeError('인식 오류: '+self.error)
                views=[view for view in self._robot_views if view['stamp']>barrier]
                if len(views)>=samples:
                    return [dict(stamp=v['stamp'],objects=[dict(o) for o in v['objects']])
                            for v in views[-samples:]]
            if time.monotonic()-started>timeout:
                raise RuntimeError('로봇 작업용 새 카메라 프레임을 받지 못했습니다.')
            time.sleep(.02)

    def stop(self):
        self.running = False
        interrupt = getattr(self.source, "interrupt", None)
        if interrupt is not None:
            interrupt()
        self.thread.join(timeout=2.0)


# --------------------------------------------------------------------------- 카메라 오버레이

class GlyphCache:
    """한글 라벨을 PIL로 한 번만 렌더링해 두고 OpenCV 이미지에 알파 합성한다 (PIL로 매 프레임 그리는 것보다 빠르다)."""

    def __init__(self, path, size):
        self.font = ImageFont.truetype(path, size)
        self.cache = {}

    def get(self, text, rgb):
        key = (text, rgb)
        if key not in self.cache:
            l, t, r, b = self.font.getbbox(text, stroke_width=2)
            im = Image.new("RGBA", (r - l + 2, b - t + 2), (0, 0, 0, 0))
            ImageDraw.Draw(im).text((1 - l, 1 - t), text, font=self.font, fill=rgb + (255,), stroke_width=2,
                                    stroke_fill=(255, 255, 255, 255))
            a = np.asarray(im, dtype=np.float32)
            self.cache[key] = (a[:, :, 2::-1], a[:, :, 3:4] / 255.0)   # BGR, alpha
        return self.cache[key]

    def put(self, img, text, x, y, rgb):
        bgr, alpha = self.get(text, rgb)
        h, w = alpha.shape[:2]
        x, y = int(x), int(y)
        x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + w, img.shape[1]), min(y + h, img.shape[0])
        if x0 >= x1 or y0 >= y1:
            return
        sa = alpha[y0 - y:y1 - y, x0 - x:x1 - x]
        roi = img[y0:y1, x0:x1].astype(np.float32)
        img[y0:y1, x0:x1] = (roi * (1 - sa) + bgr[y0 - y:y1 - y, x0 - x:x1 - x] * sa).astype(np.uint8)


def draw_overlay(frame, dets, mapper, glyphs, crop=None, detail=False, label_mode='confidence', size_estimator=None, piece_regions=None):
    """인식 결과를 그린 BGR 이미지. crop=(x0, y0, x1, y1)이면 그 영역만 잘라서 그린다."""
    x0, y0, x1, y1 = crop or (0, 0, frame.shape[1], frame.shape[0])
    img = frame[y0:y1, x0:x1].copy()
    if piece_regions:
        draw_piece_regions(img, piece_regions, (x0, y0))
    off = np.array([x0, y0], dtype=np.float64)

    def P(p):
        q = np.asarray(p, dtype=np.float64) - off
        return int(round(q[0])), int(round(q[1]))

    yellow = (0, 220, 255)
    if mapper is not None:
        g = mapper.grid_pixels()
        cv2.polylines(img, [np.array([P(g[0, 0]), P(g[0, -1]), P(g[-1, -1]), P(g[-1, 0])], np.int32)], True,
                      yellow, 2, cv2.LINE_AA)
        for y in range(ROWS):
            for x in range(COLS):
                cv2.circle(img, P(g[y, x]), 2, yellow, -1, cv2.LINE_AA)
        for (x, y), name in (((0, 0), "(0,0)"), ((8, 9), "(8,9)")):
            u, v = P(g[y, x])
            cv2.putText(img, name, (u + 6, v + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, name, (u + 6, v + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, yellow, 1, cv2.LINE_AA)
    for det in dets:
        assigned = det.board is not None
        rgb = SIDE_RGB.get(det.side, (0, 170, 145)) if assigned else (150, 150, 150)
        bgr = rgb[::-1]
        bx1, by1 = P(det.box[:2])
        bx2, by2 = P(det.box[2:])
        cv2.rectangle(img, (bx1, by1), (bx2, by2), bgr, 2 if assigned else 1, cv2.LINE_AA)
        if assigned and mapper is not None:
            cv2.line(img, P(det.center), P(mapper.grid_pixels()[det.board[1], det.board[0]]), bgr, 1, cv2.LINE_AA)
        glyphs.put(img, piece_char(det.side, det.kind), bx1, by1 - 19, rgb)
        tag = box_label(det, label_mode, size_estimator, (frame.shape[1], frame.shape[0]))
        tag += f" {det.board[0]},{det.board[1]}" if detail and assigned else ""
        if detail and det.piece_id:
            tag += " "+det.piece_id
        font_scale = .34 if label_mode.startswith(('size-', 'geometric-')) else .42
        (text_width, text_height), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
        tx = min(max(2, bx1+19), max(2, img.shape[1]-text_width-2))
        ty = min(max(text_height+2, by1-5), max(text_height+2, img.shape[0]-3))
        cv2.putText(img, tag, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(img, tag, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, font_scale, bgr, 1, cv2.LINE_AA)
    return img


def board_crop_box(mapper, w, h, margin=70):
    """판 영역(격자 + 여유)을 감싸는 잘라낼 사각형."""
    g = mapper.grid_pixels().reshape(-1, 2)
    x0, y0 = np.floor(g.min(axis=0) - margin).astype(int)
    x1, y1 = np.ceil(g.max(axis=0) + margin).astype(int)
    return max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)


# --------------------------------------------------------------------------- 장기판 그림

class BoardView(tk.Canvas):
    CELL, MARGIN = 54, 42

    def __init__(self, master, on_press, on_drag, on_release):
        w = self.MARGIN * 2 + self.CELL * (COLS - 1)
        h = self.MARGIN * 2 + self.CELL * (ROWS - 1)
        super().__init__(master, width=w, height=h, bg="#e8c68d", highlightthickness=0)
        self.bind("<ButtonPress-1>", lambda e: on_press(self.cell_at(e.x, e.y), e))
        self.bind("<B1-Motion>", on_drag)
        self.bind("<ButtonRelease-1>", lambda e: on_release(self.cell_at(e.x, e.y), e))
        self.recognition_renderer = BoardRenderer(FONT_PATH)

    def draw_recognition(self, board):
        frame = self.recognition_renderer.render(board)
        self.recognition_image = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
        self.delete("all")
        self.create_image(0, 0, image=self.recognition_image, anchor="nw")

    def xy(self, c):
        return self.MARGIN + c[0] * self.CELL, self.MARGIN + c[1] * self.CELL

    def cell_at(self, px, py):
        x = round((px - self.MARGIN) / self.CELL)
        y = round((py - self.MARGIN) / self.CELL)
        if 0 <= x < COLS and 0 <= y < ROWS and np.hypot(*(np.subtract((px, py), self.xy((x, y))))) < self.CELL * 0.45:
            return (x, y)
        return None

    def draw_grid(self):
        self.delete("all")
        for x in range(COLS):
            self.create_line(*self.xy((x, 0)), *self.xy((x, ROWS - 1)), fill="#5a3a12", width=1)
            self.create_text(self.xy((x, 0))[0], 16, text=str(x), fill="#5a3a12", font=("TkDefaultFont", 9))
        for y in range(ROWS):
            self.create_line(*self.xy((0, y)), *self.xy((COLS - 1, y)), fill="#5a3a12", width=1)
            self.create_text(16, self.xy((0, y))[1], text=str(y), fill="#5a3a12", font=("TkDefaultFont", 9))
        for cy in (1, 8):  # 궁성 대각선
            self.create_line(*self.xy((3, cy - 1)), *self.xy((5, cy + 1)), fill="#5a3a12")
            self.create_line(*self.xy((5, cy - 1)), *self.xy((3, cy + 1)), fill="#5a3a12")

    def draw_piece(self, c, side, kind, at=None, ghost=False):
        cx, cy = at if at else self.xy(c)
        r = {"king": 25, "sa": 18, "soldier": 18}.get(kind, 22)
        pts = []
        for k in range(8):  # 장기말은 팔각형
            a = np.pi / 8 + k * np.pi / 4
            pts += [cx + r * np.cos(a), cy + r * np.sin(a)]
        color = SIDE_HEX.get(side, "#00897b")
        self.create_polygon(pts, fill="#f8ecd2" if not ghost else "#ddd2bb", outline=color, width=2,
                            tags="piece")
        self.create_text(cx, cy, text=piece_char(side, kind), fill=color,
                         font=(FONT_FAMILY, int(r * 0.85), "bold"), tags="piece")

    def mark(self, c, color, r=27, width=3, dash=None):
        x, y = self.xy(c)
        self.create_oval(x - r, y - r, x + r, y + r, outline=color, width=width, dash=dash)

    def dot(self, c, color, r=7):
        x, y = self.xy(c)
        self.create_oval(x - r, y - r, x + r, y + r, fill=color, outline="")


# --------------------------------------------------------------------------- 앱

class App:
    def __init__(self, args, worker, mapper):
        self.args, self.worker, self.mapper = args, worker, mapper
        self.position_only = getattr(getattr(worker, "detector", None), "position_only", False)
        self.root = tk.Tk()
        self.root.title("장기 실시간 인식")
        self.glyphs = GlyphCache(FONT_PATH, 16)
        self._board_sig = None
        self.zoom = tk.BooleanVar(value=mapper is not None)
        self.detail = tk.BooleanVar(value=False)
        self.piece_motion_visible = tk.BooleanVar(value=getattr(args, 'show_piece_motion', False))
        self.box_label_options = {'신뢰도': 'confidence', '크기(px)': 'size-px', '크기(mm)': 'size-mm',
                                  '기하평균(px)': 'geometric-px', '기하평균(mm)': 'geometric-mm'}
        self.box_label_choice = tk.StringVar(value=next(name for name, value in self.box_label_options.items()
                                                       if value == getattr(args, 'box_label', 'confidence')))
        self._after = None
        self.mode = tk.StringVar(value="live")
        self.first = tk.StringVar(value="blue")
        self.show_diff = tk.BooleanVar(value=True)
        self.game: Game | None = None
        self.sel = None
        self.drag_xy = None
        self.last_move = None
        self.stable = {}
        self._photo = None
        self.rg = None                 # robot_game.RobotGame (로봇 대국 중)
        self.robot_busy = False
        self._motion_settings = None
        self._motion_dialog = None
        self._board_z_dialog = None
        self.robot_stop = False
        self._mover = None
        self._msgs = deque()           # 작업 스레드 → UI 기보 메시지
        self.robot_phase = "대국 시작 전: 판을 놓고 [대국 시작]"
        if args.robot:
            from dataclasses import replace
            from action.pick_place2 import load_pick_place_settings
            from manipulation import ManipulationOptions
            _, parameters=load_pick_place_settings(args.pick_place_config)
            self.manipulation_options=ManipulationOptions.from_parameters(parameters)
            for arg,field in (('adaptive_grasp','adaptive_grasp_enabled'),
                              ('refresh_before_pick','refresh_before_pick'),('auto_retry','retry_move_enabled')):
                value=getattr(args,arg,None)
                if value is not None: self.manipulation_options=replace(self.manipulation_options,**{field:value})

        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.scale = min(1.0, (sw - 760) / args.width, (sh - 140) / args.height)
        self.disp = (int(args.width * self.scale), int(args.height * self.scale))
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.bind("<Escape>", lambda e: self.cancel_select())
        self.root.bind("<Control-z>", lambda e: self.undo())
        if args.robot:
            self.root.bind("<space>", lambda e: self.robot_human_done())
        self._after = self.root.after(30, self.tick)

    def _build(self):
        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True)
        left = tk.Frame(main)
        left.pack(side="left", fill="y")
        # 고정 크기 캔버스: 매 프레임 이미지만 바꾼다 (Label 크기를 다시 지정하면 창 전체가 다시 배치돼 느리다)
        self.cam = tk.Canvas(left, bg="black", width=self.disp[0], height=self.disp[1], highlightthickness=0)
        self.cam.pack()
        self.cam_item = self.cam.create_image(self.disp[0] // 2, self.disp[1] // 2, anchor="center")
        row = tk.Frame(left)
        row.pack(fill="x", padx=6, pady=4)
        self.lbl_cam = tk.Label(row, anchor="w", font=("TkFixedFont", 10), width=56)
        self.lbl_cam.pack(side="left")
        tk.Checkbutton(row, text="판 확대", variable=self.zoom,
                       state=tk.NORMAL if self.mapper else tk.DISABLED).pack(side="right")
        tk.Checkbutton(row, text="상세(교차점 좌표)", variable=self.detail).pack(side="right")
        row2 = tk.Frame(left)
        row2.pack(fill="x", padx=6)
        tk.Label(row2, text="신뢰도 기준").pack(side="left")
        self.conf_var = tk.DoubleVar(value=self.args.conf)
        tk.Scale(row2, from_=0.05, to=0.9, resolution=0.05, orient="horizontal", length=220, variable=self.conf_var,
                 command=lambda v: setattr(self.args, "conf", float(v))).pack(side="left")
        self.roi_var = tk.BooleanVar(value=self.worker.use_roi)
        tk.Checkbutton(row2, text="판 영역만 인식", variable=self.roi_var,
                       state=tk.NORMAL if self.mapper else tk.DISABLED,
                       command=lambda: setattr(self.worker, "use_roi", self.roi_var.get())).pack(side="left", padx=10)
        tk.Label(row2, text='바깥 여유(px)').pack(side='left')
        self.roi_margin_var = tk.StringVar(value=str(getattr(self.args, 'roi_margin', 30)))
        margin_input = tk.Spinbox(row2, from_=0, to=300, increment=5, width=5,
                                  textvariable=self.roi_margin_var, command=self._set_roi_margin,
                                  state=tk.NORMAL if self.mapper else tk.DISABLED)
        margin_input.pack(side='left')
        margin_input.bind('<Return>', self._set_roi_margin)
        margin_input.bind('<FocusOut>', self._set_roi_margin)
        tk.Button(row2, text="원본 프레임 저장", command=self.save_frame).pack(side="right")
        row3 = tk.Frame(left)
        row3.pack(fill="x", padx=6, pady=3)
        tk.Label(row3, text="박스 표시").pack(side="left")
        choice = ttk.Combobox(row3, textvariable=self.box_label_choice, values=list(self.box_label_options), state="readonly", width=10)
        choice.pack(side="left", padx=4)
        choice.bind('<<ComboboxSelected>>', lambda e: setattr(self.worker, 'box_label_mode', self.box_label_options[self.box_label_choice.get()]))
        tk.Label(row3, text="mm는 보정 기반 박스 추정 길이", fg="#666").pack(side="left", padx=4)
        tk.Button(row3, text="방해 기준 다시 잡기", command=self.worker.obstruction_reset.set).pack(side="right")
        self._build_exposure_controls(left)
        sensor_row = tk.Frame(left)
        sensor_row.pack(fill='x', padx=6, pady=2)
        tk.Checkbutton(sensor_row, text='말별 방해 영역(실험)', variable=self.piece_motion_visible,
                       command=lambda: setattr(self.worker, 'show_piece_motion', self.piece_motion_visible.get())).pack(side='left')
        self.lbl_piece_motion = tk.Label(sensor_row, text='노랑=MOVE · 빨강=CONF/SIZE/MISS', fg='#666')
        self.lbl_piece_motion.pack(side='left', padx=5)
        self.lbl_obstruction = tk.Label(left, anchor="w", justify="left", width=65, fg="#a06000", text="화면 방해: 확인 중")
        self.lbl_obstruction.pack(fill="x", padx=6, pady=2)
        self.lbl_tracking = tk.Label(left, anchor='w', fg='#555', width=65)
        self.lbl_tracking.pack(fill='x', padx=6)
        tk.Label(left, anchor="w", justify="left", fg="#555", text=(
            "박스 색: 파랑=초, 빨강=한, 회색=판 교차점에 할당 안 됨.  노란 점: 장기판 교차점 (보정 파일의 네 모서리 기준)"
        )).pack(fill="x", padx=6)

        right = tk.Frame(main, padx=10, pady=6)
        right.pack(side="left", fill="both", expand=True)
        top = tk.Frame(right)
        top.pack(fill="x")
        tk.Radiobutton(top, text="실시간 인식", value="live", variable=self.mode, command=self.on_mode,
                       font=("TkDefaultFont", 11, "bold")).pack(side="left")
        tk.Radiobutton(top, text="게임", value="game", variable=self.mode, command=self.on_mode,
                       state=tk.DISABLED if self.position_only else tk.NORMAL,
                       font=("TkDefaultFont", 11, "bold")).pack(side="left", padx=8)
        self.lbl_status = tk.Label(right, anchor="w", font=(FONT_FAMILY, 13, "bold"), width=40)
        self.lbl_status.pack(fill="x", pady=(4, 2))

        mid = tk.Frame(right)
        mid.pack(fill="both", expand=True)
        self.board = BoardView(mid, self.on_press, self.on_drag, self.on_release)
        self.board.pack(side="left", anchor="n")

        side = tk.Frame(mid, padx=8)
        side.pack(side="left", fill="y")
        fg = tk.LabelFrame(side, text="게임", padx=6, pady=4)
        fg.pack(fill="x")
        fr = tk.Frame(fg)
        fr.pack(fill="x")
        tk.Label(fr, text="선수").pack(side="left")
        tk.Radiobutton(fr, text="초", value="blue", variable=self.first).pack(side="left")
        tk.Radiobutton(fr, text="한", value="red", variable=self.first).pack(side="left")
        for text, cmd in (("인식된 판으로 새 게임", self.new_game), ("판에서 둔 수 읽기", self.read_move),
                          ("되돌리기 (Ctrl+Z)", self.undo), ("한수쉼", self.pass_turn),
                          ("인식된 판으로 맞추기", self.sync_to_detection)):
            tk.Button(fg, text=text, command=cmd,
                      state=tk.DISABLED if self.position_only else tk.NORMAL).pack(fill="x", pady=1)
        tk.Checkbutton(fg, text="인식과 다른 칸 표시", variable=self.show_diff).pack(anchor="w")
        if self.args.robot:
            fr = tk.LabelFrame(side, text=f"로봇 대국 (사람 {SIDE_KO[self.args.human_side]})", padx=6, pady=4)
            fr.pack(fill="x", pady=(8, 0))
            tk.Button(fr, text="로봇 속도·가속도 설정", command=self._open_motion_settings).pack(fill="x", pady=1)
            tk.Button(fr, text="판 Z 높이 미세 보정", command=self._open_board_z_settings).pack(fill="x", pady=1)
            tk.Button(fr, text="호밍 (J1 0°)", command=self.robot_home).pack(fill="x", pady=1)
            tk.Button(fr, text="대국 시작 (인식된 판)", command=self.robot_start).pack(fill="x", pady=1)
            tk.Button(fr, text="착수 완료 (Space)", command=self.robot_human_done, bg="#2e7d32", fg="white",
                      font=(FONT_FAMILY, 12, "bold")).pack(fill="x", pady=3)
            tk.Button(fr, text="로봇 정지", command=self.robot_halt, bg="#c62828", fg="white").pack(fill="x", pady=1)
            self.manipulation_vars={}
            for name,label in (('adaptive_grasp_enabled','주변 말 피해 회전'),
                               ('refresh_before_pick','집기 직전 재인식'),('retry_move_enabled','착수 실패 1회 재시도')):
                value=tk.BooleanVar(value=getattr(self.manipulation_options,name))
                self.manipulation_vars[name]=value
                tk.Checkbutton(fr,text=label,variable=value,command=self._set_manipulation_options).pack(anchor='w')
            self.lbl_robot = tk.Label(fr, anchor="w", justify="left", wraplength=220, font=(FONT_FAMILY, 10))
            self.lbl_robot.pack(fill="x")
        tk.Label(side, text="기보", anchor="w").pack(fill="x", pady=(8, 0))
        self.log = tk.Listbox(side, height=14, width=26, font=(FONT_FAMILY, 10))
        self.log.pack(fill="both", expand=True)
        self.lbl_count = tk.Label(side, anchor="w", justify="left", font=(FONT_FAMILY, 10))
        self.lbl_count.pack(fill="x", pady=6)

    def _set_roi_margin(self, event=None):
        try:
            value = int(self.roi_margin_var.get())
            if value < 0:
                raise ValueError
        except ValueError:
            self.roi_margin_var.set(str(getattr(self.args, 'roi_margin', 30)))
            return
        self.args.roi_margin = value

    def _build_exposure_controls(self, parent):
        self.exposure_control = getattr(self.worker.source, 'exposure_control', None)
        if self.exposure_control is None:
            return
        state = self.exposure_control.snapshot()
        self._exposure_input_error = ''
        self._exposure_limits = None
        self.exposure_auto = tk.BooleanVar(value=state['auto'])
        self.exposure_value = tk.StringVar(value=f"{state['exposure']:g}" if state['exposure'] is not None else '')
        row = tk.Frame(parent)
        row.pack(fill='x', padx=6, pady=3)
        tk.Label(row, text='RGB 노출').pack(side='left')
        self.exposure_auto_button = tk.Checkbutton(row, text='자동', variable=self.exposure_auto,
                                                   command=self._queue_exposure)
        self.exposure_auto_button.pack(side='left')
        self.exposure_spin = tk.Spinbox(row, textvariable=self.exposure_value, width=9)
        self.exposure_spin.pack(side='left', padx=3)
        self.exposure_spin.bind('<Return>', lambda event:self._queue_exposure())
        self.exposure_apply = tk.Button(row, text='적용', command=self._queue_exposure)
        self.exposure_apply.pack(side='left')
        self.lbl_exposure = tk.Label(row, anchor='w', fg='#555')
        self.lbl_exposure.pack(side='left', padx=5)
        buttons = tk.Frame(parent)
        buttons.pack(fill='x', padx=6, pady=2)
        tk.Button(buttons, text='RGB 카메라 설정…', command=self._open_camera_settings).pack(side='left')
        tk.Button(buttons, text='저장값 적용', command=self._apply_saved_camera_settings).pack(side='left', padx=4)
        self.lbl_camera_settings = tk.Label(buttons, text='', anchor='w', fg='#555')
        self.lbl_camera_settings.pack(side='left', padx=4)
        self._camera_settings_dialog = None
        self._camera_preset_pending = False
        self._refresh_exposure()

    def _open_camera_settings(self):
        if self._camera_settings_dialog is not None:
            self._camera_settings_dialog.window.lift()
            return
        from realsense_settings_ui import CameraSettingsDialog
        self._camera_settings_dialog = CameraSettingsDialog(
            self.root, self.exposure_control,
            on_close=lambda:setattr(self,'_camera_settings_dialog',None))

    def _apply_saved_camera_settings(self):
        from realsense_settings import apply_saved
        try:
            values = apply_saved(self.exposure_control)
            if 'enable_auto_exposure' in values:self.exposure_auto.set(bool(values['enable_auto_exposure']))
            if 'exposure' in values:self.exposure_value.set(f"{values['exposure']:g}")
            if self._camera_settings_dialog is not None:
                self._camera_settings_dialog.set_form(values)
                self._camera_settings_dialog.awaiting = True
            self._camera_preset_pending = True
            self._exposure_input_error = ''
        except (OSError, ValueError, RuntimeError) as exc:
            self._exposure_input_error = str(exc)
        self._refresh_exposure()

    def _queue_exposure(self):
        try:
            automatic = self.exposure_auto.get()
            value = None if automatic else float(self.exposure_value.get())
            value = self.exposure_control.request(automatic, value)
            if value is not None:
                self.exposure_value.set(f'{value:g}')
            self._exposure_input_error = ''
        except (ValueError, RuntimeError) as exc:
            self._exposure_input_error = str(exc)
        self._refresh_exposure()

    def _refresh_exposure(self):
        control = getattr(self, 'exposure_control', None)
        if control is None:
            return
        state = control.snapshot()
        if not state['pending']:
            self.exposure_auto.set(state['auto'])
            if state['auto'] and state['exposure'] is not None:
                self.exposure_value.set(f"{state['exposure']:g}")
        enabled = state['connected'] and state['supported']
        automatic = self.exposure_auto.get()
        self.exposure_auto_button.config(state=tk.NORMAL if state['connected'] and state['auto_supported'] else tk.DISABLED)
        manual_state = tk.NORMAL if enabled and not automatic and not state['pending'] else tk.DISABLED
        self.exposure_spin.config(state=manual_state)
        self.exposure_apply.config(state=manual_state)
        limits = (state['minimum'], state['maximum'], state['step'])
        if enabled and limits != self._exposure_limits:
            self.exposure_spin.config(from_=limits[0], to=limits[1], increment=limits[2])
            self._exposure_limits = limits
        error = self._exposure_input_error or state['error']
        if error:
            text, colour = '카메라: '+error, '#b42318'
        elif not state['connected']:
            text, colour = '카메라 연결 대기', '#a06000'
        elif not state['supported']:
            text, colour = '이 센서는 노출 조절 미지원', '#777'
        else:
            mode = '적용 대기' if state['pending'] else '자동' if state['auto'] else '수동'
            text = f"{mode} · 현재 {state['exposure']:g} · 범위 {limits[0]:g}~{limits[1]:g} (센서 단위)"
            colour = '#555'
        self.lbl_exposure.config(text=text, fg=colour)
        if getattr(self,'_camera_preset_pending',False):
            if state['error']:
                self.lbl_camera_settings.config(text='저장값 적용 실패', fg='#b42318')
                self._camera_preset_pending = False
            elif state['pending']:
                self.lbl_camera_settings.config(text='저장값 적용 대기…', fg='#555')
            else:
                suffix=f" · 미지원 {len(state['skipped'])}개 제외" if state['skipped'] else ''
                self.lbl_camera_settings.config(text='저장값 적용 완료'+suffix, fg='#20733c')
                self._camera_preset_pending = False

    # ------------------------------------------------------------------ 모드 / 게임 조작
    def on_mode(self):
        if self.mode.get() == "game" and self.game is None:
            self.new_game()
        self.cancel_select()

    def _recognized_board(self):
        return {cell:piece for cell,piece in self.stable.items()
                if piece[0] in SIDE_KO and piece[1] in FULL_SET}

    def new_game(self):
        if self.position_only:
            self.mode.set("live")
            return
        board = self._recognized_board()
        self.game = Game(board, top_side=detect_top_side(board), turn=self.first.get())
        self.worker.mark_game_generation()
        self.last_move = None
        self.log.delete(0, "end")
        self.log.insert("end", f"새 게임: 위쪽 {SIDE_KO[self.game.top_side]}, {SIDE_KO[self.game.turn]} 먼저")
        self.mode.set("game")
        self.cancel_select()

    def sync_to_detection(self):
        """게임 진행(차례·기보)은 두고 판만 인식 결과로 덮어쓴다 (인식 오류를 손으로 고친 뒤 등)."""
        if self.game is None:
            return self.new_game()
        self.game.board = self._recognized_board()
        self.game.top_side = detect_top_side(self.game.board, default=self.game.top_side)
        self.log.insert("end", "판을 인식 결과로 맞춤")
        self.cancel_select()

    def read_move(self):
        """실제 판에서 사람이 둔 수를 인식 결과와의 차이로 읽어 게임에 반영한다."""
        if self.game is None:
            return
        observed = self._recognized_board()
        mv = infer_move(self.game.board, observed, self.game.turn)
        self.game.board = observed
        self.game.top_side = detect_top_side(observed, default=self.game.top_side)
        self.game.turn = other(self.game.turn)
        self.log.insert("end", "현재 인식된 판으로 갱신" + (f" · {mv[0]}→{mv[1]}" if mv else ""))
        self.cancel_select()

    def play(self, src, dst):
        rec = self.game.move(src, dst)
        self.last_move = (src, dst)
        n = len(self.game.history)
        cap = f" x{piece_char(*rec.captured)}" if rec.captured else ""
        self.log.insert("end", f"{n}. {SIDE_KO[rec.side]} {piece_char(*rec.piece)} {src[0]},{src[1]}→{dst[0]},{dst[1]}{cap}")
        st = self.game.status()
        if st:
            self.log.insert("end", f"   {SIDE_KO[self.game.turn]} {st}!")
        self.log.see("end")

    def undo(self):
        if self.game and self.game.undo():
            self.log.delete("end")
            if self.log.size() and self.log.get("end").startswith("   "):
                self.log.delete("end")
            h = self.game.history
            self.last_move = (h[-1].src, h[-1].dst) if h and not h[-1].passed else None
            self.cancel_select()

    def pass_turn(self):
        if self.game:
            self.game.pass_turn()
            self.log.insert("end", f"{len(self.game.history)}. {SIDE_KO[other(self.game.turn)]} 한수쉼")
            self.log.see("end")

    # ------------------------------------------------------------------ 마우스 (클릭 또는 드래그)
    def _gazebo_human_input(self):
        # Physical simulation must not teleport pieces from board clicks.
        return False

    def _move_gazebo_piece(self, src, dst):
        messagebox.showinfo('물리 시뮬레이션',
                            '화면에서 말 좌표를 직접 옮기는 기능은 꺼져 있습니다. 접촉·힘으로 이동하세요.',
                            parent=self.root)
        self.cancel_select()

    def on_press(self, cell, e):
        # Board clicks are annotations only, including real/Gazebo robot games.
        board = self.stable if self.mode.get() == "live" or self.game is None else self.game.board
        piece = board.get(cell)
        self.sel = (None if self.sel == cell else cell) if piece and piece[0] in SIDE_KO and piece[1] in FULL_SET else None
        self.drag_xy = None

    def on_drag(self, e):
        pass

    def on_release(self, cell, e):
        self.drag_xy = None
        if getattr(self, 'robot_busy', False):
            self.cancel_select()

    def selection_moves(self, board):
        from rules import Position
        known = {cell:piece for cell,piece in board.items()
                 if piece[0] in SIDE_KO and piece[1] in FULL_SET}
        top = detect_top_side(known, default=self.game.top_side if self.game else 'red')
        return Position(known, top).legal_moves(self.sel) if self.sel in known else []

    def cancel_select(self):
        self.sel, self.drag_xy = None, None

    def diff_cells(self):
        if self.game is None:
            return set()
        b, s = self.game.board, self.stable
        return {c for c in set(b) | set(s) if b.get(c) != s.get(c)}

    # ------------------------------------------------------------------ 로봇 대국
    def _set_manipulation_options(self):
        from dataclasses import replace
        if self.robot_busy:
            for name,var in self.manipulation_vars.items(): var.set(getattr(self.manipulation_options,name))
            self._msgs.append('[집기 설정] 동작이 끝난 뒤 변경하세요.')
            return
        self.manipulation_options=replace(self.manipulation_options,
                                         **{name:var.get() for name,var in self.manipulation_vars.items()})
        if self.rg is not None: self.rg.cfg.manipulation=self.manipulation_options
        self._msgs.append('[집기 설정] '+str(self.manipulation_options))

    def _get_motion_settings(self):
        from dataclasses import replace
        from action.pick_place2 import load_pick_place_settings
        saved = getattr(self, '_motion_settings', None)
        if saved is not None:
            return saved
        settings, _ = load_pick_place_settings(self.args.pick_place_config)
        velocity = getattr(self.args, 'robot_vel', None)
        if velocity is not None:
            settings = replace(settings, linear_velocity_mm_s=velocity,
                               linear_acceleration_mm_s2=velocity * 2).validate()
        return replace(settings, joint_velocity_deg_s=settings.angular_velocity_deg_s,
                       joint_acceleration_deg_s2=settings.angular_acceleration_deg_s2).validate()

    def _open_motion_settings(self):
        from robot_motion_ui import RobotMotionDialog
        if self._motion_dialog is not None:
            self._motion_dialog.window.lift()
            return
        self._motion_dialog = RobotMotionDialog(
            self.root, self._get_motion_settings(), self._apply_motion_settings,
            lambda:setattr(self, '_motion_dialog', None))

    def _apply_motion_settings(self, settings):
        from dataclasses import replace
        current = getattr(self, '_motion_settings', None)
        if current is not None:
            fields = ('linear_velocity_mm_s', 'linear_acceleration_mm_s2', 'angular_velocity_deg_s',
                      'angular_acceleration_deg_s2', 'joint_velocity_deg_s', 'joint_acceleration_deg_s2')
            settings = replace(current, **{name:getattr(settings,name) for name in fields})
        self._install_robot_settings(settings)
        self._msgs.append(f'[속도 설정] 직선 {settings.linear_velocity_mm_s:g} mm/s, '
                          f'{settings.linear_acceleration_mm_s2:g} mm/s² · '
                          f'회전·관절 {settings.angular_velocity_deg_s:g} °/s, '
                          f'{settings.angular_acceleration_deg_s2:g} °/s² · 다음 동작부터 적용')

    def _open_board_z_settings(self):
        from action.board import BoardGeometry
        from board_z_ui import BoardZDialog
        if self._board_z_dialog is not None:
            self._board_z_dialog.window.lift()
            return
        self._board_z_dialog = BoardZDialog(
            self.root, BoardGeometry.from_calibration(self.args.calibration), self._get_motion_settings(),
            self._apply_board_z_settings, lambda:setattr(self, '_board_z_dialog', None))

    def _apply_board_z_settings(self, settings):
        from dataclasses import replace
        from board_z_ui import Z_FIELDS
        settings = replace(self._get_motion_settings(), **{name:getattr(settings,name) for name in Z_FIELDS})
        self._install_robot_settings(settings)
        self._msgs.append(f'[판 Z 보정] 사용={settings.board_z_compensation_enabled} · '
                          f'전체 {settings.board_z_offset_mm:+g}mm · 모서리 추가 {settings.board_z_corner_offsets_mm}')

    def _install_robot_settings(self, settings):
        # Both this callback and _robot_job's busy transition run on the Tk thread.
        if self.robot_busy:
            raise RuntimeError('로봇 동작이 끝난 뒤 적용하세요. 진행 중인 이동은 변경하지 않습니다.')
        settings.validate()
        if self.rg is not None:
            if self.rg.picker is not None:
                self.rg.picker.set_motion_settings(settings)
            self.rg.cfg.command_settings = settings
        self._motion_settings = settings

    def _robot_job(self, phase, fn, *, done_phase=None):
        """fn을 작업 스레드에서 돌린다 (엔진·로봇 동작이 UI를 막지 않게)."""
        if self.robot_busy:
            return
        from session import TurnError

        self.robot_busy, self.robot_stop, self.robot_phase = True, False, phase

        def run():
            try:
                fn()
                self.robot_phase = done_phase if done_phase is not None else self._human_phase()
            except TurnError as e:
                self._msgs.append(f"! {e}")
                self.robot_phase = f"다시 확인: {e}"
            except Exception as e:  # noqa: BLE001
                self._msgs.append(f"! 오류: {e}")
                self.robot_phase = f"오류: {e}\n판을 바로잡고 [인식된 판으로 맞추기] 후 [착수 완료]"
            finally:
                self.worker.robot_motion = False
                self.robot_busy = False

        threading.Thread(target=run, daemon=True).start()

    def _human_phase(self):
        return "현재 판을 기준으로 [착수 완료]를 누르면 로봇이 둡니다"

    def _ensure_robot(self):
        if self.position_only:
            raise RuntimeError("위치 전용 모델은 진영·종류를 분류하지 않아 로봇 대국에 사용할 수 없다")
        if self.rg is not None:
            return
        from session import RobotGame, RobotGameConfig

        from action import BoardGeometry, BoardMover
        from action.pick_place2 import BoardPickPlace2
        from engine import JanggiEngine
        from piece_detection import CameraToBase

        if (getattr(self.args, "simulation_calibration", False) and self.args.source != "gazebo"
                and not self.args.no_motion):
            raise RuntimeError("Gazebo 녹화 재생은 --no-motion으로 확인하세요. 가상 로봇 제어에는 --source gazebo를 사용하세요.")
        if self.args.source == "gazebo" and not self.args.no_motion:
            self.worker.source.require_virtual_driver()
        self.robot_phase = "엔진 시작 중"
        engine = JanggiEngine()
        geom = BoardGeometry.from_calibration(self.args.calibration)
        cam = CameraToBase.from_calibration(self.args.calibration)
        settings = self._get_motion_settings()
        picker = None
        if not self.args.no_motion:
            self.robot_phase = "로봇 드라이버 연결 중"
            v = settings.linear_velocity_mm_s
            a = settings.linear_acceleration_mm_s2
            self._mover = BoardMover(self.args.ns, self.args.calibration,
                                    vel=(v, settings.angular_velocity_deg_s),
                                    acc=(a, settings.angular_acceleration_deg_s2))
            try:
                picker = BoardPickPlace2(self._mover, self.args.pick_place_config,
                                         virtual=self.args.source == 'gazebo',
                                         stop_check=lambda: self.robot_stop,
                                         log=self._msgs.append, yaw=self.args.pick_yaw,
                                         trace_path=(self.worker.state_store.directory if self.worker.state_store is not None
                                                     else self.args.state_dir/f'robot_session_{time.time_ns()}')/'robot_commands.jsonl')
                picker.set_motion_settings(settings)
            except Exception:
                self._mover.close()
                self._mover = None
                raise
        cfg = RobotGameConfig(human_side=self.args.human_side, movetime_ms=self.args.movetime,
                              piece_height_mm=self.args.piece_height, command_settings=settings,
                              manipulation=getattr(self,'manipulation_options',None))
        self.rg = RobotGame(engine, cam, geom, picker, cfg, log=self._msgs.append)
        self.rg.vision=self.worker
        if not self.args.no_motion:
            self.rg.on_move_planned = self.worker.expect_robot_move

    def robot_home(self):
        """Explicit home button: no board read, game start, or gripper command."""
        def job():
            self._ensure_robot()
            from action.pick_place2 import load_pick_place_settings
            settings, _ = load_pick_place_settings(self.args.pick_place_config)
            joints = list(settings.home_joint_deg)
            self._msgs.append('[호밍] 목표 관절각 [deg]: '+str(joints))
            if not self.args.no_motion:
                self.worker.robot_motion = True
                motion = getattr(self.rg.picker, 'settings', None)
                self.rg.picker.go_observe(
                    joints, motion.joint_velocity_deg_s if motion else self.rg.cfg.observe_vel,
                    motion.joint_acceleration_deg_s2 if motion else self.rg.cfg.observe_vel)
        self._robot_job('호밍 중 (J1 0°)', job,
                        done_phase='호밍 완료 · [대국 시작]을 누르세요' if not self.args.no_motion
                        else '호밍 목표 표시 완료 · --no-motion')

    def robot_start(self):
        def job():
            self._ensure_robot()
            self.robot_phase = "판 읽는 중"
            board, _ = self.worker.fresh_state(new_game=True)
            self.game = self.rg.start(board, observe=False)
            self.worker.mark_game_generation()
            self.last_move = None
        self.log.delete(0, "end")
        self.mode.set("game")
        self._robot_job("대국 준비 중", job)

    def robot_human_done(self):
        """Replace the game with the current observations, then plan and execute."""
        def job():
            self._ensure_robot()
            rg = self.rg
            self.robot_phase = "현재 판 읽는 중"
            board, centers = self.worker.fresh_state()
            if rg.game is None:
                self.game = rg.start(board, observe=False)
            rg.apply_human_move(board)
            self.game = rg.game
            self.robot_phase = "현재 판 기준 엔진 생각 중 / 로봇 두는 중"
            self.worker.robot_motion = not self.args.no_motion
            plan = rg.robot_turn(centers, stop_check=lambda: self.robot_stop)
            if plan.src is not None and not plan.is_pass:
                self.last_move = (plan.src, plan.dst)
        self.mode.set("game")
        self._robot_job("판 읽는 중", job)

    def robot_halt(self):
        self.robot_stop = True
        if self._mover is not None:
            self._mover.stop()
        self._msgs.append("! 로봇 정지 명령")

    # ------------------------------------------------------------------ 그리기
    def tick(self):
        if self.worker.error:
            messagebox.showerror("오류", f"인식 스레드 오류: {self.worker.error}", parent=self.root)
            return self.quit()
        data = self.worker.get()
        if data is not None:
            frame, dets, raw, self.stable = data
            self.worker.video_zoom = self.zoom.get()
            self.worker.video_detail = self.detail.get()
            self.worker.box_label_mode = self.box_label_options[self.box_label_choice.get()]
            self.worker.show_piece_motion = self.piece_motion_visible.get()
            local_regions = self.worker.get_piece_regions(frame) if self.piece_motion_visible.get() else []
            self.lbl_piece_motion.config(text=f"의심 {len(local_regions)}곳 · 노랑=이동 / 빨강=검출 이상"
                                         if self.piece_motion_visible.get() else "영역 표시 꺼짐 · 말별 판단은 계속")
            crop = None
            if self.zoom.get() and self.mapper is not None:
                crop = board_crop_box(self.mapper, frame.shape[1], frame.shape[0])
            img = draw_overlay(frame, dets, self.mapper, self.glyphs, crop, self.detail.get(),
                               self.worker.box_label_mode, self.worker.size_estimator, local_regions)
            k = min(self.disp[0] / img.shape[1], self.disp[1] / img.shape[0])
            if abs(k - 1.0) > 1e-3:
                img = cv2.resize(img, (int(img.shape[1] * k), int(img.shape[0] * k)),
                                 interpolation=cv2.INTER_LINEAR if k > 1 else cv2.INTER_AREA)
            pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            if self._photo is None or (self._photo.width(), self._photo.height()) != pil.size:
                self._photo = ImageTk.PhotoImage(pil)   # 크기가 바뀔 때만 새로 만든다
                self.cam.itemconfigure(self.cam_item, image=self._photo)
            else:
                self._photo.paste(pil)                  # 같은 이미지 객체에 내용만 덮어쓴다
            lag = time.time() - self.worker.last_frame_t
            stale = f"  ⚠ 영상 {lag:.0f}초 멈춤" if lag > 1.5 else ""
            status = self.worker.get_obstruction()
            title, color = {'stable': ('방해 징후 없음', '#20733c'), 'settling': ('안정 확인 중', '#a06000'),
                            'suspected': ('방해 의심', '#b42318'), 'unavailable': ('판단 불가', '#777')}.get(status['state'], ('확인 중', '#777'))
            if lag > 1.5:
                title, color = '판단 불가 · 영상 지연', '#b42318'
            self.worker.game_turn = self.game.turn if self.game is not None and self.mode.get() == 'game' else None
            self.lbl_tracking.config(text=(f'종류 {self.worker.visual_counts[1]}/{self.worker.visual_counts[0]} · 개별 보류 {self.worker.visual_counts[2]} · '
                                           +'현재 인식 판으로 진행') if self.worker.tracker is not None else '')
            self.lbl_obstruction.config(text=f"화면 방해: {title} · {status['reason']} · 움직임 {status['motion']*100:.1f}%", fg=color)
            assigned = sum(1 for d in dets if d.board is not None)
            unique = count_unique_detections(dets)
            self.lbl_cam.config(text=f"인식 {self.worker.fps:4.1f} FPS · 검출 {len(dets)} · 중복 제거 {unique} · 판 할당 {assigned}{stale}")
        while self._msgs:
            self.log.insert("end", self._msgs.popleft())
            self.log.see("end")
        if self.args.robot:
            self.lbl_robot.config(text=("⏳ " if self.robot_busy else "") + self.robot_phase)
        self._refresh_exposure()
        try:
            self.draw_board()
        except RuntimeError:   # 작업 스레드가 게임 판을 바꾸는 중 (다음 tick에 다시 그린다)
            self._board_sig = None
        self._after = self.root.after(30, self.tick)

    def draw_board(self):
        bv = self.board
        live = self.mode.get() == "live" or self.game is None
        board = self.stable if live else self.game.board
        # 그릴 내용이 그대로면 다시 그리지 않는다 (캔버스 전체를 다시 그리는 데 수십 ms 걸린다)
        sig = (live, tuple(sorted(board.items())), self.sel, self.drag_xy, self.last_move,
               None if live else (self.game.turn, len(self.game.history)),
               tuple(sorted(self.diff_cells())) if (not live and self.show_diff.get()) else None)
        if sig == self._board_sig:
            return
        self._board_sig = sig
        bv.draw_grid()
        if live:
            status = "말 위치 인식 · 진영/종류 미분류" if self.position_only else "실시간 인식 결과"
            self.lbl_status.config(text=status if self.mapper else "보정 파일이 없어 판 매핑 없음",
                                   fg="#333")
        else:
            st = self.game.status()
            self.lbl_status.config(text=f"{SIDE_KO[self.game.turn]} 차례" + (f" — {st}!" if st else ""),
                                   fg=SIDE_HEX[self.game.turn])
            if self.last_move:
                for c in self.last_move:
                    bv.mark(c, "#2e7d32", r=29, width=2, dash=(4, 2))
            if self.show_diff.get():
                for c in self.diff_cells():
                    bv.mark(c, "#ff8f00", r=30, width=3)
        if live:
            bv.draw_recognition(board)
        else:
            for c, (side, kind) in board.items():
                if c == self.sel and self.drag_xy:
                    bv.draw_piece(c, side, kind, ghost=True)
                else:
                    bv.draw_piece(c, side, kind)
        if self.sel is not None:
            bv.mark(self.sel, "#6a1b9a", r=28, width=3)
            for c in self.selection_moves(board):
                if c in board:
                    bv.mark(c, "#6a1b9a", r=26, width=3, dash=(3, 2))
                else:
                    bv.dot(c, "#6a1b9a")
            if self.drag_xy:
                bv.draw_piece(self.sel, *board[self.sel], at=self.drag_xy)
        if self.position_only:
            self.lbl_count.config(text=f"판 위 말 위치 {len(board)}개\n진영·말 종류는 분류하지 않음")
            return
        cnt = {s: {k: 0 for k in FULL_SET} for s in SIDE_KO}
        for side, kind in board.values():
            if side in cnt and kind in cnt[side]:
                cnt[side][kind] += 1
        lines = []
        for s in ("blue", "red"):
            total = sum(cnt[s].values())
            missing = [f"{piece_char(s, k)}{FULL_SET[k] - n}" for k, n in cnt[s].items() if n < FULL_SET[k]]
            lines.append(f"{SIDE_KO[s]} {total}/16" + (f"  없음: {' '.join(missing)}" if missing else ""))
        unknown = sum(side == "unknown" for side, kind in board.values())
        if unknown:
            lines.append(f"종류 미확정 {unknown}개")
        self.lbl_count.config(text="\n".join(lines))

    def save_frame(self):
        """오버레이 없는 원본 프레임과 인식 결과를 저장한다 (인식 문제 분석·추가 학습 데이터용)."""
        data = self.worker.get()
        if data is None:
            return
        frame, dets = data[0], data[1]
        out = HERE / "captures"
        out.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(str(out / f"{stamp}.png"), frame)
        with open(out / f"{stamp}.txt", "w", encoding="utf-8") as f:
            f.write(f"# conf>={self.args.conf} roi={self.worker.use_roi}  name detection_conf x1 y1 x2 y2 board classification_conf piece_id geometric_size_mm\n")
            for d in dets:
                f.write(f"{d.name}\t{d.conf:.3f}\t" + " ".join(f"{v:.1f}" for v in d.box) + f"\t{d.board}\t{d.classification_conf}\t{d.piece_id}\t{d.geometric_size_mm}\n")
        self.lbl_cam.config(text=f"저장: {out.name}/{stamp}.png (검출 {len(dets)})")
        print(f"[저장] {out / stamp}.png")

    def quit(self):
        z_dialog = getattr(self, '_board_z_dialog', None)
        if z_dialog is not None:
            z_dialog.close()
        motion_dialog = getattr(self, '_motion_dialog', None)
        if motion_dialog is not None:
            motion_dialog.close()
        dialog = getattr(self, '_camera_settings_dialog', None)
        if dialog is not None:dialog.close()
        if self._after is not None:
            self.root.after_cancel(self._after)
            self._after = None
        self.worker.stop()
        if self.rg is not None:
            self.rg.engine.close()
        if self._mover is not None:
            self._mover.stop()
            self._mover.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    args = parse_args()
    print(f"[보정] {args.calibration} ({args.calibration_origin})")
    mapper = None
    if args.calibration and Path(args.calibration).exists():
        mapper = BoardMapper.from_calibration(args.calibration, piece_height_mm=args.piece_height)
    else:
        print(f"[경고] 보정 파일 없음: {args.calibration} — 판 매핑 없이 영상 좌표에서 인식한다")
    print("[모델] 불러오는 중… (처음엔 수 초)")
    from piece_detection import PieceDetector

    detector = PieceDetector(args.model, conf=args.conf, device=args.device)
    if args.model_mode in ("hybrid", "cnn"):
        from piece_detection.classifier import MobileNetClassifier, UltralyticsClassifier, TwoStageDetector
        classifier = (MobileNetClassifier(args.classifier, args.device, args.classifier_batch)
                      if args.classifier_backend == "mobilenet" else
                      UltralyticsClassifier(args.classifier, args.device, args.classifier_batch, args.classifier_size))
        detector = TwoStageDetector(detector, classifier, args.classifier_conf)
        print(f"[분류] {args.classifier_backend} · {args.classifier} · {len(classifier.classes)}종")
        if args.robot and not detector.robot_compatible:
            print("[분류] 전체 14종 모델이 아니므로 --robot 비활성화")
            args.robot = False
    print(f"[모델] {args.model.name} on {detector.device}")
    if detector.position_only and args.robot:
        print("[위치 인식] 진영·종류를 분류하지 않는 모델이므로 --robot을 비활성화한다. 로봇 연결 없음.")
        args.robot = False
    try:
        if args.source == "gazebo":
            from ros_source import GazeboSource
            source = GazeboSource(namespace=args.ns, transport=args.gazebo_transport)
        elif args.source == "realsense":
            source = RealSenseSource(args.width, args.height)
        else:
            source = CvSource(args.source, args.width, args.height, args.playback_fps)
    except RuntimeError as e:
        sys.exit(f"[오류] 영상 입력을 열 수 없다 ({args.source}): {e}\n"
                 "  - RealSense: USB 3 포트에 연결됐는지 `lsusb | grep -i realsense`로 확인한다\n"
                 "  - 보정 도구·realsense-viewer 등 카메라를 쓰는 다른 프로그램을 끈다\n"
                 "  - 카메라 없이 확인하려면 --source <이미지/동영상 경로> 또는 --source 0 (웹캠)")
    if args.source == "gazebo":
        print(f"[영상] {source.transport}: {source.topic}")
    worker = Worker(source, detector, mapper, args)
    try:
        App(args, worker, mapper).run()
    finally:
        worker.stop()
        source.close()


if __name__ == "__main__":
    main()
