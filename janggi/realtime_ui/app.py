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
from count_log import CountLog  # noqa: E402
from video_log import BoardRenderer, EvaluationVideos  # noqa: E402

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_FAMILY = "Noto Sans CJK KR"
SIDE_RGB = {"blue": (30, 110, 255), "red": (230, 40, 40)}
SIDE_HEX = {"blue": "#1e5fd9", "red": "#d62828"}
SIDE_KO = {"blue": "초", "red": "한"}
FULL_SET = {"king": 1, "cha": 2, "po": 2, "horse": 2, "sang": 2, "sa": 2, "soldier": 5}


def parse_args():
    p = argparse.ArgumentParser(description="장기말 실시간 인식 + 게임 UI")
    p.add_argument("--source", default="realsense", help="realsense | 웹캠 번호 | 동영상/이미지 파일 경로")
    p.add_argument("--playback-fps", type=float, default=0.0,
                   help="동영상 재생 FPS (기본/0: 속도 제한 없음). 카메라·사진에는 적용 안 함")
    model_options = p.add_mutually_exclusive_group()
    model_options.add_argument("--model-mode", "-model-mode", choices=("piece", "full", "hybrid", "cnn"), default=None,
                               help="piece: 위치 전용, full: 기존 14종, hybrid/cnn: 위치 YOLO + 별도 분류 (생략 시 weights.pt)")
    model_options.add_argument("--model", type=Path, default=None, help="모델 파일 경로 직접 지정")
    p.add_argument("--classifier", type=Path, default=HERE.parents[1]/"models/piece_classifier/best.pt",
                   help="hybrid/cnn 모드의 분류 가중치")
    p.add_argument("--classifier-backend", choices=("mobilenet", "ultralytics"), default="mobilenet")
    p.add_argument("--classifier-conf", type=float, default=.5, help="종류 분류 확률 기준; 미달이면 위치만 표시")
    p.add_argument("--classifier-batch", type=int, default=32)
    p.add_argument("--classifier-size", type=int, default=224, help="Ultralytics 분류 입력 크기; MobileNet은 체크포인트 설정 사용")
    p.add_argument("--device", default=None, help="cpu 또는 cuda:0; 생략 시 자동 선택")
    p.add_argument("--box-label", choices=("confidence", "size-px", "size-mm"), default="confidence",
                   help="박스 표시: 신뢰도 또는 가로x세로 크기")
    p.add_argument("--obstruction-motion", type=float, default=.03, help="방해 의심 움직임 면적 비율 (기본 0.03)")
    p.add_argument("--obstruction-window", type=float, default=2., help="검출 수 진동 확인 시간(초)")
    p.add_argument("--obstruction-settle", type=float, default=.8, help="안정 표시 전 대기 시간(초)")
    p.add_argument("--conf", type=float, default=0.35, help="검출 신뢰도 임계값 (UI 슬라이더로 바꿀 수 있다)")
    p.add_argument("--full-frame", action="store_true", help="판 영역만 잘라 인식하지 않고 전체 프레임을 인식")
    p.add_argument("--roi-margin", type=int, default=60, help="판 영역 인식 때 격자 바깥 여유 [px]")
    p.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION,
                   help="네 모서리 픽셀을 읽을 보정 파일 (없으면 판 매핑 없이 검출만)")
    p.add_argument("--piece-height", type=float, default=12.0, help="말 높이 [mm] (시차 보정)")
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
    g.add_argument("--robot-vel", type=float, default=100.0, help="로봇 직선 속도 [mm/s]")
    g.add_argument("--ns", default="dsr01", help="로봇 네임스페이스")
    g.add_argument("--no-motion", action="store_true", help="로봇을 움직이지 않고 엔진 수와 집을 좌표만 표시")
    args = p.parse_args()
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
    if args.expected_pieces <= 0:
        p.error("--expected-pieces는 양수여야 한다")
    return args


# --------------------------------------------------------------------------- 영상 입력

class RealSenseSource:
    def __init__(self, w, h, fps=30):
        import pyrealsense2 as rs

        self.rs, self.size = rs, (w, h, fps)
        self.pipe = None
        self._open()

    def _open(self):
        rs = self.rs
        w, h, fps = self.size
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)   # 인식에는 color만 쓴다
        pipe.start(cfg)
        self.pipe = pipe

    def read(self):
        try:
            if self.pipe is None:
                self._open()
            f = self.pipe.wait_for_frames(1000).get_color_frame()
            return np.asanyarray(f.get_data()).copy() if f else None
        except RuntimeError:
            # USB가 잠깐 끊기면 파이프라인을 다시 연다
            try:
                if self.pipe is not None:
                    self.pipe.stop()
            except RuntimeError:
                pass
            self.pipe = None
            time.sleep(0.5)
            return None

    def close(self):
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
        self.view_status = dict(state='settling', reason='영상 입력 대기 중', motion=0.)
        self.size_estimator = None
        if mapper is not None and args.calibration:
            try:
                self.size_estimator = BoxSizeEstimator.from_calibration(args.calibration, getattr(args, 'piece_height', 12.))
            except (KeyError, ValueError, OSError) as exc:
                print(f"[크기 표시] mm 추정 사용 불가: {exc}")
        self.monitor_context = None
        self.board_polygon = None
        if mapper is not None:
            grid = mapper.grid_pixels()
            self.board_polygon = np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
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
                    confidence_semantics="conf=YOLO detection probability; classifier probabilities in classifications.jsonl")
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
                frame = self.source.read()
                if frame is None:
                    continue
                roi = None
                if self.use_roi and self.mapper is not None:
                    roi = board_crop_box(self.mapper, frame.shape[1], frame.shape[0], self.args.roi_margin)
                dets = self.detector.detect(frame, conf=self.args.conf, roi=roi)
                raw = self.mapper.assign(dets, self.args.max_dist) if self.mapper else {}
                unique_dets = deduplicate_detections(dets)
                context = (getattr(self.source, 'source_loop', None), self.args.conf, self.use_roi)
                if self.obstruction_reset.is_set() or context != self.monitor_context:
                    self.obstruction.reset()
                    self.obstruction_reset.clear()
                    self.monitor_context = context
                view_status = self.obstruction.update(frame, len(unique_dets), len(raw) if self.mapper else None,
                                                      time.monotonic(), self.board_polygon)
                if getattr(self, "classification_log", None) is not None:
                    import json
                    self.classification_log.write(json.dumps(dict(inference_frame=self.frames, frame=getattr(self.source, "source_frame", None),
                        source_time=getattr(self.source, "source_time", None), loop=getattr(self.source, "source_loop", None),
                        detections=[dict(box=d.box, label=d.name, detection_conf=d.conf,
                                         classification_conf=d.classification_conf) for d in dets]))+"\n")
                with self.lock:
                    stable = self.stab.update(raw) if self.mapper else {}
                    self.center_hist.append({d.board: d.center for d in dets if d.board is not None})
                    self.latest = (frame, dets, raw, stable)
                    self.view_status = view_status
                    self.frames += 1
                self.last_frame_t = time.time()
                if self.count_log:
                    crop = board_crop_box(self.mapper, frame.shape[1], frame.shape[0]) if self.video_zoom and self.mapper else None
                    overlay = draw_overlay(frame, dets, self.mapper, video_glyphs, crop, self.video_detail,
                                           self.box_label_mode, self.size_estimator)
                    self.video_log.write(overlay, board_renderer.render(stable))
                    self.count_log.record(len(dets), len(unique_dets),
                                          len(raw) if self.mapper else None, len(stable) if self.mapper else None,
                                          self.args.conf, roi is not None,
                                          getattr(self.source, "source_frame", None),
                                          getattr(self.source, "source_time", None),
                                          getattr(self.source, "source_loop", None),
                                          confidences=[d.conf for d in dets],
                                          deduplicated_confidences=[d.conf for d in unique_dets])
                n += 1
                if time.time() - t0 >= 1.0:
                    self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
        except Exception as e:  # noqa: BLE001
            self.error = repr(e)
        finally:
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

    def get(self):
        with self.lock:
            return self.latest

    def get_obstruction(self):
        with self.lock:
            return dict(self.view_status)

    def fresh_state(self, timeout: float = 10.0):
        """기록을 비우고 새로 들어온 프레임 window개로 (판 상태, 칸별 박스 중심 픽셀)을 만든다.

        로봇 팔이 비킨 뒤나 사람이 손을 뗀 뒤의 판만 보도록 한다 (robot_game vision 인터페이스).
        """
        from session import TurnError, median_centers

        with self.lock:
            self.stab.reset()
            self.center_hist.clear()
            start = self.frames
        t0 = time.time()
        while self.frames - start < self.args.window + 1:   # 비우는 순간 처리 중이던 프레임 하나는 버린다
            if self.error or time.time() - t0 > timeout:
                raise TurnError("영상이 들어오지 않는다 (카메라 확인)")
            time.sleep(0.03)
        with self.lock:
            board = self.stab.state()
            hist = list(self.center_hist)
        if getattr(self.detector, "classifier", None) is not None and any(side == 'unknown' for side, kind in board.values()):
            raise TurnError("종류를 확정하지 못한 말이 있습니다. 분류 결과를 확인하세요.")
        return board, median_centers(board, hist)

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


def draw_overlay(frame, dets, mapper, glyphs, crop=None, detail=False, label_mode='confidence', size_estimator=None):
    """인식 결과를 그린 BGR 이미지. crop=(x0, y0, x1, y1)이면 그 영역만 잘라서 그린다."""
    x0, y0, x1, y1 = crop or (0, 0, frame.shape[1], frame.shape[0])
    img = frame[y0:y1, x0:x1].copy()
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
        font_scale = .34 if label_mode.startswith('size-') else .42
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
        self.box_label_options = {'신뢰도': 'confidence', '크기(px)': 'size-px', '크기(mm)': 'size-mm'}
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
        self.robot_stop = False
        self._mover = None
        self._msgs = deque()           # 작업 스레드 → UI 기보 메시지
        self.robot_phase = "대국 시작 전: 판을 놓고 [대국 시작]"

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
        tk.Checkbutton(row2, text="판 영역만 인식 (작은 말에 유리)", variable=self.roi_var,
                       state=tk.NORMAL if self.mapper else tk.DISABLED,
                       command=lambda: setattr(self.worker, "use_roi", self.roi_var.get())).pack(side="left", padx=10)
        tk.Button(row2, text="원본 프레임 저장", command=self.save_frame).pack(side="right")
        row3 = tk.Frame(left)
        row3.pack(fill="x", padx=6, pady=3)
        tk.Label(row3, text="박스 표시").pack(side="left")
        choice = ttk.Combobox(row3, textvariable=self.box_label_choice, values=list(self.box_label_options), state="readonly", width=10)
        choice.pack(side="left", padx=4)
        choice.bind('<<ComboboxSelected>>', lambda e: setattr(self.worker, 'box_label_mode', self.box_label_options[self.box_label_choice.get()]))
        tk.Label(row3, text="mm는 보정 기반 박스 추정 길이", fg="#666").pack(side="left", padx=4)
        tk.Button(row3, text="방해 기준 다시 잡기", command=self.worker.obstruction_reset.set).pack(side="right")
        self.lbl_obstruction = tk.Label(left, anchor="w", justify="left", width=65, fg="#a06000", text="화면 방해: 확인 중")
        self.lbl_obstruction.pack(fill="x", padx=6, pady=2)
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
            tk.Button(fr, text="대국 시작 (인식된 판)", command=self.robot_start).pack(fill="x", pady=1)
            tk.Button(fr, text="착수 완료 (Space)", command=self.robot_human_done, bg="#2e7d32", fg="white",
                      font=(FONT_FAMILY, 12, "bold")).pack(fill="x", pady=3)
            tk.Button(fr, text="로봇 정지", command=self.robot_halt, bg="#c62828", fg="white").pack(fill="x", pady=1)
            self.lbl_robot = tk.Label(fr, anchor="w", justify="left", wraplength=220, font=(FONT_FAMILY, 10))
            self.lbl_robot.pack(fill="x")
        tk.Label(side, text="기보", anchor="w").pack(fill="x", pady=(8, 0))
        self.log = tk.Listbox(side, height=14, width=26, font=(FONT_FAMILY, 10))
        self.log.pack(fill="both", expand=True)
        self.lbl_count = tk.Label(side, anchor="w", justify="left", font=(FONT_FAMILY, 10))
        self.lbl_count.pack(fill="x", pady=6)

    # ------------------------------------------------------------------ 모드 / 게임 조작
    def on_mode(self):
        if self.mode.get() == "game" and self.game is None:
            self.new_game()
        self.cancel_select()

    def classification_ready(self):
        if any(side == 'unknown' for side, kind in self.stable.values()):
            messagebox.showwarning("종류 미확정", "종류를 확정하지 못한 말이 있습니다. 인식 결과를 확인하세요.", parent=self.root)
            return False
        return True

    def new_game(self):
        if self.position_only:
            self.mode.set("live")
            return
        if not self.stable:
            messagebox.showwarning("새 게임", "인식된 말이 없다. 카메라와 보정 파일을 확인한다.", parent=self.root)
            self.mode.set("live")
            return
        if not self.classification_ready():
            self.mode.set("live")
            return
        board = dict(self.stable)
        self.game = Game(board, top_side=detect_top_side(board), turn=self.first.get())
        self.last_move = None
        self.log.delete(0, "end")
        self.log.insert("end", f"새 게임: 위쪽 {SIDE_KO[self.game.top_side]}, {SIDE_KO[self.game.turn]} 먼저")
        self.mode.set("game")
        self.cancel_select()

    def sync_to_detection(self):
        """게임 진행(차례·기보)은 두고 판만 인식 결과로 덮어쓴다 (인식 오류를 손으로 고친 뒤 등)."""
        if self.game is None:
            return self.new_game()
        if not self.classification_ready():
            return
        self.game.board = dict(self.stable)
        self.log.insert("end", "판을 인식 결과로 맞춤")
        self.cancel_select()

    def read_move(self):
        """실제 판에서 사람이 둔 수를 인식 결과와의 차이로 읽어 게임에 반영한다."""
        if self.game is None:
            return
        if not self.classification_ready():
            return
        mv = infer_move(self.game.board, self.stable, self.game.turn)
        if mv is None:
            diff = self.diff_cells()
            messagebox.showinfo("수 읽기", f"{SIDE_KO[self.game.turn]}의 한 수로 읽을 수 없다 (다른 칸 {len(diff)}개).\n"
                                "말이 가려졌거나 인식이 흔들리면 잠시 뒤 다시 누른다.", parent=self.root)
            return
        try:
            self.play(*mv)
        except ValueError as e:
            messagebox.showwarning("수 읽기", f"규칙상 둘 수 없는 수다: {e}", parent=self.root)

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
    def on_press(self, cell, e):
        if self.mode.get() != "game" or self.game is None or self.rg is not None:
            return
        if self.sel and cell in self.game.legal_moves(self.sel):
            self.play(self.sel, cell)
            self.cancel_select()
            return
        if cell and cell in self.game.board and self.game.board[cell][0] == self.game.turn:
            self.sel = cell
            self.drag_xy = (e.x, e.y)
        else:
            self.cancel_select()

    def on_drag(self, e):
        if self.sel:
            self.drag_xy = (e.x, e.y)

    def on_release(self, cell, e):
        if self.sel and cell and cell != self.sel and cell in self.game.legal_moves(self.sel):
            self.play(self.sel, cell)
            self.cancel_select()
            return
        self.drag_xy = None   # 같은 칸에서 놓으면 선택 유지 → 다음 클릭으로 둔다

    def cancel_select(self):
        self.sel, self.drag_xy = None, None

    def diff_cells(self):
        if self.game is None:
            return set()
        b, s = self.game.board, self.stable
        return {c for c in set(b) | set(s) if b.get(c) != s.get(c)}

    # ------------------------------------------------------------------ 로봇 대국
    def _robot_job(self, phase, fn):
        """fn을 작업 스레드에서 돌린다 (엔진·로봇 동작이 UI를 막지 않게)."""
        if self.robot_busy:
            return
        from session import TurnError

        self.robot_busy, self.robot_stop, self.robot_phase = True, False, phase

        def run():
            try:
                fn()
                self.robot_phase = self._human_phase()
            except TurnError as e:
                self._msgs.append(f"! {e}")
                self.robot_phase = f"다시 확인: {e}"
            except Exception as e:  # noqa: BLE001
                self._msgs.append(f"! 오류: {e}")
                self.robot_phase = f"오류: {e}\n판을 바로잡고 [인식된 판으로 맞추기] 후 [착수 완료]"
            finally:
                self.robot_busy = False

        threading.Thread(target=run, daemon=True).start()

    def _human_phase(self):
        g = self.rg.game if self.rg else None
        if g is None:
            return "대국 시작 전"
        if not self.rg.has_moves(g.turn):
            return f"대국 끝: {SIDE_KO[other(g.turn)]} 승"
        st = g.status()
        if g.turn == self.rg.robot_side:
            return "로봇 차례: [착수 완료]를 누르면 로봇이 둔다"
        return "사람 차례: 판에 두고 [착수 완료]" + (f"  ({st}!)" if st else "")

    def _ensure_robot(self):
        if self.position_only:
            raise RuntimeError("위치 전용 모델은 진영·종류를 분류하지 않아 로봇 대국에 사용할 수 없다")
        if self.rg is not None:
            return
        from session import RobotGame, RobotGameConfig

        from action import BoardGeometry, BoardMover, PickConfig, PiecePicker
        from engine import JanggiEngine
        from piece_detection import CameraToBase

        self.robot_phase = "엔진 시작 중"
        engine = JanggiEngine()
        geom = BoardGeometry.from_calibration(self.args.calibration)
        cam = CameraToBase.from_calibration(self.args.calibration)
        picker = None
        if not self.args.no_motion:
            self.robot_phase = "로봇 드라이버 연결 중"
            v = self.args.robot_vel
            self._mover = BoardMover(self.args.ns, self.args.calibration, vel=(v, 30.0), acc=(v * 2, 60.0))
            picker = PiecePicker(self._mover, PickConfig())
        cfg = RobotGameConfig(human_side=self.args.human_side, movetime_ms=self.args.movetime,
                              piece_height_mm=self.args.piece_height)
        self.rg = RobotGame(engine, cam, geom, picker, cfg, log=self._msgs.append)

    def robot_start(self):
        def job():
            self._ensure_robot()
            self.robot_phase = "판 읽는 중"
            board, _ = self.worker.fresh_state()
            self.robot_phase = "관찰 자세로 이동 중"
            self.game = self.rg.start(board)
            self.last_move = None
        self.log.delete(0, "end")
        self.mode.set("game")
        self._robot_job("대국 준비 중", job)

    def robot_human_done(self):
        """착수 완료: 판 읽기 → 사람 수 반영 → 엔진 → 로봇 → 관찰 자세 → 사람 차례."""
        if self.rg is None or self.rg.game is None:
            messagebox.showinfo("로봇 대국", "먼저 [대국 시작]을 누른다.", parent=self.root)
            return

        def job():
            rg = self.rg
            self.robot_phase = "판 읽는 중"
            board, centers = self.worker.fresh_state()
            if rg.game.turn != rg.robot_side:
                rg.apply_human_move(board)
            self.robot_phase = "엔진 생각 중 / 로봇 두는 중"
            plan = rg.robot_turn(centers, stop_check=lambda: self.robot_stop)
            if plan.src is not None and not plan.is_pass:
                self.last_move = (plan.src, plan.dst)
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
            crop = None
            if self.zoom.get() and self.mapper is not None:
                crop = board_crop_box(self.mapper, frame.shape[1], frame.shape[0])
            img = draw_overlay(frame, dets, self.mapper, self.glyphs, crop, self.detail.get(),
                               self.worker.box_label_mode, self.worker.size_estimator)
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
            self.lbl_obstruction.config(text=f"화면 방해: {title} · {status['reason']} · 움직임 {status['motion']*100:.1f}%", fg=color)
            assigned = sum(1 for d in dets if d.board is not None)
            unique = count_unique_detections(dets)
            self.lbl_cam.config(text=f"인식 {self.worker.fps:4.1f} FPS · 검출 {len(dets)} · 중복 제거 {unique} · 판 할당 {assigned}{stale}")
        while self._msgs:
            self.log.insert("end", self._msgs.popleft())
            self.log.see("end")
        if self.args.robot:
            self.lbl_robot.config(text=("⏳ " if self.robot_busy else "") + self.robot_phase)
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
        if not live and self.sel:
            bv.mark(self.sel, "#6a1b9a", r=28, width=3)
            for c in self.game.legal_moves(self.sel):
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
            f.write(f"# conf>={self.args.conf} roi={self.worker.use_roi}  name detection_conf x1 y1 x2 y2 board classification_conf\n")
            for d in dets:
                f.write(f"{d.name}\t{d.conf:.3f}\t" + " ".join(f"{v:.1f}" for v in d.box) + f"\t{d.board}\t{d.classification_conf}\n")
        self.lbl_cam.config(text=f"저장: {out.name}/{stamp}.png (검출 {len(dets)})")
        print(f"[저장] {out / stamp}.png")

    def quit(self):
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
    mapper = None
    if args.calibration and Path(args.calibration).exists():
        mapper = BoardMapper.from_calibration(args.calibration, piece_height_mm=args.piece_height)
    else:
        print(f"[경고] 보정 파일 없음: {args.calibration} — 판 매핑 없이 검출만 한다")
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
        source = RealSenseSource(args.width, args.height) if args.source == "realsense" else \
            CvSource(args.source, args.width, args.height, args.playback_fps)
    except RuntimeError as e:
        sys.exit(f"[오류] 영상 입력을 열 수 없다 ({args.source}): {e}\n"
                 "  - RealSense: USB 3 포트에 연결됐는지 `lsusb | grep -i realsense`로 확인한다\n"
                 "  - 보정 도구·realsense-viewer 등 카메라를 쓰는 다른 프로그램을 끈다\n"
                 "  - 카메라 없이 확인하려면 --source <이미지/동영상 경로> 또는 --source 0 (웹캠)")
    worker = Worker(source, detector, mapper, args)
    try:
        App(args, worker, mapper).run()
    finally:
        worker.stop()
        source.close()


if __name__ == "__main__":
    main()
