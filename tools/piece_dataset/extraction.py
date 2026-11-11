"""영상 프레임과 EXIF 회전을 반영한 사진에서 YOLO Crop을 추출한다."""
import hashlib
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np

from .core import Catalog, Cancelled, file_hash, timestamp

ROOT = Path(__file__).resolve().parents[2]


def extract_video(project, video, interval=2., confidence=.35, margin=0., group=None, split='train',
                  model=None, device=None, limit=None, cancel=None, progress=None, detector=None):
    if not math.isfinite(interval) or interval <= 0 or not 0 <= margin <= 1 or not 0 < confidence < 1:
        raise ValueError('간격은 양수, 여백은 0~1, 신뢰도는 0~1 사이로 지정하세요.')
    import sys
    if str(ROOT/'janggi') not in sys.path:
        sys.path.insert(0, str(ROOT/'janggi'))
    from piece_detection.detector import PieceDetector
    video = Path(video).expanduser().resolve()
    model = Path(model or ROOT/'models/weights_only_piece.pt').resolve()
    digest, model_digest = file_hash(video), file_hash(model)
    cap = cv2.VideoCapture(str(video))
    catalog = None
    if not cap.isOpened():
        cap.release()
        raise ValueError(f'영상을 열 수 없습니다: {video}')
    added = sampled = 0
    try:
        catalog = Catalog(project)
        fps, total = cap.get(cv2.CAP_PROP_FPS), cap.get(cv2.CAP_PROP_FRAME_COUNT)
        source = catalog.register_source(video, group or video.stem, split, digest,
                                         dict(fps=fps, frames=total, registered_at=timestamp()))
        settings = dict(interval_s=interval, confidence=confidence, margin=margin, model=str(model),
                        model_sha256=model_digest, dedup_iou=.5, roi='full frame')
        detector = detector or PieceDetector(model, conf=confidence, device=device)
        settings.update(imgsz=getattr(detector, 'imgsz', 640), nms_iou=getattr(detector, 'iou', .5),
                        classes=getattr(detector, 'names', []))
        next_time, index, previous_time = 0., 0, -1.
        while True:
            if cancel is not None and cancel.is_set():
                raise Cancelled(f'추출을 중단했습니다. 저장된 {added}개와 기존 정답은 유지합니다.')
            ok, frame = cap.read()
            if not ok:
                break
            time_s = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            timestamp_mode = 'video_pos_msec'
            if not math.isfinite(time_s) or (index > 0 and time_s <= previous_time):
                if not math.isfinite(fps) or fps <= 0:
                    raise ValueError('프레임 시각과 FPS를 읽을 수 없습니다. 고정 FPS의 MP4로 변환해 다시 시도하세요.')
                time_s, timestamp_mode = index/fps, 'index_over_fps'
            previous_time = time_s
            if time_s+1e-6 >= next_time:
                if shutil.disk_usage(catalog.root).free < 1024**3:
                    raise OSError('남은 공간이 1 GB 미만이라 추출을 중단했습니다.')
                added += _extract_frame(catalog, frame, digest, model_digest, index, time_s,
                                        settings, detector, confidence, margin, cancel, timestamp_mode)
                sampled += 1
                next_time = (math.floor(time_s/interval)+1)*interval
                if limit is not None and sampled >= limit:
                    break
            index += 1
            if progress and index % 15 == 0:
                progress(index, int(total) if total > 0 else max(index, 1))
        return dict(added=added, sampled_frames=sampled, source=source['id'])
    finally:
        cap.release()
        if catalog:
            catalog.close()


def _extract_frame(catalog, frame, digest, model_digest, index, time_s, settings,
                   detector, confidence, margin, cancel, timestamp_mode):
    from piece_detection.detector import deduplicate_detections
    height, width = frame.shape[:2]
    detections = deduplicate_detections(detector.detect(frame, conf=confidence))
    added = 0
    frame_path = Path('frames')/digest/f'{index:09}.png'
    (catalog.root/frame_path).parent.mkdir(parents=True, exist_ok=True)
    if not (catalog.root/frame_path).exists() and not cv2.imwrite(str(catalog.root/frame_path), frame):
        raise OSError('원본 프레임 저장 실패')
    for detection in sorted(detections, key=lambda d: (d.box[1], d.box[0])):
        if cancel is not None and cancel.is_set():
            raise Cancelled(f'추출을 중단했습니다. 저장된 {added}개는 유지합니다.')
        x1, y1, x2, y2 = detection.box
        if x2 <= x1 or y2 <= y1:
            continue
        dx, dy = (x2-x1)*margin, (y2-y1)*margin
        requested = [math.floor(x1-dx), math.floor(y1-dy), math.ceil(x2+dx), math.ceil(y2+dy)]
        crop_box = [max(0, requested[0]), max(0, requested[1]), min(width, requested[2]), min(height, requested[3])]
        a, b, c, d = crop_box
        if c <= a or d <= b:
            continue
        crop = frame[b:d, a:c].copy()
        fingerprint = json.dumps([digest, index, model_digest, list(detection.box), crop_box], sort_keys=True)
        sample_id = hashlib.sha256(fingerprint.encode()).hexdigest()[:32]
        crop_path = Path('crops/raw')/(sample_id+'.png')
        (catalog.root/crop_path).parent.mkdir(parents=True, exist_ok=True)
        if not (catalog.root/crop_path).exists() and not cv2.imwrite(str(catalog.root/crop_path), crop):
            raise OSError('Crop 저장 실패')
        added += catalog.add_sample(dict(id=sample_id, source_id=digest, frame_index=index, time_s=time_s,
            box=json.dumps(list(detection.box)), crop_box=json.dumps(crop_box), confidence=float(detection.conf),
            frame_path=str(frame_path), crop_path=str(crop_path),
            crop_hash=hashlib.sha256(str(crop.shape).encode()+crop.tobytes()).hexdigest(),
            blur=float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()),
            clipped=int(crop_box != requested), settings=json.dumps({**settings, 'timestamp_mode': timestamp_mode})))
    return added


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'}


def image_paths(paths):
    """Validate before processing; retain order, ignore repeated selections of the same path."""
    result = list(dict.fromkeys(Path(p).expanduser().resolve() for p in paths))
    if not result:
        raise ValueError('사진을 한 장 이상 선택하세요.')
    for path in result:
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTENSIONS:
            raise ValueError(f'지원하는 사진 파일이 아닙니다: {path}')
    return result


def extract_images(project, paths, confidence=.35, margin=0., group=None, model=None,
                   device=None, cancel=None, progress=None, detector=None):
    """Each EXIF-oriented photo becomes frame zero of an independently hashed source."""
    import sys
    from PIL import Image, ImageOps
    if not 0 <= margin <= 1 or not 0 < confidence < 1:
        raise ValueError('여백은 0~1, 신뢰도는 0~1 사이로 지정하세요.')
    paths = image_paths(paths)
    if str(ROOT/'janggi') not in sys.path:
        sys.path.insert(0, str(ROOT/'janggi'))
    from piece_detection.detector import PieceDetector
    model = Path(model or ROOT/'models/weights_only_piece.pt').resolve()
    model_digest = file_hash(model)
    detector = detector or PieceDetector(model, conf=confidence, device=device)
    settings = dict(input_type='image', confidence=confidence, margin=margin, model=str(model),
                    model_sha256=model_digest, dedup_iou=.5, roi='full frame',
                    exif_orientation='transposed', imgsz=getattr(detector, 'imgsz', 640),
                    nms_iou=getattr(detector, 'iou', .5), classes=getattr(detector, 'names', []))
    added = processed = 0
    errors = []
    catalog = Catalog(project)
    try:
        for path in paths:
            if cancel is not None and cancel.is_set():
                raise Cancelled(f'추출을 중단했습니다. 저장된 {added}개와 기존 정답은 유지합니다.')
            if shutil.disk_usage(catalog.root).free < 1024**3:
                raise OSError('남은 공간이 1 GB 미만이라 추출을 중단했습니다.')
            try:
                with Image.open(path) as image:
                    frame = cv2.cvtColor(np.array(ImageOps.exif_transpose(image).convert('RGB')), cv2.COLOR_RGB2BGR)
            except (OSError, ValueError) as exc:
                errors.append(f'{path.name}: {exc}')
                if progress:
                    progress(processed + len(errors), len(paths))
                continue
            digest = file_hash(path)
            catalog.register_source(path, group or path.parent.name or 'photos', 'train', digest,
                                    dict(input_type='image', width=frame.shape[1], height=frame.shape[0],
                                         exif_orientation='transposed', registered_at=timestamp()))
            added += _extract_frame(catalog, frame, digest, model_digest, 0, 0., settings,
                                    detector, confidence, margin, cancel, 'still_image')
            processed += 1
            if progress:
                progress(processed + len(errors), len(paths))
        return dict(added=added, processed_images=processed, errors=errors)
    finally:
        catalog.close()
