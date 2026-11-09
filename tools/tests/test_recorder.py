"""카메라 없이 실제 저장 파일과 오류 시 녹화 마무리를 검증한다."""
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import realsense_recorder as recorder


INFO = dict(color_number=10, color_timestamp_ms=100.0, depth_number=12,
            depth_timestamp_ms=101.0, timestamp_domain="hardware_clock")


@pytest.mark.parametrize("extension", [".db3", ".bag"])
def test_recording_uses_sdk_required_extension(tmp_path, extension):
    attempted = []

    def open_camera(path):
        attempted.append(path)
        if path.suffix != extension:
            raise RuntimeError(f"Output file must have {extension} extension: '{path}'")

    path = recorder.start_recording_stream(tmp_path, open_camera)
    assert path.name == "rgbd" + extension
    assert len(attempted) == (1 if extension == ".db3" else 2)
    recording = recorder.Recording(tmp_path, (64, 48), 30, {}, recording_file=path.name)
    recording.finish()
    assert json.loads((tmp_path / "metadata.json").read_text())["recording_file"] == path.name


def test_recording_does_not_retry_unrelated_camera_error(tmp_path):
    attempted = []

    def open_camera(path):
        attempted.append(path)
        raise RuntimeError("USB disconnected")

    with pytest.raises(RuntimeError, match="USB disconnected"):
        recorder.start_recording_stream(tmp_path, open_camera)
    assert len(attempted) == 1


def test_snapshot_saves_only_pngs_and_keeps_color_depth_and_units(tmp_path):
    color = np.zeros((48, 64, 3), np.uint8)
    color[:, :, 2] = 201
    depth = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64)
    metadata = {"depth_scale_m": 0.001, "color_intrinsics": {"width": 64}}
    paths = [recorder.save_snapshot(tmp_path, color, depth, metadata, INFO) for _ in range(2)]
    assert paths[0] != paths[1]
    for path in paths:
        assert path.parent == tmp_path / "photos"
        assert np.array_equal(cv2.imread(str(path)), color)
        depth_path = path.with_name(path.name.replace("color_", "depth_", 1))
        loaded = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        assert loaded.dtype == np.uint16 and np.array_equal(loaded, depth)
        with recorder.Image.open(depth_path) as image:
            meta = json.loads(image.info["capture_metadata"])
        assert meta["depth_scale_m"] == 0.001 and meta["frame"] == INFO
    assert len(list((tmp_path / "photos").iterdir())) == 4
    assert all(p.is_file() and p.suffix == ".png" for p in (tmp_path / "photos").iterdir())


def test_mp4_csv_and_final_metadata_are_readable(tmp_path, monkeypatch):
    monkeypatch.setattr(recorder, "CALIBRATION", tmp_path / "absent.yaml")
    recording = recorder.Recording(tmp_path, (64, 48), 30, {"depth_scale_m": 0.001})
    for i in range(8):
        recording.append(np.full((48, 64, 3), i * 20, np.uint8), {**INFO, "color_number": i})
    recording.finish()
    capture = cv2.VideoCapture(str(tmp_path / "color.mp4"))
    shapes = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        shapes.append(frame.shape)
    capture.release()
    assert shapes == [(48, 64, 3)] * 8
    with (tmp_path / "frames.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert [int(r["color_number"]) for r in rows] == list(range(8))
    meta = json.loads((tmp_path / "metadata.json").read_text())
    assert meta["frames"] == 8 and meta["status"] == "complete"


def test_camera_stop_failure_still_finalizes_mp4(tmp_path):
    class FailedPipe:
        def stop(self):
            raise RuntimeError("USB disconnected")

    worker = recorder.CameraWorker(None)
    worker.pipe = FailedPipe()
    worker.recording = recorder.Recording(tmp_path, (64, 48), 30, {})
    worker.recording.append(np.zeros((48, 64, 3), np.uint8), INFO)
    with pytest.raises(RuntimeError, match="USB disconnected"):
        worker.stop_recording(restart=False)
    metadata = json.loads((tmp_path / "metadata.json").read_text())
    assert metadata["status"] == "interrupted" and metadata["frames"] == 1
    assert worker.recording is None and worker.pipe is None
    capture = cv2.VideoCapture(str(tmp_path / "color.mp4"))
    ok, _ = capture.read()
    capture.release()
    assert ok
