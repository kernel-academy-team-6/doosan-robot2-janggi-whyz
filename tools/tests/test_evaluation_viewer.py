import csv
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluation_viewer import Dashboard, Session, ViewOptions, export_video, graph_bounds, value_bounds, parse_args


@pytest.fixture
def session_folder(tmp_path):
    keys = ['frame', 'expected', 'deduplicated', 'assigned', 'source_frame', 'source_loop',
            'dedup_conf_mean', 'dedup_conf_min', 'dedup_conf_bottom30_mean']
    with (tmp_path / 'counts.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for i in range(8):
            writer.writerow(dict(frame=i+1, expected=32, deduplicated=32-i%2, assigned=31,
                                 source_frame=i%4, source_loop=i//4, dedup_conf_mean=.9,
                                 dedup_conf_min=.5, dedup_conf_bottom30_mean=.7))
    for name in ('overlay', 'board'):
        writer = cv2.VideoWriter(str(tmp_path / f'{name}.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10, (64, 48))
        assert writer.isOpened()
        for i in range(8):
            writer.write(np.full((48, 64, 3), i*25, dtype=np.uint8))
        writer.release()
    return tmp_path


def test_stationary_page_and_follow_windows():
    assert graph_bounds(19, 20, 'page') == (0, 20)
    assert graph_bounds(20, 20, 'page') == (20, 40)
    assert graph_bounds(30, 20, 'follow') == (15, 35)


def test_defaults_and_full_log_vertical_range(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['evaluation_viewer.py'])
    args = parse_args()
    assert args.window == ViewOptions().window == 10
    assert args.mode == ViewOptions().mode == 'follow'
    assert value_bounds(np.array([np.nan, 28., 32., 29.])) == (28., 32.)
    assert value_bounds(np.array([75., 81., 77.]), percent=True) == (75., 81.)
    low, high = value_bounds(np.array([90., 90.]), percent=True)
    assert low < 90 < high
    assert value_bounds(np.array([np.nan]), percent=True) == (0., 100.)


def test_video_and_data_follow_same_frame_and_render_without_display(session_folder):
    session = Session(session_folder)
    try:
        assert session.length == 8 and session.fps == 10
        assert session.rows[4]['source_loop'] == '1'
        assert np.allclose(session.scores('mean', 'deduplicated'), .9)
        assert np.isnan(session.scores('mean', 'raw')).all()
        session.videos['overlay'].read(6)
        assert np.asarray(session.videos['overlay'].read(1)).mean() < 30
        dashboard = Dashboard()
        frame = dashboard.render(session, 4, ViewOptions())
        assert frame.size == (1920, 1080)
        assert np.asarray(frame).std() > 10
    finally:
        session.close()


def test_export_frame_range_fps_and_safe_cancellation(session_folder, tmp_path):
    output = tmp_path / 'report.mp4'
    export_video(session_folder, output, ViewOptions(), start=1, end=4, speed=.5)
    capture = cv2.VideoCapture(str(output))
    assert capture.isOpened()
    assert capture.get(cv2.CAP_PROP_FPS) == 5
    frames = []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        frames.append(image.shape)
    capture.release()
    assert frames == [(1080, 1920, 3)] * 3
    with pytest.raises(FileExistsError):
        export_video(session_folder, output, ViewOptions())
    with pytest.raises(ValueError, match='입력 영상'):
        export_video(session_folder, session_folder / 'overlay.mp4', ViewOptions(), overwrite=True)
    cancelled = tmp_path / 'cancelled.mp4'
    event = threading.Event()
    event.set()
    assert export_video(session_folder, cancelled, ViewOptions(), cancel=event) is None
    assert not cancelled.exists()
    assert not list(tmp_path.glob('*.partial.mp4'))
