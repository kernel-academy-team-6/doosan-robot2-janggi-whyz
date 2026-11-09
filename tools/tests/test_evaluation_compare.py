import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluation_compare import ComparisonDashboard, ComparisonSession, ComparisonViewer, preview_size
from evaluation_viewer import ViewOptions, export_video
from test_evaluation_viewer import session_folder


@pytest.fixture
def comparison_paths(session_folder):
    other = session_folder / 'second'
    other.mkdir()
    for name in ('overlay.mp4', 'board.mp4'):
        writer = cv2.VideoWriter(str(other/name), cv2.VideoWriter_fourcc(*'mp4v'), 5, (64, 48))
        for i in range(8):
            writer.write(np.full((48, 64, 3), 100+i, np.uint8))
        writer.release()
    with (session_folder/'counts.csv').open() as f:
        reader = csv.DictReader(f)
        keys, rows = reader.fieldnames, list(reader)
    for i, row in enumerate(rows):
        row['deduplicated'] = str(20+i if i < 4 else 100+i)
        row['dedup_conf_mean'] = '.6'
    with (other/'counts.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    return [session_folder, other]


def test_common_ranges_include_unchecked_logs_and_time_alignment(comparison_paths):
    group = ComparisonSession(comparison_paths)
    try:
        assert group.fps == 10 and group.length == 8
        assert group.bounds(0, 'deduplicated') == (20., 32.)
        group.selected = {0}
        assert group.bounds(0, 'deduplicated') == (20., 32.)
        assert group.bounds(2, 'deduplicated') == (60., 90.)
        assert group.sample_index(0, 6) == 6
        assert group.sample_index(1, 6) == 3
        assert group.sample_index(0, 8) is None
        assert group.sample_index(1, 8) is None  # 긴 결과도 공통 구간 뒤에는 사용하지 않는다.
        assert group.sample_index(1, 7) == 3
        assert len(group.values(1, 0, 'deduplicated')) == 4
        group.axis = 'frame'
        assert group.length == 8
        assert group.sample_index(1, 6) == 6
        assert group.bounds(0, 'deduplicated') == (20., 107.)
        with pytest.raises(ValueError, match='이미 열려'):
            group.add(comparison_paths[0])
    finally:
        group.close()


def test_two_four_video_rendering_and_export_snapshot(comparison_paths):
    group = ComparisonSession(comparison_paths)
    try:
        renderer = ComparisonDashboard()
        for count in (2, 4):
            group.video_count = count
            group.slots = [(0, 'overlay'), (1, 'overlay'), (0, 'board'), (1, 'board')]
            assert renderer.render(group, 6, ViewOptions()).size == ((2366 if count == 4 else 1920), 1080)
        viewer = ComparisonViewer.__new__(ComparisonViewer)
        viewer.session = group
        paths, kwargs = viewer.export_context()
        group.selected.clear()
        clone = kwargs['session_factory'](paths)
        try:
            assert clone.selected == {0, 1}
            assert clone.video_count == 4
            assert clone.slots[1] == (1, 'overlay')
        finally:
            clone.close()
    finally:
        group.close()


def test_comparison_video_export(comparison_paths, tmp_path):
    output = tmp_path/'comparison.mp4'
    export_video(comparison_paths, output, ViewOptions(),
                 session_factory=lambda paths: ComparisonSession(paths, video_count=4),
                 dashboard_factory=ComparisonDashboard)
    capture = cv2.VideoCapture(str(output))
    assert capture.isOpened()
    assert capture.get(cv2.CAP_PROP_FPS) == 10
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 8
    ok, frame = capture.read()
    capture.release()
    assert ok and frame.shape == (1080, 2366, 3)


def test_four_video_layout_preserves_box_and_preview_scale():
    two = ComparisonDashboard.video_tiles(2)
    four = ComparisonDashboard.video_tiles(4)
    assert {side for _, _, side in two+four} == {445}
    assert four[1][0] > four[0][0] and four[1][1] == four[0][1]
    assert four[0][0]+four[0][2] == four[1][0]
    assert two[0][1]+two[0][2] == two[1][1]
    assert ComparisonDashboard.graph_left(4) == four[1][0]+four[1][2]
    assert ComparisonDashboard.graph_left(2) == two[0][0]+two[0][2]
    assert preview_size(1920, 1280, 720) == (1280, 720)
    assert preview_size(2366, 1280, 720) == (1577, 720)


def test_short_csv_limits_videos_and_graphs_at_both_axes(comparison_paths):
    csv_path = comparison_paths[1]/'counts.csv'
    lines = csv_path.read_text().splitlines()
    csv_path.write_text('\n'.join(lines[:4])+'\n')  # CSV는 3행만 남기고 영상은 8프레임을 유지한다.
    group = ComparisonSession(comparison_paths)
    try:
        assert group.length == 6  # 3 frames / 5 FPS = .6s, common 10 FPS
        assert group.sample_index(1, 5) == 2  # 마지막 프레임은 그 프레임의 표시 시간까지 유지
        assert group.sample_index(0, 6) is None
        assert len(group.values(0, 0, 'deduplicated')) == 6
        assert group.warning
        group.axis = 'frame'
        assert group.length == 3
        assert len(group.values(0, 0, 'deduplicated')) == 3
    finally:
        group.close()
