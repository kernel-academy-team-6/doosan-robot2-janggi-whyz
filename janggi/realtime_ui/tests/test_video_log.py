import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import FONT_PATH
from video_log import BoardRenderer, EvaluationVideos, fit_frame


def test_board_renderer_handles_neutral_and_full_pieces_without_tk():
    renderer = BoardRenderer(FONT_PATH)
    empty = renderer.render({}).copy()
    neutral = renderer.render({(4, 4): ('unknown', 'piece')})
    assert neutral.shape == (570, 516, 3)
    assert not np.array_equal(neutral, empty)
    assert renderer.render({(4, 4): ('unknown', 'piece')}) is neutral
    full = renderer.render({(4, 1): ('red', 'king'), (3, 8): ('blue', 'sa'), (4, 6): ('blue', 'soldier')})
    assert not np.array_equal(full, neutral)


def test_frame_fills_output_without_black_bars():
    frame = np.full((20, 40, 3), 255, np.uint8)
    fitted = fit_frame(frame, (40, 40))
    assert fitted.shape == (40, 40, 3)
    assert np.all(fitted == 255)


def test_both_videos_have_one_frame_per_record_and_fixed_size(tmp_path):
    videos = EvaluationVideos(tmp_path / 'counts_test.csv', 15)
    board = BoardRenderer(FONT_PATH).render({(4, 4): ('unknown', 'piece')})
    for shape in ((48, 32), (32, 32), (48, 64)):
        videos.write(np.full((*shape, 3), 100, np.uint8), board)
    videos.close()
    info = videos.info()
    assert info['frames'] == 3
    assert info['sizes']['overlay'] == (32, 48)  # 원본 전체 프레임이 아니라 실제 overlay 크기
    for name, path in info['files'].items():
        capture = cv2.VideoCapture(path)
        assert capture.isOpened()
        assert capture.get(cv2.CAP_PROP_FPS) == 15
        shapes = []
        while True:
            ok, image = capture.read()
            if not ok:
                break
            shapes.append(image.shape)
            if name == 'overlay':
                assert image.min() > 80  # 저장된 영상 가장자리에도 검정 여백이 없어야 한다.
        capture.release()
        width, height = info['sizes'][name]
        assert shapes == [(height, width, 3)] * 3
