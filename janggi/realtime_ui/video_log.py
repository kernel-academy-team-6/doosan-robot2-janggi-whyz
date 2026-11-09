"""평가 로그와 프레임을 맞춘 오버레이/인식판 동영상 저장. 화면 캡처나 Tk에 의존하지 않는다."""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from piece_detection.labels import piece_char


class BoardRenderer:
    CELL, MARGIN = 54, 42
    SIZE = (516, 570)

    def __init__(self, font_path):
        self.font_path = font_path
        self.fonts = {size: ImageFont.truetype(font_path, pixels)
                      for size, pixels in ((12, 12), (15, 20), (18, 24), (21, 28))}
        self.signature = self.cached = None
        self.grid = Image.new('RGB', self.SIZE, '#e8c68d')
        d = ImageDraw.Draw(self.grid)
        for x in range(9):
            d.line([self.xy(x, 0), self.xy(x, 9)], fill='#5a3a12')
            d.text((self.xy(x, 0)[0], 16), str(x), fill='#5a3a12', font=self.fonts[12], anchor='mm')
        for y in range(10):
            d.line([self.xy(0, y), self.xy(8, y)], fill='#5a3a12')
            d.text((16, self.xy(0, y)[1]), str(y), fill='#5a3a12', font=self.fonts[12], anchor='mm')
        for y in (1, 8):
            d.line([self.xy(3, y-1), self.xy(5, y+1)], fill='#5a3a12')
            d.line([self.xy(5, y-1), self.xy(3, y+1)], fill='#5a3a12')

    def xy(self, x, y):
        return self.MARGIN + x * self.CELL, self.MARGIN + y * self.CELL

    def render(self, board):
        signature = tuple(sorted(board.items()))
        if signature != self.signature:
            image = self.grid.copy()
            d = ImageDraw.Draw(image)
            for (x, y), (side, kind) in board.items():
                cx, cy = self.xy(x, y)
                r = {'king': 25, 'sa': 18, 'soldier': 18}.get(kind, 22)
                points = [(cx + r*np.cos(np.pi/8 + k*np.pi/4), cy + r*np.sin(np.pi/8 + k*np.pi/4))
                          for k in range(8)]
                color = {'blue': '#1e5fd9', 'red': '#d62828'}.get(side, '#00897b')
                d.polygon(points, fill='#f8ecd2', outline=color, width=2)
                d.text((cx, cy), piece_char(side, kind), fill=color,
                       font=self.fonts[int(r*.85)], anchor='mm')
            self.cached = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
            self.signature = signature
        return self.cached


def fit_frame(frame, size):
    """종횡비를 유지한 채 고정 크기를 채운다. 비율이 바뀌면 중앙을 잘라 여백을 만들지 않는다."""
    width, height = size
    h, w = frame.shape[:2]
    scale = max(width / w, height / h)
    resized = cv2.resize(frame, (max(width, round(w*scale)), max(height, round(h*scale))))
    rh, rw = resized.shape[:2]
    x, y = (rw-width)//2, (rh-height)//2
    return np.ascontiguousarray(resized[y:y+height, x:x+width])


class EvaluationVideos:
    def __init__(self, csv_path, fps):
        directory = Path(csv_path).parent
        self.paths = {name: directory / f'{name}.mp4' for name in ('overlay', 'board')}
        self.fps, self.frames = fps, 0
        self.writers, self.sizes = {}, {}

    def write(self, overlay, board):
        if not self.writers:
            overlay_size = (overlay.shape[1], overlay.shape[0])
            self.sizes = dict(overlay=tuple((int(v)+1)//2*2 for v in overlay_size), board=BoardRenderer.SIZE)
            try:
                for name, path in self.paths.items():
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), self.fps, self.sizes[name])
                    self.writers[name] = writer
                    if not writer.isOpened():
                        raise RuntimeError(f'평가 영상 저장을 시작할 수 없다: {path}')
            except Exception:
                self.close()
                raise
        self.writers['overlay'].write(fit_frame(overlay, self.sizes['overlay']))
        self.writers['board'].write(board)
        self.frames += 1

    def info(self):
        return dict(files={name: str(p) for name, p in self.paths.items()}, fps=self.fps,
                    frames=self.frames, sizes=self.sizes, board='stabilized live recognition board',
                    overlay_sizing='first overlay size; aspect-preserving center crop if dimensions change; no padding',
                    alignment='video frame 0 corresponds to CSV frame 1; one frame per inference')

    def close(self):
        for writer in self.writers.values():
            writer.release()
