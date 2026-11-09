#!/usr/bin/env python3
"""평가 폴더의 두 영상과 5개 지표를 동기화해 확인하고 분석 영상으로 내보낸다."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
import uuid
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
BG, PANEL, GRID, TEXT, MUTED = '#101824', '#192535', '#334254', '#edf4fc', '#9badbf'
COLORS = ['#43d6c5', '#65aaff', '#f6cc6a', '#f28f9d', '#bc9cf5']


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else math.nan
    except (TypeError, ValueError):
        return math.nan


def clock(seconds):
    return f'{int(seconds)//60:02}:{seconds % 60:05.2f}'


def graph_bounds(position, window, mode):
    start = math.floor(position / window) * window if mode == 'page' else max(0., position - .75*window)
    return start, start + window


def value_bounds(values, percent=False):
    """전체 로그 범위로 축을 고정한다. 상수/미기록 지표만 표시 가능한 여유를 둔다."""
    finite = values[np.isfinite(values)]
    if not len(finite):
        return (0., 100.) if percent else (0., 1.)
    low, high = float(finite.min()), float(finite.max())
    if low == high:
        low, high = low-.5, high+.5
        low = max(0., low)
        if percent:
            high = min(100., high)
    return low, high


@dataclass
class ViewOptions:
    axis: str = 'time'
    window: float = 10.
    mode: str = 'follow'
    confidence: str = 'deduplicated'


class Video:
    def __init__(self, path):
        self.path = path
        self.capture = cv2.VideoCapture(str(path))
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f'영상을 열 수 없습니다: {path}')
        self.frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.index, self.image = -1, None

    def read(self, index):
        if index == self.index and self.image is not None:
            return self.image
        if index != self.index + 1:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError(f'프레임 {index}을 읽을 수 없습니다: {self.path.name}')
        self.index, self.image = index, Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        return self.image

    def close(self):
        self.capture.release()


class Session:
    def __init__(self, path):
        path = Path(path).expanduser().resolve()
        if path.is_dir():
            csvs = sorted(path.glob('*.csv'))
            if len(csvs) != 1:
                raise ValueError('counts.csv와 두 영상이 들어 있는 실행별 폴더를 선택하세요.')
            path = csvs[0]
        self.path = path
        with path.open(newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            self.columns = reader.fieldnames or []
            self.rows = list(reader)
        if not self.rows:
            raise ValueError('CSV에 기록된 프레임이 없습니다.')
        if not {'deduplicated', 'assigned'}.issubset(self.columns):
            raise ValueError('이 도구는 인식 개수 로그 CSV가 필요합니다.')
        summary_path = path.with_suffix('.summary.json')
        self.summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        self.videos = {}
        try:
            for name in ('overlay', 'board'):
                video_path = path.parent / f'{name}.mp4'
                if not video_path.exists():  # 이전의 평면 폴더 저장 형식도 지원
                    video_path = path.with_name(path.stem + f'_{name}.mp4')
                self.videos[name] = Video(video_path)
            self.fps = self.videos['overlay'].fps
            if not math.isfinite(self.fps) or self.fps <= 0:
                raise ValueError('overlay 영상의 FPS를 읽을 수 없습니다.')
            board_fps = self.videos['board'].fps
            if not math.isclose(board_fps, self.fps, rel_tol=.001):
                raise ValueError('두 영상의 FPS가 달라 프레임 동기화를 할 수 없습니다.')
            lengths = [len(self.rows), *[v.frames for v in self.videos.values()]]
            self.length = min(lengths)
            if self.length <= 0:
                raise ValueError('재생할 수 있는 영상 프레임이 없습니다. 기록 프로그램을 정상 종료한 파일인지 확인하세요.')
            self.warning = '' if len(set(lengths)) == 1 else f'길이 불일치: CSV/overlay/board={lengths}; 공통 {self.length}프레임만 사용'
        except Exception:
            self.close()
            raise
        self.rows = self.rows[:self.length]
        self.data = {key: np.array([number(row.get(key)) for row in self.rows]) for key in self.columns}
        expected = number(self.rows[0].get('expected'))
        self.expected = expected if math.isfinite(expected) else 32.

    def scores(self, suffix, basis):
        prefix = 'dedup_conf_' if basis == 'deduplicated' else 'conf_'
        key = prefix + suffix
        return self.data.get(key, np.full(self.length, np.nan))

    def close(self):
        for video in self.videos.values():
            video.close()


class Dashboard:
    WIDTH, HEIGHT = 1920, 1080
    VIDEO_SIDE = 445

    def __init__(self):
        self.fonts = {size: ImageFont.truetype(FONT, size) for size in (16, 18, 20, 22, 24, 28, 34)}

    def text(self, draw, xy, text, size=20, color=TEXT, anchor=None):
        draw.text(xy, text, font=self.fonts[size], fill=color, anchor=anchor)

    def output_size(self, session):
        return self.WIDTH, self.HEIGHT

    def render(self, session, index, options):
        image = Image.new('RGB', (self.WIDTH, self.HEIGHT), BG)
        d = ImageDraw.Draw(image)
        seconds = index / session.fps
        x = np.arange(session.length, dtype=float)
        x = x/session.fps if options.axis == 'time' else x+1
        current = float(x[index])
        start, end = graph_bounds(current, options.window, options.mode)
        mask = (x >= start) & (x <= end)
        indices = np.flatnonzero(mask)
        self.text(d, (24, 15), '장기 비전 · 동기화 분석', 28)
        model = Path(session.summary.get('metadata', {}).get('model', '')).name or '모델 미기록'
        self.text(d, (1892, 18), f'{session.path.parent.name}  |  {model}', 18, MUTED, 'ra')
        source = session.rows[index]
        source_frame = source.get('source_frame') or '—'
        loop = source.get('source_loop') or '—'
        status = f'현재 {index+1:,} / {session.length:,} 프레임   ·   {clock(seconds)}   ·   원본 프레임 {source_frame}   ·   반복 {loop}'
        self.text(d, (24, 55), status, 20, MUTED)
        for y, name, title in [(100, 'overlay', '검출 오버레이'), (565, 'board', '교차점 인식판')]:
            side = self.VIDEO_SIDE
            left = 20 + (600-side)//2
            d.rounded_rectangle((left, y, left+side, y+side), radius=12, fill=PANEL)
            self.text(d, (left+16, y+10), title, 22)
            frame = session.videos[name].read(index).copy()
            area_w, area_h = side-28, side-56
            frame.thumbnail((area_w, area_h), Image.Resampling.LANCZOS)
            image.paste(frame, (left+14+(area_w-frame.width)//2, y+49+(area_h-frame.height)//2))
        metrics = [
            ('중복 제거 후 말 개수', session.data['deduplicated'], False),
            ('교차점에 할당된 말 개수', session.data['assigned'], False),
            ('평균 신뢰도', session.scores('mean', options.confidence) * 100, True),
            ('하위 30% 평균 신뢰도', session.scores('bottom30_mean', options.confidence) * 100, True),
            ('최저 신뢰도', session.scores('min', options.confidence) * 100, True),
        ]
        for row, (title, values, percent) in enumerate(metrics):
            top = 100 + row*184
            color = COLORS[row]
            d.rounded_rectangle((642, top, 1900, top+174), radius=12, fill=PANEL)
            self.text(d, (660, top+8), title, 22, color)
            value = values[index]
            label = '미기록' if not np.isfinite(value) else (f'{value:.1f}%' if percent else f'{value:g}개')
            self.text(d, (1880, top+3), label, 28, color, 'ra')
            visible = values[indices]
            finite = visible[np.isfinite(visible)]
            if len(finite):
                unit = '%' if percent else '개'
                stats = f'구간 최소 {finite.min():.1f}{unit} · 평균 {finite.mean():.1f}{unit} · 최대 {finite.max():.1f}{unit}'
                self.text(d, (1030, top+12), stats, 16, MUTED)
            left, right, upper, lower = 698, 1878, top+50, top+142
            minimum, maximum = value_bounds(values, percent)

            def px(v):
                return left + (float(v)-start)/(end-start)*(right-left)

            def py(v):
                return lower - (min(max(float(v), minimum), maximum)-minimum)/(maximum-minimum)*(lower-upper)

            for tick in (minimum, (minimum+maximum)/2, maximum):
                yy = py(tick)
                d.line((left, yy, right, yy), fill=GRID)
                decimals = 3 if maximum-minimum < .01 else 2 if maximum-minimum < 1 else 1
                label = f'{tick:.{decimals}f}' if percent else f'{tick:g}'
                self.text(d, (left-9, yy), label, 16, MUTED, 'rm')
            for tick in np.linspace(start, end, 5):
                xx = px(tick)
                d.line((xx, upper, xx, lower), fill=GRID)
                tick_label = f'{tick:.1f}s' if options.axis == 'time' else f'{tick:.0f}'
                self.text(d, (xx, lower+7), tick_label, 16, MUTED, 'ma')
            if not percent and self.expected_visible(session.expected, minimum, maximum):
                yy = py(session.expected)
                for xx in range(left, right, 12):
                    d.line((xx, yy, min(xx+5, right), yy), fill='#708395')
            # 구간 전체는 옅게, 현재까지는 밝게 그린다.
            self.curve(d, x, values, indices, px, py, '#607487')
            self.curve(d, x, values, indices[indices <= index], px, py, color)
            cursor = px(current)
            d.line((cursor, upper-2, cursor, lower+2), fill='#ffffff', width=2)
            if np.isfinite(value):
                yy = py(value)
                d.ellipse((cursor-4, yy-4, cursor+4, yy+4), fill=color, outline='#ffffff')
        basis = '중복 제거 후 conf' if options.confidence == 'deduplicated' else '전체 검출 conf (중복 포함)'
        mode = '고정 구간' if options.mode == 'page' else '현재 위치 따라가기'
        footer = f'{mode} · {options.window:g}{"초" if options.axis == "time" else "프레임"} 구간 · {basis} · 신뢰도는 정답 정확도가 아님'
        self.text(d, (24, 1038), session.warning or footer, 18, '#f6cc6a' if session.warning else MUTED)
        return image

    @staticmethod
    def expected_visible(expected, minimum, maximum):
        return minimum <= expected <= maximum

    @staticmethod
    def curve(draw, xs, values, indices, px, py, color):
        points = []
        for i in indices:
            if np.isfinite(values[i]):
                points.append((px(xs[i]), py(values[i])))
            else:
                if len(points) > 1:
                    draw.line(points, fill=color, width=2)
                points = []
        if len(points) > 1:
            draw.line(points, fill=color, width=2)


def export_video(session_path, output, options, cancel=None, progress=None, start=0, end=None, overwrite=False, speed=1.,
                 session_factory=None, dashboard_factory=None):
    output = Path(output).expanduser().resolve()
    if output.suffix.lower() != '.mp4':
        raise ValueError('출력 파일 확장자는 .mp4여야 합니다.')
    if output.exists() and not overwrite:
        raise FileExistsError(f'이미 존재하는 파일입니다: {output}')
    session = (session_factory or Session)(session_path)
    temporary = output.with_name(f'.{output.stem}.{uuid.uuid4().hex}.partial.mp4')
    writer = None
    try:
        if output in {v.path for v in session.videos.values()}:
            raise ValueError('입력 영상을 덮어쓸 수 없습니다.')
        end = session.length if end is None else min(end, session.length)
        if start < 0 or start >= end:
            raise ValueError('내보내기 프레임 범위가 잘못되었습니다.')
        output.parent.mkdir(parents=True, exist_ok=True)
        dashboard = (dashboard_factory or Dashboard)()
        writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), session.fps*speed,
                                 dashboard.output_size(session))
        if not writer.isOpened():
            raise RuntimeError('MP4 인코더를 열 수 없습니다.')
        for i in range(start, end):
            if cancel is not None and cancel.is_set():
                return None
            frame = dashboard.render(session, i, options)
            writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
            if progress is not None:
                progress(i-start+1, end-start)
        writer.release()
        writer = None
        if output.exists() and not overwrite:
            raise FileExistsError(f'출력 파일이 이미 존재합니다: {output}')
        os.replace(temporary, output)
        return output
    finally:
        if writer is not None:
            writer.release()
        session.close()
        temporary.unlink(missing_ok=True)


class Viewer:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.session = self.dashboard = None
        self.index, self.playing, self.updating, self.closing = 0, False, False, False
        self.export_thread = None
        self.child_viewers = []
        self.cancel_export = threading.Event()
        self.events = queue.Queue()
        self.axis = tk.StringVar(value='시간(초)' if args.axis == 'time' else '프레임')
        self.previous_axis = args.axis
        self.window = tk.StringVar(value=f'{args.window:g}')
        self.mode = tk.StringVar(value='고정 구간' if args.mode == 'page' else '따라가기')
        self.basis = tk.StringVar(value='중복 제거' if args.confidence == 'deduplicated' else '전체 검출')
        self.speed = tk.StringVar(value=f'{args.speed:g}')
        self.status = tk.StringVar(value='로그 폴더를 열어 주세요. Space: 재생/정지 · ←/→: 한 프레임 이동')
        root.title('장기 비전 로그 분석')
        root.geometry('1440x940')
        root.minsize(1000, 650)
        bar = ttk.Frame(root, padding=8)
        bar.pack(fill='x')
        ttk.Button(bar, text='로그 추가' if getattr(self, 'comparison_mode', False) else '로그 폴더 열기',
                   command=self.open_folder).pack(side='left', padx=3)
        if not getattr(self, 'comparison_mode', False):
            ttk.Button(bar, text='비교 모드', command=self.open_comparison).pack(side='left', padx=3)
        self.play_button = ttk.Button(bar, text='▶ 재생', command=self.toggle)
        self.play_button.pack(side='left', padx=3)
        ttk.Button(bar, text='−1초', command=lambda: self.step_seconds(-1)).pack(side='left', padx=3)
        ttk.Button(bar, text='+1초', command=lambda: self.step_seconds(1)).pack(side='left', padx=3)
        ttk.Label(bar, text='재생 배속').pack(side='left', padx=(12, 3))
        box = ttk.Combobox(bar, textvariable=self.speed, values=['0.25', '0.5', '1', '2'], width=5, state='readonly')
        box.pack(side='left')
        box.bind('<<ComboboxSelected>>', lambda e: self.restart_clock())
        self.export_button = ttk.Button(bar, text='분석 화면 MP4 저장', command=self.start_export)
        self.export_button.pack(side='right', padx=3)
        self.progress = ttk.Progressbar(bar, maximum=100, length=180)
        self.progress.pack(side='right', padx=10)
        controls = ttk.Frame(root, padding=(8, 0, 8, 6))
        controls.pack(fill='x')
        for title, var, values, width in [('X축', self.axis, ['시간(초)', '프레임'], 9),
                                         ('구간 길이', self.window, ['5', '10', '20', '30', '60', '120', '600'], 7),
                                         ('그래프 이동', self.mode, ['고정 구간', '따라가기'], 10),
                                         ('신뢰도 기준', self.basis, ['중복 제거', '전체 검출'], 10)]:
            ttk.Label(controls, text=title).pack(side='left', padx=(10, 4))
            combo = ttk.Combobox(controls, textvariable=var, values=values, width=width,
                                 state='normal' if var is self.window else 'readonly')
            combo.pack(side='left')
            if var is self.axis:
                combo.bind('<<ComboboxSelected>>', lambda e: self.change_axis())
            else:
                combo.bind('<<ComboboxSelected>>', lambda e: self.redraw())
            combo.bind('<Return>', lambda e: self.redraw())
        ttk.Button(controls, text='적용', command=self.redraw).pack(side='left', padx=8)
        self.canvas = tk.Canvas(root, width=1, height=1, bg=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.image_item = self.canvas.create_image(0, 0, anchor='center')
        self.canvas.bind('<Configure>', lambda e: self.redraw())
        self.slider = ttk.Scale(root, from_=0, to=1, command=self.seek)
        self.slider.pack(fill='x', padx=12, pady=5)
        ttk.Label(root, textvariable=self.status, padding=8, wraplength=1380).pack(fill='x')
        root.bind('<space>', self.space)
        root.bind('<Left>', lambda e: self.step(-1))
        root.bind('<Right>', lambda e: self.step(1))
        root.protocol('WM_DELETE_WINDOW', self.quit)
        if args.session:
            root.after(100, lambda: self.load(args.session))
        root.after(20, self.tick)

    def options(self):
        window = float(self.window.get())
        if not math.isfinite(window) or window <= 0:
            raise ValueError('그래프 구간 길이는 양수여야 합니다.')
        return ViewOptions('time' if self.axis.get() == '시간(초)' else 'frame', window,
                           'page' if self.mode.get() == '고정 구간' else 'follow',
                           'deduplicated' if self.basis.get() == '중복 제거' else 'raw')

    def open_comparison(self):
        from evaluation_compare import ComparisonViewer
        from types import SimpleNamespace
        self.pause()
        args = SimpleNamespace(**vars(self.args))
        args.session = self.session.path if self.session else None
        args.add_session = []
        self.child_viewers.append(ComparisonViewer(tk.Toplevel(self.root), args))

    def change_axis(self):
        axis = 'time' if self.axis.get() == '시간(초)' else 'frame'
        if axis != self.previous_axis and self.session:
            try:
                value = float(self.window.get())
                value = value*self.session.fps if axis == 'frame' else value/self.session.fps
                self.window.set(f'{value:g}')
            except ValueError:
                pass
        self.previous_axis = axis
        self.redraw()

    def open_folder(self):
        path = filedialog.askdirectory(parent=self.root, title='counts.csv와 overlay/board.mp4가 있는 폴더')
        if path:
            self.load(path)

    def load(self, path):
        self.pause()
        try:
            session = Session(path)
            dashboard = Dashboard()
        except Exception as exc:
            messagebox.showerror('로그 열기 실패', str(exc), parent=self.root)
            return
        if self.session:
            self.session.close()
        self.session, self.dashboard, self.index = session, dashboard, 0
        self.slider.configure(to=max(1, session.length-1))
        self.redraw()

    def redraw(self):
        if self.session is None or self.closing:
            return
        try:
            frame = self.dashboard.render(self.session, self.index, self.options())
            self.display_frame(frame)
            self.updating = True
            self.slider.set(self.index)
            self.updating = False
            self.status.set(self.session.warning or
                            f'{self.session.path.parent}  |  {self.index+1:,}/{self.session.length:,} 프레임 · {clock(self.index/self.session.fps)}')
        except Exception as exc:
            self.pause()
            self.status.set(f'표시 오류: {exc}')

    def display_frame(self, frame):
        frame.thumbnail((max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(frame)
        self.canvas.itemconfigure(self.image_item, image=self.photo)
        self.canvas.coords(self.image_item, self.canvas.winfo_width()/2, self.canvas.winfo_height()/2)

    def restart_clock(self):
        self.play_start, self.play_index = time.monotonic(), self.index

    def pause(self):
        self.playing = False
        self.play_button.configure(text='▶ 재생')

    def toggle(self):
        if self.session is None:
            return
        if self.playing:
            self.pause()
        else:
            if self.index >= self.session.length-1:
                self.index = 0
            self.restart_clock()
            self.playing = True
            self.play_button.configure(text='Ⅱ 일시정지')

    def space(self, event):
        if not isinstance(self.root.focus_get(), (ttk.Entry, ttk.Combobox)):
            self.toggle()
            return 'break'

    def seek(self, value):
        if not self.updating and self.session:
            self.pause()
            self.index = min(self.session.length-1, max(0, round(float(value))))
            self.redraw()

    def step(self, delta):
        if self.session:
            self.pause()
            self.index = min(self.session.length-1, max(0, self.index+delta))
            self.redraw()

    def step_seconds(self, seconds):
        if self.session:
            self.step(round(seconds*self.session.fps))

    def start_export(self):
        if self.export_thread and self.export_thread.is_alive():
            self.cancel_export.set()
            self.status.set('내보내기 취소 중…')
            return
        if self.session is None:
            return
        try:
            options = self.options()
        except ValueError as exc:
            messagebox.showerror('설정 오류', str(exc), parent=self.root)
            return
        output = filedialog.asksaveasfilename(parent=self.root, title='분석 화면 전체를 MP4로 저장',
                    initialdir=str(self.session.path.parent), initialfile='report.mp4',
                    defaultextension='.mp4', filetypes=[('MP4 동영상', '*.mp4')])
        if not output:
            return
        output = Path(output).resolve()
        if output in {v.path for v in self.session.videos.values()}:
            messagebox.showerror('출력 경로 오류', '입력 영상을 덮어쓸 수 없습니다.', parent=self.root)
            return
        self.pause()
        self.cancel_export.clear()
        path, export_kwargs = self.export_context()
        speed = float(self.speed.get())
        self.export_button.configure(text='내보내기 취소')
        self.progress['value'] = 0

        def run():
            try:
                last = -1

                def progress(done, total):
                    nonlocal last
                    percent = int(done/total*100)
                    if percent != last:
                        self.events.put(('progress', percent))
                        last = percent

                result = export_video(path, output, options, self.cancel_export, progress,
                                      overwrite=True, speed=speed, **export_kwargs)
                self.events.put(('finished', str(result) if result else '취소됨 (부분 영상 삭제)'))
            except Exception as exc:
                self.events.put(('error', str(exc)))

        self.export_thread = threading.Thread(target=run, daemon=True)
        self.export_thread.start()

    def export_context(self):
        return self.session.path, {}

    def tick(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'progress':
                self.progress['value'] = value
            else:
                self.export_button.configure(text='분석 화면 MP4 저장')
                self.status.set(('저장 완료: ' if kind == 'finished' else '내보내기 오류: ') + value)
                if kind == 'error' and not self.closing:
                    messagebox.showerror('내보내기 실패', value, parent=self.root)
        if self.closing:
            children_running = any(v.export_thread and v.export_thread.is_alive() for v in self.child_viewers)
            if (not self.export_thread or not self.export_thread.is_alive()) and not children_running:
                if self.session:
                    self.session.close()
                for child in self.child_viewers:
                    if child.session:
                        child.session.close()
                self.root.destroy()
                return
        elif self.playing and self.session:
            target = self.play_index + int((time.monotonic()-self.play_start)*self.session.fps*float(self.speed.get()))
            target = min(target, self.session.length-1)
            if target != self.index:
                self.index = target
                self.redraw()
            if target >= self.session.length-1:
                self.pause()
        self.root.after(20, self.tick)

    def quit(self):
        if self.closing:
            return
        for child in self.child_viewers:
            child.quit()
        self.pause()
        self.closing = True
        self.cancel_export.set()
        self.status.set('내보내기를 정리하고 종료하는 중…')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', nargs='?', type=Path, help='실행별 로그 폴더 또는 CSV')
    parser.add_argument('--compare', action='store_true', help='여러 로그를 체크박스로 선택해 겹쳐 보는 비교 모드')
    parser.add_argument('--add-session', action='append', type=Path, default=[], help='함께 열 로그 (여러 번 지정 가능)')
    parser.add_argument('--video-count', choices=[2, 4], type=int, default=2, help='비교 모드의 영상 패널 수')
    parser.add_argument('--window', type=float, default=10., help='그래프 구간 길이 (기본 10초)')
    parser.add_argument('--axis', choices=['time', 'frame'], default='time')
    parser.add_argument('--mode', choices=['page', 'follow'], default='follow')
    parser.add_argument('--confidence', choices=['deduplicated', 'raw'], default='deduplicated')
    parser.add_argument('--speed', type=float, choices=[.25, .5, 1., 2.], default=1., help='재생/저장 배속 (기본 1)')
    parser.add_argument('--export', type=Path, help='GUI 없이 분석 화면을 MP4로 내보내기')
    parser.add_argument('--start-frame', type=int, default=0, help='CLI 내보내기 시작 프레임 (0부터)')
    parser.add_argument('--end-frame', type=int, help='CLI 내보내기 끝 프레임 (해당 프레임 제외)')
    args = parser.parse_args()
    if not math.isfinite(args.window) or args.window <= 0:
        parser.error('--window는 양수여야 합니다.')
    if args.export and not (args.session or args.add_session):
        parser.error('--export 사용 시 로그 폴더를 지정하세요.')
    return args


def main():
    args = parse_args()
    compare = args.compare or bool(args.add_session)
    if args.export:
        last = [-1]

        def progress(done, total):
            percent = int(done/total*100)
            if percent//10 != last[0]:
                print(f'내보내기 {percent}% ({done}/{total})', flush=True)
                last[0] = percent//10

        kwargs = {}
        source = args.session
        if compare:
            from evaluation_compare import ComparisonSession, ComparisonDashboard
            source = ([args.session] if args.session else []) + args.add_session
            kwargs = dict(session_factory=lambda paths: ComparisonSession(paths, video_count=args.video_count, axis=args.axis),
                          dashboard_factory=ComparisonDashboard)
        result = export_video(source, args.export,
                              ViewOptions(args.axis, args.window, args.mode, args.confidence),
                              progress=progress, start=args.start_frame, end=args.end_frame, speed=args.speed, **kwargs)
        print(f'저장 완료: {result}')
    else:
        root = tk.Tk()
        if compare:
            from evaluation_compare import ComparisonViewer
            ComparisonViewer(root, args)
        else:
            Viewer(root, args)
        root.mainloop()


if __name__ == '__main__':
    sys.modules.setdefault('evaluation_viewer', sys.modules[__name__])
    try:
        main()
    except Exception as exc:
        print(f'[오류] {exc}', file=sys.stderr)
        sys.exit(1)
