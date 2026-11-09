"""복수 실행 로그의 공통 축 비교 및 선택 가능한 2/4 영상 패널."""
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from PIL import Image, ImageDraw, ImageTk

from evaluation_viewer import (BG, PANEL, GRID, TEXT, MUTED, Dashboard, Session, Viewer,
                               clock, graph_bounds, value_bounds)

SERIES_COLORS = ['#43d6c5', '#f6cc6a', '#bc9cf5', '#f28f9d', '#65aaff', '#ffa66b', '#c6e881', '#e8a7e7']


def preview_size(width, viewport_width, viewport_height):
    # 영상 수가 늘어도 1920 기준 배율을 유지해 각 박스가 줄어들지 않게 한다.
    scale = min(max(1, viewport_width)/1920, max(1, viewport_height)/1080, 1.)
    return max(1, round(width*scale)), max(1, round(1080*scale))


def metrics(session, basis):
    return [session.data['deduplicated'], session.data['assigned'], session.scores('mean', basis)*100,
            session.scores('bottom30_mean', basis)*100, session.scores('min', basis)*100]


class ComparisonSession:
    def __init__(self, paths=(), selected=None, slots=None, video_count=2, axis='time'):
        self.sessions = []
        self._values, self._bounds = {}, {}
        self.selected = set()
        self.slots = list(slots or [None]*4)
        self.video_count, self.axis = video_count, axis
        try:
            for path in paths:
                self.add(path)
            if selected is not None:
                self.selected = set(selected)
            if slots is not None:
                self.slots = list(slots)
        except Exception:
            self.close()
            raise

    def add(self, path):
        session = Session(path)
        if any(s.path == session.path for s in self.sessions):
            session.close()
            raise ValueError('이미 열려 있는 로그입니다.')
        i = len(self.sessions)
        self.sessions.append(session)
        self._bounds.clear()
        self.selected.add(i)
        if i == 0:
            self.slots[:2] = [(0, 'overlay'), (0, 'board')]
        elif i == 1:
            self.slots[2:] = [(1, 'overlay'), (1, 'board')]

    @property
    def path(self):
        return self.sessions[0].path

    @property
    def videos(self):
        return {f'{i}:{kind}': video for i, s in enumerate(self.sessions) for kind, video in s.videos.items()}

    @property
    def fps(self):
        return max(s.fps for s in self.sessions) if self.axis == 'time' else self.sessions[0].fps

    @property
    def length(self):
        if self.axis == 'frame':
            return min(s.length for s in self.sessions)
        duration = min(s.length/s.fps for s in self.sessions)
        return max(1, int(np.floor(duration*self.fps + 1e-7)))

    @property
    def warning(self):
        messages = [f'{self.code(i)}: {s.warning}' for i, s in enumerate(self.sessions) if s.warning]
        if len({s.expected for s in self.sessions}) > 1:
            messages.append('로그별 기준 말 개수가 다름')
        return ' / '.join(messages)

    @staticmethod
    def code(i):
        return chr(65+i) if i < 26 else f'R{i+1}'

    def label(self, i):
        session = self.sessions[i]
        model = Path(session.summary.get('metadata', {}).get('model', '')).name or '모델 미기록'
        return f'{self.code(i)} · {model} · {session.path.parent.name[-13:]}'

    def sample_index(self, session_id, index):
        if index < 0 or index >= self.length:
            return None
        session = self.sessions[session_id]
        if self.axis == 'frame':
            return index if index < session.length else None
        seconds = index/self.fps
        if seconds >= session.length/session.fps:
            return None
        return min(session.length-1, int(np.floor(seconds*session.fps + 1e-7)))

    def bounds(self, metric, basis):
        # 선택된 로그만으로 축을 잡지 않는다. 체크를 바꿔도 비교 축은 동일하다.
        key = metric, basis, self.axis
        if key not in self._bounds:
            arrays = [self.values(i, metric, basis) for i in range(len(self.sessions))]
            self._bounds[key] = value_bounds(np.concatenate(arrays), percent=metric >= 2)
        return self._bounds[key]

    def values(self, session_id, metric, basis):
        key = session_id, basis
        if key not in self._values:
            self._values[key] = metrics(self.sessions[session_id], basis)
        limit = (self.length if self.axis == 'frame' else
                 int(np.floor((self.length-1)/self.fps*self.sessions[session_id].fps + 1e-7))+1)
        return self._values[key][metric][:limit]

    def close(self):
        for session in self.sessions:
            session.close()


class ComparisonDashboard(Dashboard):
    def output_size(self, group):
        return (2366 if group.video_count == 4 else 1920), self.HEIGHT

    @classmethod
    def graph_left(cls, count):
        return 20 + cls.VIDEO_SIDE*(2 if count == 4 else 1)

    @classmethod
    def video_tiles(cls, count):
        side = cls.VIDEO_SIDE
        if count == 2:
            return [(20, 100, side), (20, 100+side, side)]
        return [(20, 100, side), (20+side, 100, side),
                (20, 100+side, side), (20+side, 100+side, side)]

    def render(self, group, index, options):
        image = Image.new('RGB', self.output_size(group), BG)
        graph_left = self.graph_left(group.video_count)
        graph_right = image.width-20
        d = ImageDraw.Draw(image)
        current = index/group.fps if options.axis == 'time' else index+1
        start, end = graph_bounds(current, options.window, options.mode)
        self.text(d, (24, 15), '장기 비전 · 로그 비교', 28)
        self.text(d, (24, 55), f'현재 {clock(index/group.fps)} · {index+1:,} 프레임 · 그래프 {len(group.selected)}/{len(group.sessions)}개 · 영상 {group.video_count}개', 20, MUTED)
        for j, i in enumerate(sorted(group.selected)):
            color = SERIES_COLORS[i % len(SERIES_COLORS)]
            x = graph_left+78+(j % 6)*195
            if j < 6:
                self.text(d, (x, 22), f'{group.code(i)} {Path(group.sessions[i].summary.get("metadata", {}).get("model", "미기록")).name[:20]}', 16, color)
        tiles = self.video_tiles(group.video_count)
        for slot, (left, top, side) in enumerate(tiles):
            d.rectangle((left, top, left+side, top+side), fill=PANEL)
            selection = group.slots[slot]
            if selection is None:
                self.text(d, (left+side/2, top+side/2), '영상 선택 없음', 18, MUTED, 'mm')
                continue
            i, kind = selection
            color = SERIES_COLORS[i % len(SERIES_COLORS)]
            self.text(d, (left+12, top+9), f'{group.code(i)} · {"오버레이" if kind == "overlay" else "인식판"}', 20, color)
            frame_index = group.sample_index(i, index)
            if frame_index is None:
                self.text(d, (left+side/2, top+side/2), '구간 종료', 20, MUTED, 'mm')
            else:
                frame = group.sessions[i].videos[kind].read(frame_index).copy()
                w, h = side-24, side-54
                frame.thumbnail((w, h))
                image.paste(frame, (left+12+(w-frame.width)//2, top+46+(h-frame.height)//2))
        titles = ['중복 제거 후 말 개수', '교차점에 할당된 말 개수', '평균 신뢰도', '하위 30% 평균 신뢰도', '최저 신뢰도']
        for metric, title in enumerate(titles):
            top = 100+metric*184
            d.rounded_rectangle((graph_left, top, graph_right, top+174), radius=0, fill=PANEL)
            self.text(d, (graph_left+18, top+8), title, 22)
            selected = sorted(group.selected)
            for j, i in enumerate(selected):
                values = group.values(i, metric, options.confidence)
                k = group.sample_index(i, index)
                value = values[k] if k is not None else np.nan
                unit = '%' if metric >= 2 else '개'
                label = '—' if not np.isfinite(value) else f'{value:.1f}{unit}' if metric >= 2 else f'{value:g}{unit}'
                column = j % 8
                row = j // 8
                if row < 2:
                    values_left = graph_left+388
                    values_width = graph_right-20-values_left
                    self.text(d, (values_left+column*(values_width/max(1, min(len(selected), 8))), top+10+row*22),
                              f'{group.code(i)} {label}', 16, SERIES_COLORS[i % len(SERIES_COLORS)])
            left, right, upper, lower = graph_left+56, graph_right-22, top+58, top+142
            minimum, maximum = group.bounds(metric, options.confidence)

            def px(v):
                return left+(float(v)-start)/(end-start)*(right-left)

            def py(v):
                return lower-(min(max(float(v), minimum), maximum)-minimum)/(maximum-minimum)*(lower-upper)

            for tick in (minimum, (minimum+maximum)/2, maximum):
                yy = py(tick)
                d.line((left, yy, right, yy), fill=GRID)
                decimals = 3 if maximum-minimum < .01 else 2 if maximum-minimum < 1 else 1
                label = f'{tick:.{decimals}f}' if metric >= 2 else f'{tick:g}'
                self.text(d, (left-9, yy), label, 16, MUTED, 'rm')
            for tick in np.linspace(start, end, 5):
                xx = px(tick)
                d.line((xx, upper, xx, lower), fill=GRID)
                self.text(d, (xx, lower+7), f'{tick:.1f}s' if options.axis == 'time' else f'{tick:.0f}', 16, MUTED, 'ma')
            if metric < 2:
                for expected in {s.expected for s in group.sessions}:
                    if minimum <= expected <= maximum:
                        for xx in range(left, right, 12):
                            d.line((xx, py(expected), min(xx+5, right), py(expected)), fill='#708395')
            for i in selected:
                session = group.sessions[i]
                values = group.values(i, metric, options.confidence)
                xs = np.arange(len(values))/session.fps if options.axis == 'time' else np.arange(len(values))+1
                indices = np.flatnonzero((xs >= start) & (xs <= end))
                color = SERIES_COLORS[i % len(SERIES_COLORS)]
                rgb = tuple(int(color[n:n+2], 16) for n in (1, 3, 5))
                dim = tuple(int(v*.45) for v in rgb)
                self.curve(d, xs, values, indices, px, py, dim)
                self.curve(d, xs, values, indices[xs[indices] <= current], px, py, color)
                k = group.sample_index(i, index)
                if k is not None and np.isfinite(values[k]):
                    xx, yy = px(current), py(values[k])
                    d.ellipse((xx-4, yy-4, xx+4, yy+4), fill=color, outline=TEXT)
            cursor = px(current)
            d.line((cursor, upper-2, cursor, lower+2), fill=TEXT, width=1)
        footer = f'가장 짧은 결과까지 사용 · Y축: 공통 구간 {len(group.sessions)}개 로그 최소~최대 · 체크 변경 시 축 유지 · {"시간" if options.axis == "time" else "행 순서"} 정렬'
        self.text(d, (24, 1038), group.warning or footer, 18, '#f6cc6a' if group.warning else MUTED)
        return image


class ComparisonViewer(Viewer):
    comparison_mode = True

    def __init__(self, root, args):
        self.checks = []
        self.video_count = tk.StringVar(master=root, value=str(args.video_count))
        self.slot_vars = [tk.StringVar(master=root, value='선택 없음') for _ in range(4)]
        self.slot_choices = {}
        super().__init__(root, args)
        scrollbar = ttk.Scrollbar(root, orient='horizontal', command=self.canvas.xview)
        scrollbar.pack(fill='x', before=self.slider)
        self.canvas.configure(xscrollcommand=scrollbar.set)
        root.title('장기 비전 · 복수 로그 비교')
        frame = ttk.LabelFrame(root, text='비교할 로그 체크 / 영상별 입력 선택', padding=5)
        frame.pack(fill='x', before=self.canvas, padx=8, pady=4)
        self.check_frame = ttk.Frame(frame)
        self.check_frame.pack(fill='x')
        controls = ttk.Frame(frame)
        controls.pack(fill='x', pady=4)
        ttk.Label(controls, text='영상 수').pack(side='left')
        count = ttk.Combobox(controls, textvariable=self.video_count, values=['2', '4'], width=3, state='readonly')
        count.pack(side='left', padx=4)
        count.bind('<<ComboboxSelected>>', lambda e: self.update_selection())
        self.slot_boxes = []
        for i, variable in enumerate(self.slot_vars):
            ttk.Label(controls, text=f'{i+1}번').pack(side='left', padx=(8, 2))
            combo = ttk.Combobox(controls, textvariable=variable, values=['선택 없음'], width=23, state='readonly')
            combo.pack(side='left')
            combo.bind('<<ComboboxSelected>>', lambda e: self.update_selection())
            self.slot_boxes.append(combo)
        for path in args.add_session:
            root.after(120, lambda p=path: self.load(p))

    def load(self, path):
        self.pause()
        if self.session is None:
            self.session = ComparisonSession(video_count=int(self.video_count.get()), axis='time' if self.axis.get() == '시간(초)' else 'frame')
            self.dashboard = ComparisonDashboard()
        try:
            self.session.add(path)
        except Exception as exc:
            if not self.session.sessions:
                self.session = None
            messagebox.showerror('로그 추가 실패', str(exc), parent=self.root)
            return
        i = len(self.session.sessions)-1
        variable = tk.BooleanVar(master=self.root, value=True)
        self.checks.append(variable)
        style = ttk.Style(self.root)
        name = f'Compare{i}.TCheckbutton'
        style.configure(name, foreground=SERIES_COLORS[i % len(SERIES_COLORS)])
        ttk.Checkbutton(self.check_frame, text=self.session.label(i), variable=variable,
                        command=self.update_selection, style=name).grid(row=i//3, column=i%3, sticky='w', padx=8)
        self.slot_choices = {'선택 없음': None}
        for j in range(len(self.session.sessions)):
            for kind in ('overlay', 'board'):
                label = f'{self.session.code(j)} · {"overlay" if kind == "overlay" else "board"} · {self.session.sessions[j].path.parent.name[-13:]}'
                self.slot_choices[label] = (j, kind)
        for slot, combo in enumerate(self.slot_boxes):
            combo.configure(values=list(self.slot_choices))
            selection = self.session.slots[slot]
            self.slot_vars[slot].set(next(k for k, v in self.slot_choices.items() if v == selection))
        self.update_selection()

    def update_selection(self):
        if self.session is None:
            return
        self.session.selected = {i for i, variable in enumerate(self.checks) if variable.get()}
        self.session.video_count = int(self.video_count.get())
        self.session.slots = [self.slot_choices.get(v.get()) for v in self.slot_vars]
        for i, combo in enumerate(self.slot_boxes):
            combo.configure(state='readonly' if i < self.session.video_count else 'disabled')
        self.redraw()

    def redraw(self):
        if self.session:
            self.session.axis = 'time' if self.axis.get() == '시간(초)' else 'frame'
            self.index = min(self.index, self.session.length-1)
            self.updating = True
            self.slider.configure(to=max(1, self.session.length-1))
            self.updating = False
        super().redraw()

    def export_context(self):
        paths = [s.path for s in self.session.sessions]
        selected, slots = set(self.session.selected), list(self.session.slots)
        count, axis = self.session.video_count, self.session.axis

        def factory(paths):
            return ComparisonSession(paths, selected=selected, slots=slots, video_count=count, axis=axis)

        return paths, dict(session_factory=factory, dashboard_factory=ComparisonDashboard)

    def display_frame(self, frame):
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        frame = frame.resize(preview_size(frame.width, width, height), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(frame)
        self.canvas.itemconfigure(self.image_item, image=self.photo)
        extent = max(width, frame.width)
        self.canvas.coords(self.image_item, extent/2, height/2)
        self.canvas.configure(scrollregion=(0, 0, extent, height))
