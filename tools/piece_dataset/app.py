"""장기말 데이터셋 추출·라벨링·학습용 회전 증강 GUI."""
import argparse
import json
import queue
import random
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageDraw, ImageTk

from .core import Catalog, Cancelled, CLASSES, KINDS, SPLITS, augment_image, export_dataset, prepare_image, sample_angles, plan_export
from .extraction import ROOT, extract_video


class DatasetApp:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.catalog = Catalog(args.project)
        self.rows, self.current = [], None
        self.thread, self.closing = None, False
        self.cancel = threading.Event()
        self.events = queue.Queue()
        self.project = tk.StringVar(value=str(self.catalog.root))
        self.video = tk.StringVar(value='')
        self.interval = tk.StringVar(value=str(args.interval))
        self.group = tk.StringVar(value='')
        self.ratios = [tk.StringVar(value=str(v)) for v in args.ratios]
        self.minimums = [tk.StringVar(value=str(v)) for v in args.min_per_class]
        self.split_mode = tk.StringVar(value='랜덤' if args.split_mode == 'random' else '순서대로')
        self.split_unit = tk.StringVar(value='사진별' if args.split_unit == 'sample' else '촬영 그룹별')
        self.split_seed = tk.StringVar(value=str(args.seed))
        self.filter = tk.StringVar(value='pending')
        self.augment = tk.StringVar(value=str(args.augment))
        self.size = tk.StringVar(value=str(args.size))
        self.angle_step = tk.StringVar(value=str(args.angle_step))
        self.status = tk.StringVar(value='영상에서 추출하거나 기존 프로젝트를 열어 정답을 붙이세요.')
        self.info = tk.StringVar()
        root.title('장기말 · 학습 데이터셋 제작')
        root.geometry('1460x930')
        root.minsize(1100, 730)
        bar = ttk.Frame(root, padding=8)
        bar.pack(fill='x')
        ttk.Label(bar, text='프로젝트').pack(side='left')
        ttk.Entry(bar, textvariable=self.project, state='readonly').pack(side='left', expand=True, fill='x', padx=6)
        ttk.Button(bar, text='프로젝트 열기 / 새로', command=self.open_project).pack(side='left')
        ttk.Button(bar, text='통계', command=self.statistics).pack(side='left', padx=6)
        source = ttk.Frame(root, padding=(8, 0))
        source.pack(fill='x')
        ttk.Entry(source, textvariable=self.video).pack(side='left', expand=True, fill='x')
        ttk.Button(source, text='영상 선택', command=self.choose_video).pack(side='left', padx=5)
        ttk.Label(source, text='추출 간격(초)').pack(side='left')
        ttk.Combobox(source, textvariable=self.interval, values=['1', '2', '3', '5', '10'], width=5).pack(side='left', padx=4)
        ttk.Button(source, text='YOLO로 추출', command=self.extract).pack(side='left', padx=3)
        ttk.Button(source, text='작업 중지', command=self.cancel.set).pack(side='left', padx=3)
        grouping = ttk.Frame(root, padding=8)
        grouping.pack(fill='x')
        ttk.Label(grouping, text='촬영 그룹').pack(side='left')
        ttk.Entry(grouping, textvariable=self.group, width=35).pack(side='left', padx=5)
        ttk.Button(grouping, text='원본·그룹 관리', command=self.manage_sources).pack(side='left', padx=6)
        ttk.Label(grouping, text='원본은 함께 보관 · Train/Val/Test는 내보낼 때 분배').pack(side='left')
        self.progress = ttk.Progressbar(grouping, maximum=100)
        self.progress.pack(side='left', fill='x', expand=True, padx=10)
        main = ttk.Frame(root, padding=8)
        main.pack(fill='both', expand=True)
        left = ttk.Frame(main)
        left.pack(side='left', fill='both', padx=(0, 10))
        ttk.Label(left, text='검수 상태').pack(anchor='w')
        combo = ttk.Combobox(left, textvariable=self.filter, values=['pending', 'confirmed', 'hold', 'rejected', 'all'], state='readonly')
        combo.pack(fill='x')
        combo.bind('<<ComboboxSelected>>', lambda e: self.refresh())
        self.tree = ttk.Treeview(left, columns=('frame', 'label'), show='headings', height=22, selectmode='browse')
        self.tree.heading('frame', text='프레임 / 상태')
        self.tree.heading('label', text='정답')
        self.tree.column('frame', width=130)
        self.tree.column('label', width=140)
        scroll = ttk.Scrollbar(left, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', self.select)
        center = ttk.Frame(main)
        center.pack(side='left', fill='both', expand=True)
        previews = ttk.Frame(center)
        previews.pack(fill='both', expand=True)
        self.canvases = []
        for title in ('원본 Crop · 픽셀 확대', '학습 이미지 · 회전 미리보기'):
            pane = ttk.LabelFrame(previews, text=title)
            pane.pack(side='left', fill='both', expand=True, padx=3)
            canvas = tk.Canvas(pane, width=280, height=280, bg='#192535', highlightthickness=0)
            canvas.pack(fill='both', expand=True)
            self.canvases.append(canvas)
        ttk.Button(center, text='회전·밝기 변형 다시 보기', command=self.preview_augmentation).pack(pady=5)
        context = ttk.LabelFrame(center, text='원본 프레임에서의 위치')
        context.pack(fill='both', expand=True)
        self.context = tk.Canvas(context, width=570, height=220, bg='#192535', highlightthickness=0)
        self.context.pack(fill='both', expand=True)
        ttk.Label(center, textvariable=self.info, wraplength=590).pack(fill='x', pady=6)
        right_panel = ttk.Frame(main, padding=(10, 0))
        right_panel.pack(side='left', fill='y')
        right_scroll = ttk.Scrollbar(right_panel, orient='vertical')
        right_scroll.pack(side='right', fill='y')
        right_canvas = tk.Canvas(right_panel, width=330, highlightthickness=0,
                                 yscrollcommand=right_scroll.set)
        right_canvas.pack(side='left', fill='y', expand=True)
        right_scroll.configure(command=right_canvas.yview)
        right = ttk.Frame(right_canvas)
        right_window = right_canvas.create_window(0, 0, window=right, anchor='nw')
        right.bind('<Configure>', lambda e: right_canvas.configure(scrollregion=right_canvas.bbox('all')))
        right_canvas.bind('<Configure>', lambda e: right_canvas.itemconfigure(right_window, width=e.width))
        right.columnconfigure((0, 1), weight=1)
        self.label_icons = {}
        icon_dir = Path(__file__).parent / 'assets' / 'pieces'
        ttk.Label(right, text='정답 선택 → 즉시 저장·다음 이미지').grid(row=0, column=0, columnspan=2, pady=5)
        for column, side in enumerate(('blue', 'red')):
            for row, (kind, ko) in enumerate(KINDS, start=1):
                label = f'{side}_{kind}'
                key = str(row) if not column else f'Shift+{row}'
                with Image.open(icon_dir / f'{label}.png') as icon:
                    self.label_icons[label] = ImageTk.PhotoImage(icon.resize((56, 56), Image.Resampling.LANCZOS))
                ttk.Button(right, text=f'{CLASSES[label]}  [{key}]', width=17,
                           image=self.label_icons[label], compound='top',
                           command=lambda c=label: self.assign(c)).grid(row=row, column=column, sticky='ew', padx=3, pady=3)
        for row, (title, command) in enumerate([
            ('보류 [H]', lambda: self.assign(status='hold')),
            ('검출 오류 / 제외 [X]', lambda: self.assign(status='rejected')),
            ('마지막 라벨 취소 [Ctrl+Z]', self.undo),
            ('이전 이미지 [←]', lambda: self.move(-1)),
            ('다음 이미지 [→]', lambda: self.move(1)),
        ], start=8):
            ttk.Button(right, text=title, command=command).grid(row=row, column=0, columnspan=2, sticky='ew', pady=3)
        ttk.Separator(right).grid(row=13, column=0, columnspan=2, sticky='ew', pady=8)
        ttk.Label(right, text='Train만 추가 증강 / 이미지').grid(row=14, column=0, sticky='w')
        ttk.Spinbox(right, textvariable=self.augment, from_=0, to=50, width=5).grid(row=14, column=1)
        ttk.Label(right, text='출력 크기(px)').grid(row=15, column=0, sticky='w', pady=4)
        ttk.Combobox(right, textvariable=self.size, values=['128', '160', '224', '320'], width=6).grid(row=15, column=1)
        ttk.Label(right, text='회전 각도 간격(°)').grid(row=16, column=0, sticky='w')
        ttk.Combobox(right, textvariable=self.angle_step, values=['1', '0.1', '0.01', '0.001'], width=6).grid(row=16, column=1)
        ttk.Button(right, text='분배 설정 / 학습 데이터셋 내보내기', command=self.export).grid(row=17, column=0, columnspan=2, sticky='ew', pady=8)
        ttk.Label(right, text='1~7: 초 / Shift+1~7: 한\n정답만 선택하세요. YOLO의 종류 예측은\n정답으로 자동 적용하지 않습니다.\n현재는 데이터 제작 도구입니다.\n모델 학습·로봇 동작은 실행하지 않습니다.',
                  wraplength=260).grid(row=18, column=0, columnspan=2, sticky='w', pady=6)

        def scroll_labels(event):
            if right_canvas.bbox('all')[3] > right_canvas.winfo_height():
                delta = -1 if event.num == 4 or getattr(event, 'delta', 0) > 0 else 1
                right_canvas.yview_scroll(delta, 'units')
            return 'break'

        for widget in (right_canvas, right, *right.winfo_children()):
            for sequence in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
                widget.bind(sequence, scroll_labels)
        ttk.Label(root, textvariable=self.status, padding=8, wraplength=1400).pack(fill='x')
        root.bind('<KeyPress>', self.key)
        root.protocol('WM_DELETE_WINDOW', self.quit)
        root.after(100, self.refresh)
        root.after(100, self.tick)

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def open_project(self):
        if self.busy():
            return
        folder = filedialog.askdirectory(parent=self.root, mustexist=False, title='데이터셋 프로젝트 폴더')
        if folder:
            try:
                new = Catalog(folder)
                self.catalog.close()
                self.catalog = new
                self.project.set(str(new.root))
                self.refresh()
            except Exception as exc:
                messagebox.showerror('프로젝트 오류', str(exc), parent=self.root)

    def choose_video(self):
        path = filedialog.askopenfilename(parent=self.root, filetypes=[('동영상', '*.mp4 *.mkv *.avi *.mov'), ('모든 파일', '*')])
        if path:
            self.video.set(path)
            self.group.set(Path(path).stem)

    def refresh(self, preferred=None):
        self.rows = self.catalog.samples(self.filter.get())
        self.tree.delete(*self.tree.get_children())
        for row in self.rows:
            self.tree.insert('', 'end', iid=row['id'], values=(f'{row["frame_index"]} / {row["status"]}', CLASSES.get(row['label'], '미지정')))
        target = preferred if any(r['id'] == preferred for r in self.rows) else self.rows[0]['id'] if self.rows else None
        self.current = None
        if target:
            self.tree.selection_set(target)
            self.tree.see(target)
            self.select()
        else:
            for canvas in self.canvases+[self.context]:
                canvas.delete('all')
            self.info.set('해당 상태의 이미지가 없습니다.')
        self.status.set(f'표시 {len(self.rows)}개 · 정답과 검수 상태는 catalog.sqlite에 즉시 저장됩니다.')

    def paint(self, canvas, image, enlarge=True):
        w, h = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        scale = min(w/image.width, h/image.height)
        if not enlarge:
            scale = min(1, scale)
        image = image.resize((max(1, round(image.width*scale)), max(1, round(image.height*scale))), Image.Resampling.NEAREST)
        canvas.photo = ImageTk.PhotoImage(image)
        canvas.delete('all')
        canvas.create_image(w/2, h/2, image=canvas.photo)

    def select(self, event=None):
        selected = self.tree.selection()
        if not selected:
            return
        self.current = next((r for r in self.rows if r['id'] == selected[0]), None)
        if self.current:
            try:
                row = self.current
                self.info.set('')
                for canvas in self.canvases+[self.context]:
                    canvas.delete('all')
                with Image.open(self.catalog.root/row['crop_path']) as image:
                    self.paint(self.canvases[0], image)
                with Image.open(self.catalog.root/row['frame_path']) as image:
                    image = image.convert('RGB')
                    self.paint(self.canvases[1], prepare_image(image, self.image_size(), box=json.loads(row['box'])))
                    ImageDraw.Draw(image).rectangle(json.loads(row['box']), outline='red', width=4)
                    self.paint(self.context, image, enlarge=False)
                box = json.loads(row['box'])
                self.info.set(f'{Path(row["source_path"]).name} · {row["time_s"]:.2f}s · 촬영 그룹 {row["group_name"]}\n'
                              f'검출 크기 {box[2]-box[0]:.1f}×{box[3]-box[1]:.1f}px · conf {row["confidence"]:.3f} · '
                              f'선명도 {row["blur"]:.1f} · 경계 잘림 {bool(row["clipped"])}\n정답: {CLASSES.get(row["label"], "미지정")}')
            except Exception as exc:
                self.status.set(f'이미지 표시 오류: {exc}')

    def image_size(self):
        size = int(self.size.get())
        if not 32 <= size <= 1024:
            raise ValueError('출력 크기는 32~1024입니다.')
        return size

    def preview_augmentation(self):
        if self.current:
            try:
                with Image.open(self.catalog.root/self.current['frame_path']) as image:
                    rng = random.Random()
                    box, step = json.loads(self.current['box']), float(self.angle_step.get())
                    angle = sample_angles(image, box, rng, 1, step)[0]
                    preview, params = augment_image(image, rng, self.image_size(), box=box, angle_step=step, angle=angle)
                self.paint(self.canvases[1], preview)
                self.status.set(f'미리보기만 변경 · 회전 {params["angle"]:.3f}° · 밝기 {params["brightness"]:.2f} · 대비 {params["contrast"]:.2f}')
            except Exception as exc:
                self.status.set(str(exc))

    def assign(self, label=None, status='confirmed'):
        if self.busy() or self.current is None:
            return
        row = self.current
        i = self.rows.index(row)
        self.catalog.label(row['id'], label, status)
        row['status'], row['label'] = status, label if status == 'confirmed' else None
        removed = self.filter.get() not in ('all', status)
        if removed:
            self.rows.pop(i)
            self.tree.delete(row['id'])
        else:
            self.tree.item(row['id'], values=(f'{row["frame_index"]} / {status}', CLASSES.get(row['label'], '미지정')))
        if self.rows:
            target = self.rows[min(i if removed else i+1, len(self.rows)-1)]['id']
            self.tree.selection_set(target)
            self.tree.see(target)
            self.select()
        else:
            self.current = None
            for canvas in self.canvases+[self.context]:
                canvas.delete('all')
            self.info.set('해당 상태의 검수가 끝났습니다.')
        self.status.set(f'정답 저장 완료 · 현재 필터에 {len(self.rows)}개 남음')

    def undo(self):
        if not self.busy():
            sample_id = self.catalog.undo()
            if sample_id:
                self.filter.set('all')
                self.refresh(sample_id)

    def move(self, delta):
        if self.current and self.rows:
            i = min(len(self.rows)-1, max(0, self.rows.index(self.current)+delta))
            self.tree.selection_set(self.rows[i]['id'])
            self.tree.see(self.rows[i]['id'])

    def key(self, event):
        if isinstance(self.root.focus_get(), (ttk.Entry, ttk.Combobox, ttk.Spinbox, tk.Entry)):
            return
        char = event.char
        if event.state & 4 and event.keysym.lower() == 'z':
            self.undo()
        elif char in '1234567' and char:
            self.assign(f'blue_{KINDS[int(char)-1][0]}')
        elif char in '!@#$%^&' and char:
            self.assign(f'red_{KINDS["!@#$%^&".index(char)][0]}')
        elif event.keysym.lower() == 'h':
            self.assign(status='hold')
        elif event.keysym.lower() == 'x':
            self.assign(status='rejected')
        elif event.keysym == 'Left':
            self.move(-1)
        elif event.keysym == 'Right':
            self.move(1)

    def job(self, title, function):
        if self.busy():
            return
        self.cancel.clear()
        self.progress['value'] = 0
        self.status.set(title)

        def run():
            try:
                result = function(lambda n, total: self.events.put(('progress', 100*n/max(1, total))))
                self.events.put(('done', str(result)))
            except Cancelled as exc:
                self.events.put(('done', str(exc)))
            except Exception as exc:
                self.events.put(('error', str(exc)))

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def extract(self):
        try:
            video, group = self.video.get(), self.group.get()
            interval = float(self.interval.get())
            if not Path(video).is_file():
                raise ValueError('입력 동영상을 선택하세요.')
            project = self.catalog.root
            self.job('추출 중… 기존 정답은 유지됩니다.', lambda progress: extract_video(project, video,
                     interval=interval, group=group or Path(video).stem, model=self.args.model,
                     device=self.args.device, confidence=self.args.conf, margin=self.args.margin,
                     cancel=self.cancel, progress=progress))
        except Exception as exc:
            messagebox.showerror('추출 설정 오류', str(exc), parent=self.root)

    def export(self):
        if self.busy():
            return
        window = tk.Toplevel(self.root)
        window.title('내보내기 · 원본 분배 후 Train 증강')
        window.transient(self.root)
        window.grab_set()
        body = ttk.Frame(window, padding=14)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='정답이 확정된 원본을 먼저 분배한 뒤 Train에만 회전 증강합니다.').grid(row=0, column=0, columnspan=3, pady=8)
        for i, (split, value) in enumerate(zip(SPLITS, self.ratios)):
            ttk.Label(body, text=f'{split} (%)').grid(row=1, column=i)
            ttk.Entry(body, textvariable=value, width=12).grid(row=2, column=i, padx=6, pady=4)
        for i, (split, value) in enumerate(zip(SPLITS, self.minimums)):
            ttk.Label(body, text=f'{split} 클래스당 최소 원본').grid(row=3, column=i)
            ttk.Spinbox(body, textvariable=value, from_=0, to=100000, width=12).grid(row=4, column=i, padx=6, pady=4)
        for row, (title, variable, values) in enumerate([
            ('분배 방식', self.split_mode, ['랜덤', '순서대로']),
            ('분배 단위', self.split_unit, ['사진별', '촬영 그룹별']),
        ], start=5):
            ttk.Label(body, text=title).grid(row=row, column=0, sticky='w', pady=4)
            ttk.Combobox(body, textvariable=variable, values=values, state='readonly').grid(row=row, column=1, columnspan=2, sticky='ew')
        ttk.Label(body, text='랜덤 seed').grid(row=7, column=0, sticky='w')
        ttk.Entry(body, textvariable=self.split_seed).grid(row=7, column=1, columnspan=2, sticky='ew')
        ttk.Label(body, text='사진별: 클래스별로 비율 분배. 순서대로는 영상 등록·프레임·검출 순서.\n'
                  '촬영 그룹별: 같은 촬영을 함께 배정하며 사진 수 비율은 달라질 수 있습니다.\n'
                  '최소 수는 정답이 있는 각 클래스에 적용합니다. 0은 최소 제한 해제입니다.\n'
                  '비율이 0인 분할은 제외합니다. 동일 Crop과 회전본으로 최소 수를 채우지 않습니다.',
                  wraplength=540).grid(row=8, column=0, columnspan=3, sticky='w', pady=10)
        preview = tk.StringVar(value='비율 합계는 100%입니다. 분배 미리보기로 개수를 확인하세요.')
        ttk.Label(body, textvariable=preview, wraplength=540).grid(row=9, column=0, columnspan=3, sticky='w', pady=8)

        table_frame = ttk.Frame(body)
        table_frame.grid(row=10, column=0, columnspan=3, sticky='ew', pady=4)
        table = ttk.Treeview(table_frame, columns=('label', *SPLITS), show='headings', height=7)
        for column in ('label', *SPLITS):
            table.heading(column, text='클래스' if column == 'label' else column)
            table.column(column, width=120 if column == 'label' else 90, anchor='center')
        scrollbar = ttk.Scrollbar(table_frame, orient='vertical', command=table.yview)
        table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        table.pack(side='left', fill='x', expand=True)

        def settings():
            return dict(ratios=tuple(float(v.get()) for v in self.ratios),
                        min_per_class=tuple(int(v.get()) for v in self.minimums),
                        split_mode='random' if self.split_mode.get() == '랜덤' else 'ordered',
                        split_unit='sample' if self.split_unit.get() == '사진별' else 'group',
                        seed=int(self.split_seed.get()))

        def show_plan():
            try:
                options = settings()
                _, info = plan_export(self.catalog.samples('confirmed'), options['ratios'], options['split_mode'],
                                      options['split_unit'], options['seed'], options['min_per_class'])
                counts = info['counts_original']
                table.delete(*table.get_children())
                for label in counts['train']:
                    table.insert('', 'end', values=(CLASSES[label], *(f'{counts[split][label]} / {info["counts_distinct"][split][label]}' for split in SPLITS)))
                lines = [f'{split}: 원본 {sum(counts[split].values())}장' for split in SPLITS]
                lines += info['warnings']
                preview.set('표: 전체 원본 수 / 동일 Crop을 제외한 고유 원본 수\n'+'\n'.join(lines))
                return options
            except Exception as exc:
                table.delete(*table.get_children())
                preview.set(str(exc))
                messagebox.showerror('분배 설정 오류', str(exc), parent=window)
                return None

        def run():
            options = show_plan()
            if options is None:
                return
            try:
                project, size, count = self.catalog.root, self.image_size(), int(self.augment.get())
                step = float(self.angle_step.get())
                window.destroy()
                self.job('원본 분배 후 Train 증강·내보내기 중…', lambda progress: export_dataset(
                    project, size, count, cancel=self.cancel, progress=progress, angle_step=step, **options))
            except Exception as exc:
                messagebox.showerror('내보내기 설정 오류', str(exc), parent=self.root)

        ttk.Button(body, text='분배 미리보기', command=show_plan).grid(row=11, column=0, pady=8)
        ttk.Button(body, text='분배 후 내보내기', command=run).grid(row=11, column=1, pady=8)
        ttk.Button(body, text='취소', command=window.destroy).grid(row=11, column=2, pady=8)

    def statistics(self):
        rows = self.catalog.samples()
        counts = {label: 0 for label in CLASSES}
        for row in rows:
            if row['status'] == 'confirmed':
                counts[row['label']] += 1
        lines = ['정답 확정 원본 수 (분배·증강 전)']
        lines += [f'{CLASSES[k]}: {count}장' for k, count in counts.items()]
        lines += [f'전체 {len(rows)} · 확정 {sum(counts.values())} · '
                  f'보류 {sum(r["status"]=="hold" for r in rows)} · 제외 {sum(r["status"]=="rejected" for r in rows)}',
                  'Train/Val/Test는 내보낼 때 선택한 비율로 분배합니다.']
        messagebox.showinfo('데이터셋 통계', '\n'.join(lines), parent=self.root)

    def manage_sources(self):
        if self.busy():
            return
        window = tk.Toplevel(self.root)
        window.title('원본별 촬영 그룹 관리')
        tree = ttk.Treeview(window, columns=('file', 'group'), show='headings', height=10)
        for key, title in [('file', '영상'), ('group', '촬영 그룹')]:
            tree.heading(key, text=title)
        tree.pack(fill='both', expand=True, padx=8, pady=8)
        group = tk.StringVar()
        ttk.Entry(window, textvariable=group).pack(fill='x', padx=8)

        def refresh():
            tree.delete(*tree.get_children())
            for source in self.catalog.sources():
                tree.insert('', 'end', iid=source['id'], values=(Path(source['path']).name, source['group_name']))

        def select(event):
            if tree.selection():
                values = tree.item(tree.selection()[0], 'values')
                group.set(values[1])

        def apply():
            if tree.selection():
                try:
                    self.catalog.rename_group(tree.selection()[0], group.get())
                    refresh()
                    self.refresh()
                except Exception as exc:
                    messagebox.showerror('그룹 수정 오류', str(exc), parent=window)

        tree.bind('<<TreeviewSelect>>', select)
        ttk.Button(window, text='촬영 그룹 이름 적용', command=apply).pack(pady=8)
        refresh()

    def tick(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'progress':
                self.progress['value'] = value
            else:
                self.refresh()
                self.status.set(('완료: ' if kind == 'done' else '오류: ')+value)
                if kind == 'error' and not self.closing:
                    messagebox.showerror('작업 오류', value, parent=self.root)
        if self.closing and not self.busy():
            self.catalog.close()
            self.root.destroy()
            return
        self.root.after(100, self.tick)

    def quit(self):
        self.cancel.set()
        self.closing = True
        self.status.set('작업을 마무리하고 종료 중…')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=ROOT/'tools/datasets/janggi_pieces')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--extract', type=Path, help='GUI 없이 영상에서 Crop 추출')
    action.add_argument('--export', action='store_true', help='GUI 없이 정답 데이터셋 내보내기')
    parser.add_argument('--interval', type=float, default=2.)
    parser.add_argument('--conf', type=float, default=.35)
    parser.add_argument('--margin', type=float, default=0., help='원본 Crop 저장 여백; 기본 0')
    parser.add_argument('--group')
    parser.add_argument('--split', choices=SPLITS, default='train', help=argparse.SUPPRESS)
    parser.add_argument('--ratios', type=float, nargs=3, default=(70, 15, 15), metavar=('TRAIN', 'VAL', 'TEST'), help='내보내기 분배 비율(%%); 합계 100')
    parser.add_argument('--min-per-class', type=int, nargs=3, default=(1, 1, 1), metavar=('TRAIN', 'VAL', 'TEST'), help='클래스당 최소 원본 수; 기본 1 1 1, 0이면 제한 해제')
    parser.add_argument('--split-mode', choices=('random', 'ordered'), default='random')
    parser.add_argument('--split-unit', choices=('sample', 'group'), default='sample')
    parser.add_argument('--model', type=Path, default=ROOT/'models/weights_only_piece.pt')
    parser.add_argument('--device')
    parser.add_argument('--limit-frames', type=int, help='확인용 최대 추출 프레임 수')
    parser.add_argument('--augment', type=int, default=5, help='Train 원본당 추가 회전 증강 개수')
    parser.add_argument('--size', type=int, default=224)
    parser.add_argument('--angle-step', type=float, default=.1, help='회전 각도 간격(도); 기본 0.1, 최소 0.001')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if args.limit_frames is not None and args.limit_frames <= 0:
        parser.error('--limit-frames는 양수여야 합니다.')
    if args.extract:
        print(extract_video(args.project, args.extract, args.interval, args.conf, args.margin, args.group,
                            args.split, args.model, args.device, args.limit_frames))
    elif args.export:
        print(export_dataset(args.project, args.size, args.augment, args.seed, angle_step=args.angle_step,
                             ratios=args.ratios, split_mode=args.split_mode, split_unit=args.split_unit, min_per_class=args.min_per_class))
    else:
        window = tk.Tk()
        DatasetApp(window, args)
        window.mainloop()
