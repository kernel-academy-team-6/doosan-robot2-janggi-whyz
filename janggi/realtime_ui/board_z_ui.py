"""Four-corner relative bed leveling controls; no robot calls in this dialog."""
from dataclasses import replace
import tkinter as tk
from tkinter import messagebox

Z_FIELDS = ('board_z_compensation_enabled', 'board_z_offset_mm', 'board_z_corner_offsets_mm')
LABELS = ('좌상단 (0,0)', '우상단 (8,0)', '우하단 (8,9)', '좌하단 (0,9)')


class BoardZDialog:
    def __init__(self, parent, geometry, settings, apply, on_close):
        self.geometry, self.settings, self.apply, self.on_close = geometry, settings, apply, on_close
        self.window = tk.Toplevel(parent)
        self.window.title('판 Z 높이 미세 보정')
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.enabled = tk.BooleanVar(value=settings.board_z_compensation_enabled)
        tk.Checkbutton(self.window, text='수동 미세 보정 사용 (기본 꺼짐)', variable=self.enabled).grid(row=0, columnspan=3, pady=8)
        tk.Label(self.window, text=f'기본 판 Z={settings.command_board_z_mm:g}mm 고정 · 과거 높이 차이는 자동 적용 안 함\n'
                 '수동 보정 양수: 팁을 위로 / 음수: 팁을 아래로 · 단위 mm').grid(row=1, columnspan=3, padx=12, pady=8)
        tk.Label(self.window, text='전체 추가 보정').grid(row=2, column=0, padx=12)
        self.offset = tk.StringVar(value=f'{settings.board_z_offset_mm:g}')
        tk.Entry(self.window, textvariable=self.offset, width=10).grid(row=2, column=2)
        for column,label in enumerate(('모서리', '과거 높이차 (미적용)', '수동 보정')):
            tk.Label(self.window, text=label).grid(row=3, column=column, padx=12, pady=8)
        self.corners = []
        delta = geometry.corners_mm[:,2] - geometry.corners_mm[:,2].min()
        for index,label in enumerate(LABELS):
            tk.Label(self.window, text=label).grid(row=4+index, column=0, padx=12)
            tk.Label(self.window, text=f'+{delta[index]:.3f}').grid(row=4+index, column=1)
            var = tk.StringVar(value=f'{settings.board_z_corner_offsets_mm[index]:g}')
            self.corners.append(var)
            tk.Entry(self.window, textvariable=var, width=10).grid(row=4+index, column=2, pady=3, padx=12)
        self.summary = tk.Label(self.window, justify='left')
        self.summary.grid(row=8, columnspan=3, padx=12, pady=8)
        tk.Button(self.window, text='다음 작업부터 적용', command=self.submit).grid(row=9, columnspan=3, pady=8)
        tk.Label(self.window, text='동작 중 적용 불가 · 현재 실행에만 유지 · 판 밖 버리는 위치는 별도',
                 fg='#555').grid(row=10, columnspan=3, padx=12, pady=8)
        for var in [self.enabled,self.offset,*self.corners]:
            var.trace_add('write', lambda *_:self.preview())
        self.preview()

    def result(self):
        return replace(self.settings, board_z_compensation_enabled=self.enabled.get(),
                       board_z_offset_mm=float(self.offset.get()),
                       board_z_corner_offsets_mm=tuple(float(var.get()) for var in self.corners)).validate()

    def preview(self):
        try:
            settings = self.result()
            heights = [self.geometry.command_surface_z(point, settings) for point in self.geometry.corners_mm]
            self.summary.config(text='보정 후 판 높이 (팁 집기 높이 제외):\n' +
                                ' / '.join(f'{name}: {z:.2f}' for name,z in zip(('좌상','우상','우하','좌하'),heights)))
        except (ValueError, tk.TclError):
            self.summary.config(text='보정값에 유한한 숫자를 입력하세요.')

    def submit(self):
        try:
            self.apply(self.result())
        except (ValueError, RuntimeError, tk.TclError) as exc:
            messagebox.showerror('판 Z 보정', str(exc), parent=self.window)
            return
        self.close()

    def close(self):
        self.on_close()
        self.window.destroy()
