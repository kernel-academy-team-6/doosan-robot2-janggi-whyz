"""Idle-only robot motion settings; importing this module does not connect ROS."""
from dataclasses import replace
import math
import tkinter as tk
from tkinter import messagebox

FIELDS = (
    ('linear_velocity_mm_s', '직선 속도 (mm/s)'),
    ('linear_acceleration_mm_s2', '직선 가속도 (mm/s²)'),
    ('angular_velocity_deg_s', '회전·관절 속도 (°/s)'),
    ('angular_acceleration_deg_s2', '회전·관절 가속도 (°/s²)'),
)


def motion_settings(settings, values, percent=100., linked=True):
    percent = float(percent)
    if not math.isfinite(percent) or percent <= 0:
        raise ValueError('조절 비율은 0보다 큰 유한한 숫자여야 합니다.')
    updates = {}
    for name, label in FIELDS:
        value = float(values[name])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{label}: 0보다 큰 유한한 숫자를 입력하세요.')
        scale = percent / 100 if linked or 'velocity' in name else 1.
        updates[name] = value * scale
    # J6-locked paths and homing use joint limits rather than MoveL limits.
    updates['joint_velocity_deg_s'] = updates['angular_velocity_deg_s']
    updates['joint_acceleration_deg_s2'] = updates['angular_acceleration_deg_s2']
    return replace(settings, **updates).validate()


class RobotMotionDialog:
    def __init__(self, parent, settings, apply, on_close):
        self.settings, self.apply, self.on_close = settings, apply, on_close
        self.window = tk.Toplevel(parent)
        self.window.title('로봇 속도·가속도')
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.values = {}
        self.anchors = {name:float(getattr(settings, name)) for name,_ in FIELDS}
        self.pending_field = None
        self.updating = False
        tk.Label(self.window, text='기준값을 입력한 뒤 비율로 함께 조절하세요.').grid(
            row=0, column=0, columnspan=2, padx=12, pady=8)
        for row, (name, label) in enumerate(FIELDS, 1):
            tk.Label(self.window, text=label).grid(row=row, column=0, sticky='w', padx=12)
            var = tk.StringVar(value=f'{getattr(settings, name):g}')
            self.values[name] = var
            entry = tk.Entry(self.window, textvariable=var, width=12)
            entry.grid(row=row, column=1, padx=12, pady=3)
            entry.bind('<FocusOut>', lambda event,key=name:self.commit_edit(key))
        self.linked = tk.BooleanVar(value=True)
        tk.Checkbutton(self.window, text='통합 조절: 직접 입력·슬라이더 모두 속도·가속도 비례 연동',
                       variable=self.linked, command=self.preview).grid(row=5, columnspan=2, pady=6)
        self.percent = tk.DoubleVar(value=100.)
        tk.Scale(self.window, from_=10, to=200, resolution=5, orient='horizontal',
                 label='속도 조절 (%) · 해제 시 가속도는 기준값 유지',
                 variable=self.percent, command=lambda _:self.preview()).grid(
                     row=6, columnspan=2, sticky='ew', padx=12)
        self.summary = tk.Label(self.window, justify='left')
        self.summary.grid(row=7, columnspan=2, padx=12, pady=8)
        tk.Button(self.window, text='다음 동작부터 적용', command=self.submit).grid(
            row=8, columnspan=2, pady=8)
        tk.Label(self.window, text='동작 중에는 적용할 수 없습니다. 현재 실행에만 적용됩니다.',
                 fg='#555').grid(row=9, columnspan=2, padx=12, pady=8)
        for var in self.values.values():
            var.trace_add('write', lambda *_:self.preview())
        for name,var in self.values.items():
            var.trace_add('write', lambda *_,key=name:self.mark_edit(key))
        self.preview()

    def mark_edit(self, name):
        if not self.updating:
            self.pending_field = name

    def commit_edit(self, name):
        if self.pending_field != name:
            return
        try:
            current = {key:float(var.get()) for key,var in self.values.items()}
            if any(not math.isfinite(v) or v <= 0 for v in current.values()):
                return
            if self.linked.get():
                ratio = current[name] / self.anchors[name]
                current = {key:value*ratio for key,value in self.anchors.items()}
            self.updating = True
            for key,value in current.items():
                self.values[key].set(f'{value:.12g}')
            self.anchors = current
            self.pending_field = None
        except (ValueError, tk.TclError):
            return
        finally:
            self.updating = False
        self.preview()

    def result(self):
        return motion_settings(self.settings, {key:var.get() for key,var in self.values.items()},
                               self.percent.get(), self.linked.get())

    def preview(self):
        if not hasattr(self, 'summary'):
            return
        try:
            value = self.result()
            self.summary.config(text=(f'적용값: 직선 {value.linear_velocity_mm_s:g} mm/s · '
                                      f'{value.linear_acceleration_mm_s2:g} mm/s²\n'
                                      f'회전·관절 {value.angular_velocity_deg_s:g} °/s · '
                                      f'{value.angular_acceleration_deg_s2:g} °/s²'))
        except (ValueError, tk.TclError):
            self.summary.config(text='각 항목에 양수를 입력하세요.')

    def submit(self):
        try:
            if self.pending_field is not None:
                self.commit_edit(self.pending_field)
            self.apply(self.result())
        except (ValueError, RuntimeError, tk.TclError) as exc:
            messagebox.showerror('로봇 설정', str(exc), parent=self.window)
            return
        self.close()

    def close(self):
        self.on_close()
        self.window.destroy()
