"""Tk operator workflow. Hardware calls run on one worker; Tk stays on its thread."""
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk
import webbrowser

from .controller import MockController, RosController
from .model import MeasurementSession, number
from .report import export_fit


class CalibrationApp:
    def __init__(self, args):
        self.args=args;self.controller=None;self.busy=False;self.closing=False;self.fit=None
        self.events=queue.Queue();self.root=tk.Tk();self.root.title('그리퍼 너비·플랜지 높이 실측 보정')
        self.session=MeasurementSession.load(args.resume) if args.resume else MeasurementSession(args.output,backend=args.backend,namespace=args.ns)
        self.backend=tk.StringVar(value=args.backend);self.namespace=tk.StringVar(value=args.ns)
        self.x=tk.StringVar(value='400');self.y=tk.StringVar(value='0');self.step=tk.StringVar(value='0.1')
        if self.session.samples:
            self.x.set(f'{self.session.samples[0].flange_pose[0]:g}');self.y.set(f'{self.session.samples[0].flange_pose[1]:g}')
        self.width=tk.StringVar();self.evaluate_width=tk.StringVar()
        self.status=tk.StringVar(value='연결 후 XY 위치를 잡고 [보정 시작]을 누르세요.')
        self.pose_text=tk.StringVar(value='마지막 읽은 6번 플랜지 좌표: 연결 전')
        self.current=tk.StringVar();self.controls=[];self._build();self.refresh()
        self.root.protocol('WM_DELETE_WINDOW',self.quit)
        self.timer=self.root.after(50,self.poll)

    def button(self,parent,text,command,**kwargs):
        widget=ttk.Button(parent,text=text,command=command);widget.pack(**kwargs)
        self.controls.append(widget);return widget

    def _build(self):
        frame=ttk.Frame(self.root,padding=12);frame.pack(fill='both',expand=True)
        row=ttk.Frame(frame);row.pack(fill='x')
        ttk.Label(row,text='연결 모드').pack(side='left')
        self.backend_box=ttk.Combobox(row,textvariable=self.backend,values=('real','virtual','mock'),state='readonly',width=9)
        self.backend_box.pack(side='left',padx=5)
        ttk.Label(row,text='네임스페이스').pack(side='left')
        self.namespace_input=ttk.Entry(row,textvariable=self.namespace,width=12)
        self.namespace_input.pack(side='left',padx=5)
        self.connect_button=self.button(row,'연결',self.connect,side='left',padx=5)
        # Stop remains usable while other controls are disabled.
        ttk.Button(row,text='로봇 정지',command=self.stop).pack(side='right')
        ttk.Label(frame,text='기준: DR_BASE · unnamed J6 플랜지 · 개폐면 XZ · ABC [0°,180°,0°]').pack(anchor='w',pady=6)
        ttk.Label(frame,textvariable=self.pose_text,font=('TkDefaultFont',11,'bold')).pack(anchor='w')
        setup=ttk.LabelFrame(frame,text='1. 측정 위치 설정',padding=8);setup.pack(fill='x',pady=8)
        row=ttk.Frame(setup);row.pack(fill='x')
        for label,var in (('X (mm)',self.x),('Y (mm)',self.y)):
            ttk.Label(row,text=label).pack(side='left');ttk.Entry(row,textvariable=var,width=11).pack(side='left',padx=5)
        self.xy_button=self.button(row,'현재 Z 유지 · 자세 맞춤 + XY 이동',self.move_xy,side='left',padx=5)
        self.button(row,'현재 좌표 읽기',self.read_pose,side='left')
        self.button(row,'호밍 초기화',self.home,side='left',padx=5)
        ttk.Label(setup,text='이 프로그램의 홈 관절각: [0, 0, 90, 0, 90, 0]°').pack(anchor='w',pady=(5,0))
        ttk.Label(setup,text='XY 이동은 현재 플랜지 Z에서 방향을 먼저 맞춥니다. 자세를 돌릴 공간에서 시작하세요. 측정 중 XY는 첫 기록 위치로 고정됩니다.').pack(anchor='w',pady=(5,0))
        jog=ttk.LabelFrame(frame,text='2. Z 수동 미세 이동 (자동 바닥 접촉 감지 없음)',padding=8);jog.pack(fill='x')
        ttk.Label(jog,text='이동량 (양수 mm, 소수점 가능)').pack(side='left')
        ttk.Entry(jog,textvariable=self.step,width=10).pack(side='left',padx=5)
        self.button(jog,'↑ 위로',lambda:self.jog(1),side='left',padx=5)
        self.button(jog,'↓ 아래로',lambda:self.jog(-1),side='left',padx=5)
        measure=ttk.LabelFrame(frame,text='3. 700 → 650 → … → 0 측정',padding=8);measure.pack(fill='x',pady=8)
        ttk.Label(measure,textvariable=self.current,font=('TkDefaultFont',11,'bold')).pack(anchor='w')
        row=ttk.Frame(measure);row.pack(fill='x',pady=5)
        self.start_button=self.button(row,'보정 시작 (700)',self.start,side='left')
        self.apply_button=self.button(row,'현재 단계 그리퍼 적용',self.apply_current,side='left',padx=5)
        ttk.Label(row,text='자로 잰 전체 너비 (mm)').pack(side='left')
        self.width_input=ttk.Entry(row,textvariable=self.width,width=10);self.width_input.pack(side='left',padx=5)
        self.next_button=self.button(row,'기록하고 다음',self.next,side='left',padx=5)
        self.undo_button=self.button(row,'이전 측정 다시하기',self.undo,side='left')
        ttk.Label(measure,text='각 단계에서 Z를 조절해 같은 바닥에 닿기 직전까지 내린 뒤, 손가락 안쪽 간격을 입력하세요.').pack(anchor='w')
        columns=('command','width','z','relative')
        self.table=ttk.Treeview(frame,columns=columns,show='headings',height=15)
        for key,label in zip(columns,('그리퍼 명령','실측 전체 너비 mm','실제 플랜지 Z mm','0 기준 상대 높이 mm')):
            self.table.heading(key,text=label);self.table.column(key,width=180,anchor='e')
        self.table.pack(fill='both',expand=True)
        result=ttk.LabelFrame(frame,text='4. 결과',padding=8);result.pack(fill='x',pady=8)
        self.result_text=tk.StringVar(value='0 단계까지 기록하면 원 피팅과 그래프를 자동 저장합니다.')
        ttk.Label(result,textvariable=self.result_text,wraplength=900,justify='left').pack(anchor='w')
        row=ttk.Frame(result);row.pack(fill='x',pady=5)
        self.button(row,'피팅 다시 계산',self.calculate,side='left')
        ttk.Button(row,text='결과 HTML 열기',command=self.open_report).pack(side='left',padx=5)
        ttk.Label(row,text='확인할 너비 (mm)').pack(side='left')
        ttk.Entry(row,textvariable=self.evaluate_width,width=10).pack(side='left',padx=5)
        ttk.Button(row,text='0 기준 높이 계산',command=self.evaluate).pack(side='left')
        ttk.Label(frame,textvariable=self.status,wraplength=950,foreground='#155e75').pack(anchor='w',pady=4)
        ttk.Label(frame,text=f'저장 폴더: {self.session.directory}',wraplength=950).pack(anchor='w')
        ttk.Label(frame,text=f'XY {self.args.velocity:g}mm/s · {self.args.acceleration:g}mm/s² / '
                  f'Z {self.args.z_velocity:g}mm/s · {self.args.z_acceleration:g}mm/s² / '
                  f'홈 {self.args.angular_velocity:g}°/s · {self.args.angular_acceleration:g}°/s² · '
                  'TCP/게임 보정값 자동 변경 없음',foreground='#555',wraplength=950).pack(anchor='w')

    def job(self,label,fn,success=None,*,requires_connection=True):
        if self.busy:return
        if requires_connection and self.controller is None:
            messagebox.showerror('연결 필요','먼저 로봇에 연결하세요.',parent=self.root);return
        self.busy=True;self.status.set(label)
        if self.controller is not None:self.controller.stopped.clear()
        self.refresh()
        def run():
            try:self.events.put((True,fn(),success))
            except Exception as exc:self.events.put((False,str(exc),None))
        threading.Thread(target=run,daemon=True).start()

    def poll(self):
        try:
            while True:
                okay,result,callback=self.events.get_nowait();self.busy=False
                if okay:
                    if callback:callback(result)
                    else:self.status.set('완료')
                else:
                    self.status.set('오류: '+result)
                    if not self.closing:messagebox.showerror('보정 도구',result,parent=self.root)
                self.refresh()
        except queue.Empty:pass
        if self.closing and not self.busy:
            if self.controller is not None:self.controller.close()
            self.root.destroy();return
        self.timer=self.root.after(50,self.poll)

    def pose(self,value):
        self.pose_text.set('마지막 읽은 6번 플랜지: '+ '  '.join(f'{k}={v:.4f}' for k,v in zip(('X','Y','Z','A','B','C'),value)))
        self.status.set('완료 · 플랜지 높이이며 팁 오프셋은 가산하지 않았습니다.')

    def connect(self):
        if self.controller is not None:return
        backend=self.backend.get();namespace=self.namespace.get().strip('/')
        if self.session.samples and (self.session.backend not in ('unknown',backend) or self.session.namespace!=namespace):
            messagebox.showerror('측정 재개','기록과 같은 backend/네임스페이스로 연결하세요.',parent=self.root);return
        def fn():
            controller=(MockController() if backend=='mock' else RosController(backend=backend,namespace=namespace,
                        velocity=self.args.velocity,acceleration=self.args.acceleration,
                        angular_velocity=self.args.angular_velocity,angular_acceleration=self.args.angular_acceleration,
                        z_velocity=self.args.z_velocity,z_acceleration=self.args.z_acceleration))
            self.controller=controller
            return controller.read()
        def connected(pose):
            self.session.backend=backend;self.session.namespace=namespace
            self.pose(pose);self.status.set(('MOCK: 실제 로봇이 움직이지 않습니다. ' if backend=='mock' else '')+'연결 완료 · 아직 이동/그리퍼 명령을 보내지 않았습니다.')
        self.job('로봇 상태 읽는 중',fn,connected,requires_connection=False)

    def move_xy(self):
        try:
            if self.session.samples:
                x,y=self.session.samples[0].flange_pose[:2]
                self.x.set(f'{x:.4f}');self.y.set(f'{y:.4f}')
            else:x=number(self.x.get(),'X');y=number(self.y.get(),'Y')
        except ValueError as exc:self.status.set(str(exc));return
        self.job('자세 맞춤 및 XY 이동 중',lambda:self.controller.move_xy(x,y),self.pose)

    def read_pose(self):self.job('현재 플랜지 좌표 읽는 중',lambda:self.controller.read(),self.pose)
    def home(self):self.job('홈 관절각 [0, 0, 90, 0, 90, 0]°로 이동 중',lambda:self.controller.home(),self.pose)
    def jog(self,direction):
        try:
            step=number(self.step.get(),'Z 이동량')
            if step<=0:raise ValueError('이동량은 양수로 입력하고 위/아래 버튼을 선택하세요.')
        except ValueError as exc:self.status.set(str(exc));return
        self.job('Z 이동 중',lambda:self.controller.jog(direction*step),self.pose)

    def start(self):
        if self.session.samples or self.session.active_command is not None:return
        self.apply_current()

    def apply_current(self):
        command=self.session.next_command
        if command is None:return
        def fn():
            from janggi_pick_place2.geometry import orientation_error_deg
            pose=self.controller.read()
            if orientation_error_deg(pose[3:],[0.,180.,0.])>.1:
                raise ValueError('먼저 XY 이동 버튼으로 XZ 개폐 자세를 맞추세요.')
            self.controller.set_gripper(command);self.session.mark_active(command)
            return self.controller.read()
        def ready(pose):
            self.pose(pose);self.width.set('');self.width_input.focus_set()
            self.status.set(f'그리퍼 {command} 적용 완료 · Z를 조절하고 자로 잰 너비를 입력하세요.')
        self.job(f'그리퍼 {command} 적용 중',fn,ready)

    def next(self):
        try:
            width=number(self.width.get(),'실측 전체 너비')
            if width<0:raise ValueError('너비는 0 이상이어야 합니다.')
        except ValueError as exc:self.status.set(str(exc));return
        def fn():
            from janggi_pick_place2.geometry import orientation_error_deg
            pose=self.controller.read()
            if orientation_error_deg(pose[3:],[0.,180.,0.])>.1:raise ValueError('플랜지 자세가 XZ 개폐 자세와 다릅니다.')
            self.session.record(width,pose)
            if self.session.next_command is None:return ('fit',export_fit(self.session),pose)
            command=self.session.next_command
            self.controller.set_gripper(command);self.session.mark_active(command)
            return ('step',command,self.controller.read())
        def done(result):
            kind,value,pose=result;self.pose(pose);self.width.set('')
            if kind=='fit':self.show_fit(value)
            else:self.status.set(f'기록 저장 완료 · 그리퍼 {value} 적용 완료. 다음 측정을 진행하세요.')
        self.job('실제 플랜지 Z 기록 및 다음 단계 적용 중',fn,done)

    def undo(self):
        if not self.session.samples:return
        def fn():
            sample=self.session.samples.pop();self.session.active_command=None
            try:self.session.save()
            except BaseException:self.session.samples.append(sample);raise
            self.controller.set_gripper(self.session.next_command);self.session.mark_active(self.session.next_command)
            return self.controller.read()
        self.fit=None;self.job('이전 측정 단계 다시 적용 중',fn,self.pose)

    def calculate(self):
        self.job('원 피팅 계산 중',lambda:export_fit(self.session),self.show_fit,requires_connection=False)

    def show_fit(self,fit):
        self.fit=fit
        self.result_text.set(f'(w/2 − ({fit.center_half_width_mm:.5f}))² + (h − ({fit.center_relative_z_mm:.5f}))² = '
                             f'{fit.radius_mm:.5f}²\n반지름 {fit.radius_mm:.5f}mm · 방사 RMSE {fit.radial_rmse_mm:.5f}mm · '
                             f'최대 오차 {fit.radial_max_error_mm:.5f}mm · command 0 플랜지 Z={fit.reference_flange_z_mm:.4f}mm')
        self.status.set('피팅·CSV·그래프·HTML 저장 완료. 기존 로봇 보정에는 적용하지 않았습니다.')

    def evaluate(self):
        if self.fit is None:self.status.set('먼저 피팅을 계산하세요.');return
        try:
            width=number(self.evaluate_width.get(),'너비')
            measured=[s.width_mm for s in self.session.samples]
            if not min(measured)<=width<=max(measured):raise ValueError('실측 너비 범위 안에서만 계산하세요.')
            self.status.set(f'너비 {width:g}mm의 command 0 대비 플랜지 높이 변화: {self.fit.height_from_open(width):+.5f}mm')
        except ValueError as exc:self.status.set(str(exc))

    def open_report(self):
        path=self.session.directory/'report.html'
        if path.exists():webbrowser.open(path.resolve().as_uri())
        else:self.status.set('아직 결과 HTML이 없습니다. 0까지 기록하고 피팅을 계산하세요.')

    def refresh(self):
        for widget in self.controls:widget.state(['disabled'] if self.busy else ['!disabled'])
        if self.controller is not None:self.connect_button.state(['disabled'])
        self.backend_box.configure(state='disabled' if self.busy or self.controller is not None else 'readonly')
        self.namespace_input.configure(state='disabled' if self.busy or self.controller is not None else 'normal')
        if self.session.samples or self.session.active_command is not None:self.start_button.state(['disabled'])
        if not self.session.samples:self.undo_button.state(['disabled'])
        if self.session.next_command is None:self.next_button.state(['disabled']);self.apply_button.state(['disabled'])
        elif self.session.active_command!=self.session.next_command:self.next_button.state(['disabled'])
        command=self.session.next_command
        self.current.set(f'완료 {len(self.session.samples)}/15 · '+ ('모든 측정 완료' if command is None else
                        f'현재 단계 {command} · '+('적용됨' if self.session.active_command==command else '그리퍼 적용 대기')))
        for item in self.table.get_children():self.table.delete(item)
        for sample in self.session.samples:
            relative='0 측정 후 계산' if self.session.reference_z is None else f'{sample.flange_pose[2]-self.session.reference_z:.5f}'
            self.table.insert('','end',values=(sample.command,f'{sample.width_mm:.5f}',f'{sample.flange_pose[2]:.5f}',relative))

    def stop(self):
        if self.controller is not None:self.controller.stop()
        self.status.set('정지 요청 · 진행 중인 다음 단계는 실행하지 않습니다.')

    def quit(self):
        self.closing=True
        if self.busy:self.stop()

    def run(self):self.root.mainloop()
