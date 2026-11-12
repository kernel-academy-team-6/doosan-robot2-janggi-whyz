"""Editable RGB sensor settings window; all SDK writes stay in capture thread."""
import tkinter as tk
from tkinter import ttk

from realsense_settings import OPTION_SPECS, PROFILE_PATH, apply_saved, save_profile


class CameraSettingsDialog:
    def __init__(self,parent,control,on_close=lambda:None):
        self.control,self.on_close=control,on_close
        self.window=tk.Toplevel(parent)
        self.window.title('RealSense RGB 카메라 설정')
        self.window.transient(parent)
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.variables={};self.widgets={};self.actual={}
        self.notice='입력값을 수정한 뒤 [현재값 적용]을 누르세요.'
        self.awaiting=False;self.timer=None
        body=ttk.Frame(self.window,padding=10);body.pack(fill='both',expand=True)
        for col,text in enumerate(['옵션','설정값','카메라 값 / 허용 범위']):
            ttk.Label(body,text=text).grid(row=0,column=col,sticky='w',padx=4,pady=3)
        state=control.snapshot()
        for row,(key,(label,kind)) in enumerate(OPTION_SPECS.items(),1):
            meta=state['controls'].get(key,{})
            ttk.Label(body,text=label).grid(row=row,column=0,sticky='w',padx=4,pady=3)
            value=meta.get('value')
            variable=tk.BooleanVar(value=bool(value)) if kind=='bool' else tk.StringVar(value=f'{value:g}' if value is not None else '')
            self.variables[key]=variable
            if kind=='bool':
                widget=ttk.Checkbutton(body,variable=variable,command=self._enable_inputs)
            elif kind=='enum':
                choices=[f'{number}: {description}' for number,description in meta.get('choices',{}).items()]
                widget=ttk.Combobox(body,textvariable=variable,values=choices,state='readonly',width=18)
                if value is not None:self._set_value(key,value,state)
            else:
                widget=tk.Spinbox(body,textvariable=variable,width=18,
                                  from_=meta.get('minimum') or 0,to=meta.get('maximum') or 1,
                                  increment=meta.get('step') or 1)
            widget.grid(row=row,column=1,sticky='ew',padx=4,pady=3)
            self.widgets[key]=widget
            actual=ttk.Label(body,text='');actual.grid(row=row,column=2,sticky='w',padx=4)
            self.actual[key]=actual
        buttons=ttk.Frame(body);buttons.grid(row=len(OPTION_SPECS)+1,column=0,columnspan=3,sticky='ew',pady=8)
        ttk.Button(buttons,text='현재값 적용',command=self.apply_current).pack(side='left',padx=3)
        ttk.Button(buttons,text='현재값 저장',command=self.save_current).pack(side='left',padx=3)
        ttk.Button(buttons,text='저장값 적용',command=self.apply_saved).pack(side='left',padx=3)
        ttk.Label(body,text='자동 노출 사용 시 노출·게인, 자동 WB 사용 시 화이트밸런스 수동값은 고정하지 않습니다.').grid(
            row=len(OPTION_SPECS)+2,column=0,columnspan=3,sticky='w',pady=3)
        self.status=ttk.Label(body,text='',wraplength=650)
        self.status.grid(row=len(OPTION_SPECS)+3,column=0,columnspan=3,sticky='w',pady=3)
        self._poll()

    def _set_value(self,key,value,state=None):
        kind=OPTION_SPECS[key][1]
        if kind=='bool':self.variables[key].set(bool(value))
        elif kind=='enum':
            meta=(state or self.control.snapshot())['controls'].get(key,{})
            description=meta.get('choices',{}).get(int(value),str(int(value)))
            self.variables[key].set(f'{int(value)}: {description}')
        else:self.variables[key].set(f'{float(value):g}')

    def set_form(self,values):
        state=self.control.snapshot()
        for key,value in values.items():
            if key in self.variables:self._set_value(key,value,state)
        self._enable_inputs(state)

    def _collect(self):
        state=self.control.snapshot();values={}
        for key,variable in self.variables.items():
            if not state['controls'].get(key,{}).get('writable'):continue
            raw=variable.get()
            if OPTION_SPECS[key][1]=='enum':raw=raw.split(':',1)[0]
            values[key]=float(raw)
        return values

    def _enable_inputs(self,state=None):
        state=state or self.control.snapshot()
        auto_exp=self.variables['enable_auto_exposure'].get()
        auto_wb=self.variables['enable_auto_white_balance'].get()
        for key,widget in self.widgets.items():
            enabled=state['connected'] and state['controls'].get(key,{}).get('writable',False)
            if key in ('exposure','gain') and auto_exp:enabled=False
            if key=='white_balance' and auto_wb:enabled=False
            widget.configure(state=('readonly' if OPTION_SPECS[key][1]=='enum' else 'normal') if enabled else 'disabled')

    def apply_current(self):
        try:
            values=self.control.request_settings(self._collect())
            self.set_form(values);self.awaiting=True;self.notice='입력값 적용 요청'
        except (ValueError,RuntimeError) as exc:
            self.awaiting=False;self.notice=str(exc)

    def save_current(self):
        try:
            values=self.control.validate_settings(self._collect())
            save_profile(values)
            self.awaiting=False
            self.notice='저장 완료: '+str(PROFILE_PATH)
        except (OSError,ValueError,RuntimeError) as exc:
            self.awaiting=False;self.notice=str(exc)

    def apply_saved(self):
        try:
            values=apply_saved(self.control)
            self.set_form(values);self.awaiting=True;self.notice='저장값 적용 요청'
        except (OSError,ValueError,RuntimeError) as exc:
            self.awaiting=False;self.notice=str(exc)

    def _poll(self):
        if self.window is None:return
        state=self.control.snapshot();self._enable_inputs(state)
        for key,label in self.actual.items():
            meta=state['controls'].get(key,{})
            if not meta.get('supported'):text='미지원'
            elif meta.get('kind')=='bool':text='켜짐' if meta['value'] else '꺼짐'
            else:text=f"{meta['value']:g} / {meta['minimum']:g}~{meta['maximum']:g}"
            if meta.get('supported') and not meta.get('writable'):text+=' (읽기 전용)'
            label.configure(text=text)
        if self.awaiting and not state['pending']:
            self.awaiting=False
            if not state['error']:self.notice='적용 완료'
        text=state['error'] or ('카메라 연결 대기 중' if not state['connected'] else
                               '적용 대기 중…' if state['pending'] else self.notice)
        if state['skipped']:text+=' · 미지원/읽기 전용 제외: '+', '.join(state['skipped'])
        self.status.configure(text=text)
        self.timer=self.window.after(250,self._poll)

    def close(self):
        if self.window is not None:
            if self.timer is not None:self.window.after_cancel(self.timer)
            self.window.destroy();self.window=None
        self.on_close()
