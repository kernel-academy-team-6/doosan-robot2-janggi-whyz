from collections import deque
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import App
from action.board import BoardGeometry
from action.pick_place2 import load_pick_place_settings
import board_z_ui


def test_z_dialog_applies_current_fields_without_robot_or_display(monkeypatch):
    class Widget:
        def __init__(self,*args,**kwargs): self.options=kwargs
        def grid(self,**kwargs): pass
        def title(self,*args): pass
        def protocol(self,*args): pass
        def config(self,**kwargs): self.options.update(kwargs)
        def destroy(self): pass
    class Var:
        def __init__(self,value=None): self.value=value
        def get(self): return self.value
        def trace_add(self,*args): pass
    for name in ('Toplevel','Label','Checkbutton','Entry','Button'):
        monkeypatch.setattr(board_z_ui.tk,name,Widget)
    for name in ('BooleanVar','StringVar'):
        monkeypatch.setattr(board_z_ui.tk,name,Var)
    settings,_=load_pick_place_settings(); applied=[]
    dialog=board_z_ui.BoardZDialog(None,BoardGeometry.from_calibration(),settings,applied.append,lambda:None)
    assert not dialog.enabled.get()
    dialog.enabled.value=True
    dialog.offset.value='1.5'; dialog.corners[0].value='2'
    dialog.preview(); assert '좌상: -16.50' in dialog.summary.options['text']
    dialog.submit()
    assert applied[0].board_z_offset_mm==1.5 and applied[0].board_z_corner_offsets_mm==(2.,0.,0.,0.)


def test_z_and_motion_dialogs_merge_fields_and_busy_update_is_rejected():
    settings,_=load_pick_place_settings()
    ui=App.__new__(App); ui.robot_busy=False; ui.rg=None; ui._msgs=deque()
    ui._motion_settings=replace(settings,linear_velocity_mm_s=40.)
    ui._apply_board_z_settings(replace(settings,board_z_offset_mm=1.))
    assert ui._motion_settings.linear_velocity_mm_s==40
    ui._apply_motion_settings(replace(settings,linear_velocity_mm_s=50.))
    assert ui._motion_settings.board_z_offset_mm==1.
    assert ui._motion_settings.linear_velocity_mm_s==50
    ui.robot_busy=True
    before=ui._motion_settings
    with pytest.raises(RuntimeError): ui._apply_board_z_settings(settings)
    assert ui._motion_settings is before
