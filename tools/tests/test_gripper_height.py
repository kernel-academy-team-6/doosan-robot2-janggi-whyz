import csv
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

WS=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(WS/'tools'))
from gripper_height.model import COMMANDS, MeasurementSession, fit_circle


def synthetic(tmp_path,noise=0.):
    session=MeasurementSession(tmp_path,backend='mock')
    random=np.random.default_rng(12)
    for command in COMMANDS:
        angle=math.radians(30.+60.*command/700.)
        width=2*(4.1+57*math.cos(angle))
        z=150.+57*(math.sin(angle)-.5)+random.normal(0,noise)
        session.mark_active(command);session.record(width,[400.,20.,z,0.,180.,0.])
    return session


def test_sequence_records_pose_and_zero_reference_and_survives_resume(tmp_path):
    s=MeasurementSession(tmp_path)
    with pytest.raises(ValueError):s.record(6,[400,0,175,0,180,0])
    s.mark_active(700);s.record(6,[400,0,175.125,0,180,0])
    assert s.next_command==650 and s.active_command is None and s.reference_z is None
    rows=list(csv.DictReader((tmp_path/'measurements.csv').open()))
    assert rows[0]['flange_z_mm']=='175.125' and rows[0]['relative_z_mm']==''
    resumed=MeasurementSession.load(tmp_path/'measurements.csv')
    assert resumed.next_command==650 and resumed.active_command is None
    resumed.mark_active(650)
    with pytest.raises(ValueError):resumed.record(float('nan'),[400,0,170,0,180,0])
    with pytest.raises(ValueError):resumed.record(12,[401,0,170,0,180,0])
    assert len(resumed.samples)==1


def test_circle_recovers_free_center_radius_and_compensation_sign(tmp_path):
    s=synthetic(tmp_path);fit=fit_circle(s)
    assert fit.center_half_width_mm==pytest.approx(4.1,abs=1e-5)
    assert fit.center_relative_z_mm==pytest.approx(-28.5,abs=1e-5)
    assert fit.radius_mm==pytest.approx(57.,abs=1e-5)
    assert fit.radial_rmse_mm<1e-6 and fit.branch==1
    assert fit.height_from_open(s.samples[-1].width_mm)==0.
    assert fit.height_from_open(s.samples[0].width_mm)==pytest.approx(28.5,abs=1e-5)
    rows=list(csv.DictReader((tmp_path/'measurements.csv').open()))
    assert float(rows[-1]['relative_z_mm'])==0.
    assert float(rows[0]['relative_z_mm'])==pytest.approx(28.5)


def test_noisy_fit_stays_finite_and_incomplete_or_collinear_data_is_not_fabricated(tmp_path):
    fit=fit_circle(synthetic(tmp_path/'noise',.02))
    assert fit.radius_mm==pytest.approx(57.,abs=.5) and fit.radial_rmse_mm<.1
    s=MeasurementSession(tmp_path/'empty')
    with pytest.raises(ValueError):fit_circle(s)
    for command in COMMANDS:
        s.mark_active(command);s.record(command/10,[400,0,150,0,180,0])
    with pytest.raises(ValueError,match='직선'):fit_circle(s)


def test_report_has_circle_plot_baseline_zero_and_no_robot_configuration_writes(tmp_path):
    from gripper_height.report import export_fit
    s=synthetic(tmp_path);fit=export_fit(s)
    assert (tmp_path/'fit.png').stat().st_size>1000
    assert '그리퍼 열림' in (tmp_path/'report.html').read_text()
    metadata=json.loads((tmp_path/'fit.json').read_text())
    assert not metadata['automatic_robot_compensation_applied']
    lookup=list(csv.DictReader((tmp_path/'lookup.csv').open()))
    assert len(lookup)==701 and float(lookup[0]['height_change_from_command_0_mm'])==0.
    assert float(lookup[-1]['height_change_from_command_0_mm'])==pytest.approx(28.5)
    s.samples.pop();s.save()
    assert not (tmp_path/'fit.json').exists() and not (tmp_path/'report.html').exists()


def test_next_reads_fresh_pose_then_sends_exact_next_command(tmp_path):
    from gripper_height.app import CalibrationApp
    class Var:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    ui=CalibrationApp.__new__(CalibrationApp);ui.session=MeasurementSession(tmp_path)
    ui.session.mark_active(700);ui.width=Var('6.25');ui.status=Var('')
    events=[]
    def read():events.append('read');return [400,20,173.1234,0,180,0]
    ui.controller=SimpleNamespace(read=read,set_gripper=lambda value:events.append(value))
    ui.pose=lambda pose:None
    ui.job=lambda label,fn,callback:callback(fn())
    ui.next()
    assert ui.session.samples[0].flange_pose[2]==173.1234
    assert ui.session.samples[0].width_mm==6.25
    assert events==['read',650,'read'] and ui.session.active_command==650


def test_mock_xy_preserves_flange_z_and_decimal_z_is_relative():
    from gripper_height.controller import MockController
    c=MockController();before=c.read()[2]
    assert c.move_xy(338.22,-168.66)==[338.22,-168.66,before,0.,180.,0.]
    assert c.jog(-.125)[2]==pytest.approx(before-.125)
    assert c.jog(.025)[2]==pytest.approx(before-.1)
    c.set_gripper(700);assert c.command==700
    c.stop()
    with pytest.raises(RuntimeError):c.jog(.1)


def test_ros_transport_uses_raw_flange_coordinates_and_aligns_before_xy_without_tip_offset():
    from gripper_height.controller import RosController
    from janggi_pick_place2.executor import Settings
    import threading
    c=RosController.__new__(RosController);c.stopped=threading.Event();c.settings=Settings()
    state={'pose':[400.,0.,175.25,90.,180.,0.]};events=[]
    def call(kind,path,request,check):
        check();events.append(('align',list(request.pos)));state['pose']=list(request.pos)
    def move(target,check):
        check();events.append(('move',list(target)));state['pose']=list(target)
    c.arm=SimpleNamespace(check_tcp=lambda check:check(),current_pose=lambda check:list(state['pose']),
                          _solution=lambda check:0,_inverse=lambda *args:[0,0,90,0,90,0],
                          _check_joint6=lambda value:None,io=SimpleNamespace(call=call),
                          _wait=lambda reached,check:reached(),move=move)
    c.move_xy(338.22,-168.66)
    assert events==[('align',[400.,0.,175.25,0.,180.,0.]),
                    ('move',[338.22,-168.66,175.25,0.,180.,0.])]
    assert c.jog(-.125)==[338.22,-168.66,175.125,0.,180.,0.]


def test_gui_constructs_and_advances_manual_measurement_without_display_or_hardware(monkeypatch,tmp_path):
    import gripper_height.app as app
    class Widget:
        def __init__(self,*args,**kwargs):self.options=kwargs;self.rows={}
        def pack(self,**kwargs):pass
        def state(self,*args):pass
        def configure(self,**kwargs):self.options.update(kwargs)
        def heading(self,*args,**kwargs):pass
        def column(self,*args,**kwargs):pass
        def title(self,*args):pass
        def protocol(self,*args):pass
        def after(self,*args):return 1
        def get_children(self):return list(self.rows)
        def delete(self,item):self.rows.pop(item,None)
        def insert(self,*args,**kwargs):self.rows[str(len(self.rows))]=kwargs
        def focus_set(self):pass
    class Var:
        def __init__(self,value=''):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    monkeypatch.setattr(app.tk,'Tk',Widget);monkeypatch.setattr(app.tk,'StringVar',Var)
    for name in ('Frame','Button','Label','Combobox','Entry','LabelFrame','Treeview'):
        monkeypatch.setattr(app.ttk,name,Widget)
    args=SimpleNamespace(resume=None,output=tmp_path,backend='mock',ns='dsr01',velocity=10.,acceleration=20.)
    ui=app.CalibrationApp(args)
    def run(label,fn,success=None,**kwargs):
        result=fn()
        if success:success(result)
        ui.refresh()
    ui.job=run;ui.connect();ui.x.set('338.22');ui.y.set('-168.66');ui.move_xy();ui.start()
    assert ui.session.active_command==700 and not ui.session.samples
    ui.step.set('0.125');ui.jog(-1);ui.width.set('6.1');ui.next()
    assert ui.session.samples[0].flange_pose[2]==249.875
    assert ui.session.active_command==650 and len(ui.table.rows)==1
