"""RGB exposure control with fake SDK sensors; no device/robot access."""
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from realsense_exposure import ExposureControl
import app

OPTIONS=SimpleNamespace(exposure='exposure',enable_auto_exposure='auto')


class Sensor:
    def __init__(self,supported=True):
        self.values={'auto':1.,'exposure':100.};self.writes=[];self.supported=supported;self.fail=False
    def supports(self,option):return self.supported
    def get_option(self,option):return self.values[option]
    def get_option_range(self,option):return SimpleNamespace(min=10.,max=1000.,step=5.)
    def set_option(self,option,value):
        if self.fail and option=='exposure':raise RuntimeError('SDK rejected exposure')
        self.writes.append((option,value));self.values[option]=value


def control():
    sensor=Sensor();result=ExposureControl(OPTIONS);result.bind(sensor)
    assert not sensor.writes  # Startup reads current settings; does not force auto/manual.
    return result,sensor


def test_manual_request_queues_and_disables_auto_before_setting_exposure():
    c,s=control()
    assert c.request(False,123.)==125.
    assert not s.writes and c.snapshot()['pending']
    c.process_pending()
    assert s.writes==[('auto',0.),('exposure',125.)]
    state=c.snapshot()
    assert not state['auto'] and state['exposure']==125. and not state['pending']


def test_auto_request_does_not_write_manual_exposure_and_latest_request_wins():
    c,s=control();c.request(False,125);c.request(False,300);c.process_pending()
    assert s.writes==[('auto',0.),('exposure',300.)]
    c.request(True);c.process_pending()
    assert s.writes[-1]==('auto',1.) and c.snapshot()['auto']


@pytest.mark.parametrize('value',[0.,1001.,float('nan'),float('inf'),'bad'])
def test_invalid_values_never_write_options(value):
    c,s=control()
    with pytest.raises(ValueError):c.request(False,value)
    assert not s.writes and not c.snapshot()['pending']


def test_reconnect_restores_only_the_users_successful_choice():
    c,s=control();c.request(False,225);c.process_pending();c.unbind()
    with pytest.raises(ValueError,match='연결'):c.request(False,300)
    second=Sensor();c.bind(second)
    assert not second.writes and c.snapshot()['pending']
    c.process_pending()
    assert second.writes==[('auto',0.),('exposure',225.)]


def test_option_failure_is_reported_without_raising_or_losing_actual_state():
    c,s=control();s.fail=True;c.request(False,225);c.process_pending()
    state=c.snapshot()
    assert 'SDK rejected' in state['error'] and not state['auto']
    assert state['exposure']==100. and not state['pending']


def test_unsupported_sensor_and_unavailable_camera_do_not_accept_manual_request():
    c=ExposureControl(OPTIONS)
    with pytest.raises(ValueError,match='연결'):c.request(False,100)
    c.bind(Sensor(False))
    assert not c.snapshot()['supported']
    with pytest.raises(ValueError,match='지원'):c.request(False,100)


def test_realsense_source_selects_colour_sensor_and_applies_in_capture_read(monkeypatch):
    import numpy as np
    sensor=Sensor();events=[]
    device=SimpleNamespace(first_color_sensor=lambda:sensor)
    profile=SimpleNamespace(get_device=lambda:device)
    frame=SimpleNamespace(get_data=lambda:np.zeros((2,2,3),np.uint8))
    pipe=SimpleNamespace(start=lambda cfg:profile,stop=lambda:events.append('stop'),
                         wait_for_frames=lambda timeout:SimpleNamespace(get_color_frame=lambda:frame))
    cfg=SimpleNamespace(enable_stream=lambda *args:events.append(args))
    sdk=SimpleNamespace(option=OPTIONS,pipeline=lambda:pipe,config=lambda:cfg,
                         stream=SimpleNamespace(color='colour'),format=SimpleNamespace(bgr8='bgr8'))
    monkeypatch.setitem(sys.modules,'pyrealsense2',sdk)
    source=app.RealSenseSource(1280,720)
    try:
        assert events==[('colour',1280,720,'bgr8',30)]
        source.exposure_control.request(False,250)
        assert not sensor.writes
        assert source.read().shape==(2,2,3)
        assert sensor.writes==[('auto',0.),('exposure',250.)]
    finally:source.close()


def test_ui_queues_manual_selection_and_shows_invalid_input():
    class Value:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    ui=app.App.__new__(app.App);ui.exposure_control,sensor=control()
    ui.exposure_auto=Value(False);ui.exposure_value=Value('123')
    ui._refresh_exposure=lambda:None
    ui._queue_exposure()
    assert ui.exposure_value.get()=='125' and ui.exposure_control.snapshot()['pending']
    assert not sensor.writes
    ui.exposure_value.set('not a number');ui._queue_exposure()
    assert ui._exposure_input_error


def test_exposure_ui_build_and_mode_switch_without_display(monkeypatch):
    class Widget:
        def __init__(self,*args,**kwargs):self.options=dict(kwargs)
        def pack(self,*args,**kwargs):pass
        def bind(self,*args,**kwargs):pass
        def config(self,**kwargs):self.options.update(kwargs)
    class Variable:
        def __init__(self,value=None):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    for name in ['Frame','Label','Checkbutton','Spinbox','Button']:
        monkeypatch.setattr(app.tk,name,Widget)
    monkeypatch.setattr(app.tk,'BooleanVar',Variable)
    monkeypatch.setattr(app.tk,'StringVar',Variable)
    c,s=control();ui=app.App.__new__(app.App)
    ui.worker=SimpleNamespace(source=SimpleNamespace(exposure_control=c))
    ui._build_exposure_controls(object())
    assert ui.exposure_spin.options['state']==app.tk.DISABLED
    assert ui.exposure_spin.options['from_']==10. and ui.exposure_spin.options['to']==1000.
    ui.exposure_auto.set(False);ui.exposure_value.set('123');ui._queue_exposure()
    assert not s.writes
    c.process_pending();ui._refresh_exposure()
    assert ui.exposure_spin.options['state']==app.tk.NORMAL
    assert ui.exposure_apply.options['state']==app.tk.NORMAL and ui.exposure_value.get()=='125'
    ui.exposure_auto.set(True);ui._queue_exposure();c.process_pending();ui._refresh_exposure()
    assert ui.exposure_spin.options['state']==app.tk.DISABLED
