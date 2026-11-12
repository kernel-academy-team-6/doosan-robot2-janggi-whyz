"""Full RGB profiles, SDK ordering, persistence and settings UI without hardware."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from realsense_exposure import ExposureControl
from realsense_settings import OPTION_SPECS, PROFILE_PATH, load_profile, save_profile, apply_saved
import realsense_settings_ui as settings_ui
import app


class Sensor:
    def __init__(self):
        self.values={key:0. for key in OPTION_SPECS}
        self.values.update(enable_auto_exposure=1.,exposure=100.,gain=10.,gamma=200.,
                           enable_auto_white_balance=1.,white_balance=5000.)
        self.writes=[];self.unsupported=set();self.readonly=set()
    def supports(self,key):return key not in self.unsupported
    def is_option_read_only(self,key):return key in self.readonly
    def get_option(self,key):return self.values[key]
    def get_option_range(self,key):
        limits={'exposure':(1,10000,1),'gain':(0,128,1),'power_line_frequency':(0,3,1),
                'brightness':(-64,64,1),'gamma':(100,500,1),'hue':(-180,180,1),
                'white_balance':(2800,6500,10),'backlight_compensation':(0,1,1)}
        lo,hi,step=limits.get(key,(0,100,1))
        return SimpleNamespace(min=lo,max=hi,step=step)
    def get_option_value_description(self,key,value):return {0:'Disabled',1:'50 Hz',2:'60 Hz',3:'Auto'}[int(value)]
    def set_option(self,key,value):
        if key in ('exposure','gain') and self.values['enable_auto_exposure']:
            raise RuntimeError('auto exposure must be disabled first')
        if key=='white_balance' and self.values['enable_auto_white_balance']:
            raise RuntimeError('auto WB must be disabled first')
        self.writes.append((key,value));self.values[key]=value


def setup():
    sensor=Sensor();options=SimpleNamespace(**{key:key for key in OPTION_SPECS})
    control=ExposureControl(options);control.bind(sensor)
    return control,sensor


def test_screenshot_profile_applies_all_manual_values_and_keeps_auto_white_balance():
    c,s=setup();values=apply_saved(c)
    assert values['exposure']==70. and values['gain']==6. and values['brightness']==5.
    assert not s.writes
    c.process_pending()
    for key,value in load_profile().items():
        if key!='white_balance':assert s.values[key]==value
    assert s.values['white_balance']==5000.  # AE/WB dependencies are respected.
    assert 'white_balance' not in [k for k,v in s.writes]
    assert s.writes[:2]==[('enable_auto_exposure',0.),('enable_auto_white_balance',1.)]
    assert c.snapshot()['controls']['power_line_frequency']['choices'][3]=='Auto'


def test_manual_white_balance_disable_precedes_setting_and_auto_exposure_skips_gain():
    c,s=setup()
    c.request_settings(dict(enable_auto_exposure=1,exposure=70,gain=6,
                            enable_auto_white_balance=0,white_balance=5080))
    c.process_pending()
    assert s.values['white_balance']==5080.
    assert ('enable_auto_white_balance',0.) in s.writes
    assert not {'exposure','gain'} & {key for key,value in s.writes}


def test_saved_preset_skips_unsupported_and_read_only_and_reconnects():
    c,s=setup();s.unsupported.add('global_time_enabled');s.readonly.add('sharpness');c.bind(s)
    apply_saved(c);c.process_pending()
    assert set(c.snapshot()['skipped'])=={'global_time_enabled','sharpness'}
    assert not {'global_time_enabled','sharpness'} & {key for key,value in s.writes}
    c.unbind();new=Sensor();c.bind(new);c.process_pending()
    assert new.values['brightness']==5. and new.values['exposure']==70.


def test_whole_batch_validated_before_any_write_and_snapshot_cannot_mutate_control():
    c,s=setup()
    with pytest.raises(ValueError):c.request_settings(dict(brightness=5,contrast=10000))
    assert not c.snapshot()['pending'] and not s.writes
    state=c.snapshot();state['controls']['brightness']['maximum']=100000
    with pytest.raises(ValueError):c.request_settings(dict(brightness=1000))


def test_profile_persists_without_applying_and_rejects_unknown_or_nonfinite(tmp_path):
    c,s=setup();path=tmp_path/'profile.json'
    values=c.validate_settings(dict(brightness=8,contrast=60))
    save_profile(values,path)
    assert load_profile(path)==values and not s.writes and not c.snapshot()['pending']
    apply_saved(c,path);c.process_pending()
    assert s.values['brightness']==8.
    path.write_text(json.dumps(dict(schema_version=1,options={'emitter_enabled':1})))
    with pytest.raises(ValueError):load_profile(path)
    with pytest.raises(ValueError):save_profile({'brightness':float('nan')},path)


def test_main_saved_profile_button_queues_without_sdk_calls(monkeypatch):
    class Variable:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    c,s=setup();ui=app.App.__new__(app.App)
    ui.exposure_control=c;ui.exposure_auto=Variable(True);ui.exposure_value=Variable('100')
    ui._camera_settings_dialog=None;ui._refresh_exposure=lambda:None
    ui._apply_saved_camera_settings()
    assert not s.writes and c.snapshot()['pending']
    assert not ui.exposure_auto.get() and ui.exposure_value.get()=='70'


def test_full_settings_window_edits_saves_and_applies_without_display(monkeypatch,tmp_path):
    class Widget:
        def __init__(self,*args,**kwargs):self.options=dict(kwargs)
        def pack(self,*args,**kwargs):pass
        def grid(self,*args,**kwargs):pass
        def configure(self,**kwargs):self.options.update(kwargs)
        def title(self,*args):pass
        def transient(self,*args):pass
        def protocol(self,*args):pass
        def after(self,*args):return 1
        def after_cancel(self,*args):pass
        def destroy(self):pass
    class Variable:
        def __init__(self,value=None):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    for name in ['Toplevel','Spinbox']:monkeypatch.setattr(settings_ui.tk,name,Widget)
    for name in ['Frame','Label','Checkbutton','Combobox','Button']:monkeypatch.setattr(settings_ui.ttk,name,Widget)
    monkeypatch.setattr(settings_ui.tk,'BooleanVar',Variable);monkeypatch.setattr(settings_ui.tk,'StringVar',Variable)
    c,s=setup();dialog=settings_ui.CameraSettingsDialog(object(),c)
    try:
        dialog.apply_saved()
        assert not s.writes and dialog.variables['exposure'].get()=='70'
        c.process_pending();dialog._poll()
        assert dialog.notice=='적용 완료'
        assert dialog.widgets['white_balance'].options['state']=='disabled'
        dialog.variables['brightness'].set('8')
        path=tmp_path/'profile.json'
        monkeypatch.setattr(settings_ui,'save_profile',lambda values:save_profile(values,path))
        count=len(s.writes);dialog.save_current()
        assert load_profile(path)['brightness']==8. and len(s.writes)==count
        dialog.variables['brightness'].set('12');dialog.apply_current();c.process_pending()
        assert s.values['brightness']==12.
    finally:dialog.close()
