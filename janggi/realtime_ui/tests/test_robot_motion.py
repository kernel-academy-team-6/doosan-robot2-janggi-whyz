from collections import deque
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
from robot_motion_ui import FIELDS, RobotMotionDialog, motion_settings
from action.pick_place2 import load_pick_place_settings


def base():
    return load_pick_place_settings()[0]


def values():
    return dict(linear_velocity_mm_s=80., linear_acceleration_mm_s2=160.,
                angular_velocity_deg_s=20., angular_acceleration_deg_s2=40.)


@pytest.mark.parametrize('linked', [True, False])
def test_scaling_covers_linear_rotation_and_locked_joint_timing(linked):
    settings = motion_settings(base(), values(), 50, linked)
    assert settings.linear_velocity_mm_s == 40
    assert settings.angular_velocity_deg_s == settings.joint_velocity_deg_s == 10
    assert settings.linear_acceleration_mm_s2 == (80 if linked else 160)
    assert settings.angular_acceleration_deg_s2 == settings.joint_acceleration_deg_s2 == (20 if linked else 40)
    assert settings.joint6_locked_deg == 90
    assert settings.piece_grip_heights_mm == [2., 2., 3.]


@pytest.mark.parametrize('bad', [0, -1, float('nan'), float('inf'), 'bad'])
def test_invalid_input_never_reaches_robot(bad):
    for name, _ in FIELDS:
        inputs = values(); inputs[name] = bad
        with pytest.raises(ValueError):
            motion_settings(base(), inputs)
    with pytest.raises(ValueError):
        motion_settings(base(), values(), bad)


def test_busy_apply_rejected_and_idle_updates_existing_session_without_motion():
    ui = app.App.__new__(app.App)
    ui.robot_busy = True; ui._msgs = deque(); ui._motion_settings = None
    calls = []
    ui.rg = SimpleNamespace(picker=SimpleNamespace(set_motion_settings=calls.append),
                            cfg=SimpleNamespace(command_settings=None))
    settings = motion_settings(base(), values())
    with pytest.raises(RuntimeError): ui._apply_motion_settings(settings)
    assert not calls and ui._motion_settings is None
    ui.robot_busy = False; ui._apply_motion_settings(settings)
    assert calls == [settings] and ui.rg.cfg.command_settings is settings
    assert ui._get_motion_settings() is settings


def test_home_and_observation_return_use_selected_rotation_limits():
    from session import RobotGame
    settings = motion_settings(base(), values(), 50.)
    calls = []
    picker = SimpleNamespace(settings=settings, go_observe=lambda *args:calls.append(args))
    ui = app.App.__new__(app.App)
    ui.args = SimpleNamespace(no_motion=False, pick_place_config=app.SRC_JANGGI.parent /
                              'janggi_pick_place2/config/pick_place.yaml')
    ui.worker = SimpleNamespace(robot_motion=False); ui._msgs = deque()
    ui.rg = SimpleNamespace(picker=picker, cfg=SimpleNamespace(observe_vel=30.,
                                                             observe_joints=list(settings.home_joint_deg)))
    ui._ensure_robot = lambda:None
    ui._robot_job = lambda phase, job, **kwargs:job()
    ui.robot_home()
    RobotGame.go_observe(ui.rg)
    assert len(calls) == 2
    assert all(call[1:] == (10.,20.) for call in calls)


def test_dialog_preview_and_submit_without_display(monkeypatch):
    import robot_motion_ui as module
    class Widget:
        def __init__(self, *args, **kwargs): self.options = kwargs
        def grid(self, **kwargs): pass
        def bind(self, *args): pass
        def title(self, *args): pass
        def protocol(self, *args): pass
        def config(self, **kwargs): self.options.update(kwargs)
        def destroy(self): pass
    class Variable:
        def __init__(self, value=None): self.value = value; self.callbacks = []
        def get(self): return self.value
        def set(self, value):
            self.value = value
            for callback in self.callbacks: callback()
        def trace_add(self, mode, callback): self.callbacks.append(callback)
    for name in ('Toplevel', 'Label', 'Entry', 'Checkbutton', 'Scale', 'Button'):
        monkeypatch.setattr(module.tk, name, Widget)
    for name in ('StringVar', 'DoubleVar', 'BooleanVar'):
        monkeypatch.setattr(module.tk, name, Variable)
    applied = []; closed = []
    dialog = RobotMotionDialog(None, base(), applied.append, lambda:closed.append(True))
    dialog.percent.value = 50.; dialog.preview()
    assert '60 mm/s' in dialog.summary.options['text']
    # Direct edits now affect all four limits, including the joint limits used
    # by pick/place. Blur and submit must not apply the ratio twice.
    dialog.values['linear_velocity_mm_s'].set('240')
    dialog.commit_edit('linear_velocity_mm_s')
    assert dialog.result().linear_acceleration_mm_s2 == 240
    assert dialog.result().joint_velocity_deg_s == 60
    dialog.commit_edit('linear_velocity_mm_s')
    dialog.submit()
    assert applied[0].joint_acceleration_deg_s2 == 120 and closed == [True]
    independent = RobotMotionDialog(None, base(), applied.append, lambda:None)
    independent.linked.value = False
    independent.values['linear_velocity_mm_s'].set('240')
    independent.commit_edit('linear_velocity_mm_s')
    assert independent.result().angular_velocity_deg_s == 60
    assert independent.result().linear_acceleration_mm_s2 == 240


def test_initial_ui_settings_unify_joint_and_cartesian_rotation():
    ui = app.App.__new__(app.App)
    ui.args = SimpleNamespace(pick_place_config=app.SRC_JANGGI.parent /
                              'janggi_pick_place2/config/pick_place.yaml', robot_vel=30.)
    settings = ui._get_motion_settings()
    assert settings.linear_velocity_mm_s == 30
    assert settings.joint_velocity_deg_s == settings.angular_velocity_deg_s
    assert settings.joint_acceleration_deg_s2 == settings.angular_acceleration_deg_s2
