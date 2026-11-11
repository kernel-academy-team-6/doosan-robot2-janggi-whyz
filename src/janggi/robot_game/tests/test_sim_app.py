"""Setup-only must build the scene without initializing motion or scheduling play."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sim_app


@pytest.fixture
def setup_harness(monkeypatch):
    events = []

    class Root:
        def __init__(self):
            self.callbacks = {}

        def title(self, text):
            events.append(('title', text))

        def protocol(self, name, callback):
            self.callbacks[name] = callback

        def after(self, *args):
            pytest.fail('Setup-only scheduled an automatic callback')

    class App:
        def __init__(self, args, worker, mapper):
            assert args.robot is False
            assert args.no_motion is True
            self.root = Root()
            self.log = SimpleNamespace(get=lambda *args: ())

        def _ensure_robot(self):
            pytest.fail('Setup-only initialized robot control')

        def robot_start(self):
            pytest.fail('Setup-only started a game')

        def robot_human_done(self):
            pytest.fail('Setup-only advanced a turn')

        def run(self):
            events.append('run')
            self.root.callbacks['WM_DELETE_WINDOW']()

        def quit(self):
            events.append('quit')

    class World:
        grasp_log = []

        def __init__(self, *args):
            pass

        def human_move(self, *args):
            pytest.fail('Setup-only changed a piece position')

    monkeypatch.setattr(sim_app.ui, 'App', App)
    monkeypatch.setattr(sim_app, 'SimWorld', World)
    monkeypatch.setattr(sim_app.BoardGeometry, 'from_calibration', lambda *a, **k: object())
    monkeypatch.setattr(sim_app.BoardMapper, 'from_calibration', lambda *a, **k: object())
    monkeypatch.setattr(sim_app.CameraToBase, 'from_calibration', lambda *a, **k: object())
    return events


def test_setup_only_opens_2d_view_without_any_motion_or_automation(monkeypatch, setup_harness):
    monkeypatch.setattr(sys, 'argv', ['sim_app', '--setup-only'])
    sim_app.main()
    assert setup_harness[-2:] == ['run', 'quit']


def test_setup_only_builds_gazebo_without_a_picker_and_cleans_up(monkeypatch, setup_harness):
    events = setup_harness

    class Scene:
        def __init__(self, world, **kwargs):
            events.append('scene')

        def start(self):
            # No tip callback: there must be no motion client or pose query.
            events.append('start')

        def stop(self):
            events.append('stop')

    monkeypatch.setitem(sys.modules, 'gz_scene', SimpleNamespace(GzScene=Scene))
    monkeypatch.setattr(sys, 'argv', ['sim_app', '--setup-only', '--gazebo'])
    sim_app.main()
    assert events[-5:] == ['scene', 'start', 'run', 'quit', 'stop']


def test_setup_scene_failure_still_stops_scene_without_playing(monkeypatch, setup_harness):
    events = setup_harness

    class Scene:
        def __init__(self, world, **kwargs):
            pass

        def start(self):
            raise RuntimeError('Gazebo unavailable')

        def stop(self):
            events.append('stop')

    monkeypatch.setitem(sys.modules, 'gz_scene', SimpleNamespace(GzScene=Scene))
    monkeypatch.setattr(sys, 'argv', ['sim_app', '--setup-only', '--gazebo'])
    with pytest.raises(RuntimeError, match='Gazebo unavailable'):
        sim_app.main()
    assert 'run' not in events and events[-1] == 'stop'
