"""Transport failures and the calibrated robot frame must be explicit."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sim_app  # application's import paths
from gz_scene import GzScene, GzRobotMirror


def test_absent_models_are_not_sent_to_remove_service():
    scene = GzScene.__new__(GzScene)
    scene._existing_models = {'janggi_board'}
    calls = []
    scene._svc = lambda *args, **kwargs: calls.append(args) or True
    assert scene._remove('janggi_table')
    assert calls == []
    assert scene._remove('janggi_board')
    assert len(calls) == 1
    assert not scene._existing_models


def test_failed_creation_does_not_claim_model_exists():
    scene = GzScene.__new__(GzScene)
    scene._existing_models = set()
    scene._svc = lambda *args, **kwargs: False
    with pytest.raises(RuntimeError, match='creation failed: example'):
        scene._create('<sdf version="1.9"><model name="example"/></sdf>')
    assert not scene._existing_models


def test_robot_fk_uses_base_link_even_with_offset_urdf_root():
    robot = GzRobotMirror.__new__(GzRobotMirror)
    robot.root, robot.q = 'world', {}
    offset = np.eye(4)
    offset[:3, 3] = [.3, -.2, .1]
    # Include a rotation so a translation-only rebase cannot pass.
    offset[:3, :3] = [[0,-1,0],[1,0,0],[0,0,1]]
    tip = np.eye(4)
    tip[:3, 3] = [.15, .02, .4]
    robot.joints = [dict(parent=a, child=b, T=t, axis=[0,0,1], type='fixed', mimic=None)
                    for a,b,t in [('world','base_link',offset),('base_link','tip',tip)]]
    poses = robot.fk()
    np.testing.assert_allclose(poses['base_link'], np.eye(4), atol=1e-12)
    np.testing.assert_allclose(poses['tip'], tip, atol=1e-12)


def test_pose_driven_robot_and_piece_motion_are_disabled():
    robot=GzRobotMirror.__new__(GzRobotMirror)
    with pytest.raises(RuntimeError,match='disabled'):robot.sync()
    scene=GzScene.__new__(GzScene)
    with pytest.raises(RuntimeError,match='disabled'):scene.sync()
    with pytest.raises(RuntimeError,match='disabled'):scene.refresh_board_models()


def test_gui_readiness_waits_for_initial_state_callback_to_finish(monkeypatch):
    from types import SimpleNamespace
    import gz_scene
    responses=iter([['/gui/move_to/pose','/12345/state_async'],*([['/gui/move_to/pose']]*3)])
    scene=GzScene.__new__(GzScene)
    scene.node=SimpleNamespace(service_list=lambda:next(responses))
    sleeps=[];monkeypatch.setattr(gz_scene.time,'sleep',lambda duration:sleeps.append(duration))
    scene.wait_for_gui(timeout=1.)
    assert len(sleeps)==3
