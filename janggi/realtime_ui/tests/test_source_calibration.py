from pathlib import Path
import json
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from source_calibration import resolve_calibration


def test_recorded_video_automatically_uses_its_own_calibration(tmp_path):
    video=tmp_path/'color.mp4';video.touch()
    saved=tmp_path/'calibration.yaml';saved.write_text('simulation: {source: gazebo}\n')
    path,origin=resolve_calibration(str(video),None,Path('/hardware.yaml'))
    assert path==saved and origin=='recording sidecar'


def test_realsense_recording_uses_board_calibration_sidecar(tmp_path):
    video=tmp_path/'color.mp4';video.touch()
    saved=tmp_path/'board_calibration.yaml';saved.touch()
    assert resolve_calibration(str(video),None,Path('/hardware.yaml'))[0]==saved


def test_explicit_calibration_always_wins_including_explicit_hardware_default(tmp_path):
    video=tmp_path/'color.mp4';video.touch();(tmp_path/'calibration.yaml').touch()
    hardware=Path('/hardware.yaml')
    assert resolve_calibration(str(video),hardware,hardware)==(hardware,'explicit')
    assert resolve_calibration('gazebo',hardware,hardware)==(hardware,'explicit')


def test_known_gazebo_recording_without_saved_calibration_does_not_use_hardware(tmp_path):
    video=tmp_path/'color.mp4';video.touch()
    (tmp_path/'metadata.json').write_text(json.dumps({'transport':'gazebo'}))
    with pytest.raises(ValueError,match='calibration.yaml'):resolve_calibration(str(video),None,Path('/hardware.yaml'))


def test_recorded_gazebo_parse_selects_sidecar_and_hybrid(tmp_path,monkeypatch):
    import app
    video=tmp_path/'color.mp4';video.touch()
    (tmp_path/'calibration.yaml').write_text('simulation: {source: gazebo}\n')
    monkeypatch.setattr(sys,'argv',['app','--source',str(video)])
    args=app.parse_args()
    assert args.calibration==tmp_path/'calibration.yaml'
    assert args.simulation_calibration and args.model_mode=='hybrid'


def test_recorded_simulation_pixels_do_not_start_robot_motion():
    from types import SimpleNamespace
    import app
    ui=app.App.__new__(app.App)
    ui.position_only=False;ui.rg=None
    ui.args=SimpleNamespace(source='recording.mp4',simulation_calibration=True,no_motion=False)
    with pytest.raises(RuntimeError,match='녹화 재생'):ui._ensure_robot()
