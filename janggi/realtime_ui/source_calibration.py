"""Choose calibration paired with the pixels, preserving explicit user overrides."""
from pathlib import Path
import json


def resolve_calibration(source, explicit, hardware_default):
    if explicit is not None:
        return Path(explicit).expanduser(), 'explicit'
    if source == 'gazebo':
        from simulation_calibration import ensure_simulation_calibration
        return ensure_simulation_calibration(), 'live Gazebo'
    path=Path(source).expanduser()
    if path.is_file():
        for filename in ('calibration.yaml','board_calibration.yaml'):
            sidecar=path.parent/filename
            if sidecar.is_file():return sidecar.resolve(), 'recording sidecar'
        metadata=path.parent/'metadata.json'
        if metadata.is_file():
            data=json.loads(metadata.read_text())
            if data.get('transport')=='gazebo' or 'janggi_sim/d455f' in str(data.get('topic','')):
                raise ValueError('Gazebo 녹화 폴더에 calibration.yaml이 없습니다. 녹화 당시 파일을 복원하거나 --calibration으로 지정하세요.')
    return hardware_default, 'workspace hardware default'


def is_simulation_calibration(path):
    import yaml
    path=Path(path)
    if not path.is_file():return False
    data=yaml.safe_load(path.read_text())
    return isinstance(data,dict) and bool(data.get('simulation'))
