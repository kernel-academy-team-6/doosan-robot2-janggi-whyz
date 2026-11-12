"""RGB option catalogue and persistent user profile (SDK sensor units)."""
import json
import math
from pathlib import Path

PROFILE_PATH = Path(__file__).resolve().parent/'config/realsense_rgb.json'
# key: (UI label, widget kind). Only RGB sensor controls are permitted.
OPTION_SPECS = {
    'enable_auto_exposure': ('자동 노출', 'bool'),
    'exposure': ('노출 / Exposure', 'number'),
    'gain': ('게인 / Gain', 'number'),
    'power_line_frequency': ('전원 주파수', 'enum'),
    'auto_exposure_priority': ('자동 노출 우선', 'bool'),
    'global_time_enabled': ('Global Time', 'bool'),
    'backlight_compensation': ('역광 보정', 'number'),
    'brightness': ('밝기 / Brightness', 'number'),
    'contrast': ('대비 / Contrast', 'number'),
    'gamma': ('감마 / Gamma', 'number'),
    'hue': ('색조 / Hue', 'number'),
    'saturation': ('채도 / Saturation', 'number'),
    'sharpness': ('선명도 / Sharpness', 'number'),
    'enable_auto_white_balance': ('자동 화이트밸런스', 'bool'),
    'white_balance': ('화이트밸런스', 'number'),
}


def load_profile(path=PROFILE_PATH):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema_version') != 1 or not isinstance(data.get('options'),dict):
        raise ValueError('지원하지 않는 카메라 설정 파일입니다.')
    values=data['options']
    if not values or not set(values)<=set(OPTION_SPECS):
        raise ValueError('카메라 설정에 알 수 없는 옵션이 있습니다.')
    if any(isinstance(v,str) or not isinstance(v,(int,float,bool)) or not math.isfinite(v) for v in values.values()):
        raise ValueError('카메라 설정값은 유한한 숫자여야 합니다.')
    return dict(values)


def save_profile(values,path=PROFILE_PATH):
    if not values or not set(values)<=set(OPTION_SPECS):
        raise ValueError('저장할 카메라 옵션이 잘못되었습니다.')
    normalized={key:float(value) for key,value in values.items()}
    if not all(math.isfinite(v) for v in normalized.values()):
        raise ValueError('카메라 설정값은 유한한 숫자여야 합니다.')
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(dict(schema_version=1,options=normalized),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)


def apply_saved(control,path=PROFILE_PATH):
    values=load_profile(path)
    return control.request_settings(values,ignore_unsupported=True)
