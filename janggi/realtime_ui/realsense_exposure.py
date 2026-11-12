"""RGB option requests are queued by the UI and applied by the capture thread."""
import copy
import math
import threading
import time
from realsense_settings import OPTION_SPECS


class ExposureControl:
    def __init__(self, options):
        self.options = options
        self.lock = threading.Lock()
        self.sensor = None
        self.pending = self.desired = None
        self.applying = False
        self.last_poll = 0.
        self.state = dict(connected=False, supported=False, auto_supported=False,
                          auto=False, exposure=None, minimum=None, maximum=None, step=None, error=None,
                          controls={}, skipped=[])

    def snapshot(self):
        with self.lock:
            return dict(copy.deepcopy(self.state), pending=self.pending is not None or self.applying)

    def _read(self, sensor):
        controls={}
        for key,(label,kind) in OPTION_SPECS.items():
            meta=dict(label=label,kind=kind,supported=False,writable=False,value=None,
                      minimum=None,maximum=None,step=None,choices={})
            option=getattr(self.options,key,None)
            try:
                if option is not None and sensor.supports(option):
                    value=float(sensor.get_option(option))
                    readonly=getattr(sensor,'is_option_read_only',lambda option:False)(option)
                    if kind=='bool':minimum,maximum,step=0.,1.,1.
                    else:
                        limits=sensor.get_option_range(option)
                        minimum,maximum,step=float(limits.min),float(limits.max),float(limits.step)
                    if not all(math.isfinite(v) for v in (value,minimum,maximum,step)) or minimum>maximum or step<=0:
                        raise ValueError('센서 옵션 범위가 올바르지 않습니다.')
                    meta.update(supported=True,writable=not readonly,value=value,
                                minimum=minimum,maximum=maximum,step=step)
                    if kind=='enum':
                        for number in range(int(minimum),int(maximum)+1):
                            try:description=getattr(sensor,'get_option_value_description',lambda *args:None)(option,float(number))
                            except RuntimeError:description=None
                            meta['choices'][number]=description or str(number)
            except (RuntimeError,ValueError) as exc:
                meta.update(supported=False,writable=False,error=str(exc))
            controls[key]=meta
        exposure,automatic=controls['exposure'],controls['enable_auto_exposure']
        return dict(connected=True,controls=controls,supported=exposure['writable'],
                    auto_supported=automatic['writable'],auto=bool(automatic['value']),
                    exposure=exposure['value'],minimum=exposure['minimum'],
                    maximum=exposure['maximum'],step=exposure['step'])

    def bind(self, sensor):
        info = self._read(sensor)
        with self.lock:
            self.sensor = sensor
            self.state.update(info, error=None)
            if self.pending is None and self.desired is not None:
                self.pending = dict(self.desired)
            self.last_poll = time.monotonic()

    def unbind(self, error=None):
        with self.lock:
            self.sensor = None
            self.state.update(connected=False, error=error)

    def request(self, automatic, exposure=None):
        values={'enable_auto_exposure':float(bool(automatic))}
        if not automatic:
            values['exposure']=self.snapshot()['exposure'] if exposure is None else exposure
        result=self.request_settings(values)
        return result.get('exposure')

    def _normalize(self,values,ignore_unsupported):
        if not self.state['connected']:
            raise ValueError('카메라가 연결되지 않았습니다.')
        result={};skipped=[]
        for key,value in values.items():
            if key not in OPTION_SPECS:
                raise ValueError('알 수 없는 RGB 옵션: '+key)
            meta=self.state['controls'].get(key,{})
            if not meta.get('writable'):
                if ignore_unsupported:
                    skipped.append(key);continue
                raise ValueError(OPTION_SPECS[key][0]+' 옵션을 지원하지 않거나 읽기 전용입니다.')
            value=float(value)
            if not math.isfinite(value) or not meta['minimum']<=value<=meta['maximum']:
                raise ValueError(f"{meta['label']}: {meta['minimum']:g}~{meta['maximum']:g} 범위여야 합니다.")
            if meta['kind']=='bool' and value not in (0.,1.):
                raise ValueError(meta['label']+': 0 또는 1만 가능합니다.')
            value=meta['minimum']+round((value-meta['minimum'])/meta['step'])*meta['step']
            result[key]=min(meta['maximum'],max(meta['minimum'],value))
        if not result:
            raise ValueError('적용할 수 있는 RGB 옵션이 없습니다.')
        return result,skipped

    def validate_settings(self,values,ignore_unsupported=False):
        with self.lock:
            return self._normalize(values,ignore_unsupported)[0]

    def request_settings(self,values,ignore_unsupported=False):
        with self.lock:
            normalized,skipped=self._normalize(values,ignore_unsupported)
            self.pending=dict(self.pending or {},**normalized)
            self.state['skipped']=skipped
            self.state['error'] = None
            return normalized

    def _apply(self,sensor,request):
        def write(key,value):
            option=getattr(self.options,key,None)
            if option is None or not sensor.supports(option) or getattr(sensor,'is_option_read_only',lambda o:False)(option):
                return
            try:sensor.set_option(option,value)
            except RuntimeError as exc:raise RuntimeError(f'{OPTION_SPECS[key][0]} 적용 실패: {exc}') from exc
        for key in ('enable_auto_exposure','enable_auto_white_balance'):
            if key in request:write(key,request[key])
        def automatic(key):
            if key in request:return bool(request[key])
            option=getattr(self.options,key,None)
            return bool(sensor.get_option(option)) if option is not None and sensor.supports(option) else False
        auto_exposure=automatic('enable_auto_exposure')
        auto_white=automatic('enable_auto_white_balance')
        for key,value in request.items():
            if key in ('enable_auto_exposure','enable_auto_white_balance'):continue
            if key in ('exposure','gain') and auto_exposure:continue
            if key=='white_balance' and auto_white:continue
            write(key,value)

    def process_pending(self):
        with self.lock:
            sensor = self.sensor
            if sensor is None:
                return
            request = self.pending
            self.pending = None
            self.applying = request is not None
            if request is None and time.monotonic()-self.last_poll < 1.:
                return
        try:
            if request is not None:
                self._apply(sensor,request)
            info = self._read(sensor)
            with self.lock:
                self.state.update(info)
                if request is not None:
                    self.desired = dict(self.desired or {},**request)
                    self.state['error'] = None
        except (RuntimeError, ValueError) as exc:
            try:
                actual = self._read(sensor)
            except (RuntimeError, ValueError):
                actual = {}
            with self.lock:
                self.state.update(actual, error=str(exc))
        finally:
            with self.lock:
                self.applying = False
                self.last_poll = time.monotonic()
