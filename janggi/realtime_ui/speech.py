"""Async prerecorded Supertonic ko/F1 playback; no runtime model or network.

KoreanSynthesizer remains an optional injected diagnostic backend. The game
defaults to ClipBank and never constructs a synthesizer.
"""
from array import array
import ctypes
import ctypes.util
from collections import deque
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import wave
import sys
from speech_assets import ClipBank


class KoreanSynthesizer:
    """eSpeak NG synchronous PCM retrieval: never opens an audio device itself."""
    def __init__(self, library=None):
        library = library or ctypes.util.find_library('espeak-ng')
        if not library:
            raise RuntimeError('한국어 음성 엔진이 없습니다 (libespeak-ng1 필요).')
        self.lib = ctypes.CDLL(library)
        self.lib.espeak_Initialize.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_char_p,ctypes.c_int]
        self.lib.espeak_Initialize.restype = ctypes.c_int
        self.rate = self.lib.espeak_Initialize(2,0,None,0x8000)  # synchronous, don't exit on missing data
        if self.rate <= 0:
            raise RuntimeError('한국어 음성 데이터를 불러오지 못했습니다 (espeak-ng-data 확인).')
        self.lib.espeak_SetVoiceByName.argtypes = [ctypes.c_char_p]
        if self.lib.espeak_SetVoiceByName(b'ko') != 0:
            self.lib.espeak_Terminate()
            raise RuntimeError('한국어 음성이 지원되지 않습니다.')
        self.lib.espeak_SetParameter.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_int]
        self.lib.espeak_SetParameter(1,185,0)  # words/minute
        self.callback_type = ctypes.CFUNCTYPE(ctypes.c_int,ctypes.POINTER(ctypes.c_short),ctypes.c_int,ctypes.c_void_p)
        self.lib.espeak_SetSynthCallback.argtypes = [self.callback_type]
        self.lib.espeak_SetSynthCallback.restype = None
        self.lib.espeak_Synth.argtypes = [ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint,
                                         ctypes.c_int,ctypes.c_uint,ctypes.c_uint,
                                         ctypes.POINTER(ctypes.c_uint),ctypes.c_void_p]
        self._callback = self.callback_type(self._receive)
        self.lib.espeak_SetSynthCallback(self._callback)
        self._chunks = []
        self._cancel = threading.Event()

    def _receive(self, samples, count, events):
        if self._cancel.is_set():
            return 1
        if samples and count:
            self._chunks.append(ctypes.string_at(samples,count*2))
        return 0

    def render(self, text, destination, cancel=None):
        self._cancel = cancel or threading.Event()
        self._chunks = []
        data = ctypes.create_string_buffer(text.encode('utf-8'))
        code = self.lib.espeak_Synth(data,len(data),0,1,0,1|0x1000,None,None)
        if self._cancel.is_set():
            return False
        if code != 0 or not self._chunks:
            raise RuntimeError('한국어 음성 합성 실패')
        with wave.open(str(destination),'wb') as output:
            output.setnchannels(1);output.setsampwidth(2);output.setframerate(self.rate)
            output.writeframes(b''.join(self._chunks))
        self._chunks = []
        return True

    def close(self):
        self.lib.espeak_Terminate()


class Speech:
    """Bounded, expiring notifications. UI/robot commands never wait for audio."""
    def __init__(self, *, synthesizer=None, players=None, on_error=lambda message:None,
                 audio_dir=None, volume=80):
        self._factory = synthesizer
        self._bank = (ClipBank() if audio_dir is None else ClipBank(audio_dir)) if synthesizer is None else None
        self._players = ([p for p in (shutil.which('aplay'),shutil.which('ffplay')) if p]
                         if players is None else list(players))
        self.available = bool(self._players) and (self._bank.available if self._bank is not None else
                         synthesizer is not KoreanSynthesizer or bool(ctypes.util.find_library('espeak-ng')))
        self.unavailable_reason = (self._bank.error if self._bank is not None and not self._bank.available else
                                   '음성 플레이어(aplay 또는 ffplay)가 없습니다.' if not self._players else '')
        self.on_error = on_error
        self._condition = threading.Condition()
        self._queue = deque(maxlen=4)
        self._enabled = self._closed = False
        self._thread = self._player = self._active_key = None
        self._cancel = threading.Event()
        self.set_volume(volume)

    def set_volume(self, percent):
        value=float(percent)
        if not 0<=value<=100:raise ValueError('volume must be between 0 and 100')
        with self._condition:self._volume=value

    @staticmethod
    def scaled_wav(source, destination, percent):
        """Software gain on a temporary copy, never changes the reusable asset."""
        with wave.open(str(source)) as wav:
            params=wav.getparams();samples=array('h',wav.readframes(wav.getnframes()))
        if params.sampwidth!=2:raise ValueError('PCM16 WAV required for software volume')
        if sys.byteorder!='little':samples.byteswap()
        samples=array('h',(round(v*percent/100.) for v in samples))
        if sys.byteorder!='little':samples.byteswap()
        with wave.open(str(destination),'wb') as wav:
            wav.setparams(params);wav.writeframes(samples.tobytes())

    def _interrupt(self):
        self._queue.clear()
        self._cancel.set()
        if self._player is not None:
            try:self._player.terminate()
            except OSError:pass

    def set_enabled(self, enabled):
        with self._condition:
            self._enabled = bool(enabled) and self.available and not self._closed
            if not self._enabled:
                self._interrupt()
            self._condition.notify_all()
        return self._enabled

    def say(self, text, *, key=None, replace=False):
        key = text if key is None else key
        with self._condition:
            if not self._enabled or self._closed:
                return False
            if (key == self._active_key and not self._cancel.is_set()) or any(item[0]==key for item in self._queue):
                return False
            if replace:self._interrupt()
            self._queue.append((key,text,time.monotonic()+8.))
            if self._thread is None:
                self._thread = threading.Thread(target=self._run,daemon=True,name='janggi-korean-speech')
                self._thread.start()
            self._condition.notify_all()
        return True

    @staticmethod
    def player_command(player, path):
        if Path(player).name == 'ffplay':
            return [player,'-nodisp','-autoexit','-loglevel','error',str(path)]
        return [player,'-q',str(path)]

    def _play(self, path, cancel):
        with self._condition:volume=self._volume
        if volume==0:return
        if volume==100:return self._play_file(path,cancel)
        with tempfile.TemporaryDirectory(prefix='janggi_volume_') as folder:
            scaled=Path(folder)/'volume.wav'
            self.scaled_wav(path,scaled,volume)
            return self._play_file(scaled,cancel)

    def _play_file(self, path, cancel):
        failures=[]
        for player in self._players:
            if cancel.is_set():return
            try:
                process=subprocess.Popen(self.player_command(player,path),stdin=subprocess.DEVNULL,
                                         stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            except OSError as exc:
                failures.append(str(exc));continue
            with self._condition:
                self._player=process
            while process.poll() is None and not cancel.wait(.02):pass
            if cancel.is_set() and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            _,error=process.communicate()
            with self._condition:
                self._player=None
            if cancel.is_set() or process.returncode == 0:return
            failures.append(error.decode('utf-8',errors='replace').strip()[-180:])
        raise RuntimeError('음성 출력 장치를 확인하세요: '+' / '.join(failures))

    def _run(self):
        engine=None
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda:self._closed or (self._enabled and self._queue))
                    if self._closed:return
                    key,text,expires=self._queue.popleft()
                    if time.monotonic()>expires:continue
                    self._active_key=key;cancel=self._cancel=threading.Event()
                try:
                    if self._bank is not None:
                        for path in self._bank.resolve(text):
                            if cancel.is_set():break
                            self._play(path,cancel)
                    else:  # Explicitly injected legacy diagnostic/test backend.
                        if engine is None:engine=self._factory()
                        with tempfile.TemporaryDirectory(prefix='janggi_speech_') as folder:
                            path=Path(folder)/'message.wav'
                            if engine.render(text,path,cancel) and not cancel.is_set():
                                self._play(path,cancel)
                except Exception as exc:
                    self.on_error(str(exc))
                finally:
                    with self._condition:
                        self._active_key=None
                        self._condition.notify_all()
        finally:
            if engine is not None:engine.close()

    def close(self):
        with self._condition:
            self._closed=True;self._enabled=False;self._interrupt();self._condition.notify_all()
        if self._thread is not None:self._thread.join(timeout=.5)

    def wait_idle(self, timeout=15.):
        """CLI/testing only; game/UI never calls this blocking helper."""
        with self._condition:
            return self._condition.wait_for(lambda:not self._queue and self._active_key is None,timeout)
