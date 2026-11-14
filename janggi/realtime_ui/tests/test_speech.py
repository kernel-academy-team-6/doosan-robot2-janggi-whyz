"""No real speaker/device is opened by these tests; Korean synthesis writes WAV only."""
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
import wave
import ctypes.util
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import speech
from speech import Speech,KoreanSynthesizer
from game_notices import automatic_notice,position_notice,position_key
from automatic_turn import AutomaticTurn
from rules import Game
from app import App


class FakeSynth:
    def render(self,text,path,cancel):
        with wave.open(str(path),'wb') as wav:
            wav.setnchannels(1);wav.setsampwidth(2);wav.setframerate(24000)
            wav.writeframes(b'\x10\x00'*100)
        return not cancel.is_set()
    def close(self):pass


class Player:
    def __init__(self,done=False):
        self.done=done;self.terminated=False
    def poll(self):return 0 if self.done else None
    def terminate(self):self.terminated=True;self.done=True
    def kill(self):self.done=True
    def wait(self,timeout=None):return 0
    def communicate(self):return b'',b''
    @property
    def returncode(self):return self.poll()


def test_disabled_speech_never_initializes_engine_or_plays(monkeypatch):
    monkeypatch.setattr(speech.subprocess,'Popen',lambda *a,**k:pytest.fail('unexpected audio'))
    service=Speech(synthesizer=lambda:pytest.fail('unexpected synthesis'),players=['aplay'])
    try:
        assert not service.say('장군입니다.')
        assert service._thread is None
    finally:service.close()


def test_audio_worker_is_nonblocking_and_mute_stops_only_its_player(monkeypatch):
    started=threading.Event();player=Player();threads=[];commands=[]
    def popen(command,**kwargs):
        commands.append(command);threads.append(threading.current_thread().name);started.set();return player
    monkeypatch.setattr(speech.subprocess,'Popen',popen)
    service=Speech(synthesizer=FakeSynth,players=['/usr/bin/aplay'])
    try:
        assert service.set_enabled(True)
        assert service.say('착수를 인식했습니다.',key='move')
        assert started.wait(2.)
        assert not service.say('착수를 인식했습니다.',key='move')
        service.set_enabled(False)
        assert player.terminated and not service._queue
        assert commands[0][0:2]==['/usr/bin/aplay','-q']
        assert 'janggi-korean-speech' in threads
    finally:service.close()


def test_reset_replaces_queued_countdown_speech(monkeypatch):
    first=threading.Event();second=threading.Event();texts=[];players=[]
    class Engine(FakeSynth):
        def render(self,text,path,cancel):
            texts.append(text)
            return super().render(text,path,cancel)
    def popen(command,**kwargs):
        player=Player(done=bool(players));players.append(player)
        (first if len(players)==1 else second).set();return player
    monkeypatch.setattr(speech.subprocess,'Popen',popen)
    service=Speech(synthesizer=Engine,players=['aplay']);service.set_enabled(True)
    try:
        service.say('착수 인식');assert first.wait(2.)
        service.say('대기 중');service.say('카운트 초기화',replace=True)
        assert second.wait(2.)
        assert players[0].terminated and texts==['착수 인식','카운트 초기화']
    finally:service.close()


def test_missing_backend_remains_off_without_affecting_game():
    service=Speech(synthesizer=FakeSynth,players=[])
    assert not service.available and not service.set_enabled(True)
    assert not service.say('장군')
    service.close()


def test_korean_engine_generates_nonempty_pcm_without_playing(tmp_path):
    if not ctypes.util.find_library('espeak-ng'):pytest.skip('system Korean TTS engine unavailable')
    engine=KoreanSynthesizer()
    try:
        target=tmp_path/'notice.wav'
        assert engine.render('착수를 인식했습니다. 장군입니다.',target)
        with wave.open(str(target)) as stream:
            assert stream.getnchannels()==1 and stream.getsampwidth()==2
            assert stream.getframerate()>0 and stream.getnframes()>1000
            pcm=np.frombuffer(stream.readframes(stream.getnframes()),dtype='<i2')
            assert np.max(np.abs(pcm))>100
    finally:engine.close()


def test_game_notices_use_same_check_rules_and_do_not_mutate_game():
    game=Game({(4,1):('red','king'),(4,8):('blue','king'),(3,8):('red','cha')},top_side='red',turn='blue')
    original=dict(game.board)
    key,text=position_notice(game)
    assert text=='초 진영 장군입니다.' and key==position_key(game)
    assert game.board==original and game.turn=='blue'
    game.board.pop((4,8));assert position_notice(game)[1]=='초 궁이 인식되지 않습니다.'


def test_countdown_notice_is_one_event_per_deadline_and_detects_capture():
    before={(0,6):('blue','soldier'),(0,5):('red','soldier')}
    after={(0,5):('blue','soldier')}
    monitor=AutomaticTurn();monitor.set_baseline(before)
    def state(now,**kw):return monitor.update(after,'blue',now=now,stamp=now,clear=True,epoch=kw.get('epoch',0))
    key,text=automatic_notice(state(1.),before,after)
    assert '포획' in text and automatic_notice(state(2.),before,after)[0]==key
    new_key,_=automatic_notice(state(2.5,epoch=1),before,after)
    assert new_key!=key


def test_large_countdown_and_progress_render_current_remaining_time():
    ui=App.__new__(App);values={}
    for name in ['lbl_auto_move','lbl_auto_seconds','lbl_auto_banner','auto_progress']:
        setattr(ui,name,SimpleNamespace(config=lambda n=name,**kw:values.__setitem__(n,kw)))
    ui._show_auto_status('착수 인식',2.4)
    assert values['lbl_auto_seconds']['text']=='2.4초'
    assert values['auto_progress']['value']==2.4
    ui._show_auto_status('방해 · 초기화')
    assert values['lbl_auto_seconds']['text']=='—' and values['auto_progress']['value']==0.


def test_speech_failure_never_propagates_into_robot_work():
    ui=App.__new__(App);ui._speech_errors=[]
    def fail(*args,**kwargs):raise RuntimeError('audio unavailable')
    ui.speech=SimpleNamespace(say=fail)
    ui._announce('로봇이 말을 옮깁니다.')
    assert ui._speech_errors==['audio unavailable']


def test_notification_timer_is_independent_of_video_redraw():
    ui=App.__new__(App);calls=[]
    ui.args=SimpleNamespace(robot=True);ui._tick_auto_move=lambda:calls.append('countdown')
    ui.rg=None;ui.game=None;ui.robot_busy=False;ui.position_only=False
    ui._speech_errors=[];ui._speech_caption='음성 안내 꺼짐'
    ui.lbl_speech_state=SimpleNamespace(config=lambda **kwargs:None)
    ui.root=SimpleNamespace(after=lambda delay,callback:calls.append(delay) or 'timer')
    ui._tick_notifications()
    assert calls==['countdown',50] and ui._notice_after=='timer'


def test_expired_notifications_are_not_read_after_newer_events(monkeypatch):
    finished=threading.Event();texts=[]
    class Engine(FakeSynth):
        def render(self,text,path,cancel):
            texts.append(text);finished.set();return super().render(text,path,cancel)
    monkeypatch.setattr(speech.subprocess,'Popen',lambda *a,**k:Player(done=True))
    service=Speech(synthesizer=Engine,players=['aplay']);service.set_enabled(True)
    try:
        service._queue.append(('old','지난 착수',time.monotonic()-1.))
        service.say('현재 장군')
        assert finished.wait(2.) and texts==['현재 장군']
    finally:service.close()


def test_failed_alsa_player_falls_back_to_ffplay_without_global_cancel(monkeypatch):
    finished=threading.Event();commands=[];errors=[]
    class Failed(Player):
        def poll(self):return 1
        def communicate(self):return b'',b'no ALSA device'
    def popen(command,**kwargs):
        commands.append(command)
        if len(commands)==1:return Failed(done=True)
        finished.set();return Player(done=True)
    monkeypatch.setattr(speech.subprocess,'Popen',popen)
    service=Speech(synthesizer=FakeSynth,players=['aplay','ffplay'],on_error=errors.append)
    service.set_enabled(True)
    try:
        service.say('장군');assert finished.wait(2.)
        assert commands[1][:4]==['ffplay','-nodisp','-autoexit','-loglevel']
        assert not errors and all('spd-say' not in command for command in commands)
    finally:service.close()


def test_unchanged_check_is_spoken_once_not_every_notification_tick():
    ui=App.__new__(App);spoken=[]
    ui.args=SimpleNamespace(robot=False);ui.rg=None;ui.robot_busy=False;ui.position_only=False
    ui.game=Game({(4,1):('red','king'),(4,8):('blue','king'),(3,8):('red','cha')},turn='blue')
    ui._spoken_game_key=None;ui._speech_errors=[];ui._speech_caption=''
    ui.speech=SimpleNamespace(say=lambda text,**kw:spoken.append(text))
    ui.lbl_speech_state=SimpleNamespace(config=lambda **kw:None)
    ui.root=SimpleNamespace(after=lambda *args:'timer')
    ui._tick_notifications();ui._tick_notifications()
    assert spoken==['초 진영 장군입니다.']
    ui.game.board.pop((3,8));ui._tick_notifications()
    assert len(spoken)==1
    ui.game.board[(3,8)]=('red','cha');ui._tick_notifications()
    assert len(spoken)==2
