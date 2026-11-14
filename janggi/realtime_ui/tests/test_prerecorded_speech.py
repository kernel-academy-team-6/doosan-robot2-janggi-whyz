"""Offline game events, WAV integrity, queue cancellation and software volume."""
import ast
from array import array
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import wave
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import speech
from speech import Speech
from speech_catalog import MESSAGES, clip_name, sentences
from speech_assets import ClipBank


@pytest.fixture
def bank(tmp_path):
    clips={}
    for text in MESSAGES.values():
        path=tmp_path/clip_name(text)
        with wave.open(str(path),'wb') as wav:
            wav.setnchannels(1);wav.setsampwidth(2);wav.setframerate(24000)
            wav.writeframes(array('h',[20000,-20000,1000,-1000]).tobytes())
        clips[text]={'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    (tmp_path/'manifest.json').write_text(json.dumps({'generation':{'lang':'ko','voice':'F1'},'clips':clips}))
    return tmp_path


def test_all_literal_announcement_sentences_have_prerecorded_catalog_entries():
    folder=Path(__file__).resolve().parents[1]
    tree=ast.parse((folder/'app.py').read_text())
    for call in ast.walk(tree):
        if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Attribute):continue
        if call.func.attr not in ('_announce','_announce_position'):continue
        for value in ast.walk(call):
            if isinstance(value,ast.Constant) and isinstance(value.value,str):
                text=value.value.strip()
                # f-string tails are fragments; side variants are covered below.
                if not text or not text.endswith('.') or text.startswith(('의 궁','승리로','승리입니다')):continue
                for part in sentences(text):
                    assert part in MESSAGES.values(),part
    for side in ('초','한'):
        for text in (f'{side} 진영 장군입니다.',f'{side} 진영 외통입니다.',
                     f'{side} 궁이 인식되지 않습니다.',f'{side} 승리입니다.',
                     f'{side}의 궁 포획 승리 수를 계산했습니다.',f'{side} 승리로 대국이 끝났습니다.'):
            assert text in MESSAGES.values()


def test_composed_notices_reuse_clips_without_runtime_model_or_network(bank):
    store=ClipBank(bank)
    paths=store.resolve('착수를 확인했습니다. 로봇이 다음 수를 생각합니다. 초 진영 장군입니다.')
    assert paths==[bank/clip_name(MESSAGES[k]) for k in ('move_confirmed','thinking','초_check')]
    with pytest.raises(ValueError,match='미리 생성되지'):store.resolve('새로운 알림입니다.')
    code=f"""
import sys,socket
sys.path.insert(0,{str(Path(__file__).resolve().parents[1])!r})
socket.socket=lambda *a,**k: (_ for _ in ()).throw(AssertionError('network forbidden'))
from speech import Speech
s=Speech(audio_dir={str(bank)!r},players=['fake-player'],volume=100)
played=[]
s._play=lambda path,cancel: played.append(path)
assert s.set_enabled(True)
assert s.say('착수를 확인했습니다. 로봇이 다음 수를 생각합니다.')
assert s.wait_idle(2) and len(played)==2
assert not any(n in sys.modules for n in ('supertonic','onnxruntime','numpy','huggingface_hub'))
s.close()
"""
    subprocess.run([sys.executable,'-S','-c',code],check=True,capture_output=True,text=True)


def test_async_playback_does_not_block_caller_and_mute_interrupts(bank):
    started=threading.Event();released=threading.Event()
    service=Speech(audio_dir=bank,players=['fake'],volume=100)
    def play(path,cancel):
        started.set();cancel.wait(2);released.set()
    service._play=play
    try:
        assert service.set_enabled(True)
        assert service.say(MESSAGES['robot_move'])
        assert started.wait(1)
        # Worker is still playing, caller can continue and submit another event.
        assert service.say(MESSAGES['robot_complete'])
        service.set_enabled(False)
        assert released.wait(1) and service.wait_idle(1)
        assert not service.say(MESSAGES['voice_test'])
    finally:service.close()


def test_volume_scales_temporary_pcm_without_mutating_original(bank,tmp_path):
    source=bank/clip_name(MESSAGES['voice_test']);before=source.read_bytes()
    target=tmp_path/'scaled.wav';Speech.scaled_wav(source,target,50)
    with wave.open(str(target)) as wav:
        assert list(array('h',wav.readframes(wav.getnframes())))==[10000,-10000,500,-500]
    assert source.read_bytes()==before
    service=Speech(audio_dir=bank,players=['fake'],volume=0)
    service._play_file=lambda *a:pytest.fail('muted volume opened device')
    try:
        assert service.set_enabled(True)
        service.say(MESSAGES['voice_test']);assert service.wait_idle(1)
        with pytest.raises(ValueError):service.set_volume(101)
        service.set_volume(25);assert service._volume==25
    finally:service.close()


def test_missing_or_corrupt_wav_fails_only_audio_and_worker_continues(bank):
    errors=[];played=[]
    service=Speech(audio_dir=bank,players=['fake'],on_error=errors.append)
    service._play=lambda path,cancel:played.append(path)
    try:
        assert service.set_enabled(True)
        (bank/clip_name(MESSAGES['voice_test'])).write_bytes(b'broken wav')
        assert service.say(MESSAGES['voice_test']) and service.wait_idle(1)
        assert len(errors)==1 and not played
        assert service.say(MESSAGES['robot_move']) and service.wait_idle(1)
        assert len(played)==1
    finally:service.close()


def test_missing_assets_do_not_start_synthesizer_or_download(tmp_path):
    service=Speech(audio_dir=tmp_path,players=['fake'])
    try:
        assert not service.available and not service.set_enabled(True)
        assert service._thread is None and 'setup.sh' in service.unavailable_reason
    finally:service.close()


def test_player_failure_is_reported_without_escaping_game_event(bank,monkeypatch):
    def fail(*args,**kwargs):raise OSError('device unavailable')
    monkeypatch.setattr(speech.subprocess,'Popen',fail)
    errors=[];service=Speech(audio_dir=bank,players=['aplay','ffplay'],on_error=errors.append)
    try:
        assert service.set_enabled(True)
        assert service.say(MESSAGES['robot_capture']) and service.wait_idle(1)
        assert errors and 'device unavailable' in errors[0]
    finally:service.close()


def test_ui_check_notice_and_volume_use_cached_async_player(bank):
    from app import App
    from rules import Game
    played=[];service=Speech(audio_dir=bank,players=['fake'],volume=100)
    service._play=lambda path,cancel:played.append(path.name)
    ui=App.__new__(App);ui.speech=service;ui._speech_errors=[]
    game=Game({(4,1):('red','king'),(4,8):('blue','king'),(3,8):('red','cha')},turn='blue')
    try:
        assert service.set_enabled(True)
        ui._announce_position(game,'로봇 동작이 끝났습니다. 사람 차례입니다.')
        assert service.wait_idle(1)
        assert played==[clip_name(MESSAGES[k]) for k in ('robot_complete','human_turn','초_check')]
        ui._set_speech_volume('45.0');assert service._volume==45.
        assert '장군' in ui._speech_caption
        service.set_enabled(False);before=list(played)
        ui._announce('로봇이 상대 말을 잡습니다.')
        assert service.wait_idle(1) and played==before
    finally:service.close()


@pytest.mark.parametrize('kind',['move','place','capture'])
def test_robot_retry_event_plays_matching_f1_wav_and_honors_mute(bank,kind):
    from app import App
    from speech_catalog import RETRY_MESSAGES
    played=[];service=Speech(audio_dir=bank,players=['fake'],volume=100)
    service._play=lambda path,cancel:played.append(path.name)
    ui=App.__new__(App);ui.speech=service;ui._speech_errors=[]
    try:
        assert service.set_enabled(True)
        ui._announce_robot_retry(kind)
        assert service.wait_idle(1)
        assert played==[clip_name(RETRY_MESSAGES[kind])]
        service.set_enabled(False);ui._announce_robot_retry(kind)
        assert service.wait_idle(1) and len(played)==1
    finally:service.close()
