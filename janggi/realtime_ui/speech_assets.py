"""Offline-only WAV lookup; importing this module never imports an ML package."""
import hashlib
import json
from pathlib import Path
import wave
from speech_catalog import DEFAULT_AUDIO_DIR, MESSAGES, sentences


class ClipBank:
    def __init__(self, folder=DEFAULT_AUDIO_DIR):
        self.folder=Path(folder).resolve()
        self.clips={}
        self.error='음성 파일이 없습니다. tools/tts/setup.sh를 실행하세요.'
        try:
            manifest=json.loads((self.folder/'manifest.json').read_text())
            generation=manifest['generation']
            if generation['lang']!='ko' or generation['voice']!='F1':
                raise ValueError('한국어 F1 음성이 아닙니다.')
            self.clips=manifest['clips']
            for text in MESSAGES.values():
                self._path(self.clips[text])
            self.error=''
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.error=f'한국어 F1 음성 준비 필요: tools/tts/setup.sh ({exc})'
        self.available=not self.error

    def _path(self, entry):
        path=(self.folder/entry['file']).resolve()
        if path.parent!=self.folder or not path.is_file() or path.suffix!='.wav':
            raise ValueError('음성 파일 경로가 없거나 올바르지 않습니다.')
        return path

    def resolve(self, text):
        paths=[]
        for sentence in sentences(text):
            entry=self.clips.get(sentence)
            if entry is None:
                raise ValueError(f'미리 생성되지 않은 음성입니다: {sentence}')
            path=self._path(entry)
            if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:
                raise ValueError(f'음성 파일 검증 실패: {path.name}')
            with wave.open(str(path)) as wav:
                if wav.getsampwidth()!=2 or wav.getnchannels()!=1 or not wav.getnframes():
                    raise ValueError(f'유효한 PCM16 음성이 아닙니다: {path.name}')
            paths.append(path)
        return paths
