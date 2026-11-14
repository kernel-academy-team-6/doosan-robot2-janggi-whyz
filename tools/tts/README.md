# Supertonic 3 ko/F1 음성 생성 도구

설치·생성: `./tools/tts/setup.sh`

기존 모델로 오프라인 생성:

```bash
env -u PYTHONPATH -u PYTHONHOME tools/.venv-supertonic/bin/python tools/tts/generate.py --offline
```

실제 출력 테스트: `python3 tools/tts/play_test.py --volume 60`

문구는 `janggi/realtime_ui/speech_catalog.py`에 추가한다. 같은 설정·문구·정상 WAV는 다시 생성하지 않는다.
모델과 생성 환경, WAV는 Git에서 제외한다. 게임 실행 시 이 환경이나 모델을 로드하지 않는다.

설치 패키지 전체는 `requirements.lock.txt`, 자세한 구조와 검증은
[Supertonic 기술 문서](../../docs/SUPERTONIC_TTS.md)를 참고한다.
