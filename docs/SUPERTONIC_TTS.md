# 장기 게임의 한국어 여성 음성 안내

검증일: 2026-11-14 (한국 시간)

현재 기본 음성은 **Supertonic 3 / 한국어 `ko` / 여성 `F1`**이다.
음성 생성 도구와 게임 실행 환경을 분리했다. 게임 로직, 엔진, 카메라, ROS 2 제어 코드는 변경하지 않고
기존 음성 이벤트를 미리 만든 WAV에 연결했다.

## 실행 방법

이 컴퓨터는 설치와 음성 생성이 완료돼 있다. 평소 사용하던 UI 명령으로 실행한 뒤,
오른쪽 상단의 **음성 안내**를 켜거나 **음성 테스트**를 누른다.
**음성 볼륨 (%)**으로 0~100%를 선택한다. 기본 볼륨은 80%, 음성 기본 상태는 꺼짐이다.
볼륨은 다음 문장부터 적용하며, 음성을 끄면 현재 재생과 대기열을 취소한다.
설정은 현재 실행 동안 유지한다. UI 재시작만 필요하며 colcon 빌드는 필요 없다.

```bash
cd ~/doosan_janggi_ws
# UI를 켜지 않고 실제 음성 출력만 확인
python3 tools/tts/play_test.py --volume 60
# 특정 이벤트만 확인
python3 tools/tts/play_test.py --message king_buried --message 한_victory
```

새 컴퓨터에서 최초 준비하거나 문구를 추가했을 때:

```bash
cd ~/doosan_janggi_ws
./tools/tts/setup.sh
```

최초 설치·모델 다운로드에는 인터넷을 사용한다. 이미 있는 WAV의 SHA-256과 생성 설정이 같으면
다운로드·모델 로딩·합성을 모두 생략한다. 설치된 모델로만 생성하려면:

```bash
env -u PYTHONPATH -u PYTHONHOME tools/.venv-supertonic/bin/python tools/tts/generate.py --offline
```

오프라인 재생은 별도 가상환경을 활성화할 필요 없이 기존 UI Python으로 작동한다.
파일이 없거나 새 문구가 등록되지 않았다면 자동 다운로드나 실시간 합성을 시도하지 않고
음성 오류만 안내한다. 게임과 로봇 제어는 계속 진행한다.

## 기존 구조 조사와 변경 구조

기존 `app.py`는 `_announce()`와 `_announce_position()`에서 게임 이벤트를 음성으로 전달했고,
`speech.py`는 eSpeak NG로 실행 중 WAV를 합성한 뒤 `aplay` 또는 `ffplay`로 재생했다.
이미 있던 비동기 대기열, 중복 억제, 장군/외통 판정, 자동 착수 카운트 연결을 유지했다.

```text
준비 시에만:
setup.sh → 전용 venv → 공식 아카이브 ONNX 모델 + F1 → generate.py → WAV + manifest

게임 실행 시:
기존 게임 이벤트 → _announce() → Speech.say() → 제한된 대기열 → 별도 재생 스레드
                                                        ↓
                                    문장별 WAV 조회 → 임시 볼륨 조절 → aplay / ffplay
```

게임 경로에는 Supertonic / ONNX Runtime / Hugging Face import나 네트워크 호출이 없다.
`KoreanSynthesizer` 클래스는 기존 진단 테스트용으로 남아 있지만 UI의 기본 재생 경로에서는 생성하지 않는다.
음성 생성 프로세스가 종료되면 모델도 메모리에서 해제된다. TTS 서버나 상주 모델 프로세스는 띄우지 않는다.

## 공식 패키지·모델 호환성

- 공식 Python 패키지: `supertonic==1.3.1`.
- 모델 저장소: `supertone-oss-archive/supertonic-3`.
- 모델 리비전: `aafc6e32416a594460b32413efc49d7fe4ce6d46`로 고정.
- SDK의 기본 다운로드 주소는 `Supertone/supertonic-3`이므로 그대로 사용하지 않았다.
  Hugging Face에서 지정한 아카이브 리비전을 다운로드하고,
  `TTS(model='supertonic-3', model_dir=..., auto_download=False)`로 공식 SDK에 전달한다.
- `get_voice_style(voice_name='F1')`, `synthesize(..., lang='ko')`로 실제 합성했다.
  별도 SDK 수정이나 모델 변환 없이 ONNX 4개, 설정·문자 인덱스, F1 스타일이 호환된다.
- 생성 설정: 10 steps, speed 1.05, seed 42, CPU 4 threads.
- 출력: **44,100Hz, 모노, PCM16 WAV**. 총 48개, 약 116.47초, 약 9MB.

공식 근거: [Python SDK와 한국어/F1 사용법](https://github.com/supertone-oss-archive/supertonic-py),
[아카이브 모델](https://huggingface.co/supertone-oss-archive/supertonic-3).
SDK는 MIT, 모델은 OpenRAIL-M이며 내려받은 모델 폴더에 원본 LICENSE도 함께 보관한다.

## 안내 이벤트 목록

문장 목록의 원본은 `janggi/realtime_ui/speech_catalog.py`의 `MESSAGES`이다.

| 상황 | 안내 내용 |
|---|---|
| 음성 제어·테스트 | 음성 켜짐, 음성 테스트, 테스트 장군 |
| 사람 착수 | 착수 인식·확인, 포획 인식·확인, 현재 판 확인 |
| 게임 진행 | 대국 시작, 사람 차례, 로봇 생각, 이동, 포획, 동작 완료, 계산 미리보기, 한수쉼, 수 없음 |
| 로봇 조작 | 호밍 시작·미리보기, 정지 요청, 착수 재확인, 동작 오류, 화면 확인 |
| 자동 착수 | 자동 모드 켜짐·꺼짐, 카운트 재시작, 영상 지연, 방해 감지, 인식 취소 |
| 장군·외통 | 초/한 각각 장군·외통·궁 미검출 |
| 승리 | 궁 묘지 포획 완료, 초/한 승리, 승리 수 미리보기, 대국 종료, 새 대국 안내 |
| 포획 미완료 | 궁 포획 미완료, 판 확인 |
| 실패 재시도 | 원위치 말 다시 집어 이동, 미놓임 추정 시 놓기만 재시도, 남은 포획 말 다시 치우기 |

예를 들어 “로봇 동작이 끝났습니다. 사람 차례입니다. 초 진영 장군입니다.”는
각 문장 WAV 3개를 차례로 재생한다. 같은 문장에 대한 파일을 중복 생성하지 않는다.
실제 궁 포획 후 승리 안내를 하는 기존 타이밍도 유지한다.

## 비동기·실패 처리

- UI/로봇 스레드는 `say()`로 큐에 넣기만 한다. 음성 종료를 기다리지 않는다.
- 큐 최대 4개, 대기 유효기간 8초, 같은 알림 중복 억제, 중요한 새 알림의 기존 재생 교체를 유지한다.
- 볼륨은 원본 파일을 변경하지 않고 임시 PCM 복사본에 적용한다. 재생 후 임시 파일을 삭제한다.
- 음소거는 이 UI가 시작한 재생 프로세스만 중지한다.
- `aplay` 실패 시 `ffplay`를 시도한다. 모두 실패해도 오류를 UI에 전달한 뒤 다음 알림을 처리한다.
- WAV 손상은 SHA-256/PCM 검사로 감지한다. 알림 재생 실패가 로봇 명령을 취소하거나 게임 스레드로 전파되지 않는다.
- `wait_idle()`은 재생 검증 CLI에서만 사용한다. 게임에서는 호출하지 않는다.

## 경로와 의존성

| 경로 | 용도 |
|---|---|
| `tools/.venv-supertonic/` | 음성 생성 전용 Python 3.12 환경, 약 203MB |
| `tools/tts/model/` | 고정 리비전 모델·F1 스타일·라이선스, 약 381MB |
| `janggi/realtime_ui/assets/speech/ko_F1/` | 재사용 WAV 48개와 `manifest.json`, 약 9MB |
| `tools/tts/setup.sh` | 독립 환경 설치와 다운로드·생성 실행 |
| `tools/tts/generate.py` | 누락·손상·설정 변경 파일 생성, 원자적 저장과 해시 기록 |
| `tools/tts/play_test.py` | 게임과 동일한 비동기 플레이어로 실제 출력 검증 |
| `tools/tts/requirements.lock.txt` | 이번 설치에 사용한 전체 패키지 버전 |
| `janggi/realtime_ui/speech_catalog.py` | 문구·모델 리비전·파일명 규칙 |
| `janggi/realtime_ui/speech_assets.py` | 오프라인 파일 조회와 무결성 확인 |
| `janggi/realtime_ui/speech.py` | 비동기 큐·볼륨·재생·음소거 |
| `janggi/realtime_ui/app.py` | 기존 이벤트 연결과 볼륨 UI |

주요 설치 패키지는 Supertonic 1.3.1, ONNX Runtime 1.32.0, NumPy 2.5.4,
SoundFile 0.14.0, Hugging Face Hub 2.4.0이다. 전부 생성 전용 환경에만 설치했다.
`PYTHONPATH`/`PYTHONHOME`을 비워 ROS 경로가 설치·생성에 섞이지 않게 한다.
전용 환경의 `pip check`는 정상이며, 기존 `janggi/.venv`에는 Supertonic이 설치되지 않았음을 확인했다.
기존 게임 requirements와 ROS 패키지는 수정하지 않았다.

외부 모델, 전용 환경, 생성 WAV는 `.gitignore`로 제외했다. Git에는 코드·문구·설정·문서만 남기며,
다른 컴퓨터에서는 `setup.sh`로 생성한다.

## 검증 결과

- 공식 아카이브에서 실제 모델·F1 스타일 다운로드 및 한국어 48문장 생성 성공.
- WAV 48개 모두 해시 일치, PCM16/모노/44.1kHz 형식 확인.
- 실제 호스트 오디오 출력 장치에서 볼륨 60%로 다음 세 문장 재생 완료:
  “장기 음성 안내 테스트입니다.” / “초 진영 장군입니다.” / “한 승리입니다.”
  재생 프로세스 정상 종료를 확인했다. 사람의 청취에 의한 음질 평가는 별도다.
- 샘플 피크 PCM 값은 각각 13057 / 11289 / 8346으로, 무음이나 PCM 포화 출력이 아니다.
- 일반 Python에서 `generate.py --offline`을 재실행해 **48개 재사용, 모델 로딩·다운로드·합성 없음** 확인.
- 소켓 사용을 차단하고 ML 라이브러리가 없는 Python `-S` 프로세스에서도 게임 음성 경로 재생 검증.
- 실제 UI 안내 함수 → 장군 판정 → 해당 WAV 순차 재생 및 볼륨 콜백 검증.
- 음소거·비동기 진행·볼륨 샘플 변환·손상 파일·플레이어 실패 후 복구 검증.
- 관련 전체 테스트 **457개 통과**. 기존 protobuf 관련 경고 3개는 남아 있다.
- 실제 로봇 이동 명령은 실행하지 않았다.

검증 명령:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH=src/janggi_pick_place2:src/janggi/robot_game:janggi/realtime_ui:$PYTHONPATH \
  python3 -m pytest -q janggi/realtime_ui/tests src/janggi/robot_game/tests \
  src/janggi/action/tests src/janggi/engine/tests src/janggi_pick_place2/tests
```

### 재시도 음성 추가

`ManipulationCoordinator`가 기존 관측 조건에 따라 실제 재시도를 선택한 직후
`on_manipulation_retry` 콜백으로 UI에 알린다. 음성 때문에 기다리거나 재시도 조건을 바꾸지 않는다.

- 이동 재시도: “말이 이동하지 않아 다시 집어 옮기겠습니다.”
- 놓기 재시도: “말을 놓지 못한 것으로 보여 놓기를 다시 시도하겠습니다.”
- 포획 재시도: “포획할 말이 남아 있어 다시 집어 치우겠습니다.”

그리퍼 통신 오류라고 단정하는 안내는 하지 않는다. 현재 감지는 홈에서 본 말 위치에 기반한다.
음성이 꺼져 있으면 재생하지 않으며, 콜백이나 오디오 오류가 발생해도 기존 재시도는 진행한다.
기존 WAV 45개를 재사용하고 추가 3개만 오프라인 생성했다.
