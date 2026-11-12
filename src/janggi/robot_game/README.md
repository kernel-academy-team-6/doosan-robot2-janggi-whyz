# robot_game — 사람 ↔ 로봇 장기 대국

사람이 실제 판에 두고 **착수 완료**를 누르면, 로봇이 판을 읽고 엔진(Fairy-Stockfish)으로 수를 정해 직접 둔다.

```text
1. 사람이 두고 [착수 완료] (Space)
2. 판 읽기     카메라 → YOLO → 교차점 할당 → 최근 8프레임 다수결   (판 + 칸별 박스 중심 픽셀)
               인식된 판을 검증 없이 그대로 게임 판으로 쓴다 (한 수로 읽히면 기보에만 남김)
3. 엔진        판 → FEN → Fairy-Stockfish (variant janggi) → 로봇의 수
4. 로봇        잡는 수면: 상대 말 집기(move_abs) → 판 밖 버리는 자리(보정 사각형 바깥)에 놓기
               로봇 말 집기(move_abs) → 목표 교차점에 놓기(move)
               집을 때는 박스 중심을 카메라-로봇 보정으로 base 좌표로 바꿔 실제 말 위치로 간다
               (박스 중심·말 위치가 교차점과 꼭 맞지 않으므로). 교차점에서 18 mm 넘게 벗어나면 교차점으로 집는다.
5. 관찰 자세   보정 도구의 홈 자세(src/janggi/calibration/config/robot_poses.yaml)로 돌아가 카메라를 비키고 다시 사람 차례
```

## 준비 (처음 한 번)

```bash
cd ~/doosan_janggi_ws
./janggi/setup_venv.sh              # 인식용 venv (이미 했으면 생략)
./src/janggi/engine/get_engine.sh   # Fairy-Stockfish largeboard 바이너리 → src/janggi/engine/bin/fairy-stockfish
```

## 실제 대국

터미널 1에서 로봇 드라이버(RUNBOOK 9절), 보정이 끝난 상태에서:

```bash
cd ~/doosan_janggi_ws/janggi
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot                       # 사람 초(먼저 둠), 로봇 한
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --human-side red      # 사람 한
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --no-motion           # 로봇을 움직이지 않고 엔진 수·집을 좌표만 기보에 표시
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --robot-vel 50 --movetime 3000
```

오른쪽 **로봇 대국** 패널: `대국 시작 (인식된 판)` → 사람이 두고 `착수 완료 (Space)` → 반복. `로봇 정지`는 즉시 정지.
인식된 판을 그대로 쓰므로, 엔진 수가 이상하면 화면의 인식 결과(판·조명)를 확인한다.
로봇 동작 중 오류가 나면 판을 손으로 바로잡고 `인식된 판으로 맞추기` 후 `착수 완료`를 누르면 로봇 차례부터 다시 한다.

## Gazebo 카메라를 기존 UI로 인식

```bash
# 터미널 1 (workspace 환경 설정 후): 가상 드라이버 + Gazebo + 장면 + 브리지
cd ~/doosan_janggi_ws
ros2 launch src/janggi/robot_game/sim_gazebo.launch.py

# 터미널 2: 기존 실시간 UI, 실제 렌더링 영상과 실제 YOLO+CNN
cd ~/doosan_janggi_ws/janggi
./realtime_ui/run.sh --source gazebo --model-mode hybrid --robot
```

현재는 RGB 전용 카메라로 깊이·포인트 클라우드·그림자 렌더링을 끈다. 설정 변경 적용에는 Gazebo 재시작이 필요하다.
별도 `sim_app.py` 창은 필요 없다. 화면 클릭으로 Gazebo 말 좌표를 바꾸는 동작은 비활성화했다.
사람 수는 접촉·외력으로 말을 움직인 뒤 영상 인식 갱신을 확인하고 `[착수 완료]`를 누른다.
로봇은 가상 드라이버의 관절 피드백을 목표로 토크 PID로 구동한다. 실제 링크와 말은 물리 엔진이 움직인다.
말은 5mm 두께, 밀도 1,200kg/m³를 가정한 동적 충돌체이며 가상 부착·목적지 스냅은 없다.
집기 제어는 PickPlace2의 고정 160mm 플랜지 연장를 사용한다. 물리적 집기에는 그리퍼와 실제 장착 형상의 일치 검증이 필요하다.
회색 받침은 수평 Z=−30mm, 판 두께는 10mm, 윗면은 −20mm다.

렌더링 영상의 CNN 오분류도 그대로 표시하므로 대국 전에 인식 결과를 확인한다.
[설정·보정·영상 캡처·동작 한계](../../../docs/GAZEBO_CALIBRATED_SCENE.md)를 참고한다.

### 레거시 합성 검사

`sim_check.py` / `sim_app.py`는 가짜 검출을 쓰는 별도 테스트 도구다.
이 도구를 쓸 때만 launch에 `scene:=false`를 지정하고 실행한다. 기존 실시간 UI 경로와 함께 실행하지 않는다.
`--setup-only`는 여전히 엔진·홈 이동·자동 착수 없이 배치만 표시하는 레거시 옵션이다.

## 구성

| 파일 | 내용 |
|---|---|
| `session.py` | 대국 진행 로직 `RobotGame` (사람 수 읽기 → 엔진 → 집기/놓기/버리기 → 관찰 자세). GUI·카메라 무관 |
| `../engine/fairy.py` | Fairy-Stockfish UCI 래퍼, 판 ↔ FEN (초 위쪽 판은 180° 돌려서 넘김) |
| `janggi/piece_detection/world.py` | `CameraToBase`: 픽셀 → base 좌표 (왜곡 제거 → 광선 → 판 평면 + 말 높이) |
| `../action/` | `move_abs(x, y, z)`, `move(x, y)`(= 교차점 → move_abs), `PiecePicker` |
| `sim_check.py`, `sim_app.py`, `gz_scene.py`, `sim_gazebo.launch.py` | 시뮬레이터 |

## 실제 로봇에서 맞춰야 할 값 (`action/pick_place.py` `PickConfig`)

시뮬레이터는 말 크기·그리퍼 개도를 모른다. 처음에는 `--robot-vel 30`으로 한 수씩 확인한다.

| 값 | 기본 | 내용 |
|---|---|---|
| `grasp_height_mm` | 0 | 집을 때 팁 높이 (0 = 판 표면) |
| `place_height_mm` | 0 | 놓을 때 팁 높이 (0 = 판 표면) |
| `open_cmd` | 350 | 집기 전 개도. 0(완전히 열림)이면 손가락이 이웃 말에 닿을 수 있다 |
| `close_cmd` | 700 | 쥐기 |
| `travel_mm` | 40 | 말을 든 채 지나가는 높이 |
| `discard_x` | -1.5 | 버리는 자리 판 좌표 x (판 왼쪽 바깥, y 0~9 순서로 채움). 오른쪽에 두려면 9.5 |
| `discard_height_mm` | 0 | 버릴 때 팁 높이 (판 평면 기준, 판 밖 바닥은 판보다 낮다) |

컨트롤러 TCP는 J6 플랜지 원점이다. 집기·놓기 명령의 팁 연장은 고정 160mm다. 보정·원점·장면 계산에는 원래 222mm를 유지하며 합산하지 않는다.

실시간 UI 로봇 대국의 집기·놓기는 이제 `janggi_pick_place2` 실행기를 직접 재사용한다.
말 폭별 개도와 고정 160mm 플랜지 연장, 현재 높이 수평 접근·하강·복귀를 적용한다.
기본 설정은 `src/janggi_pick_place2/config/pick_place.yaml`(현재 고정 플랜지 연장=160mm)이다.
별도 액션 서버나 빌드 없이 UI 재시작으로 적용된다.
[상세 동작·파라미터](../../../janggi/realtime_ui/README.md#로봇-대국의-집기놓기-방식)를 참고한다.

UI와 독립 액션 서버는 PickPlace2의 `runtime.create_clients`로 같은 백엔드를 만든다.
UI 기본 yaw는 0°, 속도는 YAML 설정이다. 이전의 보정 yaw 자동 적용은 제거했다.
관찰 자세 파일이 없으면 별도의 과거 기본값 대신 PickPlace2 홈 자세를 사용한다.
실제 요청 좌표·관찰 자세 명령은 UI 세션의 `robot_commands.jsonl`에서 확인한다.
