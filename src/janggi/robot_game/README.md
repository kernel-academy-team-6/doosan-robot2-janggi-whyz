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
               (박스 중심·말 위치가 교차점과 꼭 맞지 않으므로). 집기는 항상 검출된 말 중심 XY를 사용한다. 중심이 없거나 변환이 불가능하면 격자점으로 대체하지 않는다.
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
로봇은 Gazebo 표준 JointTrajectoryController의 낮은 위치 PI·실제 속도 감쇠·중력 보상으로 구동한다. 실제 링크와 말은 물리 엔진이 움직인다.
가상 드라이버의 `/joint_states`는 100Hz로 발행되지만 DRCF 자세는 약 0.1초(10Hz)마다만 바뀌고 속도는 항상 0이다.
그래서 값이 실제로 바뀐 샘플만 갱신 주기에 맞춰 시각을 보정해 쓰고(`joint_targets.HeldStreamSampler`), 지연을 갱신 주기의 2배+30ms(약 0.23초)로 둔 3차 보간 결과를 명목 200Hz로 전달한다.
느린 Gazebo에서는 simulation time에 맞춰 경로를 재생한다.
현재 게인은 `physics_robot.py`의 `ARM_GAINS`에 있다. [2026-11-14 물리 파지·타이밍 검증](../../../docs/analysis/physical_grasp_20261114/README.md)에 원인과 실제 시험 결과를 기록했다.
게인은 Gazebo 전용이며 실제 두산 제어기의 공개된 내부 게인을 복사한 값은 아니다.
ROS/RViz 표시 환산은 원래의 700 → 1.0rad다. Gazebo는 별도로 공통 W(c)에 맞는 CAD 관절각을 계산한다.
[패드 간격 물리 검증](../../../docs/analysis/gripper_gap/README.md)에서 결과와 적용 범위를 확인할 수 있다.
말은 팔각형 최소 너비/두께가 소형 22/6mm, 중형 28/7mm, 궁 34/8mm인 동적 충돌체다.
사진·몸체·충돌 형상의 폭과 종류별 배치 높이를 맞췄으며, 밀도 1,200kg/m³로 질량·관성을 계산한다.
가상 부착·목적지 스냅은 없다.
집기 제어는 PickPlace2의 132.6717mm 기준값 + H(c) 플랜지 연장을 사용한다. 물리적 집기에는 그리퍼와 실제 장착 형상의 일치 검증이 필요하다.
Gazebo 생성 모델만 손끝 길이에 맞춘 15.377546mm 장착 어댑터를 사용한다. 원본 URDF/RViz/실물의 장착값은 유지한다.
말은 기본 박스 4개의 합집합으로 정확한 팔각형 옆면을 만들고, 판에는 0° 박스 하나만 닿는다(나머지 3개는 위아래 0.1mm 안쪽). 물리 스텝은 0.5ms다.
충돌 필터(`collision_masks.py`): 말끼리, 그리고 base_link~link_5와 말 사이는 충돌 검사를 하지 않는다. 손가락·그리퍼 본체·link_6은 말과 충돌한다.
이 두 가지가 없으면 말들이 잠들지 못하거나 하나의 island로 묶여, 팔이 판 근처에 있을 때 RTF가 0.1~0.3(카메라 3~9FPS)으로 떨어진다.
가상 백엔드는 `/dsr01/gazebo/joint_states`의 실제 물리 관절이 안정된 뒤 다음 단계를 실행한다.
회색 받침은 수평 Z=−30mm, 판 두께는 10mm, 윗면은 −20mm다.
UI를 `--source gazebo`로 실행하면 집기·놓기 및 좌표 미리보기에서도 판 표면 Z를 −20mm로 고정한다.
실물의 4점 높이 실측값과 수동 Z 보정은 Gazebo에 적용하지 않는다. 팁 집기 높이는 모든 말에서 판 위 2mm이므로 −18mm이며, 플랜지 목표에는 기존 그리퍼 개도별 툴 길이를 더한다.

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

### 궁 포획으로 승리 마무리

로봇 차례에 규칙상 바로 잡을 수 있는 상대 궁이 있으면 그 포획 수를 우선한다.
일반 수가 외통을 만들었을 때도 로컬 규칙의 외통 판정과 Fairy-Stockfish의 합법 응수 없음이
일치하면 상대 궁을 별도로 집어 묘지로 옮긴다. 단순 장군에는 궁을 치우지 않는다.
궁은 다른 포획 말과 같은 검출 중심 좌표·크기 프로필·묘지 위치를 사용한다.
궁 치우기와 홈 복귀가 끝난 뒤 승리를 확정하고 UI에 표시하며, 음성 안내가 켜져 있으면
“상대 궁을 포획해 묘지로 옮겼습니다. … 승리입니다.”라고 안내한다.
종료 후 자동 착수는 꺼지고 [대국 시작]으로 새 게임을 시작할 수 있다.
`--no-motion`은 실제 포획 완료 대신 승리 수 계산으로 안내한다.

모든 포획 말의 묘지 놓기는 **열기 → 기존 작동 대기 → 같은 폭으로 다시 열기 → 기존 작동 대기**이다.
두 명령 사이에는 팔을 움직이지 않는다. 실제 기본 대기는 각각 2초(총 약 4초)이며
시리얼 응답을 기다리는 방식은 아니다. 보드 위 일반 플레이스의 열림 횟수는 그대로다.
정지 요청은 두 번째 명령 전에도 확인한다. 두 번째 열기에서 오류가 나면 포획 완료로 기록하지 않는다.

### 관측 오류 복구

`observations.py`의 `SnapshotTimeout`, `TargetNotFound`만 자동 복구 대상으로 구분한다.
`ManipulationCoordinator.observe_home()`은 홈 복귀 후 안정 영상 수신을 최대 3회(기본 3/6/9초) 시도한다.
`find_at_home()`은 후보가 없거나 모호할 때 홈에서 새 영상을 얻어 대상을 최대 3회 조회한다.
복구는 관측/대상 조회 단계에서 수행하며, 완료된 포획·집기·놓기 작업 전체를 재시작하지 않는다.
계획 단계에서 검출 중심이 없을 때는 실행 전에 홈 관측과 중심 조회를 다시 수행한다.
사용자 정지나 모션 예외는 이 재시도에서 잡지 않는다. 모든 복구는 기존 홈 자세·제어기를 사용한다.
