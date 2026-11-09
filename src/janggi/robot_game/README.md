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
./realtime_ui/run.sh --robot                       # 사람 초(먼저 둠), 로봇 한
./realtime_ui/run.sh --robot --human-side red      # 사람 한
./realtime_ui/run.sh --robot --no-motion           # 로봇을 움직이지 않고 엔진 수·집을 좌표만 기보에 표시
./realtime_ui/run.sh --robot --robot-vel 50 --movetime 3000
```

오른쪽 **로봇 대국** 패널: `대국 시작 (인식된 판)` → 사람이 두고 `착수 완료 (Space)` → 반복. `로봇 정지`는 즉시 정지.
인식된 판을 그대로 쓰므로, 엔진 수가 이상하면 화면의 인식 결과(판·조명)를 확인한다.
로봇 동작 중 오류가 나면 판을 손으로 바로잡고 `인식된 판으로 맞추기` 후 `착수 완료`를 누르면 로봇 차례부터 다시 한다.

## 시뮬레이터로 확인 (카메라·실제 말 없이)

가상 세계(`sim_check.SimWorld`): 말은 교차점에서 몇 mm씩 비켜 놓이고, 가상 카메라가 보정 결과로 말을 픽셀에 투영해
가짜 YOLO 박스를 만든다. 그리퍼가 닫히면 에뮬레이터가 보고한 실제 팁 위치 근처의 말을 잡고, 열리면 그 자리에 놓는다.

```bash
# 터미널 1: 가상 로봇 + Gazebo (Gazebo가 필요 없으면 bringup.launch.py mode:=virtual만 띄워도 된다)
cd ~/doosan_janggi_ws && ros2 launch src/janggi/robot_game/sim_gazebo.launch.py

# 터미널 2
cd ~/doosan_janggi_ws/src/janggi/robot_game
./run_sim_check.sh --gazebo                # 파이프라인 + 턴마다 검사 (Gazebo에 판·말·로봇 표시)
./run_sim_check.sh --top-side blue --seed 7  # 초가 카메라 위쪽인 판
./run_sim_check.sh --no-robot              # 로봇 없이 엔진·인식 경로만
./run_sim_app.sh --gazebo --turns 3        # 실제 대국 UI를 가상 세계로 자동 진행
```

`sim_check` 검사 항목: 처음 인식한 판 = 실제 판, 집을 때 팁-말 중심 거리 < 3 mm(교차점으로 집었을 때 거리도 함께 표시),
로봇이 둔 뒤 실제 판 = 게임 판, 잡은 말이 판 밖에 있음, 관찰 자세 복귀.

Gazebo는 보기 전용이다: `gz_scene.py`가 바닥을 지우고 판 평면 높이에 테이블·장기판·말을 만들고,
로봇은 `/dsr01/joint_states` 순기구학으로 URDF 링크 메쉬를 옮긴다 (물리·컨트롤러 없음).
`e0509_gripper_description/bringup_real_gazebo.launch.py`는 지금 빌드에서 뜨지 않아(실행 파일 이름, mimic 관절) 쓰지 않는다.

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

팁 오프셋 222 mm는 그리퍼 650 기준으로 잰 값이라 개도에 따라 팁 높이가 조금 달라진다.
