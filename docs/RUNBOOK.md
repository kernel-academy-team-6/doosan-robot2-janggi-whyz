# 실행 및 검증 Runbook

## 1. 환경 불러오기

```bash
source /opt/ros/jazzy/setup.bash
source ~/doosan_janggi_ws/install/setup.bash
```

매 터미널마다 실행한다. 자동 source를 원하면 사용자가 직접 `.bashrc`에 추가한다.

## 2. 정적 검증

```bash
./scripts/verify_workspace.sh ~/doosan_janggi_ws
```

패키지 여섯 개와 Tool Flange Serial 서비스 네 개가 모두 `OK`여야 한다.

## 3. 에뮬레이터 준비

```bash
./scripts/pull_emulator.sh
nproc
```

Doosan의 `run_drcf.sh`는 CPU 네 개를 할당하므로 논리 CPU가 4개 이상인 환경을 권장한다.

## 4. 가상 로봇

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=virtual host:=127.0.0.1 port:=12345 name:=dsr01
```

가상 모드에서 RViz의 그리퍼 형상과 joint state는 확인할 수 있다. 실제 Tool Flange Serial 하드웨어가 없으므로 그리퍼 open/close 통신은 성공하지 않는다.

## 5. 실제 로봇

먼저 PC의 유선 네트워크를 로봇과 같은 대역으로 설정하고 통신을 확인한다.

```bash
ping -c 3 ROBOT_IP
nc -zv ROBOT_IP 12345
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=ROBOT_IP port:=12345 name:=dsr01
```

실제 IP를 Git에 커밋하지 않는다. 문서와 스크립트에는 항상 `ROBOT_IP` placeholder를 사용한다.

## 6. 팔 명령

```bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

실제 로봇에서는 저속·낮은 가속도, 충분한 작업 공간, 비상정지 접근성을 먼저 확인한다.

## 7. 그리퍼 명령

```bash
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

`position_cmd`의 범위는 0~700이며, 이 통합 코드에서는 0이 열림, 700이 닫힘이다.

## 8. 장애 확인

```bash
docker ps -a --filter name=dsr01_emulator
docker logs dsr01_emulator 2>&1 | head -n 60
docker exec dsr01_emulator ps aux
docker inspect -f '{{.Path}} {{.Args}}' dsr01_emulator
docker exec dsr01_emulator env | grep ROBOT_MODEL
nc -zv 127.0.0.1 12345
```

에뮬레이터를 재생성해야 할 때만 다음을 사용한다. 실행 중인 상태는 삭제되므로 먼저 `docker ps -a`로 대상 이름을 확인한다.

```bash
docker rm -f dsr01_emulator
```

그 뒤 launch를 다시 실행하고 최대 1~2분 기다린다.

## 9. 장기 보정·이동 도구

터미널 세 개를 쓴다. 각 터미널에서 1번(환경 불러오기)을 먼저 한다. 실제 로봇 IP는 `ROBOT_IP` 자리에 넣는다.

**터미널 1 — 로봇 드라이버.** 작업이 끝날 때까지 켜 둔다. 이 터미널을 Ctrl+C로 끄거나 닫으면 다른 도구도 로봇과 끊긴다.

```bash
ping -c 3 ROBOT_IP
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=ROBOT_IP port:=12345 name:=dsr01        # RViz가 필요 없으면 rviz:=false
```

로그에 `Connected RT control stream`과 `Successfully switched controllers!`가 나오면 연결된 것이다.

**터미널 2 — 카메라-로봇 보정 / 조그.** 자세한 사용법은 [src/janggi/calibration/README.md](../src/janggi/calibration/README.md).

```bash
cd ~/doosan_janggi_ws/src/janggi/calibration
./setup_venv.sh          # 처음 한 번
./run.sh                 # 보정 (카메라 + 조그/그리퍼/홈 패널) → config/calibration.yaml
./run.sh --jog-only      # 카메라 없이 로봇 패널만
```

**터미널 3 — 장기판 좌표 이동 예제.** 보정이 끝난 뒤에 쓴다. 자세한 내용은 [src/janggi/action/README.md](../src/janggi/action/README.md).

```bash
cd ~/doosan_janggi_ws/src/test
./run_move_example.sh --dry-run      # 목표 좌표만 출력 (로봇 불필요)
./run_move_example.sh --vel 50       # (0,0)→(1,1)→…→(8,8)→(8,9), 각 교차점 판 표면까지 (사이 이동은 30 mm 높이)
```

**사람 ↔ 로봇 대국.** 보정이 끝난 뒤에 쓴다. 자세한 내용은 [src/janggi/robot_game/README.md](../src/janggi/robot_game/README.md).

```bash
cd ~/doosan_janggi_ws
./src/janggi/engine/get_engine.sh    # 처음 한 번: Fairy-Stockfish
./janggi/realtime_ui/run.sh --model models/weights_14class.pt --robot          # [대국 시작] → 사람이 두고 [착수 완료](Space) → 로봇이 둔다
```

카메라·로봇 없이 확인하려면 가상 로봇(+ Gazebo)으로:

```bash
ros2 launch src/janggi/robot_game/sim_gazebo.launch.py                 # 워크스페이스 루트에서
./src/janggi/robot_game/run_sim_app.sh --gazebo --setup-only          # 다른 터미널: 배치만 생성, 팔 이동 없이 대기
./src/janggi/robot_game/run_sim_check.sh --gazebo                   # 다른 터미널
```

배치 전용과 자동 대국 프로그램은 하나씩 실행한다. `--setup-only`는 홈 이동·엔진 시작·자동 착수를
하지 않으며 현재 로봇 자세를 표시한다. 자동 대국으로 전환할 때는 배치 전용 창을 닫고
`./src/janggi/robot_game/run_sim_app.sh --gazebo --turns 3`을 실행한다.

Gazebo의 판·D455f는 저장된 보정 좌표를 사용한다. 판 외곽은 400×400mm 추정값이며 격자는 실측 보정값을 유지한다.
카메라 영상은 `/janggi_sim/d455f/color/image_raw`, 깊이는
`/janggi_sim/d455f/aligned_depth_to_color/image_raw`에서 확인한다.
새 Sensors world를 읽도록 기존 Gazebo와 드라이버를 종료하고 첫 터미널부터 재시작한다.
[보정 장면 문서와 실제 렌더링 캡처](GAZEBO_CALIBRATED_SCENE.md)에 치수 검증 및 한계를 정리했다.

### 연결이 끊길 때

| 증상 | 원인 확인 | 대응 |
|---|---|---|
| 드라이버 로그 `Disconnected.. Please check out Ethernet Cable..` 후 `RT data is stale`가 계속 늘어남 | `journalctl --since "10 min ago" \| grep -iE "wlo1\|enp"` | PC↔로봇 네트워크가 끊겼다. 와이파이로 연결했다면 유선 랜으로 바꾼다. 와이파이를 써야 하면 절전을 끈다: `nmcli con modify <로봇 와이파이> 802-11-wireless.powersave 2`. 복구 후 드라이버(터미널 1)만 다시 실행한다 |
| `RT data is stale ... 0.1 s`가 가끔 나옴 | — | 네트워크 지연. 짧게 회복되면 동작에는 문제없지만 와이파이라면 유선을 권장한다 |
| 보정 화면 `CAMERA LOST`, `Frame didn't arrive` | `journalctl -k \| grep -iE "usb\|uvc" \| tail` (`error -71`, `USB disconnect`) | 카메라 USB가 잠깐 빠졌다. 도구가 자동 재연결한다. 자주 끊기면 허브·연장선 없이 USB 3 포트에 직접, 짧은 케이블로 연결한다 |
| `Package 'e0509_gripper_description' not found` | `echo $AMENT_PREFIX_PATH` | 이 워크스페이스의 `install/setup.bash`를 source하지 않았다 (`~/ros2_ws`만 source된 셸) |
| `ModuleNotFoundError: rclpy` | `which python3` | 다른 venv의 python3이 잡혔다. `run.sh`, `run_move_example.sh`로 실행하거나 `/usr/bin/python3`를 쓴다 |

같은 PC에서 에뮬레이터(`mode:=virtual`)와 실제 로봇 드라이버를 동시에 띄우지 않는다. 둘 다 `dsr01` 이름을 쓰므로 명령이 어느 쪽으로 갈지 보장되지 않는다.

## 알려진 호환성 판단

- `fhekwn549/e0509_gripper_description`만 공식 Jazzy 드라이버 위에 빌드하면 `FlangeSerialOpen` 헤더가 없어 실패한다.
- `fhekwn549/doosan-robot2` Humble 포크 전체를 Jazzy에 빌드하면 Jazzy DRFL 바이너리가 없고 `ros2_control` 하드웨어 API가 달라 실패한다.
- 이 저장소는 공식 Doosan Jazzy 드라이버에 필요한 시리얼 서비스만 패치한다.
