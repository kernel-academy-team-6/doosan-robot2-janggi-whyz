# doosan-robot2-janggi

Ubuntu 24.04 + ROS 2 Jazzy에서 Doosan E0509와 ROBOTIS RH-P12-RN-A 그리퍼를 재현 가능하게 설치하고 실행하기 위한 환경 저장소다.

발표자를 위한 [워크스페이스 기술 설명서 (HTML)](docs/WORKSPACE_TECHNICAL_GUIDE.html): 전체 동작, 비전·추적·좌표 보정·로봇 제어, 포함된 ROS 패키지 37개와 주요 도구의 용도를 그림 10개로 정리했다. 파일을 브라우저에서 열면 인터넷 없이 읽을 수 있고 인쇄·PDF 저장도 지원한다.

## 현재 상태

- 공식 Doosan Jazzy 드라이버 사용
- DRCF 3.0 에뮬레이터 사용
- `fhekwn549/e0509_gripper_description` 통합
- Tool Flange Serial 기반 RH-P12 Modbus RTU 제어
- upstream 저장소를 `src/` 아래 git submodule로 커밋 SHA 고정

## 빠른 시작

```bash
git clone --recursive https://github.com/kernel-academy-team-6/doosan-robot2-janggi-whyz.git ~/doosan_janggi_ws
cd ~/doosan_janggi_ws
./scripts/install_host_ubuntu_24_04.sh
./scripts/bootstrap_workspace.sh
source /opt/ros/jazzy/setup.bash
source ~/doosan_janggi_ws/install/setup.bash
./scripts/verify_workspace.sh ~/doosan_janggi_ws
```

이 저장소 자체가 colcon 워크스페이스다. `bootstrap_workspace.sh`는 submodule 초기화, `patches/doosan-jazzy-flange-serial.patch` 적용(이미 적용된 경우 건너뜀), rosdep, colcon build를 수행한다.

설치 스크립트를 일반 사용자로 실행하고, 요청될 때 sudo 암호를 입력한다. 실행 전에 내용을 확인한다. Docker 그룹 가입은 사실상 root 수준 권한을 주므로 자동으로 수행하지 않는다.

## 실행

가상 E0509 + RViz:

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=virtual host:=127.0.0.1 port:=12345 name:=dsr01
```

실제 E0509 + RViz:

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=ROBOT_IP port:=12345 name:=dsr01
```

팔 이동:

```bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"
```

그리퍼:

```bash
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

## 장기 대국 실행 (사람 ↔ 로봇)

현재 기본 비전 모델은 `models/weights.pt`이며 말 위치만 검출한다. 이 모델에서는 게임·로봇 대국을 비활성화한다. 아래 대국 예시는 기존 14클래스 모델을 명시해서 사용한다.

처음 한 번만:

```bash
cd ~/doosan_janggi_ws
./janggi/setup_venv.sh              # 인식용 venv
./src/janggi/engine/get_engine.sh   # Fairy-Stockfish 엔진
```

보정(`src/janggi/calibration/config/calibration.yaml`)이 끝난 상태여야 한다. 시뮬레이터(`sim_gazebo.launch.py`, `mode:=virtual`, 에뮬레이터 컨테이너)는 **켜지 않는다**. 같은 `dsr01` 이름을 쓰므로 명령이 에뮬레이터로 갈 수 있다.

터미널 1 — 실제 로봇 드라이버:

```bash
source /opt/ros/jazzy/setup.bash
source ~/doosan_janggi_ws/install/setup.bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.50 port:=12345 name:=dsr01
```

로그에 `Serial port opened`가 나오면 그리퍼까지 준비된 것이다.

터미널 2 — 게임 UI (ROS 환경은 `run.sh`가 불러온다):

```bash
cd ~/doosan_janggi_ws/janggi
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --robot-vel 30    # 사람 초(먼저 둠), 로봇 한. 처음에는 천천히
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --human-side red  # 사람 한
./realtime_ui/run.sh --model ../models/weights_14class.pt --robot --no-motion       # 로봇은 움직이지 않고 엔진 수만 표시
```

진행: 오른쪽 **로봇 대국** 패널에서 `대국 시작 (인식된 판)` → 사람이 두고 `착수 완료 (Space)` → 로봇이 두고 홈 자세로 복귀 → 반복. `로봇 정지`는 즉시 정지.
인식된 판을 검증 없이 그대로 쓰므로 로봇 수가 이상하면 화면의 인식 결과를 먼저 확인한다. 자세한 내용은 [src/janggi/robot_game/README.md](src/janggi/robot_game/README.md).

종료: 터미널 2에서 창을 닫고, 터미널 1에서 Ctrl+C.

장기 보정·이동 도구 실행 순서는 [docs/RUNBOOK.md 9장](docs/RUNBOOK.md#9-장기-보정이동-도구), 실행 및 장애 대응은 [docs/RUNBOOK.md](docs/RUNBOOK.md), 통신 계층은 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), 장기 대국 애플리케이션 설계는 [docs/JANGGI_DESIGN.md](docs/JANGGI_DESIGN.md)를 참고한다.
