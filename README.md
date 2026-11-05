# doosan-robot2-janggi

Ubuntu 24.04 + ROS 2 Jazzy에서 Doosan E0509와 ROBOTIS RH-P12-RN-A 그리퍼를 재현 가능하게 설치하고 실행하기 위한 환경 저장소다.

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

실행 및 장애 대응은 [docs/RUNBOOK.md](docs/RUNBOOK.md), 통신 계층은 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)를 참고한다.
