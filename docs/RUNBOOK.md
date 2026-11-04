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

## 알려진 호환성 판단

- `fhekwn549/e0509_gripper_description`만 공식 Jazzy 드라이버 위에 빌드하면 `FlangeSerialOpen` 헤더가 없어 실패한다.
- `fhekwn549/doosan-robot2` Humble 포크 전체를 Jazzy에 빌드하면 Jazzy DRFL 바이너리가 없고 `ros2_control` 하드웨어 API가 달라 실패한다.
- 이 저장소는 공식 Doosan Jazzy 드라이버에 필요한 시리얼 서비스만 패치한다.
