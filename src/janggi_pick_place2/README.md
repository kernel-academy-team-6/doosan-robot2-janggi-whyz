# janggi_pick_place2

기존 `janggi_pick_place`를 변경하지 않고 사용할 수 있는 독립 ROS 2 패키지입니다.
**실기 그리퍼는 기존 `/dsr01/gripper/position_cmd` 토픽으로만 제어합니다.** 팔의 현재 위치, 이동 완료 및 TCP 이름 확인은 유지합니다.
`janggi_pick_place` Python 모듈이나 액션 타입에 의존하지 않습니다.

## 동작과 성공의 의미

- 별도 Home / Pick / Place / PickAndPlace 액션을 제공합니다. 시작 시 또는 픽 전에 자동 홈 이동하지 않습니다.
- 픽: 현재 플랜지 높이에서 XY/yaw 이동 → 물체 폭+여유만큼 개방 → 목표 팁 높이까지 하강 → 물체 폭-1mm로 닫기 명령 → 시작 높이 복귀.
- 플레이스: 현재 높이에서 XY/yaw 이동 → 목표 팁 높이까지 하강 → 물체 폭+5mm로 개방 → 시작 높이 복귀.
- 목표는 베이스 기준 `[x, y, 절대 팁 z, yaw]`, mm/degree입니다. 하강량을 입력하는 방식이 아닙니다.
- 플랜지 Z축은 아래를 향해야 합니다. 소프트웨어에서 `플랜지 목표 Z = 팁 z + L_TCP(물체 폭)`을 사용합니다. 제어기의 TCP 설정은 바꾸지 않습니다.
- 팔과 그리퍼를 동시에 보정하는 기능은 포함하지 않습니다.

실기에서 사용하는 명령은 사용자가 동작을 확인한 아래 명령과 같습니다. 값만 물체 폭에 맞게 계산합니다.

```bash
ros2 topic pub --once /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 700}"
```

새 패키지는 시리얼 서비스를 직접 호출하거나 전류·프로파일·토크 레지스터를 쓰지 않습니다. 기존 `gripper_service_node`가 포트 열기·토크 활성화·위치 전송·포트 닫기와 stroke 발행을 그대로 담당합니다. 토픽 구독자가 없으면 팔을 움직이기 전에 오류를 반환합니다.

**이 토픽은 위치만 전달할 수 있습니다.** 따라서 기존 그리퍼의 전류·속도 설정을 그대로 사용합니다. 액션의 `grip_current_ma`와 CLI `--current`는 호출 호환성을 위해 남겨둔 입력이며 실제 그리퍼에는 전달하지 않습니다. 상태에 `current_limit_command_sent: false`로 표시하고, 실기 완료 메시지에도 이를 명시합니다. 전류 제한을 새로 설정하는 기능은 이 패키지에 포함하지 않습니다.

`success=True`는 팔 이동 확인과 그리퍼 토픽 발행 및 대기 순서가 끝났다는 의미입니다. 실제 도착, 파지, 릴리스 또는 전류 제한 적용을 확인한 결과가 아닙니다.

- `empty_unverified`: 시작 시 빈 그리퍼로 가정하거나 릴리스 명령/대기를 완료한 상태.
- `holding_unverified`: 닫기 명령/대기를 완료한 상태. 실제 물체 파지는 확인하지 않음.
- `unknown`: 파지/릴리스 도중 실패 또는 취소 등으로 재시도 전에 수동 확인이 필요한 상태.

Place는 **같은 서버 세션에서 완료한 Pick 명령**의 물체 폭을 사용합니다. 전류 입력은 기록만 유지하며 전송하지 않습니다. 외부 명령으로 그리퍼를 움직여도 이 상태는 자동 갱신되지 않습니다. 서버를 재시작할 때는 그리퍼가 비어 있는지 확인해야 합니다.

## 빌드

```bash
cd ~/doosan_janggi_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select janggi_pick_place2
source install/setup.bash
```

## 실기 실행

기존 bringup을 그대로 실행합니다. bringup은 시작 시 기존 동작에 따라 그리퍼를 엽니다.

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=ROBOT_IP port:=12345 name:=dsr01
```

초기 그리퍼 개방이 완료된 뒤, 다른 터미널에서:

```bash
cd ~/doosan_janggi_ws
source install/setup.bash
ros2 launch janggi_pick_place2 pick_place.launch.py backend:=real \
  params_file:="$PWD/src/janggi_pick_place2/config/pick_place.yaml"
```

로봇은 원점 TCP를 선택해야 합니다. 그리퍼는 기존 `position_cmd` 명령이 실물에서 정상 동작하는 설정을 사용합니다. 새 패키지는 모델·모드·전류·속도 설정을 조회하거나 변경하지 않습니다.

`janggi_pick_place`, `janggi_tcp_test` 또는 다른 팔 제어기를 동시에 실행/명령하지 마세요. 새 액션 실행 중에는 기존 `/dsr01/gripper/open`, `/close`, `/position_cmd`와 시리얼 서비스를 다른 프로그램에서 호출하지 마세요. 액션 이름은 분리되어도 물리적인 팔과 시리얼 포트는 같습니다. 기존 bringup의 `gripper_service_node`는 계속 켜두어야 합니다.

## 명령 예제

새 터미널에서 `source install/setup.bash` 후 실행합니다. 좌표는 실제 작업 공간에 맞게 입력하세요.

```bash
# 필요할 때만 별도 홈 이동. 그리퍼 명령은 보내지 않음.
ros2 run janggi_pick_place2 pick_place_example --operation home

# 픽만 실행
ros2 run janggi_pick_place2 pick_place_example --operation pick \
  --pick-target 338.22 -168.66 10 0 --width 30 --allowance 10

# 위 픽을 실행한 동일 서버에서 플레이스만 실행
ros2 run janggi_pick_place2 pick_place_example --operation place \
  --place-target 438.22 -168.66 10 0

# 픽앤플레이스 한 번에 실행 (빈 그리퍼 상태에서)
ros2 run janggi_pick_place2 pick_place_example \
  --pick-target 338.22 -168.66 10 0 \
  --place-target 438.22 -168.66 10 0 \
  --width 30 --allowance 10
```

`--width`, `--allowance`를 생략하거나 -1로 입력하면 YAML 기본값을 사용합니다. 기본 물체 폭 30mm, 추가 개방 10mm(좌우 각 5mm)입니다. `--current`를 입력해도 위치 토픽에는 전달되지 않습니다.

ROS 액션 CLI로도 실행할 수 있습니다.

```bash
ros2 action send_goal /janggi_pick_place2/home \
  janggi_pick_place2/action/Home '{}' --feedback

ros2 action send_goal /janggi_pick_place2/pick \
  janggi_pick_place2/action/Pick \
  '{target: [338.22, -168.66, 10.0, 0.0], object_width_mm: 30.0, opening_allowance_mm: 10.0}' --feedback

ros2 action send_goal /janggi_pick_place2/place \
  janggi_pick_place2/action/Place \
  '{target: [438.22, -168.66, 10.0, 0.0]}' --feedback

ros2 service call /janggi_pick_place2/status std_srvs/srv/Trigger '{}'
```

## 가상 실행

기존 `mode:=virtual host:=127.0.0.1` bringup과 함께:

```bash
ros2 launch janggi_pick_place2 pick_place.launch.py backend:=virtual
```

그리퍼는 명령 간격으로 애니메이션만 표시합니다. 기존 RViz 핑거 모델의 각도 변환은 측정식과 다르므로 화면상의 팁 높이를 정량적인 검증으로 사용하지 마세요.

로봇 드라이버 없이 순서만 시험하려면:

```bash
ros2 launch janggi_pick_place2 pick_place.launch.py backend:=mock
```

## 설정 변경

`config/pick_place.yaml`은 시작 시 읽습니다. 파일을 수정한 뒤 위처럼 `params_file`로 지정해 서버를 재시작하면 재빌드 없이 적용됩니다. 설치본 기본 설정을 사용하려면 패키지를 다시 빌드하세요.

```yaml
tcp_length_at_6mm_mm: 160.0
gripper_command_wait_sec: 5.0
```

- `gripper_command_wait_sec`: 각 개폐 토픽을 발행한 뒤 기다리는 시간. 기존 노드가 명령을 처리하는 시간과 실제 개폐 시간을 모두 포함해야 합니다. 도착 여부를 읽지 않으므로 대기 시간이 너무 짧으면 개폐 완료 전에 팔 이동이 진행될 수 있습니다.
- 그리퍼 속도·전류는 기존 설정을 사용합니다. `grip_current_ma` 입력은 적용되지 않으며 실제 파지를 확인하지 않습니다.
- 취소/실패 시 팔에 정지 요청을 보내며, 그리퍼는 자동으로 열거나 토크를 끄지 않습니다. 이미 발행된 그리퍼 명령은 계속 진행될 수 있습니다.

L만 실행 시 바꾸려면:

```bash
ros2 launch janggi_pick_place2 pick_place.launch.py backend:=real \
  tcp_length_at_6mm_mm:=170.0
```

기본 홈 관절은 패키지 생성 당시 기존 YAML 값을 복사했습니다. 새 패키지의 `home_joint_deg`를 독립적으로 수정할 수 있습니다.

간격 W는 6~106mm이고, 개방/릴리스 여유를 더한 간격도 이 범위에 들어야 합니다. 측정식은 다음과 같습니다.

```text
u(W) = clip(round((degrees(acos((W-8.2)/114))-30.331197)/0.086670), 0, 700)
L_TCP(W) = L - sqrt(3249-1.21) + sqrt(3249-(W/2-4.1)^2)
```

## 검증

단위 테스트는 파지 순서, 기하 계산, 기존 토픽을 통한 전송, 구독자 부재, 대기 중 취소 및 가상 상태 표시를 검사합니다. 소프트웨어 테스트 통과는 실제 파지나 전류 제한의 하드웨어 검증을 의미하지 않습니다.
