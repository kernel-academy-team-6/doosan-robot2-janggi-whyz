# janggi_pick_place2

기존 `janggi_pick_place`를 변경하지 않고 사용할 수 있는 독립 ROS 2 패키지입니다.
**실기 그리퍼는 기존 `/dsr01/gripper/position_cmd` 토픽으로만 제어합니다.** 팔의 현재 위치, 이동 완료 및 TCP 이름 확인은 유지합니다.
`janggi_pick_place` Python 모듈이나 액션 타입에 의존하지 않습니다.

## 동작과 성공의 의미

- 별도 Home / Pick / Place / PickAndPlace 액션을 제공합니다. 시작 시 또는 픽 전에 자동 홈 이동하지 않습니다.
- 픽: 현재 플랜지 높이에서 XY/yaw 이동 → 물체 폭+여유만큼 개방 → 목표 팁 높이까지 하강 → 물체 폭-1mm로 닫기 명령 → 시작 높이 복귀.
- 플레이스: 현재 높이에서 XY/yaw 이동 → 목표 팁 높이까지 하강 → 물체 폭+5mm로 개방 → 시작 높이 복귀.
- 목표는 베이스 기준 `[x, y, 절대 팁 z, yaw]`, mm/degree입니다. 하강량을 입력하는 방식이 아닙니다.
- 플랜지 Z축은 아래를 향해야 합니다. 소프트웨어에서 `플랜지 목표 Z = 팁 z + 160mm`을 사용합니다. 제어기의 TCP 설정은 바꾸지 않습니다.
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

`tcp_length_at_6mm_mm`는 호환성을 위해 남은 이름이며, 현재 프로젝트에서는 고정 160mm만 허용합니다.
말 폭에 따른 TCP 길이 변화나 별도 그리퍼 TCP는 사용하지 않습니다. 컨트롤러는 기본 J6 플랜지 원점 TCP여야 합니다.

현재 기본 홈은 `[0.0, -13.694, 134.216, 0.0, 59.485, 90.0]°`입니다.
실물에서 확인해 사용자가 지정한 자세이며 보정 도구·대국 관찰 자세에도 같은 값을 저장했습니다.
UI의 별도 호밍 버튼도 이 패키지 YAML의 `home_joint_deg`와 Home 실행기를 사용합니다.

간격 W는 6~106mm이고, 개방/릴리스 여유를 더한 간격도 이 범위에 들어야 합니다. 측정식은 다음과 같습니다.

```text
u(W) = clip(round((degrees(acos((W-8.2)/114))-30.331197)/0.086670), 0, 700)
L_TCP = 160mm (J6 flange → tip, fixed; independent of W)
```

## 검증

단위 테스트는 파지 순서, 기하 계산, 기존 토픽을 통한 전송, 구독자 부재, 대기 중 취소 및 가상 상태 표시를 검사합니다. 소프트웨어 테스트 통과는 실제 파지나 전류 제한의 하드웨어 검증을 의미하지 않습니다.

카메라/판 보정과 시뮬레이터 기준 계산에는 원래 222mm를 계속 사용합니다. 이 실행기의 집기·놓기 목표에는 별도 160mm만 적용하며 합산하지 않습니다.

## J6 고정 비활성화 시 XY 이동 중 플랜지 축 고정

현재 기본은 `joint6_lock_enabled: false`인 고정 자세 MoveL이다. UI는 첫 집기 전 읽은 홈/진입 자세의 회전행렬을 유지하고, 다음 명시적 홈 이동 후에는 새 홈 자세를 다시 읽는다.

집기·놓기의 이동 목표 자세는 `[yaw, 180°, 0°]`(Doosan ZYZ)다.
따라서 J6 플랜지 로컬 +Z축은 베이스 −Z를 향하며, 베이스 Z축과 평행하다.

진입 자세가 목표 자세와 다르면 현재 플랜지 XYZ를 유지한 J6 단독 관절 회전을 먼저 보낸다.
드라이버에서 읽은 자세가 목표와 0.1° 이내인지 확인한 뒤 XY 이동을 시작한다.
XY 이동·Z 하강·Z 상승에는 같은 자세를 사용한다. XY 이동과 자세 정렬을 동시에 하지 않는다.
초기 축 기울기가 2°를 넘으면 기존처럼 픽/플레이스를 거부하며 자동 홈 이동하지 않는다.
J6만으로 목표 축 방향을 맞출 수 없는 경우(축 방향 차이 0.1° 초과)에도 자동 보정하지 않고 거부한다.
A/C 오일러 숫자가 달라도 같은 회전행렬이면 불필요한 정렬을 하지 않는다.

UI의 관찰 자세 이동도 이 실행기의 명시적 Home 경로를 사용한다. Home은 관절 이동이며,
집기·놓기의 고정 자세 Cartesian 구간과 구분된다.
Gazebo의 실제 토크 PID 추종 오차를 이 자세 명령만으로 없애는 것은 아니다.

## J6 회전 범위와 역방향 선택

현재 기본에서는 그리퍼의 베이스 기준 방향을 유지하며 J6는 팔 위치에 따라 변한다. 아래 등가 각도 선택/되감기를 적용한다.

실물 정지 로그에서 J6=362.501°가 global-normal 상한 360°를 초과한 알람 9008이 확인됐다.
PickPlace2의 실물·virtual 백엔드에 다음 작업 범위를 적용한다.

```yaml
joint6_min_deg: -270.0
joint6_max_deg: 270.0
joint6_limit_margin_deg: 5.0
joint6_path_step_mm: 10.0
joint6_rewind_clearance_mm: 50.0
```

- 정상 목표와 예측 경로는 여유 5°를 둔 **-265°~+265°** 안에서 계획한다.
  실제 피드백이 작업 한계 -270°~+270°를 벗어나면 MoveStop을 요청한다.
- 홈·관찰 자세와 제자리 J6 정렬은 `q6 + 360*k` 중 범위 안의 해를 선택한다.
  예: +300°는 -60°로 명령할 수 있다. 이동량이 같으면 중앙에 가까운 해를 우선한다.
  예: 현재 +90°에서 목표 +270° 방향이면 -90° 해를 선택한다.
- 집기 전 정렬에서는 접근·하강·상승 경로(복합 액션은 놓기 경로까지)를 미리 확인한다.
  현재 solution space를 유지한 IK를 최대 10mm 간격으로 조회하고 J6를 연속적으로 풀어서
  검사한다. IK 결과만 매번 ±360° 정규화해 경로 초과를 숨기지 않는다.
- 한 바퀴 풀기가 필요한 경우 빈 그리퍼의 집기 전 단계에서, 진입 높이가 목표 높이보다
  최소 50mm 높은 경우에만 J6 단독 회전을 허용한다. 다른 다섯 관절은 유지한다.
  이 높이 조건은 주변 장애물 전체의 충돌 검사를 대체하지 않는다.
- 말을 잡은 뒤 놓기 단계에서는 한 바퀴 풀기를 허용하지 않는다. 유효 경로가 없으면
  이동을 시작하지 않고 오류를 반환한다. 임의 Home이나 XYZ 우회 이동을 추가하지 않는다.
- 실제 XY·Z 이동은 기존 MoveL을 유지하고 각 구간 직전에도 IK 경로를 다시 검사한다.
- 시작 J6가 이미 작업 범위를 벗어나면 자동 복구하지 않는다. 기존 SAFE_OFF2를 해제하거나
  안전 제한을 바꾸지 않으며, 먼저 사용자가 정상적인 복구 절차로 범위 안으로 이동해야 한다.

명령·경로 선택은 `joint6_path_selection`과 `move_joint` 추적에 남긴다.
이 제한은 PickPlace2 실행 경로에 적용하는 소프트웨어 검사다. DART의 안전 설정은 변경하지 않으며,
직접 ROS 서비스 호출 등 다른 제어 클라이언트에는 적용되지 않는다.
10mm IK 샘플 검사와 주기적인 피드백 정지는 모든 중간 시점의 관절 범위를 보증하는 안전 기능이 아니다.
실물에서의 동작 검증은 별도로 필요하며, 이 변경 작업 중 실물 이동 명령은 실행하지 않았다.

## 선택 실험 모드: J6=90° 고정과 실측 말 프로필

아래는 `joint6_lock_enabled: true`로 명시적으로 켰을 때만 사용하는 과거 실험 모드다. 현재 기본은 false다.
전역 yaw를 90°로 쓰는 것과 다르다. Cartesian 위치를 따라 움직일 때 yaw는 자유롭게 바꾸고,
모든 관절 스플라인 경유점의 J6를 정확히 90°로 지정한다. 기존 집기 전 yaw=0 정렬은 하지 않는다.
향후 고정 해제/각도 변경을 위해 설정과 `locked_wrist.py` 플래너를 분리했다.

`LockedWristPlanner`는 현재 solution space에서 task yaw를 수치적으로 풀어 J6를 고정한다.
SDK IK와 FK로 각 경유점을 검증하고 J3/J5 ±155°, 나머지 관절 ±360°를 검사한다.
기본 5mm 간격이며 SDK 100점 제한에 걸릴 때는 최대 10% 간격 증가를 허용한다.
각 인접 관절점 사이의 1/4·1/2·3/4 위치도 FK로 검사하며, 선형 XYZ에서 0.25mm 이내인지 확인한다.
최대 100점의 `MoveSplineJoint` 절대·비동기 요청 한 번으로 각 XY/Z 구간을 실행한다.
관절 스플라인이므로 완전한 MoveL 직선 보장은 아니며, 실제 위치 피드백으로 경로 이탈 0.5mm,
J6 오차 3°, 플랜지 축 기울기 0.1°를 감시해 초과 시 MoveStop을 요청한다.
물리 추종 오차나 통신 지연 자체를 없애는 기능은 아니다.

시간 지정 movesj는 SDK에서 vel/acc를 무시하므로, 이동 시간은 직선 거리·직선 속도/가속도와
누적 툴 회전량·회전 속도/가속도, 전체 관절 이동량·관절 속도/가속도에서 보수적으로 계산한다.
UI의 속도·가속도 설정은 회전 값을 관절 제한에도 연결하며 호밍·관찰 복귀에도 적용한다.
대기 중에만 적용할 수 있고, 변경 시 이전 속도로 계산한 스플라인 캐시는 폐기한다.
실제 Cartesian 순간 속도가 `linear_velocity_mm_s`로 정확하게 제한됨을 보증하지 않는다.
현재 J6가 고정 각도에서 3° 넘게 벗어나면 자동 회전하지 않고 명시적 홈 이동을 요구한다.
명시적 홈 이동에서는 J6=90°로 이동할 수 있다. 설정 각도를 변경하면 홈 J6도 일치시킨다.
고정 모드에서는 UI의 `--pick-yaw`를 함께 지정하지 않는다.

| 그룹 | 최소 너비 | 두께 | 닫힘 | 개방·릴리스 | 판 위 집기 높이 | 팁 Z | 플랜지 Z |
|---|---:|---:|---:|---:|---:|---:|---:|
| 사·졸·병 | 21 | 6 | 20 | 30 | 2 | -18 | 142 |
| 차·마·상·포 | 27 | 7 | 26 | 36 | 2 | -18 | 142 |
| 궁 | 33 | 8 | 32 | 42 | 3 | -17 | 143 |

위 표는 기본 고정 판 Z=-20mm 기준이며 모두 mm다. 과거 보정 파일의 모서리 Z 차이는
명령 높이에 자동 가산하지 않는다. `board_z_compensation_enabled`는 기본 false다.
UI에서 명시적으로 수동 보정을 켠 경우에만 전체 및 TL/TR/BR/BL 오프셋을 보간해 더한다.
카메라/222mm 보정값은 그대로 유지하며 수동 보정과 합산하지 않는다.
판 밖 버리는 위치와 standalone 절대 TIP Z에는 이 보정을 자동 적용하지 않는다.

기본 닫힘 추가 이동(`grasp_overtravel_mm`)은 0, 개방·릴리스 여유는 10이다.
말 프로필에 이미 최소 너비보다 1mm 작은 닫힘 목표가 들어 있으므로 다시 1mm를 빼지 않는다.
`BoardPickPlace2`는 명령용 판 Z=-20에 그룹별 높이를 한 번 더한다.
원래 카메라 보정 XY·평면·222mm 기준은 수정하지 않고 명령 Z만 별도로 선택한다.
standalone 액션의 절대 tip target Z는 사용자가 지정한 값을 그대로 쓰며 프로필을 자동 가산하지 않는다.
SDK 그리퍼 폭→명령식과 고정 명령 연장 160mm는 유지한다.

오프라인 실제 E0509 URDF/독립 수치 IK 검증은
`docs/analysis/locked_wrist_validation/validate.py`와 `results.json`에 있다.
실제 로봇에 새 이동 명령을 보내 검증하지는 않았다.

## 현재 기본 속도 경로

기본 MoveL의 `time=0`이며 추가 스플라인 이동 시간 계산이나 안전 감속 계수를 적용하지 않는다.
`linear_velocity_mm_s=120`, `linear_acceleration_mm_s2=240`,
`angular_velocity_deg_s=60`, `angular_acceleration_deg_s2=120`을 UI에서 변경할 수 있다.
홈/관찰 복귀는 같은 회전 설정을 관절 속도·가속도에 사용한다.
UI 옵션 `--robot-vel 30`은 YAML보다 우선해 직선 속도 30, 가속도 60을 설정한다.
수평 이동과 Z 하강·상승 모두 동일한 MoveL 설정을 사용한다.
J6 연속 경로 사전 검사는 유지하되 실행 시작 상태가 일치하면 검사 결과를 재사용하여
같은 구간의 IK 서비스 호출을 중복하지 않는다. 시작 상태가 달라지면 다시 검사한다.
실기 그리퍼 명령 후 `gripper_command_wait_sec=1.0`, 가상 `virtual_gripper_delay_sec=0.4`는
팔 이동 속도가 아니라 개폐 완료를 기다리는 별도 설정이다. UI가 사용하는 동일 YAML에서 조절한다.
드라이버 MoveLine 콜백은 전달받은 vel/acc/time을 그대로 DRFL에 넘긴다.
컨트롤러 자체 운전 속도 비율·안전 모드·물리 관절 한계는 이 프로그램의 추가 감속과 별개다.

## UI의 시각 기반 집기 및 그리퍼 ACK

실시간 UI는 `robot_game/manipulation.py`에서 주변 말 여유에 따른 yaw 선택,
각 말 집기 직전/수평 접근 뒤의 위치 재인식, 착수 후 최대 1회 재시도를 조합한다.
절대 좌표 standalone 액션에는 카메라가 없으므로 이 세 기능을 자동 적용하지 않는다.
YAML의 `adaptive_grasp_enabled`, `refresh_before_pick`, `retry_move_enabled`가 UI 초기값이며
UI 체크박스 또는 `--no-adaptive-grasp`, `--no-refresh-before-pick`, `--no-auto-retry`로 끌 수 있다.
하강 전 재인식 콜백이 새 tip target을 반환하면 같은 높이에서 XY/자세를 다시 맞추고 내려간다.

실기 기본 그리퍼 전송은 요청 ID가 있는 `/gripper/position_request`이다.
새 C++ gripper_service_node의 `/gripper/command_result`에서 같은 ID와 위치의
성공 응답을 받은 후 다음 팔 단계로 진행한다. 이 결과는 명령 처리 완료이며 파지 검증은 아니다.
기본 `gripper_wait_for_result=true`, 응답 시간 한도는 `gripper_result_timeout_sec=20`이다.
false로 바꾸면 기존 position_cmd/고정 대기 방식이다. 가상 backend는 stroke를 유지한다.
C++ 그리퍼 브리지를 빌드·재실행해야 새 프로토콜을 사용할 수 있다.

`retry_place`는 완료된 직전 플레이스의 폭·팁 높이·목표 기록을 이용하여
새 집기/닫힘 없이 플레이스만 한 번 더 실행하기 위한 UI API다.
영상에서 미놓임으로 추정했을 뿐 실제 파지로 확인한 상태로 기록하지 않는다.
기존 캘리브레이션, 판 Z=-20, 말 크기별 팁 높이와 160mm 플랜지 연장은 유지한다.
