> 현재 프로젝트 TCP 기준: 컨트롤러=J6 플랜지 원점, 소프트웨어 연장=132.6717mm + H(c).
> 말 폭에 따른 길이 보정을 제거했으므로 이 실험의 개도 변화에 따른 Z 보정량은 0이다.
> 독립 실행 설치본은 `janggi_pick_place2`와 이 패키지를 다시 빌드한 뒤 사용한다.

# janggi_tcp_test — 명령 기반 개폐 / Z 보정 시험

초기 간격과 목표 간격 두 값을 받아 그리퍼를 움직이면서, 예상 개폐 프로파일에 맞춰 플랜지 Z를 보정하는 ROS 2 Jazzy 시험 액션입니다. 홈·XY 이동·자세 변경은 하지 않습니다.

**실기 그리퍼는 동작을 확인한 기존 `/dsr01/gripper/position_cmd` 토픽만 사용합니다.** 기존 `gripper_service_node`가 시리얼 포트와 토크·위치 전송을 담당합니다. 시험 노드는 시리얼 서비스를 호출하지 않고 그리퍼의 개도·전류를 읽지 않습니다. 기존 `janggi_pick_place`와 `janggi_pick_place2` 파일은 변경하지 않습니다. 기구학과 팔 클라이언트는 `janggi_pick_place`에서 읽기 전용으로 재사용합니다.

## 무엇을 확인하는 시험인가

이 액션은 **예상 그리퍼 개도에 맞춰 Z를 움직이는 시험**입니다. 실제 개도에 따라 Z를 수정하는 제어는 아닙니다.

`position_cmd`는 위치만 전달하므로 **속도·가속도·전류 설정을 보내지 않습니다.** `gripper_profile_velocity`, `gripper_profile_acceleration`은 Z 계산용 모델 값일 뿐입니다. 이 값을 낮춰도 실제 그리퍼가 느려지지 않습니다. 실제 그리퍼 속도는 기존 설정을 사용합니다.

실제 개폐가 모델보다 빠르거나 늦으면 팁 높이가 변할 수 있습니다. 팔 위치와 모델 개도를 합성한 보고서의 오차가 작아도 실제 팁 높이가 일정하다고 입증되지는 않습니다. 빈 그리퍼를 충분히 띄우고, 눈금이나 카메라로 실제 움직임을 함께 관찰하며 모델 시간과 지연을 맞추는 용도로 사용합니다.

## 입력과 순서

액션 `/janggi_tcp_test/test`, 타입 `janggi_tcp_test/action/TestTcpHeight`.

| 입력 | 의미 |
|---|---|
| `initial_width_mm` | 초기 총 손가락 간격, mm |
| `target_width_mm` | 최종 총 손가락 간격, mm. 변화량이 아님 |

둘 다 6~106mm이며 필수입니다. 같은 그리퍼 입력값으로 변환되는 두 간격은 거절합니다. `40 → 60`은 벌리기, `60 → 40`은 닫기입니다.

1. 로봇 시스템 모드, 플랜지 TCP 이름, 아래 방향 자세(2도 이내), Z 경로 한계를 확인합니다.
2. 팔은 고정한 채 초기 간격을 토픽으로 전송하고 `gripper_command_wait_sec`만큼 기다립니다. 초기 간격의 실제 도착 여부는 확인하지 않습니다. 이 구간은 높이 유지 시험에 포함되지 않습니다.
3. 명령한 초기 간격과 현재 팔 자세로 기준 팁 높이를 계산합니다.
4. 목표 간격을 토픽으로 한 번 전송하고, 발행 시각을 기준으로 예상 `u(t) → W(t) → L_TCP(t)`를 계산합니다.
5. 기본 20Hz로 `/dsr01/servol_stream`에 Z 목표를 보냅니다. X/Y/A/B/C는 시작 자세를 유지합니다.
6. 별도 작업에서 **팔 자세만** 조회해 모델 간격과 합성한 추정 팁 높이를 기록합니다.
7. 모델 시간이 끝나고 팔의 최종 Z가 확인되면 종료합니다. 그리퍼 도착·정지 확인은 아닙니다. 초기 간격·높이로 자동 복귀하지 않습니다.

수직 하향 자세에서 사용하는 식은 다음과 같습니다. 실제 시작 자세의 미세한 기울기는 `cos(B)`로 반영합니다.

```text
기준 팁 높이 = 시작 플랜지 Z − L_TCP(초기 명령 간격)
플랜지 Z(t) = 기준 팁 높이 + L_TCP(예상 W(t))
```

제어기의 TCP 설정은 변경하지 않습니다. 제어점은 플랜지 원점이어야 합니다.

## 빌드

기존 `janggi_pick_place`가 설치된 워크스페이스에서:

```bash
cd ~/doosan_janggi_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select janggi_tcp_test
source install/setup.bash
```

## 실기 실행

기존 bringup을 실행하고 초기 그리퍼 개방이 끝날 때까지 기다립니다. 기존 `gripper_service_node`는 계속 켜둡니다.

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=ROBOT_IP port:=12345 name:=dsr01
```

팔을 아래 방향의 빈 공간에 준비한 뒤 다른 팔/그리퍼 액션을 종료하고, 별도 터미널에서 시험 노드를 실행합니다.

```bash
cd ~/doosan_janggi_ws
source install/setup.bash
ros2 launch janggi_tcp_test tcp_test.launch.py backend:=real \
  params_file:="$PWD/src/janggi_tcp_test/config/tcp_test.yaml"
```

다른 터미널에서:

```bash
source ~/doosan_janggi_ws/install/setup.bash
ros2 action send_goal /janggi_tcp_test/test \
  janggi_tcp_test/action/TestTcpHeight \
  '{initial_width_mm: 40.0, target_width_mm: 60.0}' --feedback
```

반대 방향은 두 간격을 바꿔 입력합니다. 동시에 다른 프로그램에서 팔 이동·그리퍼 토픽/서비스를 호출하지 마세요. 두 픽앤플레이스 서버의 상태 서비스가 있으면 busy/파지 상태를 검사하지만, 다른 프로그램의 이후 명령까지 막는 잠금은 아닙니다.

## 주요 파라미터

이 패키지의 `config/tcp_test.yaml`을 편집한 뒤 위처럼 `params_file`로 지정해 노드를 재시작하면 재빌드 없이 적용됩니다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `gripper_command_wait_sec` | 5.0 | 초기 간격 명령 후 기다리는 시간. 목표 전환과 Z 스트림 사이에 넣는 대기가 아님 |
| `gripper_start_delay_sec` | 0.3 | 토픽 발행부터 실제 개폐 시작까지 예상 지연. 기존 노드의 준비 대기를 반영한 초기 추정값이며 실측 조정 필요 |
| `gripper_profile_velocity` | 50 | 계산 모델의 속도 값. **실제 그리퍼에 전송하지 않음** |
| `gripper_profile_acceleration` | 100 | 계산 모델의 가속도 값. **실제 그리퍼에 전송하지 않음** |
| `position_unit_deg` | 0.088 | 모델의 속도 단위를 위치 입력 u로 환산하는 가정 |
| `tcp_length_at_6mm_mm` | 132.6717 | 고정 기준값. 계산 시 명령별 H(c)를 더함, mm |
| `stream_rate_hz` | 20.0 | Z 목표 발행 주기. 하드 실시간 보증은 아님 |
| `arm_lookahead_sec` | 0.1 | 팔 추종 지연을 고려해 모델을 미리 보는 시간 |
| `servo_time_sec` | 0.1 | 각 servol 목표의 도달 시간 |
| `arm_velocity_mm_s` / `arm_acceleration_mm_s2` | 5 / 10 | 팔의 Z 속도·가속도 제한 |
| `max_z_displacement_mm` | 30.0 | 한 시험의 Z 변화 상한 |
| `feedback_interval_sec` | 0.5 | 팔 자세 조회 사이 대기 |
| `max_feedback_age_sec` | 3.0 | 팔 자세가 오래 갱신되지 않으면 중단 |
| `max_estimated_tip_error_mm` | 3.0 | 모델 개도+팔 자세의 추정 오차 한계. 실제 팁 오차 한계가 아님 |

초기 속도 0인 사다리꼴/삼각형 프로파일을 사용합니다. 모델 단위는 velocity raw ×0.01rev/min, acceleration raw rev/min²로 가정합니다. 모터 원축 회전속도와 핑거 링크 회전속도가 같다는 의미는 아닙니다. 측정 기구학 계수 0.086670과 프로파일 단위 가정 0.088은 별개입니다.

L만 실행 시 변경하려면:

```bash
ros2 launch janggi_tcp_test tcp_test.launch.py backend:=real \
  tcp_length_at_6mm_mm:=132.6717
```

## 가상 실행

기존 virtual bringup과 함께:

```bash
ros2 launch janggi_tcp_test tcp_test.launch.py backend:=virtual
```

팔은 에뮬레이터, 그리퍼는 기존 stroke 애니메이션을 사용합니다. RViz 핑거 각도 변환은 실측 보정식과 다르므로 메시 팁 높이를 정량적인 검증으로 사용하지 않습니다.

로봇 없이 모델과 보고서만 보려면:

```bash
ros2 launch janggi_tcp_test tcp_test.launch.py backend:=mock
```

액션 입력은 동일합니다. mock은 실제 팔·그리퍼 명령을 보내지 않습니다.

## 보고서와 취소

액션 결과의 `report_directory`에 `samples.csv`, `summary.json`, `report.html`을 저장합니다. 기본 경로는 `/tmp/janggi_tcp_test/run_.../`입니다.

- real: `gripper_model_and_arm_pose` — **예상 간격 + 실제 팔 자세**.
- virtual: `virtual_gripper_model_and_arm_pose` — 모의 간격 + 에뮬레이터 팔 자세.
- mock: `mock_model_only` — 모델만 계산. 오차 0은 실제 높이 검증이 아님.

모든 보고서에 `gripper_feedback_received: false`, `profile_settings_sent: false`를 기록합니다. `feedback_samples`는 팔/모델 표본 수이며 그리퍼 실측 표본 수가 아닙니다. `success=True`는 명령/모델 시간과 팔 위치 확인 순서의 완료를 의미합니다.

```bash
ros2 topic echo /janggi_tcp_test/profile
ros2 service call /janggi_tcp_test/status std_srvs/srv/Trigger '{}'
```

취소 또는 실패 시 팔 정지를 요청합니다. **이미 보낸 그리퍼 목표는 계속 진행될 수 있습니다.** 실측 위치가 없으므로 그리퍼에 현재 위치 유지 명령이나 자동 개방을 보내지 않습니다. 실패/취소 보고서는 가능한 범위에서 저장합니다.

`scripts/gripper_read_probe.py`, `scripts/gripper_read_probe_legacy.drl`은 이전 수신 문제를 조사한 별도 진단 도구로 남아 있습니다. 시험 액션이 이 도구들을 실행하거나 호출하지는 않습니다.

## 소프트웨어 검증

```bash
colcon test --packages-select janggi_tcp_test
```

모델의 끝점·연속성·방향과 Z 한계, 기존 토픽을 통한 단일 명령, 초기 대기·취소·구독자 부재, 보고서의 예측/실측 구분을 검사합니다. 실기에서 실제 팁 높이가 유지되는지는 별도 관찰이 필요합니다.
