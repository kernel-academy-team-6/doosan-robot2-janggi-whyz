# ROS 초기화 실패 시 복구 모드 요청

`doosan_recovery.sh`는 저장소에 포함된 두산 DRFL SDK를 직접 사용한다.
ROS source나 controller 서비스가 필요하지 않다. 스크립트는 실행 시 C++ 도구를 빌드한다.

## 발견된 초기화 실패

`~/.ros/log/2026-11-12-17-27-57-784205-cse-pc-16012/launch.log`에서 다음을 확인했다.

1. `Received robot description from topic.` — URDF 수신 성공.
2. `Connected to DRCF`, `INITIAL AUTHORITY GRANTED` — 실물 컨트롤러 연결 성공.
3. `ROBOT_STATE : STATE_SAFE_OFF2` — 안전 복구가 필요한 상태.
4. 하드웨어 `on_init`에서 Servo On을 반복 요청했으나 STANDBY를 유지하지 못함.
5. `INITIAL STATE CALL FAILURE`, `Failed to initialize hardware 'e0509'`.
6. 초기화 실패 후 모델 대기 경고 반복, 두산 ROS 서비스는 생성되지 않음.

따라서 이 기록에서 원인은 URDF 경로나 모델 토픽 이름이 아니라 안전 상태로 인한
하드웨어 초기화 실패다. CLI의 별도 5초 echo 타임아웃 원인은 이 로그만으로 확정하지 않는다.

## 사용

복구 전용 런치도 제공한다. 기존 bringup 및 자동 대국을 먼저 종료하고 실행한다.
이 런치는 ros2_control, RViz, 일반 로봇 제어기 또는 그리퍼 제어기를 시작하지 않는다.

```bash
cd ~/doosan_janggi_ws
source /opt/ros/jazzy/setup.bash
ros2 launch tools/doosan_recovery.launch.py host:=110.120.1.50 enter_recovery:=true
```

`enter_recovery`를 생략하면 상태만 조회한다. 이 파일은 경로로 직접 실행하므로
colcon 빌드 및 workspace의 install/setup.bash는 필요 없다.
복구 진입 요청 뒤 결과를 확인하고 연결을 닫으며, 수동 이동 서버를 유지하는 런치는 아니다.

자동 대국/UI와 기존 bringup을 먼저 정상 종료한다. 기본값은 일반 권한 요청이다.
`Control access not granted`는 10초 내 권한 승인을 받지 못했다는 뜻이며,
이 로그만으로 기존 권한 보유자가 누구인지 확정하지 않는다.
다른 제어 클라이언트가 동작 중이지 않은 상태에서 명시적인 제어권 강제 이전을
선택하려면 다음 옵션을 사용한다. 이는 두산 SDK의 `MANAGE_ACCESS_CONTROL_FORCE_REQUEST`이며
안전 제한 해제·Servo On·이동 요청과는 별개다. 기본 요청이 실패해도 자동으로 강제 이전하지 않는다.

```bash
ros2 launch tools/doosan_recovery.launch.py \
  host:=110.120.1.50 enter_recovery:=true force_access:=true
# 직접 실행 시: bash tools/doosan_recovery.sh --host 110.120.1.50 --enter-recovery --force-access
```

강제 이전 요청도 거부되거나 상태 8이 확인되지 않으면 종료하며, 반복적인 권한 탈취·자동 재시도는 하지 않는다.

### 복구 안전 모드의 ENTER 경로

권한 GRANT 후에도 상태 10과 안전 상태 2(VIOLATION_STO)가 유지되는 경우,
`set_robot_control` 경로로 복구 진입이 완료되지 않은 것이다.
두산 저장소의 제조사 답변에는 별도 진입 경로인
`set_safety_mode(RECOVERY, ENTER)`가 안내돼 있다:
https://github.com/DoosanRobotics/doosan-robot2/issues/162
(해당 답변의 전체 이동 절차를 이 도구에서 실행하지는 않는다.)

```bash
ros2 launch tools/doosan_recovery.launch.py \
  host:=110.120.1.50 enter_recovery:=true force_access:=true \
  entry_method:=safety-enter
```

이 옵션은 SAFE_STOP2/SAFE_OFF2 및 MANUAL 모드에서만 요청한다.
`EVENT_ENTER`만 보내며 `EVENT_MOVE`, Servo On, 관절 이동은 보내지 않는다.
기존 control 요청에 자동으로 이어서 실행하는 fallback이 아니라 명시적으로 선택하는 경로다.
복구 모드에서는 충돌 감지 등 일부 안전 기능이 제한될 수 있으므로 현장 확인이 필요하다.
API 반환값과 별도로 robot_state 및 safety_state로 결과를 확인하며,
실물 V2.10.3에서 이 경로가 성공하는지는 아직 검증되지 않았다.

```bash
cd ~/doosan_janggi_ws
# 상태 조회만
bash tools/doosan_recovery.sh --host 110.120.1.50

# 안전 복구 진입 요청
bash tools/doosan_recovery.sh --host 110.120.1.50 --enter-recovery
```

- 상태 9(SAFE_STOP2): `CONTROL_RECOVERY_SAFE_STOP` 요청.
- 상태 10(SAFE_OFF2): `CONTROL_RECOVERY_SAFE_OFF` 요청.
- 상태 8(RECOVERY): 추가 전환 없음.
- 그 밖의 상태: 전환 거부.
- 전환 전 권한·상태 재확인, 요청 후 상태 8 확인, 최대 10초 대기.
- 연결 후 컨트롤러/SDK 버전, 로봇 모드 및 마지막 알람을 출력한다.
  `enter_recovery` 생략 시 이 진단 정보만 읽으며 제어권이나 모드 변경을 요청하지 않는다.
- 권한을 얻었는데 상태 10이 유지되는 경우, 추가 모드 전환을 자동 시도하지 않는다.
  출력된 알람의 group/index/param 및 컨트롤러 버전으로 원인을 확인한다.
- 별도의 안전 상태 콜백(`safety_state`)도 등록한다. `7=SAFETY_STATE_RV_SOS`는
  복구 안전 정지 상태이며, 일반 `robot_state=8`과 서로 다른 열거형이다.
  `robot_state=10`만으로 복구 안전 상태 진입 여부를 단정하지 않는다.
  콜백이 없으면 `-1=not received`로 표시하며 성공으로 간주하지 않는다.
- `robot_state=8`이면 RECOVERY 확인, `safety_state=7`이면 복구 안전 정지 관측으로
  구분해 출력한다. 후자를 움직일 수 있는 상태라고 주장하지 않는다.
- 사용자가 읽은 컨트롤러는 `GF02100300`(E0509), SDK는 `GL013304`, 모드는 0(MANUAL),
  상태는 10이었다. 알람 필드 0은 현재 연결에서 유효한 알람 정보가 없음을 의미하며,
  위험 원인이 해소됐다는 증거가 아니다. 도구 빌드의 `DRCF_VERSION=2`를 기존 하드웨어
  드라이버 기본값과 맞췄다. 이 매크로 변경이 이전 실패의 원인이라는 근거는 없다.
- Servo On, 정상 상태 리셋, 자동 프로그램 재개, 팔 이동, Backdrive 요청은 구현하지 않았다.
- 복구 모드 진입만 수행하는 도구다. 수동 Jog/이동 제어 기능은 포함하지 않는다.
- 종료 시 SDK 연결을 닫는다. 이후 다른 클라이언트 연결 시 해당 클라이언트의
  자동 Servo On/초기화 동작을 별도로 고려해야 한다. 복구 완료 전 기존 bringup을 재실행하지 않는다.

검증: 실제 SDK 링크 빌드 성공 및 `--self-test`로 상태 0~15의 전환 허용 검사 통과.
실물 로봇에는 이 도구를 실행하지 않았으므로 실물 전환 성공은 아직 검증하지 않았다.

```bash
bash tools/doosan_recovery.sh --self-test
```
