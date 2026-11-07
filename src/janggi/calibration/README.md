# calibration — 장기판 4점 카메라 ↔ 로봇 보정

카메라(RealSense D455F)와 장기판이 고정된 상태에서, 장기판 네 모서리를 **카메라 화면 픽셀**과 **그리퍼 팁 위치**로 각각 찍어 카메라 자세 `T_base_camera`를 구한다. 결과는 `config/calibration.yaml`에 저장된다 (보정을 한 번 끝내야 생긴다).

```text
p_base = R · p_cam + t        (단위 m, base_link ← camera_color_optical_frame)
```

같은 창에 로봇 패널(조그 · 그리퍼 · 홈)이 있어 팔을 따로 움직일 도구가 필요 없다. 점을 찍지 않고 로봇만 움직이려면 `--jog-only`로 띄운다.

## 방식: PnP (depth 미사용)

장기판(무늬 없는 나무) 위에서는 D455F의 depth가 거의 잡히지 않는다. 실측에서 판 영역의 유효 depth가 default 프리셋 38%, high_accuracy 프리셋 21%였고, 판 중앙은 0이었다. 그래서 보정에는 depth를 쓰지 않는다.

- 네 점의 픽셀(RealSense 왜곡 모델로 정규화)과 로봇이 준 base 3D 점으로 PnP(IPPE + LM)를 푼다. 1 px 오차가 판 위에서 약 1 mm다.
- 팁이 닿은 네 점으로 판 평면을 구해 `board_plane_base`로 저장한다. 이후 픽셀 → base 변환은 광선과 판 평면의 교점으로 하므로 depth가 필요 없다.
- depth가 잡힌 점이 있으면 교차검증 값으로만 로그와 yaml에 남긴다.

## 설치 (한 번)

시스템 Python의 `rclpy`, `python3-numpy`, `python3-opencv`, `python3-tk`, `python3-pil`을 그대로 쓰고 `pyrealsense2`만 venv에 추가한다. pip의 numpy 2.x를 섞으면 ROS 메시지 모듈(`dsr_msgs2`)이 깨지므로 numpy는 따로 설치하지 않는다.

```bash
cd ~/doosan_janggi_ws/src/janggi/calibration
./setup_venv.sh
```

이 디렉터리에는 `COLCON_IGNORE`가 있어 colcon 빌드 대상이 아니다.

## 실행

1. 로봇 드라이버를 띄운다. `ROBOT_IP`는 실제 로봇 주소로 바꾼다. 이 워크스페이스의 `install/setup.bash`를 source해야 `e0509_gripper_description`이 보인다 (`~/ros2_ws`만 source된 셸에서는 패키지가 없다고 나온다).

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/doosan_janggi_ws/install/setup.bash
   ros2 launch e0509_gripper_description bringup.launch.py \
     mode:=real host:=ROBOT_IP port:=12345 name:=dsr01
   ```

2. 다른 터미널에서 실행한다. `realsense-viewer`, `cam_visualizer`처럼 카메라를 쓰는 프로그램은 먼저 끈다.

   ```bash
   ./run.sh                          # 보정 (카메라 + 로봇 패널)
   ./run.sh --jog-only               # 로봇 패널만 (카메라 불필요)
   ./run.sh --gripper-cmd            # 시작 시 그리퍼를 650으로
   ./run.sh --jog-speed 1            # 조그 시작 속도 단계 (기본 2 = 3%)
   ./run.sh --home-vel 20            # 홈 이동 관절 속도 [deg/s] (기본 30)
   ./run.sh --tcp-offset 0 0 0       # 펜던트 TCP가 이미 그리퍼 팁으로 설정된 경우
   ./run.sh --manual-robot           # ROS 없이 펜던트 좌표 입력 (로봇 패널 비활성)
   ```

   그리퍼 팁 오프셋은 **그리퍼 위치 명령 650** 상태에서 J6 중심으로부터 아래로 6.3 + 2.5 + 4.9 + 8.5 = 22.2 cm로 실측한 값이다 (`robot_ui.GRIPPER_TIP_OFFSET_MM`). 펜던트에 TCP를 따로 설정해 두었다면 오프셋이 이중으로 들어가니 `--tcp-offset 0 0 0`을 쓴다.

3. 장기판 기준 **좌상단 → 우상단 → 우하단 → 좌하단** 순서로 네 번 반복한다. 왼쪽 위는 카메라 영상, 그 아래는 단계 안내와 로그, 오른쪽은 로봇 패널이다.

   | 단계 | 할 일 | 키 |
   |---|---|---|
   | 카메라 | 화면에서 점을 정확히 클릭 (확대창 참고, 방향키로 1픽셀 이동). 점이 팔에 가리지 않게 한다 | `SPACE` |
   | 로봇 | 조그로 그리퍼 팁을 그 점에 맞춤. 그리퍼는 650 | `SPACE` |

   4번째 점에서는 앞 3점으로 추정한 대략적 위치를 안내에 보여준다. 로봇 단계에서 그리퍼 명령값이 650이 아니면 기록 전에 확인을 묻는다.

4. 네 쌍이 모이면 계산해 저장하고 점별 판 위 잔차(mm)와 재투영 오차(px), 네 팁 점의 평면 이탈을 로그에 출력한다. 잔차가 큰 점은 `U`로 되돌려 다시 찍는다. 기존 파일은 `calibration.yaml.bak`으로 남는다.

5. 검증: 저장 후 화면의 **판 안쪽** 점을 클릭하면 base 좌표가 나온다. 팁을 그 점에 대고 `SPACE`를 누르면 차이가 나온다.

## 로봇 패널

### 조그

키나 패널 버튼을 **누르고 있는 동안** base 좌표계로 이동하고, 떼면 정지한다. 보정 어느 단계에서나 쓸 수 있다 (카메라 점 측정 중 1초만 막힘).

| 키 | 동작 |
|---|---|
| `W` / `S` | X+ / X− |
| `A` / `D` | Y+ / Y− |
| `R` / `F` | Z+ (위) / Z− (아래) |
| `1` ~ `6` | 속도 1 / 3 / 10 / 25 / **50 / 75** % |
| 패널 `STOP` | 즉시 정지 (홈 이동 포함) |

- 키를 여러 개 함께 누르면 대각선으로 움직인다. 화면 방향과 base 축 방향은 다르니 처음에는 속도 1로 짧게 눌러 방향을 확인한다.
- 짧게 톡 치면 0.1 mm 단위로도 움직인다.
- 에뮬레이터 기준 0.5초 동안 3%는 약 6 mm, 75%는 약 41 mm 움직였다. 50 / 75 %는 판 근처에서 쓰지 않는다.

### 그리퍼

| 버튼 / 키 | 동작 |
|---|---|
| 열기 (0) / 닫기 (700) | `gripper_service_node`의 open/close와 같은 값 |
| 팁 기준 650 (`G`) | 팁 오프셋을 잰 개도 |
| 위치 입력 + 적용 | 0~700 임의 값 (RUNBOOK의 position_cmd 범위) |

현재 명령값은 `/dsr01/gripper/stroke`로 표시한다. 그리퍼 노드는 명령을 받을 때만 이 값을 내보내므로, 처음에는 "모름"으로 나온다. 한 번 설정하면 표시된다.

### 홈 자세

- `H` 또는 "홈으로 이동": 확인창 뒤 `move_joint`로 이동한다. 이동 중에 `STOP`, 조그 키, `ESC`를 누르거나 창 포커스를 잃으면 `move_stop`으로 멈춘다.
- "현재 자세를 홈으로 저장": 지금 관절값을 `config/robot_poses.yaml`에 저장한다. 파일이 없으면 기본값 `[0, 0, 90, 0, 90, 0]`(툴 하향)을 쓴다.

### 지연과 안전

지연 (에뮬레이터 실측, joint_states 10 Hz라 ±100 ms):

- 키 누름 → 명령 전송: 수 ms. 뗌 → 정지 명령: 30 ms (X11 키 자동반복 구분용)
- 명령 → 실제 움직임 시작 약 130–200 ms(가속 포함), 정지 명령 → 멈춤 약 90 ms. 이 부분은 컨트롤러 쪽이다.

안전 장치:

- 창이 포커스를 잃거나, 종료·`ESC`·`SPACE`(로봇 단계) 때 조그와 홈 이동을 정지한다.
- UI가 0.3 s 이상 멈추면 감시 스레드가 직접 정지 명령을 보낸다.
- 프로그램이 강제 종료(kill -9)되면 정지 명령을 못 보낸다. **비상정지 버튼을 손 닿는 곳에 둔다.**

## 키 전체

| 키 | 동작 |
|---|---|
| 클릭 | 점 선택 (보정 후에는 검증 점) |
| 방향키 | 선택 점 1픽셀 이동 |
| `W` `S` `A` `D` `R` `F` | 조그 |
| `1` ~ `6` | 조그 속도 |
| `G` | 그리퍼 650 |
| `H` | 홈 이동 (확인 후) |
| `SPACE` / `ENTER` | 현재 단계 확정 |
| `U` | 한 단계 되돌리기 |
| `ESC` | 모든 이동 정지 + 종료 확인 (그리퍼 입력창에서는 입력창 빠져나오기) |

## 결과 사용

```bash
# yaml 첫 줄 주석에 값이 채워진 명령이 있다
ros2 run tf2_ros static_transform_publisher --x ... --qw ... \
  --frame-id base_link --child-frame-id camera_color_optical_frame
```

픽셀 (u, v) → base 점: `rs2_deproject_pixel_to_point(intr, [u, v], 1)`로 광선 방향을 구해 `R`로 돌리고, 원점 `t`에서 `board_plane_base`와 교차시킨다. `corners`에는 네 모서리의 픽셀과 base 좌표가 함께 있어 판 격자(교차점 90개) 계산에도 쓸 수 있다.

## 주의

- 판 평면 위 좌표는 정확하지만(시뮬레이션 0.5 mm), 카메라 위치 자체는 수 mm, 각도는 0.5° 정도 흔들릴 수 있다. 평면 4점 PnP의 특성이다. 판에서 높이 방향으로 멀어질수록 오차가 커지므로 말 높이 정도까지만 신뢰한다.
- 팁 오프셋을 잘못 주면 결과 전체가 일정량 어긋난다. 네 점을 같은 툴 자세로 찍으면 잔차로도 드러나지 않는다. 펜던트의 TCP 설정을 확인한다.
- 팁 높이가 점마다 다르면(팁이 판을 누르거나 떠 있음) "평면 이탈" 값이 커진다. 닿기 직전 높이를 일정하게 맞춘다.
- 카메라나 판을 움직이면 다시 보정한다.

## 카메라 연결이 끊길 때

USB 통신 오류(`journalctl -k`에 `error -71`, `USB disconnect`)로 카메라가 잠깐 빠졌다 다시 잡히는 일이 있다.

- 도구가 자동으로 다시 연결한다 (수 초). 그동안 화면에 `CAMERA LOST`가 뜨고 카메라 점 기록이 막힌다. 이미 찍은 점과 진행 단계는 유지된다.
- 자주 끊기면 하드웨어를 확인한다: 노트북 포트에 직접 꽂기(허브·연장선 빼기), 짧은 USB 3 케이블(정품 또는 인증 케이블), 로봇 케이블 묶음에서 떨어뜨리기. 연장이 필요하면 전원이 들어가는 USB 3 허브나 액티브 케이블을 쓴다.

## 파일

| 파일 | 내용 |
|---|---|
| `calibrate_board.py` | 보정 앱, 카메라, 계산·저장 |
| `robot_ui.py` | 로봇 클라이언트, 로봇 패널(조그/그리퍼/홈), 조그 전용 창 |
| `jog.py` | 누름/뗌 → 조그 명령 상태 관리 (ROS/GUI 무관) |
| `calib_math.py` | PnP, 평면, 좌표 변환 (ROS/카메라 무관) |
| `config/calibration.yaml` | 보정 결과 |
| `config/robot_poses.yaml` | 홈 자세 (저장 버튼으로 생성) |

## 테스트

```bash
.venv/bin/python -m pytest tests
```
