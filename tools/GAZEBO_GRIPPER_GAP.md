# Gazebo 그리퍼 패드 간격 직접 측정

실행 중인 Gazebo의 **실제 손가락 링크 pose**와 **현재 로드된 패드 STL/스케일/visual pose**로
양쪽 안쪽 평면 패드 사이의 최단거리를 측정한다. 목표 stroke, ROS `/joint_states`,
관절 FK, 700→각도→너비 환산식을 사용하지 않는다. 읽기 전용이며 로봇이나 그리퍼를 움직이지 않는다.

## 사용

Gazebo가 켜진 상태에서 다른 터미널:

```bash
cd ~/doosan_janggi_ws
./tools/measure_gazebo_gripper.sh
```

기본 초당 2회 갱신. Ctrl+C 종료. 한 번만 읽으려면:

```bash
./tools/measure_gazebo_gripper.sh --once
```

표시 예시(실행 중인 시뮬레이터에서 실제로 읽은 값):

```text
sim=2714.091s  gap=106.926 mm  centers=106.926 mm  tilt=0.0000 deg
```

- **gap**: 두 유한한 안쪽 패드 표면의 최단거리. 주로 비교할 값이다.
- **centers**: 패드 중심점 간 거리. 평행하고 높이가 맞으면 gap과 거의 같다.
- **tilt**: 패드 평면이 평행에서 벗어난 각도. 손가락이 구부러지거나 기울어졌는지 판단하는 보조값이다.
- **sim**: 해당 위치 메시지의 Gazebo 시뮬레이션 시각.

현재 scene/world 기본값 `empty`, 로봇 모델 `janggi_physics_robot`이다.
다르면 `--world 이름 --model 이름`을 지정한다. Gazebo와 같은 `GZ_PARTITION` 환경에서 실행해야 한다.
런처 스크립트가 ROS/Gazebo Python 환경을 source하므로 별도 빌드나 Gazebo 재시작은 필요 없다.

## 실측 비교용 CSV

그리퍼를 원하는 값으로 **다른 제어 도구에서** 움직이고 정지한 뒤 측정한다.
`--label`은 기록용 문자열이며 동작 명령이 아니다.

```bash
./tools/measure_gazebo_gripper.sh \
  --seconds 5 --rate 5 --label stroke_700 \
  --csv docs/analysis/gripper_gap/stroke_700.csv
```

CSV에 label, 실제 시각, 시뮬레이션 시각, gap_mm, center_mm, tilt_deg,
양쪽 최근접점 좌표(m)를 저장한다. 기존 CSV는 덮어쓰지 않으므로 새 파일명을 사용한다.
여러 stroke마다 같은 방식으로 저장하고 실제 장치도 **같은 안쪽 평면 패드 위치**에서 재면 비교하기 좋다.
정지 후 여러 샘플의 평균·범위를 비교하고, 측정 당시의 물체 접촉 여부도 별도로 남기는 것을 권장한다.
현재 상태의 읽기 전용 저장 예시는 [current_sample.csv](../docs/analysis/gripper_gap/current_sample.csv)에 있다.

## 측정 기준과 구현

1. `/world/empty/scene/info`에서 대상 모델·손가락의 entity ID와 visual mesh 경로·스케일·pose를 읽는다.
2. r2 STL의 최소 Y 평면, l2 STL의 최대 Y 평면에서 안쪽 평평한 패드를 추출한다.
   현재 CAD의 해당 면은 26×20mm이며 링크 원점에서 ±3.9mm 떨어져 있다.
   이 수치는 계산에 하드코딩하지 않고 로드된 STL에서 직접 추출한다.
3. `/world/empty/dynamic_pose/info`의 **동일한 메시지**에 있는 실제 r2/l2 링크 pose를 사용한다.
   native mimic 제약에서 미세하게 어긋난 손가락도 독립적인 실제 pose로 측정한다.
4. STL 패드를 각 링크 pose로 변환해 두 사각형의 최근접점을 구한다.
   무한 평면 간 거리나 링크 원점 간 거리가 아니므로 기울거나 높이가 어긋난 경우도 반영한다.

**gap은 CAD 표면의 기하학적 거리다.** Bullet 물리 엔진의 collision margin은 포함하지 않는다.
따라서 접촉이 시작되는 순간의 gap이 정확히 0mm가 아닐 수 있다. 이 값을 엔진 내부의 접촉 거리라고 부르지는 않는다.
또한 모델 링크가 없는 정적 미러, 수정되어 다른 형상이 된 STL, 대상 모델 respawn으로 바뀐 entity ID는
계속 추정하지 않고 오류/시간 초과로 알린다. 모델을 재생성했다면 측정 도구도 다시 실행한다.

이 도구는 제어기·TCP·실제 로봇 설정을 변경하지 않는다. 지금은 측정·저장 기능만 추가했으며
가상 모델을 실측에 맞추는 보정은 비교 데이터를 얻은 후 따로 적용한다.

검증: 실제 실행 중인 Gazebo에서 106.926mm 읽기 및 CSV 기록 확인.
평행 패드, 공통 강체 변환, 10° 기울어진 패드, 유한 패드의 모서리 거리, STL 스케일 적용을 검증하는 테스트 5개 통과.

```bash
python3 -m pytest -q tools/tests/test_gazebo_gripper_gap.py
```
