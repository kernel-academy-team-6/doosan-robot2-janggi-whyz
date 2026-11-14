# 독립 4점 실측 판 높이 보정

## 새 측정 기준

기존 `src/janggi/calibration/config/calibration.yaml`의 카메라 보정, 네 모서리 XYZ, 222mm 기준은 수정하지 않았다.
이번 접촉 측정은 별도 `src/janggi/calibration/config/board_height.yaml`에 저장했다.

측정 당시 그리퍼 명령은 700, J6 플랜지→팁 길이는 **160.3681mm**다.
이 길이는 측정 당시 기준으로 파일에 고정 저장한다. 나중에 그리퍼 모델 파라미터를 바꾸더라도
이미 측정한 판 높이가 따라서 달라지지 않게 했다.

| 모서리 | 실측 플랜지 Z | 판 표면 Z=플랜지 Z−160.3681 | 기준 바닥 −30 대비 높이 |
|---|---:|---:|---:|
| 좌상단 (0,0) | 143.5573 | −16.8108 | 13.1892 |
| 우상단 (8,0) | 143.4812 | −16.8869 | 13.1131 |
| 우하단 (8,9) | 137.4137 | −22.9544 | 7.0456 |
| 좌하단 (0,9) | 137.6356 | −22.7325 | 7.2675 |

모든 값은 mm, 로봇 베이스 좌표계 기준이다. `130.3681 = 160.3681 − 30`이므로 사용자가 계산한
`143.5573 − 130.3681 = 13.1892`는 기준 바닥 대비 높이다.
명령용 절대 Z는 `−30 + 13.1892 = −16.8108`이다.
구현에서는 같은 결과인 `143.5573 − 160.3681`을 사용하므로 −30을 중복 가산하지 않는다.
이 측정만으로 판 두께·바닥·로봇 설치 기울기를 개별 분리 추정하지는 않는다. 필요한 접촉 표면 Z를 직접 사용한다.

## XY 위치별 이중선형 보간

`src/janggi/action/board_height.py`의 `BoardHeightMap.surface_z(x_mm, y_mm)`가 독립 계산 함수다.

```python
from action.board_height import BoardHeightMap
bed = BoardHeightMap.from_file()
z_board = bed.surface_z(500, 100)          # 약 -19.250598mm
height = bed.height_above_support(500,100) # 약 10.749402mm
z_tip = z_board + 2                       # 약 -17.250598mm
```

실측 네 모서리 XY 사각형을 `(u,v)` 좌표로 역변환한 뒤 아래 가중치를 적용한다.

```text
weights = [(1-u)(1-v), u(1-v), uv, (1-u)v]   # TL, TR, BR, BL
Z_board = weights · [Z_TL, Z_TR, Z_BR, Z_BL]
```

직사각형이라고 가정해 X/Y 길이를 대충 나누지 않는다. 좌하단 Y가 다른 실제 사각형도 반영한다.
네 점을 평균 평면에 맞춰 잔차를 버리지 않고 네 실측점을 모두 통과한다.
중앙의 Z는 약 **−19.84615mm**다.

터미널에서도 읽기 전용 계산을 할 수 있다.

```bash
cd ~/doosan_janggi_ws
python3 src/janggi/action/board_height.py 500 100
```

입력은 **판 로컬 좌표가 아닌 로봇 베이스 XY(mm)**다. 예시 `(100,100)`은 이 판의
X 범위 약 288.417~642.958mm 밖이므로 내부 보간점이 아니다.
기본적으로 판 밖 XY는 `(u,v)`를 0~1로 제한해 가장자리 높이를 사용한다.
이는 카메라로 인식한 가장자리 말 중심이 조금 벗어날 때 높이를 무제한 외삽하지 않기 위한 동작이다.
분석 목적으로 외삽이 필요하면 아래처럼 명시할 수 있다. 실제 집기에서는 이 옵션을 사용하지 않는다.

```bash
python3 src/janggi/action/board_height.py 100 100 --extrapolate
```

## 실행 경로

`BoardGeometry.from_calibration()`은 카메라/판 좌표 보정과 별개로 새 높이 파일도 읽는다.
기존 `corners_mm`, `tool_abc`, `tcp_offset_mm` 값을 바꾸지 않는다.
`command_surface_z()`에서만 새 실측면을 선택한다.

```text
실제 말 중심 XY (집기 직전 재인식 포함)
 → 별도 실측면의 Z_board(X,Y)
 → Z_tip = Z_board + 2mm                # 작은 말·중간 말·궁 모두 동일
 → c = 요청 너비의 역함수
 → Z_flange = Z_tip + 132.6717 + H(c)
```

놓을 곳도 해당 목적지 XY의 판 표면 Z+2mm를 사용한다.
그리퍼 너비, 말의 두께, 종류별 닫힘/개방 너비는 유지한다. 종류별 집기 높이만 `[2,2,2]mm`로 통일했다.
이전에 구현한 개폐 구간 최대 유효 길이 검사와 진입 높이/경로 검사는 유지한다.

추가 수동 미세 보정(`board_z_compensation_enabled`)은 기존처럼 기본 false다.
**이 값이 false여도 별도 실측면은 적용된다.** true일 때만 새 실측면 위에 전체/모서리 추가값을 더한다.
UI는 실측 판 Z와 최종 보정 높이를 표시한다. 별도 높이 파일의 `enabled: false` 또는 명시적인
`height_map_path=None`으로 맵을 사용하지 않는 경우에만 `command_board_z_mm`의 −20mm로 돌아간다.

판 밖 버리는 위치는 기존 별도 높이 기준을 유지한다. standalone 액션/CLI에 사용자가 직접 주는
절대 팁 Z에는 이 판 전용 함수를 자동 가산하지 않는다.
Gazebo의 판 배치/카메라 원본 캘리브레이션을 새 측정에 맞춰 변형한 작업도 아니다.

## 검증과 적용

네 모서리 재현, 내부/중앙/가장자리 XY 역변환, 판 밖 제한/외삽, 비정상 좌표,
모든 말 종류의 픽/플레이스 +2mm, 추가 수동 보정, 기존 보정 파일 불변을 검증했다.
관련 action·PickPlace2·Gazebo·높이/속도 UI 테스트 **202개 통과**.

기존 calibration.yaml의 작업 전후 SHA-256:
`631d76a7f1cd8db6863e35f38c53c5c3769ff747787fd84d573ed19df548db15`.
실제 로봇 이동 명령은 보내지 않았다. 실제 기구 기준/접촉 측정 오차까지 소프트웨어 테스트가 검증한 것은 아니다.

PickPlace2 패키지를 재빌드했다. 실행 중인 UI·실행기를 재시작하면 새 설정/맵을 읽는다.
기존처럼 `source /opt/ros/jazzy/setup.bash`와 `source install/setup.bash`를 적용한 환경을 사용한다.
