# action — 장기판 좌표로 로봇 이동

```python
from action import move

move(0, 0)          # 좌상단 교차점, 판 표면 (높이 0)
move(4, 4, 10)      # (4, 4) 교차점 위 10 mm에서 멈춤
```

## 좌표계

```text
(0,0) ───────────── (8,0)      x: 0~8 (가로 방향 교차점 9개)
  │                   │        y: 0~9 (세로 방향 교차점 10개)
  │       판          │
  │                   │
(0,9) ───────────── (8,9)
```

방향은 보정 때 찍은 순서(좌상단 → 우상단 → 우하단 → 좌하단)와 같다. 정수가 아닌 값(예: `move(3.5, 2)`)도 받는다.

## 동작

1. `calibration/config/calibration.yaml`의 네 모서리 `base_xyz`를 읽는다. 보정 때 그리퍼 팁을 직접 대서 잰 값이다.
2. (x, y)는 네 모서리를 이중선형 보간해 base 좌표로 바꾼다. 카메라 변환은 쓰지 않는다.
3. 도착점은 판 법선 방향으로 `height_mm`(기본 **0** = 보정 때 팁을 댄 판 표면) 띄운 점이다.
4. 툴 자세는 보정 때 자세를 쓰고, 팁 오프셋(222 mm)을 빼서 플랜지 목표를 만든다.
5. 말·판에 부딪히지 않게 세 단계로 움직인다 (`move_line`):
   1. 현재 팁이 이동 높이 `travel_mm`(기본 30 mm)보다 낮으면 그 자리에서 수직으로 올린다
   2. 이동 높이로 목표 교차점 위까지 직선 이동한다
   3. 도착 높이까지 수직으로 내린다
6. 이동이 끝날 때까지 기다린다. 실패하면 `RuntimeError`, 범위 밖 좌표는 `ValueError`를 낸다.

## API

| 이름 | 설명 |
|---|---|
| `move(x, y, height_mm=None, travel_mm=None)` | 기본 연결(dsr01)로 이동 (기본 도착 0 mm, 이동 30 mm). 도착한 팁 위치(base, mm)를 돌려준다 |
| `shutdown()` | `move()`가 만든 연결을 닫는다 |
| `BoardMover(ns, calibration, height_mm, travel_mm, vel, acc)` | 설정을 바꿔 쓸 때. `with` 문 지원 |
| `BoardMover.stop()` | 이동 정지 (`move_stop`) |
| `BoardGeometry.from_calibration(path)` | 로봇 없이 좌표만 계산 (`board_to_base`, `tip_target`) |

## 실행 환경

시스템 Python(`/usr/bin/python3`, ROS rclpy가 붙은 3.12)으로 실행하고 ROS 환경을 source한다. 셸의 기본 `python3`이 다른 venv를 가리키면 rclpy를 못 찾는다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/doosan_janggi_ws/install/setup.bash
/usr/bin/python3 your_script.py
```

예제: [`src/test/move_board_example.py`](../../test/move_board_example.py) — `(0,0) → (1,1) → … → (8,8) → (8,9)`.

```bash
cd ~/doosan_janggi_ws/src/test
./run_move_example.sh --dry-run      # 목표 좌표만 출력 (로봇 불필요)
./run_move_example.sh                # 실제 이동 (판 표면까지, 사이 이동 30 mm 높이, 100 mm/s)
./run_move_example.sh --height 10 --vel 50   # 교차점 위 10 mm에서 멈춤, 천천히
```

## 주의

- 보정을 다시 하면 그대로 새 값이 반영된다. 카메라나 판을 움직였으면 다시 보정한다.
- 보정 때 그리퍼 명령값과 같은 개도로 이동해야 팁 높이가 맞는다. 지금 보정 파일은 700으로 찍혀 있다(팁 오프셋 222 mm는 650 기준으로 잰 값).
- 높이 0은 팁이 판 표면에 닿는 높이다. 처음에는 `--height 10` 정도로 띄워서 위치를 확인한 뒤 0으로 내린다. 판 위에 말이 있는 교차점에서는 팁이 말에 닿는다.
- 처음에는 낮은 속도로 확인한다. 비상정지 버튼을 손 닿는 곳에 둔다.

## 테스트

```bash
/usr/bin/python3 -m pytest src/janggi/action/tests
```
