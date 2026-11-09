# piece_detection — 장기말 인식

기본 모델은 `models/weights.pt`(입력 640, 단일 클래스 `0: janggi_dol`)다.
말의 박스·중심·신뢰도만 검출하며 진영과 종류는 분류하지 않는다.
결과의 `side='unknown'`, `kind='piece'`는 미분류를 뜻한다. 보정 파일의 네 모서리 픽셀로 교차점에 할당할 수 있다.
UI는 말 위치를 중립색의 `알`로 표시하며 게임·로봇 대국 기능은 비활성화한다.
기존 `models/weights_14class.pt`는 보존하며, 명시적으로 선택하면 기존 14클래스 분류도 지원한다.

```python
import sys; sys.path.insert(0, "~/doosan_janggi_ws/janggi")
from piece_detection import PieceDetector, BoardMapper, BoardStabilizer

det = PieceDetector()                      # GPU 있으면 cuda:0
mapper = BoardMapper.from_calibration()    # src/janggi/calibration/config/calibration.yaml
stab = BoardStabilizer(window=8)

dets = det.detect(bgr)                     # list[Detection]: side, kind, conf, box, board
board = mapper.assign(dets)                # {(x, y): ('unknown', 'piece'), ...}
stable = stab.update(board)                # 최근 8프레임 다수결
```

## 기존 14클래스 모델의 라벨

| id | 라벨 | 진영 | 말 | | id | 라벨 | 진영 | 말 |
|---|---|---|---|---|---|---|---|---|
| 0 | blue cha | 초 | 차 | | 7 | red cha | 한 | 차 |
| 1 | blue horse | 초 | 마 | | 8 | red horse | 한 | 마 |
| 2 | blue king | 초 | 궁(초) | | 9 | red king | 한 | 궁(한) |
| 3 | blue po | 초 | 포 | | 10 | red po | 한 | 포 |
| 4 | blue sa | 초 | 사 | | 11 | red sa | 한 | 사 |
| 5 | blue sang | 초 | 상 | | 12 | red sang | 한 | 상 |
| 6 | blue solder | 초 | 졸 | | 13 | red solder | 한 | 병 |

`PieceDetector`는 단일 `janggi_dol` 또는 위의 기존 14클래스 목록만 허용한다.

## 장기판 좌표

`src/janggi/action`과 같다. (0,0) 좌상단 · (8,0) 우상단 · (8,9) 우하단 · (0,9) 좌하단, x 0~8, y 0~9.

- 네 모서리 픽셀로 이미지 → 판 호모그래피를 만든다.
- 말 윗면은 판보다 높아 카메라 중심에서 멀수록 바깥쪽으로 밀려 보인다. 카메라-판 거리(보정 결과)와 말 높이(`piece_height_mm`, 기본 12 mm)로 되돌린 뒤 가장 가까운 교차점에 할당한다.
- 교차점에서 0.45칸 넘게 떨어진 검출은 버린다. 한 교차점에 여러 검출이 있으면 신뢰도가 높은 것만 남긴다.

카메라나 판을 움직였으면 보정을 다시 해야 매핑이 맞는다.

## 환경

`janggi/.venv` (torch CUDA + ultralytics + pyrealsense2, ROS와 분리). 처음 한 번 `janggi/setup_venv.sh`.
RTX 4050에서 1280×720 프레임 추론은 약 15~23 ms.

## 테스트

```bash
/usr/bin/python3 -m pytest janggi/piece_detection/tests     # 매핑·안정화 (torch 불필요)
```
