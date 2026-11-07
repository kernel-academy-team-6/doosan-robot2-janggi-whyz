# cam_visualizer

RealSense D455 컬러 포인트클라우드를 Open3D 창에 실시간으로 표시한다. ROS 없이 `pyrealsense2`로 카메라에 직접 붙는다.

## 설치

Ubuntu 24.04 시스템 Python에는 `venv`/`pip`이 없어서, pip 없이 venv를 만든 뒤 `get-pip.py`로 pip을 설치한다. `sudo apt install python3.12-venv`가 되어 있다면 `python3 -m venv .venv`로 만들어도 된다.

```bash
cd ~/doosan_janggi_ws/janggi/cam_visualizer
/usr/bin/python3 -m venv --without-pip .venv
curl -sSL https://bootstrap.pypa.io/get-pip.py | .venv/bin/python
.venv/bin/pip install -r requirements.txt
```

## 실행

```bash
./run.sh                    # 기본: 848x480@30, decimate 2, 필터 ON
./run.sh --show-2d          # color | depth 2D 창도 함께
./run.sh --decimate 1       # 전체 해상도 (점 약 37만 개, 무거움)
./run.sh --max-dist 1.2     # 1.2 m 이내만 표시 (장기판 볼 때)
./run.sh --help
```

ROS 드라이버(`realsense2_camera`)나 `realsense-viewer`가 카메라를 쓰고 있으면 열리지 않는다. 먼저 종료한다.

| 키 | 동작 |
|---|---|
| Q / ESC | 종료 |
| P | 일시정지 / 재개 |
| F | depth 후처리 필터 on/off |
| S | 저장: `captures/`에 PLY + color PNG + depth PNG |
| R | 시점 초기화 |
| 마우스 | 드래그 회전, 휠 줌, Shift/Ctrl+드래그 이동 |

## 처리 흐름

1. 별도 프로세스에서 color(rgb8)와 depth(z16)를 받는다. 렌더링과 GIL을 나눠 쓰지 않게 하려는 것이다.
2. depth 필터: decimation → 거리 threshold → disparity 변환 → spatial → temporal → depth 복원 → hole filling.
3. `rs.pointcloud`로 3D 점을 계산하고, 각 점을 color 카메라에 투영한 텍스처 좌표로 색을 가져온다.
4. 메인 프로세스가 최신 프레임만 받아 Open3D 창을 갱신한다. 렌더링이 밀리면 오래된 프레임은 버린다.

화면에서는 y 위, z 뒤 방향으로 축을 뒤집어 보여준다. 저장하는 PLY는 RealSense 광학 좌표계(x 오른쪽, y 아래, z 앞, 단위 m)를 그대로 쓴다. depth PNG는 decimation과 필터를 적용한 뒤의 16bit 원시값이며(1 = depth_scale, 보통 1 mm), color 이미지와 정렬되어 있지 않다.

## 성능

필터를 켜면 CPU 부하가 크다. 노트북이 배터리 + power-saver 모드라 CPU 클럭이 낮으면 약 15 FPS까지 떨어진다. 전원을 연결하고 성능 모드로 두면 FPS가 오른다. 그래도 느리면 `F`로 필터를 끄거나 `--decimate 3`을 쓴다.
