# RealSense RGB-D 녹화 / 촬영

학습용 말 이미지 추출·정답 라벨링·Train 회전 증강은 **[장기말 데이터셋 제작 도구](PIECE_DATASET.md)**를 사용한다:

```bash
./tools/run_piece_dataset.sh
```

녹화 후 평가 로그의 두 영상과 그래프를 함께 보려면 **[비전 로그 분석 뷰어](EVALUATION_VIEWER.md)**를 사용한다:

```bash
./tools/run_evaluation_viewer.sh
./tools/run_evaluation_viewer.sh --compare  # 여러 로그를 체크해 겹쳐 보고 영상 2/4개 선택
```

로봇과 ROS 없이 D455 계열 카메라의 컬러·깊이를 보면서 테스트 데이터를 저장한다.
기존 `janggi/.venv`를 사용한다. 없다면 워크스페이스 루트에서 `./janggi/setup_venv.sh`를 먼저 실행한다.
다른 인식·보정 프로그램은 종료한 뒤 USB 3 포트에 카메라를 연결한다.

```bash
./tools/run_recorder.sh
./tools/run_recorder.sh --output /원하는/저장폴더
```

- **녹화 시작 / 정지 (R)**: 컬러 MP4와 원본 RGB-D 파일 동시 녹화 (`.db3`, SDK에 따라 `.bag`). 정지 후에도 미리보기는 계속된다.
- **사진 촬영 (Space)**: `photos/`에 컬러·깊이 PNG만 저장한다 (별도 JSON·보정 파일 없음). 녹화 중에도 촬영할 수 있다.
- **종료 (Esc / 창 닫기)**: 녹화를 끝내고 파일을 닫은 뒤 종료한다.
- 저장 폴더는 화면 상단에서 바꾼다. 녹화 중에는 변경할 수 없다.

기본 저장 위치는 `tools/captures/`이며 Git에서 제외된다. **집에 가져갈 때 이 폴더를 별도로 복사한다.**
녹화는 촬영별 폴더에, 사진은 공용 `photos/` 폴더에 시간·마이크로초가 포함된 파일명으로 저장한다.

## 저장 형식

```text
tools/captures/
├── video_날짜_시간_마이크로초/
│   ├── color.mp4                 # 인식 테스트용 컬러, mp4v 압축
│   ├── rgbd.db3                  # SDK 원본 컬러 + uint16 깊이 스트림
│   ├── frames.csv                # MP4 프레임 순서, SDK 프레임 번호·타임스탬프
│   ├── metadata.json             # 카메라 내부/외부 파라미터, 깊이 단위, 녹화 결과
│   └── board_calibration.yaml    # 현재 프로젝트의 보정 파일 사본 (파일이 있을 때)
└── photos/
    ├── color_날짜_시간_마이크로초.png   # 무손실 컬러
    └── depth_날짜_시간_마이크로초.png   # 같은 시각의 정렬된 uint16 깊이
```

깊이 미리보기의 색상은 표시 전용이다. 사진의 실제 깊이[m]는
`depth_*.png` 값 × PNG 내부 메타데이터의 `depth_scale_m`로 계산한다. 0은 깊이를 얻지 못한 픽셀이다.
PNG는 `cv2.imread(path, cv2.IMREAD_UNCHANGED)`로 읽어야 16비트 값이 유지된다.
사진 메타데이터는 별도 파일 대신 PNG 내부 텍스트에 저장한다.
`PIL.Image.open(path).info["capture_metadata"]`를 `json.loads()`로 읽으면 된다.
정렬된 깊이는 `color_intrinsics` 기준이다. 원본 RGB-D 파일에는 정렬 **전** 원본 스트림을 보존한다.
보정 사본은 녹화 당시 프로젝트 파일을 복사한 것으로, 카메라·판 위치가 달라졌다면 재보정해야 한다.

## 집에서 테스트

컬러만 사용할 때는 현재 인식 UI에 MP4나 사진을 바로 넣는다 (로봇 옵션 불필요).

```bash
./janggi/realtime_ui/run.sh --source /복사한/video_.../color.mp4
./janggi/realtime_ui/run.sh --source /복사한/photos/color_날짜_시간_마이크로초.png
```

원본 컬러·깊이를 재생할 때는 녹화에 사용한 것과 호환되는 RealSense SDK를 사용한다. 카메라 없이 실행할 수 있다.
현재 환경은 `.db3`를 요구하며, `.bag`을 요구하는 SDK에서는 자동으로 `.bag`으로 전환한다.
실제 파일명은 `metadata.json`의 `recording_file`에 기록된다. 아래 경로는 실제 확장자에 맞게 지정한다.

```python
import pyrealsense2 as rs

pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_device_from_file('/복사한/video_.../rgbd.db3', repeat_playback=False)
profile = pipe.start(cfg)
profile.get_device().as_playback().set_real_time(False)  # 느린 분석 중 프레임 건너뛰기 방지
align = rs.align(rs.stream.color)
try:
    while True:
        try:
            frames = pipe.wait_for_frames(5000)
        except RuntimeError:  # 파일 끝 (또는 재생 오류)
            break
        aligned = align.process(frames)
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        # np.asanyarray(color.get_data()), np.asanyarray(depth.get_data())로 분석
finally:
    pipe.stop()
```

## 설정과 촬영 요령

기본값은 컬러 1280×720, 깊이 848×480, 30 FPS다.
깊이 미리보기는 0~1.5 m이며 `--max-depth 2`로 범위를 바꿀 수 있다 (원본에는 영향 없음).
지원하지 않는 스트림 조합은 오류로 표시된다. 예를 들어 대역폭을 줄이려면:

```bash
./tools/run_recorder.sh --width 640 --height 480 --depth-width 640 --depth-height 480
```

녹화 시작·정지 시 SDK 기록 대상을 전환하려고 스트림을 다시 열기 때문에 미리보기가 잠시 멈춘다.
MP4는 받은 컬러 프레임을 고정 FPS로 저장한다. 처리 지연으로 누락되면 실제 시간과 MP4 재생 시간이 다를 수 있다.
정확한 타임스탬프·원본 깊이가 필요한 검증에는 원본 RGB-D 파일을 쓰고, MP4 프레임 시간은 `frames.csv`를 참고한다.
동기 사진 저장 중에는 프레임 처리가 잠시 늦어질 수 있다. 원본 RGB-D 파일과 MP4의 프레임 수가 꼭 같지는 않다.
남은 디스크가 1 GB 미만이면 녹화를 자동 종료한다. 강제 종료 대신 정지/종료 버튼으로 원본 RGB-D·MP4 저장을 마친다.

집에서 비교할 자료로는 빈 판, 초기 배치, 여러 중간 배치, 겹치거나 비스듬한 말, 손이 들어오는 착수 장면,
밝기·그림자가 다른 장면을 각각 촬영하면 좋다. 미세한 글자 인식 비교용 정지 장면은 PNG도 함께 남긴다.

카메라 없이 저장 형식과 오류 시 파일 마무리를 검사하려면:

```bash
/usr/bin/python3 -m pytest -q tools/tests
```

실제 카메라에서는 출발 전에 짧게 녹화·정지한 뒤 `color.mp4`가 열리는지 확인하고,
`photos/` 폴더에 같은 시각의 `color_*.png`, `depth_*.png`가 함께 있는지 확인한다.
