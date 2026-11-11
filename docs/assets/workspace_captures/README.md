# 실행 화면과 개발 이력 근거

2026-11-11, 루트 HEAD `44f41ea`를 기준으로 생성했다. 현재 작업 트리의 미커밋 변경(문서·캡처 스크립트와 SimWorker UI 호환 수정)을 함께 사용했다. 과거 커밋의 화면을 복원한 자료가 아니라 **현재 실행 화면**이다.

- `piece.png`, `hybrid.png`: 동일한 실제 촬영 프레임에 CPU YOLO / YOLO+MobileNet을 실행한 Tkinter 창.
- `obstruction.png`: 같은 프레임에 검정 사각형을 추가한 인위적 가림 입력. 모델 결과·UI는 실제 실행 결과.
- `dataset.png`: 기존 SQLite 카탈로그의 임시 복사본과 원본 Crop으로 연 DatasetApp. 정답 변경 없음.
- `evaluation.png`: `evaluation_inputs.json`의 두 기존 CSV·MP4를 현재 비교 뷰어로 연 결과. 프레임 인덱스 45, 즉 표시 46번째 처리 프레임.
- `simulation.png`, `simulation.log`: 실제 Fairy-Stockfish와 SimWorld/UI의 `--no-motion --turns 2 --seed 1` 실행. 양쪽 턴에서 가상 판과 게임 판 일치. 합성 검출이며 YOLO 성능이나 실제 파지 실험이 아님. 30 FPS는 SimWorker의 고정 표시값.
- `tcp.png`, `tcp_model/`: Profile·CompensationPlan·save_report 코드가 만든 순수 모델 계산의 보고서와 브라우저 캡처. 그리퍼 개도·팔 자세 피드백 모두 없음.
- `capture_manifest.json`: 각 실행 시각·명령·입력/모델 SHA-256.
- `development_history.json`: 루트 저장소의 참조 가능한 17개 커밋의 SHA·시각·제목·변경 파일.
- `sim_worker_compatibility.patch`: 최신 UI 계약에 맞춰 SimWorker에 추가한 최소 호환 변경.
- `*_observations.json`: Hybrid 캡처 시점의 실제 객체 상태 일부. 정답 궤적이 아님.

캡처는 `janggi/.venv`와 별도 Xvfb 디스플레이에서 실행했다. 로봇·카메라·Docker·Gazebo에 연결하지 않았다. 사용자의 데스크톱을 촬영한 자료가 아니다. 원본 카탈로그를 SQLite backup으로 임시 복사하고 프레임·Crop만 읽었다. 녹화·데이터셋·CNN 체크포인트는 Git 제외 경로이므로 재실행 시 별도 준비가 필요하다.

재현은 워크스페이스 루트에서 `DISPLAY=:93 janggi/.venv/bin/python tools/capture_workspace_docs.py --mode hybrid`처럼 실행한다. Xvfb를 먼저 별도로 준비하고 DISPLAY를 맞춰야 한다. 다른 모드는 `piece`, `obstruction`, `dataset`, `evaluation`, `simulation`, `tcp`이다. 기본 출력은 이 폴더이며 `--output`으로 변경할 수 있다. 기존 PNG를 다시 생성하므로 보존하려면 다른 출력 폴더를 지정한다. 시뮬레이터는 엔진 탐색 시간에 따라 선택 수가 달라질 수 있다.

`tcp_model/report.html`의 브라우저 캡처에는 headless Chrome을 사용했다. Xvfb 실행 파일은 시스템 설치 없이 Ubuntu 배포 패키지를 `/tmp/janggi_capture_runtime`에 내려받아 추출해 사용했다.
