# MobileNetV3-Small 장기말 분류 학습과 실시간 적용

YOLO는 `weights_only_piece.pt`로 위치를 찾고, 그 박스의 중심에서 추출한 RGB 이미지를 별도 분류기로 판별한다.
기존 `piece`, `full`, 옵션을 생략한 실행은 그대로 사용할 수 있다.

## 1. 학습 이미지 준비

```bash
cd ~/doosan_janggi_ws
./tools/run_piece_dataset.sh
```

정답 라벨링 후 **분배 설정 / 학습 데이터셋 내보내기**를 사용한다.
Train/Val/Test에 학습할 각 클래스의 원본이 있어야 한다. 일반 대국용은 14종 모두 준비한다.
회전 증강은 Train에만 적용하며, 동일 원본의 파생 이미지는 분할을 넘지 않는다.
다른 촬영 환경의 성능을 평가하려면 촬영 그룹별 분배를 사용한다.

학습 경로는 `train`, `val`, `test`, `dataset.json`, `manifest.jsonl`이 들어 있는 **내보내기 폴더**다.
`crops/raw`나 `catalog.sqlite` 자체를 학습 입력으로 지정하지 않는다.

## 2. 학습 실행

기존 `janggi/.venv`의 PyTorch·torchvision을 사용한다. 최초 실행은 공식 ImageNet 사전학습 가중치 약 10MB를
`tools/datasets/.torch/`에 내려받으므로 인터넷 연결이 필요하다. 다운로드 실패 시 임의 초기화로 대체하지 않는다.

```bash
cd ~/doosan_janggi_ws
./tools/train_piece_classifier.sh \
  --dataset tools/datasets/janggi_pieces/exports/dataset_20261110_205100_825553 \
  --output models/piece_classifier_run2 \
  --epochs 20 --freeze-epochs 5 --batch-size 32 --device auto
```

`--dataset`은 사용할 내보내기 폴더로 바꾼다. `--device auto`는 CUDA가 있으면 GPU, 없으면 CPU를 사용한다.
GPU를 명시하려면 `--device cuda:0`, CPU는 `--device cpu`다. GPU 메모리가 부족하면 배치를 16이나 8로 줄인다.
기본 `--workers 0`, CPU 스레드 4이며 `--threads`로 조절한다.

`--output`을 생략하면 `models/piece_classifier/`다. **비어 있지 않은 결과 폴더는 덮어쓰지 않는다.**
재학습할 때는 위 예시처럼 새 결과 폴더를 사용한다. 기존 모델은 계속 사용할 수 있다.

학습은 두 단계다.

1. 기본 5 epoch: 특징 추출층을 고정하고 분류층 학습
2. 나머지 epoch: 특징 추출층도 낮은 학습률로 미세 조정

배치 정규화의 사전학습 통계는 유지하고, 클래스 빈도의 역수로 손실 가중치를 조절한다.
추가 RandomResizedCrop, CenterCrop, 반전, 회전은 적용하지 않는다. 이미 증강된 전체 정사각형 이미지를
224×224로 읽고 ImageNet 평균/표준편차로 정규화한다. 학습 데이터 제작과 실시간 추출은
`janggi/piece_detection/crops.py`를 공유한다. 참고: [공식 모델](https://docs.pytorch.org/vision/main/models/generated/torchvision.models.mobilenet_v3_small.html),
[전이학습 설명](https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial.html).

## 3. 결과 확인

```text
models/선택한_결과_폴더/
├── best.pt                  # Val macro-F1이 가장 좋은 모델, UI에 사용할 파일
├── last.pt                  # 마지막 epoch 모델
├── config.json              # 설정·클래스 순서·데이터셋/manifest 해시·분배 조건
├── history.csv              # epoch별 Train loss/accuracy, Val loss/accuracy/macro-F1
├── val_metrics.json
├── test_metrics.json        # best.pt 선택 후 Test 평가; 모델 선택에는 사용하지 않음
├── val_confusion.csv
├── test_confusion.csv       # 행=실제 정답, 열=예측
├── val_predictions.csv
└── test_predictions.csv     # 이미지 경로·정답·예측·확률·정오
```

전체 정확도 외에 macro-F1, 클래스별 precision/recall/F1, 초 차→상·상→차 횟수를 확인한다.
학습 평가의 분류 확률은 모델의 softmax 값이며 정확히 교정된 성공 확률을 뜻하지는 않는다.
실시간 추적 모드의 C는 과거 확률과 크기 점수까지 결합한 값이다.
같은 촬영의 유사 이미지가 Train/Test에 섞인 경우 높은 점수만으로 다른 촬영 조건의 성능을 판단하지 않는다.

2026-11-10 초기 실행은 위 내보내기 데이터(Train 1,614장, Val 59장, Test 55장)로
CPU에서 12 epoch, 고정 4 epoch로 수행했다. 초기 실행 결과는 기본 `models/piece_classifier/`에 저장한다.
초기 best.pt는 epoch 5 모델이며 Val 정확도 58/59(98.3%), Test 정확도 51/55(92.7%),
Test Macro-F1 0.919였다. Test의 초 차·상은 각각 3장이고 두 종류 간 혼동은 없었다.
사진별 분배에 대한 결과이므로 독립 촬영 성능을 보장하지 않는다.
원본 프레임 한 장에서 탐지 32개·분류 32개 및 실시간 Worker 로그 저장을 확인했다.
GUI 창/실제 카메라/로봇 동작은 이 검증에 포함하지 않았다.
모델과 대용량 결과는 Git에서 제외하며 별도로 백업한다.

## 4. 실시간 UI 연결

```bash
cd ~/doosan_janggi_ws

# 기본 경로 models/piece_classifier/best.pt를 사용, 실제 카메라
./janggi/realtime_ui/run.sh --model-mode hybrid

# 새로운 학습 결과를 골라 저장된 영상으로 확인
./janggi/realtime_ui/run.sh --model-mode hybrid \
  --classifier models/piece_classifier_run2/best.pt \
  --source tools/captures/video_20261109_223623_956498/color.mp4 \
  --playback-fps 30 --log-counts
```

`--model-mode cnn`도 같은 동작이다. 위치는 `weights_only_piece.pt`로 찾고, 클래스 무관 중복 제거 후
최대 32장 단위로 분류한다. 32개를 넘으면 여러 배치로 처리한다. 분류 모델은 체크포인트에 저장된
클래스 순서와 크기를 사용하며, 기존 14클래스 YOLO의 라벨 순서를 가정하지 않는다.

- `--conf`: YOLO 탐지 기준, 기존 슬라이더와 동일
- `--classifier-conf 0.5`: 종류 확정 기준. 미달이거나 원본 밖으로 나가는 Crop이면 위치만 `알`로 표시
- `--classifier-batch 32`: 분류 배치 크기
- `--device cpu` / `--device cuda:0`: 탐지와 분류 실행 장치

오버레이 `D90/C85`는 탐지 90%, 종류 확률 85%다. 보정이 있는 추적 모드에서는 C에 과거/크기를 융합한다. 기존 개수/신뢰도 CSV의 conf는 탐지 확률을 유지한다.
`--log-counts`에서 분류 결과는 세션 폴더의 **classifications.jsonl**에 별도로 기록한다.
원본 프레임 저장 TXT에도 분류 확률을 추가한다.
종류 미확정 말이 있으면 게임 시작·판 동기화·착수 읽기를 막으며, 14종이 없는 모델은 로봇 옵션을 비활성화한다.
이 작업에서는 실제 로봇을 연결하거나 움직이지 않는다.

## 다른 분류 모델

`CropClassifier.predict(images)` 인터페이스로 탐지와 분류를 분리했다. MobileNet 외에
같은 전체 RGB Crop으로 학습한 Ultralytics Classification 가중치를 다음처럼 선택할 수 있다.

```bash
./janggi/realtime_ui/run.sh --model-mode hybrid \
  --classifier-backend ultralytics --classifier /경로/classification_best.pt --classifier-size 224
```

해당 어댑터는 0~1 RGB 전체 Resize 전처리를 적용한다. 학습 전처리도 일치해야 하며,
탐지용 `.pt`는 분류기로 사용할 수 없다. 이 문서의 학습 스크립트는 MobileNet 전용이다.


실시간 분류는 보정 파일이 있으면 기본 **0.2초 비동기 갱신**이다. 방해 중에는 분류·크기 이력 갱신을 보류하고,
말별 확률·이동/포획 상태는 `janggi/realtime_ui/states/`에 별도 저장한다. 자세한 기준·조절 옵션은
[실시간 UI 문서](../janggi/realtime_ui/README.md#02초-비동기-분류와-말별-상태)를 참고한다.
