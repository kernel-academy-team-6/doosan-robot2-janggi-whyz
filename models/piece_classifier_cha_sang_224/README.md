# 차·상 전용 보조 CNN

현재 사용자 요청으로 실시간 UI 연결은 제거했다. 이 모델과 아래 평가 결과는 실험 기록으로 보관한다.
UI는 기존 14종 CNN만 사용하며 차·상 재분류와 색 보정을 실행하지 않는다.

- 파일: `best.pt` (실시간용), `last.pt` (마지막 epoch).
- 구조: ImageNet 사전학습 MobileNetV3-Small, 입력 RGB 224×224, 출력 `cha`/`sang` 2종.
- 원본: `tools/datasets/janggi_pieces2/exports/224`. 기존 train/val/test 분할을 유지하고
  초·한의 차·상만 필터링한 뒤 진영을 합쳤다. 이미지 복제나 split 재추첨은 하지 않았다.
- 학습: NVIDIA RTX 4050 Laptop GPU, 25 epoch, 초기 5 epoch backbone 동결,
  batch 32, seed 42. Train 550장(55개 원본의 내보낸 파생 이미지), val 28장, test 28장.
- 검증 Macro F1이 처음 최고점에 도달한 epoch 7의 체크포인트를 `best.pt`로 선택했다.
- val: 28/28 (100%), test: 27/28 (96.43%), test macro F1 약 0.9642.
  test의 상 1장이 차로 오분류됐다.

작은 표본이며 사진 단위 분할이다. 동일 촬영의 유사 사진이 다른 split에 있으므로
새 조명·카메라·장기말·실시간 영상에 대한 일반화 성능으로 해석하지 않는다.

## 기존 14종 분류기에 결합

`piece_detection.refinement.RefinedClassifier`가 기존 CNN을 유지하면서 보조 CNN을 선택 호출한다.
차 또는 상이 종류별 1위이고 그 확률(초·한 합산)이 0.85 미만이며,
차+상의 전체 확률이 0.5 이상일 때만 추가 배치 추론한다.
차·상 조건부 확률은 기존 분포 25%, 보조 모델 75%의 로그 확률 혼합으로 결합한다.
차·상 전체 확률 및 각 진영의 차·상 합은 유지한다. 포·마 등 다른 종류 확률은 이 단계에서 바뀌지 않는다.

이후 말 중앙 반경 약 38%의 원형 영역에서 HSV 채도 64 이상·명도 30 이상의 색 픽셀을 조사한다.
빨강은 한, 파랑과 초록은 모두 초의 증거로 사용한다. 흰색·회색·갈색은 제외한다.
색 픽셀 수와 진영 색 우세도로 신뢰도를 낮추며, 증거가 부족하면 보정하지 않는다.
색은 각 종류의 초/한 비율에만 유한한 likelihood를 곱하고 재정규화한다.
종류별 확률 합은 보존하므로 빨간색이라는 이유만으로 말 종류를 바꾸지 않는다.
기본 색 강도 1.5에서 한쪽 multiplier는 최대 exp(1.5)이고 반대쪽은 최소 exp(-1.5)다.
이 점수들은 별도 확률 calibration을 수행한 결과는 아니다.

순서: 위치 YOLO → 기존 14종 CNN → 조건부 차·상 보조 CNN → 글자 색 진영 보정 → 기존 말 상태/크기 보정.
알/정상 말의 재분류 주기는 기존 설정을 유지한다. 보조 CNN은 요청된 Crop 중 조건을 만족하는 것만 처리한다.
`--model-mode hybrid` 또는 `cnn`에 적용하며, `full` YOLO 단독 경로에는 적용하지 않는다.

## 통합 평가

`pipeline_metrics.json`은 CPU에서 기존 전체 val/test 이미지로 비교한 결과다.
test는 파라미터 선택에 사용하지 않았다.

| 분할 | 기존 정확도 | 보조 CNN+색 보정 | 보조 CNN 호출 |
|---|---:|---:|---:|
| val (114장) | 111/114 = 97.37% | 112/114 = 98.25% | 1장 |
| test (110장) | 108/110 = 98.18% | 109/110 = 99.09% | 0장 |

val의 한 상→차 오류 1건을 수정했다. test의 한 사→초 사 오류 1건을 색 보정으로 수정했다.
test 진영 정확도는 109/110에서 110/110으로 변했다. 이 작은 평가에서의 개선이며 실시간 개선을 보증하지 않는다.

## 실험 재현

아래 독립 평가 스크립트는 실험용 RefinedClassifier를 사용한다.
현재 UI의 `--classifier`에는 기존 14종 모델을 사용하며, 이 2종 모델로 바꾸지 않는다.
이전에 추가했던 차·상/색 관련 UI 옵션은 제거했다.

재학습은 결과를 덮어쓰지 않도록 새 output 경로로 실행한다:

```bash
./tools/train_piece_classifier.sh \
  --dataset tools/datasets/janggi_pieces2/exports/224 \
  --output models/piece_classifier_cha_sang_224_new \
  --kind-only cha sang --size 224 --epochs 25 --freeze-epochs 5 \
  --batch-size 32 --device cuda:0

env -u PYTHONPATH janggi/.venv/bin/python tools/evaluate_classifier_refinement.py --device cpu
```

GPU 학습은 완료했다. GPU 통합 평가 요청은 자동 승인 검토 시간 초과로 실행되지 않았으며,
통합 평가는 CPU에서 완료했다. 로봇·카메라 연결 및 실제 로봇 이동은 하지 않았다.
