# cnn2 — 장기말 분류기 재학습 (2026-11-14)

기존 코드와 학습 스크립트는 수정하지 않았다. 이 폴더의 새 파일만 사용한다.

## 결과물

| 경로 | 내용 |
|---|---|
| `models/piece_classifier_cnn2/best.pt` | 14종 주 분류기. 기존 UI `--classifier`에 그대로 사용 |
| `models/piece_classifier_cnn2/expert_cha_sang/best.pt` | 차↔상 2클래스 보조 분류기 (진영 무관) |
| `models/piece_classifier_cnn2/expert_soldier_po/best.pt` | 졸/병↔포 2클래스 보조 분류기 |
| `models/piece_classifier_cnn2/bundle.json` | 주 분류기 + 보조 분류기 + 글자 색 진영 보정 묶음 |
| `models/piece_classifier_cnn2_preview/best.pt` | 230434 영상 + 기존 데이터만으로 학습한 중간 모델 |

## 사용

```bash
cd ~/doosan_janggi_ws/janggi
# 주 분류기만: 기존 실행 그대로
./realtime_ui/run.sh ... --classifier ../models/piece_classifier_cnn2/best.pt
# 보조 분류기 + 색 보정 번들: 새 실행기 (옵션은 run.sh와 동일)
../tools/cnn2/run_ui.sh ... --classifier ../models/piece_classifier_cnn2/bundle.json
```

`run_ui.sh`는 `app.py`를 수정 없이 실행하며, `--classifier`가 `.json`일 때만 계층형 분류기를 쓴다.

## 데이터

| 그룹 | 샘플 | 정답 출처 |
|---|---:|---|
| video_20261109 | 391 | 기존 카탈로그 `janggi_pieces` 확정 라벨 |
| photos_20261112 | 607 | 기존 카탈로그 `janggi_pieces2` 확정 라벨 |
| video_20261113_230434 | 19,648 (32말 × 614프레임) | 초기 배치. 위치·글자로 수동 확정 (`labels.json`) |
| video_20261113_230554 | 6,144 (32말 × 192프레임) | 대국 중 배치. 글자로 수동 확정, 경첩 오검출 1개 제외 |

두 영상은 말이 정지해 있어 위치 YOLO(`weights_only_piece.pt`) 검출을 위치로 묶어 트랙 단위로 라벨을 붙였다.

## 학습 방식 (`train.py`)

- 모델: MobileNetV3-Small (ImageNet 사전학습). UI 로더가 이 구조만 받으므로 유지했다.
- Crop을 매번 원본 프레임에서 실시간과 같은 `prepare_image`로 다시 추출하며 증강한다:
  회전 0~360°, 박스 크기 0.90~1.14배·위치 ±7%, 밝기 0.65~1.4, 대비 0.75~1.25, 채도 0.7~1.3,
  흐림 25%, 빛 반사 원형 패치 20%. 색상(hue)은 바꾸지 않는다(빨강/초록 진영 정보 보존).
- 그룹별 동일 비중 + 그룹 내 클래스 균형 샘플링. epoch당 8,000장, 24 epoch, batch 64,
  AdamW(lr 6e-4, wd 1e-4), 1 epoch warmup + cosine, label smoothing 0.05, AMP. 약 5분(RTX 4050).
- 고정 epoch로 저장한다. 평가 영상을 모델 선택에 쓰지 않는다.

## 평가 (`evaluate.py`, 프레임 단위)

| 시험 | 새 방식 | 기존 모델 |
|---|---:|---:|
| 230554 (학습에서 제외) | **100.00%** | dataset_real 97.85% (상→차 61건) |
| 230434 (학습에서 제외) | **100.00%** | dataset_real 98.15% (상→차 123, 졸→포 23) |
| 230434 / 230554 (두 영상 모두 제외, 기존 데이터만 학습) | 99.99% / 100% | — |
| video_20261109 (학습에서 제외) | **98.98%** | dataset_real 98.47% |
| photos_20261112 (학습에서 제외) | **95.55%** | piece_classifier 84.18% |

제외 시험은 해당 그룹을 빼고 같은 방식으로 학습한 별도 모델의 결과다. 최종 모델은 4그룹 모두로 학습했다.

## 계층형 번들 (`hierarchical.py`)

1. 주 분류기 14종 확률.
2. 최고 종류가 차/상(또는 졸/포)이고, 그 쌍의 확률 합 ≥ 50%, 최고 종류 확률 < 0.9일 때만 보조 분류기 실행.
   두 분포를 로그 공간에서 결합(보조 가중 0.7)하고 각 진영 안에서만 재분배한다. 진영은 바꾸지 않는다.
3. 기존 `glyph_colour_evidence`(중앙 글자의 빨강/초록 픽셀)로 같은 종류의 초/한 비율을 보정한다.

`bundle.json`에서 threshold/strength/colour_strength를 조정할 수 있다.

효과(photos_20261112를 주·보조 분류기 학습에서 모두 제외): 정확도 95.55% → 95.55%(동일),
진영 99.18% → 99.34%. 새 주 분류기에서는 차↔상, 졸↔포 혼동이 거의 사라져서 보조 분류기가 고칠 오류가 적다.
남은 오류는 졸→상·궁, 차→궁 등 쌍 밖이다. 기본 권장은 `best.pt` 단독이며, 번들은 선택 사항이다.

## 재현

```bash
cd ~/doosan_janggi_ws
janggi/.venv/bin/python tools/cnn2/extract_video_tracks.py tools/captures/video_20261113_*/color.mp4 --out tools/cnn2/videos
janggi/.venv/bin/python tools/cnn2/build_index.py
janggi/.venv/bin/python tools/cnn2/train.py --output models/새폴더 \
  --train-groups video_20261109 photos_20261112 video_20261113_23043 video_20261113_23055
janggi/.venv/bin/python tools/cnn2/evaluate.py models/새폴더/best.pt
```

새 영상을 추가하려면 `extract_video_tracks.py`로 트랙을 만들고 `labels.json`에 트랙별 정답을 넣은 뒤 `build_index.py`를 다시 실행한다.
정지 영상만 지원한다(말이 움직이면 트랙이 섞인다).
