# 중형·궁 사진 스킨 가림 수정

실행 중인 `/world/empty/scene/info`를 읽어 32개 말에 올바른 `photo_top`과 개별 PNG 경로가
등록되어 있음을 확인했다. RGB 카메라 한 프레임을 읽은 `before.png`에서는 소형만 사진이 보였다.
사용자 장면의 로봇·말 위치나 물리 상태는 수정하지 않았다.

원인은 `body_geometry()`가 충돌용 convex decomposition을 시각적 몸체에도 적용한 것이다.
근사된 몸체 윗면이 중형·궁의 사진 표면을 덮었다. 충돌 설정은 충돌체에만 사용하고,
눈에 보이는 몸체는 원본 `body.dae`를 그대로 사용하도록 분리했다.

- 사진 원본·글자·색·개별 말 PNG는 유지했다.
- 사진 높이는 기존 몸체 윗면 +0.1mm를 유지했다. 사진을 1mm 띄우는 우회는 적용하지 않았다.
- 실측 크기 21/27/33mm, 두께 6/7/8mm와 충돌·질량·관성은 유지했다.
- 별도 GZ_PARTITION의 Ogre2 headless 렌더에서 32개 사진을 모두 확인했다: `all_32_skins.png`.
- `skin_comparison.png`: 세 크기별 사진 높이 +0.1mm / +1mm 비교. 수정 후 두 경우 모두 보인다.
- robot_game 테스트 51개 통과. 시각 메시와 충돌 메시 설정을 구분하는 회귀 검사를 포함한다.

```bash
source /opt/ros/jazzy/setup.bash
python3 docs/analysis/piece_skin_fix/check_render.py --all-pieces
```

Python 장면 코드 수정이므로 colcon 재빌드는 필요 없다. 기존 Gazebo를 종료하고 같은 launch
명령으로 재실행하면 반영된다.
