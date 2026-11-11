# 물리 모드 전환 검증

현재 구현은 말/로봇의 pose setter를 사용하지 않는다. 초기 배치만 SDF pose로 정하고,
그 이후에는 중력·충돌·마찰·관절 토크가 움직임을 결정한다.

- `runtime.json`: 독립 Gazebo 파티션의 headless 실험. 사용자 실행 중인 월드·로봇에는 명령을 보내지 않았다.
- 중형 말 3.8998g을 판 위 30mm에서 시작해 중력으로 안착시켰다.
- 안착 높이 오차 약 0.00088mm, 초기 교차점 대비 최종 위치 오차 약 0.0046mm.
- 관절 1 목표 0.25rad에 대한 실제 응답 약 0.25007rad. PID는 토크 모드이며 pose/velocity 강제 지정 없음.
- -0.08N 외력을 짧게 적용하고 제거했을 때 말이 약 12.98mm 움직였다.
- 32개 동적 말이 유지됨을 확인했다.
- 단위/회귀 테스트 153개 통과. 질량·관성·충돌체, 토크 PID 생성, 좌표 변경 API 차단 등을 포함한다.

질량은 실측이 아니다. 말 지름과 두께 5mm, 플라스틱 밀도 1,200kg/m³로 계산한 초기값이다.
소형 2.434g / 중형 3.900g / 궁 5.921g. 마찰 계수 0.8도 초기 가정이다.

완전한 로봇 집기·놓기 성공을 검증한 실험은 아니다. 보정된 TCP는 플랜지 Z +222mm인데
기본 RH-P12-RN-A 메시에는 그 지점까지의 실제 장착/연장 형상이 없다.
이 형상이 맞지 않으면 실제 접촉 집기는 실패하며, 가상 부착으로 성공처럼 만들지 않는다.

Gazebo API 근거:
- [JointPositionController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html): force PID, `use_velocity_commands=false`.
- [ApplyLinkWrench](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1ApplyLinkWrench.html): 외력 적용·해제.
