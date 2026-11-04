# Jazzy 호환성 테스트 보고서

테스트 환경: Ubuntu 24.04.5 LTS, ROS 2 Jazzy, x86_64

## 통과

- 공식 Doosan Jazzy `dsr_msgs2`에 Tool Flange Serial 서비스 4종 생성 및 컴파일
- `e0509_gripper_description` C++ 노드 3종 컴파일
  - `gripper_service_node`
  - `gripper_joint_publisher`
  - `gazebo_bridge`
- 공식 Doosan Jazzy `dsr_controller2`에 서비스 callback 이식 및 DRCF v3 링크
- E0509 + RH-P12 통합 xacro 생성
- 생성된 URDF를 `check_urdf`로 파싱
- `bringup.launch.py --show-args`를 ROS 2 Jazzy에서 로드
- 깨끗한 공식 Doosan Jazzy checkout에 배포 patch가 `git apply --check` 통과

## 의도적으로 채택하지 않은 구성

### 그리퍼 패키지만 공식 Jazzy 드라이버 위에 추가

다음 헤더가 공식 Jazzy `dsr_msgs2`에 없어 컴파일이 중단됐다.

```text
fatal error: dsr_msgs2/srv/flange_serial_open.hpp: No such file or directory
```

### `fhekwn549/doosan-robot2` Humble 포크 전체를 Jazzy에서 빌드

포크에는 `dsr_common2/lib/jazzy/.../libDRFL.a`가 없다. 공식 Jazzy DRFL을 임시 연결한 뒤에도 Humble용 `dsr_hardware2`가 Jazzy `ros2_control` API와 맞지 않아 컴파일되지 않았다.

## 아직 필요한 실제 장비 검증

- DRCF Docker 에뮬레이터와 전체 virtual launch
- 실제 E0509의 TCP 12345 연결
- Tool Flange Serial open/write/read/close
- RH-P12-RN-A 실제 open/close 및 0~700 stroke 방향

실제 장비 검증 전까지 저장소 상태는 “컴파일 및 정적 launch 검증 완료”다. 실제 장비 동작 완료로 과장하지 않는다.
