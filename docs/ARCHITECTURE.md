# 제어 구조

이 저장소는 공식 Doosan ROS 2 Jazzy 드라이버를 기준으로 RH-P12-RN-A 그리퍼 제어에 필요한 Tool Flange Serial 서비스만 추가한다.

```text
사용자 명령 / 애플리케이션
  ├─ 팔: /dsr01/motion/* 서비스
  └─ 그리퍼: /dsr01/gripper/open, close, position_cmd
                 │
                 ▼
e0509_gripper_description/gripper_service_node
                 │  Modbus RTU 프레임
                 ▼
/dsr01/gripper/flange_serial_{open,close,write,read}
                 │
                 ▼
dsr_controller2 → DRFL → E0509 Tool Flange Serial → RH-P12-RN-A
```

가상 모드에서 팔은 DRCF Docker 에뮬레이터와 TCP 12345로 통신한다. Tool Flange Serial은 실제 플랜지 하드웨어가 필요하므로 가상 모드에서는 서비스가 실패하는 것이 정상이다. RViz의 그리퍼 조인트 표시는 별도 joint publisher가 담당한다.

## 왜 Humble 포크 전체를 쓰지 않는가

`fhekwn549/doosan-robot2`의 Humble 브랜치는 그리퍼 서비스가 포함되어 있지만 Jazzy의 `ros2_control` API와 호환되지 않는다. 이 환경은 공식 Doosan Jazzy 소스를 유지하고 네 개의 시리얼 서비스만 최소 이식한다.
