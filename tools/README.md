# 검증 도구

펌웨어와 하드웨어 없는 통합 검증 도구를 관리한다. 가사 작업 실행 코드는 `ros2_ws/src/home_robot_tasks`에 둔다.

| 진입점 | 역할 |
|---|---|
| `run/verify_offline.py` | PC 회귀·C 빌드·모의 실행·반복 시험·소스/의존성 해시 보고; `--stm32 --ros --bundle` 선택 |
| `run/soak_mobile_core.py` | Python host→실제 C 코어→모의 STS 버스의 가속 반복 시험 |
| `run/check_nav2_action_port.py` | PC 내부 합성 ROS 액션 서버의 성공·접수 전 취소, 실제 Nav2 planner 시험은 아님 |
| `run/check_mobile_model.py` | 조립체 모델 확장·단일 TF root·좌우 팔의 리프트 위치 변환 |
| `run/validate_protocol_manifest.py` | 기존 펌웨어 메시지 ID·필수 명령 검사 |
| `setup/firmware/generate_protocol_header.py --check` | manifest와 생성된 C 헤더 일치 검사 |
| `setup/firmware/generate_joint_limits.py --check` | 관절 JSON·생성 C 표·ROS 설정 사본 검사; `--write`로 사본 갱신 |
| `setup/firmware/validate_phase0.py` | 수동 기록한 하드웨어 기본 측정값 검사 |
| `lib/actuator_protocol.py` | 보존 펌웨어의 v1 codec 단위 시험 지원 |

`hardware/phase0_baseline.json`은 빈 측정 양식이며 검증 완료 자료가 아니다. 예전 접기·캔 집기·카메라 보정·실물 자세 반복 도구는 현재 실행 경로에서 제거했다.

휴대폰 지도 좌표 시험에는 Node.js가 필요하다. Python/ROS 개발 의존성과 별개이며, CI는 Node.js 22를 설치한다. 실제 서버 UI는 추가 JavaScript 패키지나 외부 CDN 없이 동작한다.
