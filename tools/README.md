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

## 새 방향의 평가 도구 범위

현재 회귀/재생 도구는 통신·기하·상태 전환의 근거다. VLM 인식이나 자동 시맨틱 매핑의 성능을
측정한 도구로 해석하지 않는다. [로드맵](../docs/ROADMAP.md)의 U1/M1/Q1/E1에 맞춰 기존 기록·재생 경계를
확장할 예정이며, 아직 존재하지 않는 실행 명령이나 모델 다운로드 절차를 완료 기능처럼 안내하지 않는다.

평가 기록은 요청/모델·프롬프트 revision, 영상/시각/TF/지도 revision, 수동 정답, 후보/선택·보류 이유,
실행 결과·개입·구간 시간을 연결한다. 같은 입력과 시간 예산에서 한 요소씩 바꿔 재생·실물 비교하고,
모의 성공·실영상 인식 성공·물리적 배달 성공을 별도 집계한다. 상세 기준은 별도 메인 문서 없이 로드맵에 둔다.

## D415 실물 RGB-D 실험

[RealSense 안내](../docs/REALSENSE.md): 독립 뷰어, Grounding DINO 키워드 검색, SAM 마스킹과 카메라 좌표를 확인한다.
`bash tools/run/realsense_remote_viewer.sh near`로 Jetson의 RGB·깊이·검색 결과를 PC 브라우저에서 본다.
이 도구의 좌표는 RGB 카메라 기준 표면점이며 로봇/작업대 TF, 파지 자세, ROS 실행기와 아직 연결하지 않는다.
