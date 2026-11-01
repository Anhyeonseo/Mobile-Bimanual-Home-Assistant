# 검증 도구

펌웨어와 하드웨어 없는 통합 검증 도구를 관리한다. 가사 작업 실행 코드는 `ros2_ws/src/home_robot_tasks`에 둔다.

| 진입점 | 역할 |
|---|---|
| `run/fixed_pick_demo.py` | 카메라 없는 고정 위치 촬영: 오른팔 실측 자세 저장·무동작 검사·명시적 재생. [사용 절차](../docs/FIXED_PICK_DEMO.md) |
| `run/verify_offline.py` | PC 회귀·C 빌드·모의 실행·반복 시험·소스/의존성 해시 보고; `--stm32 --ros --bundle` 선택 |
| `run/soak_mobile_core.py` | Python host→실제 C 코어→모의 STS 버스의 가속 반복 시험 |
| `run/check_nav2_action_port.py` | PC 내부 합성 ROS 액션 서버의 성공·접수 전 취소, 실제 Nav2 planner 시험은 아님 |
| `run/check_mobile_model.py` | 조립체 모델 확장·단일 TF root·좌우 팔의 리프트 위치 변환 |
| `diagnostics/s2_coordinates.py` | S2 설치 목록 조회·고정 카메라 대응점 보정/독립 오차 검사. 장치 접근 없음. [S2 절차](../docs/HOME_ASSISTANT_S2.md) |
| `run/validate_protocol_manifest.py` | 기존 펌웨어 메시지 ID·필수 명령 검사 |
| `setup/firmware/generate_protocol_header.py --check` | manifest와 생성된 C 헤더 일치 검사 |
| `setup/firmware/generate_joint_limits.py --check` | 관절 JSON·생성 C 표·ROS 설정 사본 검사; `--write`로 사본 갱신 |
| `setup/firmware/generate_development_ranges.py --check` | 새 양팔 연속 좌표 프로필·검증용 C 헤더·ROS 사본 검사; `--write`로 생성. 실제 구동 허가 없음 |
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

### STM32 S1 연결 진단

`diagnostics/stm32_link_check.py`는 식별된 구형 텔레옵 펌웨어의 INFO 전용 검사다. 포트 독점·기존 backend 잠금을 사용하며 첫 실패에서 중단한다. [S1 안내](../docs/HOME_ASSISTANT_S1.md)를 참고한다.

`diagnostics/stm32_device_inventory.py`는 현재 S1 조회 펌웨어용이다. 기본은 제어기 상태만,
`--read-motors`는 양팔·바퀴·리프트의 고정 레지스터를 읽는다. 모터 설정·이동 명령은 없다.
지원 버전·주소·실패 표현은 [장치 조회 계약](../docs/DEVICE_INVENTORY_PROTOCOL.md)에 명시한다.

`run/calibrate_arm_home.py`는 보정 전용 `arm-calibration-v1` 이미지에서 한 팔씩 기준 자세를
2048로 보정한다. 기본은 읽기 백업이며 실제 보정은 `--center`로 명시한다.
현재 동작용/조회용 이미지에서 실행하지 않는다. [양팔 보정 절차](../docs/ARM_HOME_CALIBRATION.md)를 참고한다.

`setup/firmware/rebase_arm_home.py`는 양팔 보정·전원 재인가 증거를 확인하고 과거 범위를
새 영점으로 조건부 환산한다. 모터 I/O 없이 검토용 JSON만 생성하며, 기존 실행용 범위는
변경하지 않는다. [현재 환산 결과와 남은 확인](../docs/ARM_HOME_CALIBRATION.md#새-home-기준-범위-환산)을 참고한다.

`run/measure_arm_range.py`는 같은 유지보수 이미지에서 한 관절의 손측정 경로를 연속으로
읽는다. 현재 자세에서 시작하고 다른 관절 조정을 허용한다. Home 복귀 없이 양쪽 사용 끝점 측정 후 stdin에
`finish`를 입력한다. `abort`는 기록을 미완료로 남긴다. 모터 쓰기는 없으며 결과는 검토용이다.


### S1 리프트 수동 측정

`run/measure_lift_range.py capture`는 읽기 전용 점검 펌웨어의 ID 10만 연속 조회한다.
손으로 조절한 `low`/`high` 표식과 원본 기록을 보존하며 모터 쓰기는 없다.
`finalize`는 동일 지점의 실측 최저/최고 높이(mm)를 받아 높이 환산 파일을 만든다.
기준점은 세션 한정이며 자동 원점 복귀나 전원 재인가 후 높이 복원을 승인하지 않는다.
상세 순서는 [S1 기록](../docs/HOME_ASSISTANT_S1.md)을 따른다.

`run/diagnose_lift_link.py`는 리프트 통신 오류와 복구 및 같은 버스 모터의 비교 응답을
읽기 전용으로 기록한다. 전체 높이 측정을 반복하기 전 연결 장애를 구분하는 용도이며
엔코더-높이 환산이나 모터 구동을 수행하지 않는다.

`run/teleop_lift.py`는 별도 `lift-teleop-v4` 펌웨어를 사용하는 ID 10 전용 하강 도구다.
기본 실행은 INFO 조회이며, `--prepare`는 설정 백업 후 속도 모드를 준비한다.
`--check-direction`은 S 입력에만 반응하는 짧은 방향 검사이고,
`--run --direction-confirmed`는 S 유지 하강 / Space 정지 / B 최저점 저장 / X 종료다.
정지 시 토크를 유지하며 원래 모드로 자동 복구하지 않는다. 전체 조작은
[S1 리프트 텔레옵 절차](../docs/HOME_ASSISTANT_S1.md#2026-10-29-리프트-단독-텔레옵으로-측정-방식-변경)를 따른다.
모든 쓰기/조작 모드에 새 `--file`을 지정해야 하며 기존 기록을 덮어쓰지 않는다.

`teleop_lift.py --link-check`는 모터 I/O 없이 지연 중 연속 UART 수신을 검사한다.
`--idle-check`는 명시적 전원/지지 확인 후 토크 ON·속도 0으로 100개 응답과 정지를
검사한다. 두 모드 모두 `--file`이 필요하고 하강 명령은 보내지 않는다.

짧은 방향 검사를 육안으로 판단하기 어려우면 `--check-direction --check-duration 2`를
사용한다. 방향 검사에서만 S 한 번으로 같은 저속 동작을 시작하고 2초 뒤 종료한다.
중간 정지는 Space/X이며, 키 반복에 의존하지 않는다. 전체 하강 run의 입력 lease는 유지한다.
이는 호스트 타이머와 MCU 통신 watchdog을 함께 사용하며 자동 전체 하강이 아니다.

## D415 실물 RGB-D 실험

[RealSense 안내](../docs/REALSENSE.md): 독립 뷰어, Grounding DINO 키워드 검색, SAM 마스킹과 카메라 좌표를 확인한다.
`bash tools/run/realsense_remote_viewer.sh near`로 Jetson의 RGB·깊이·검색 결과를 PC 브라우저에서 본다.
이 도구의 좌표는 RGB 카메라 기준 표면점이며 로봇/작업대 TF, 파지 자세, ROS 실행기와 아직 연결하지 않는다.
