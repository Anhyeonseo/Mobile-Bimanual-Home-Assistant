# 검증 도구

펌웨어와 하드웨어 없는 통합 검증 도구를 관리한다. 가사 작업 실행 코드는 `ros2_ws/src/home_robot_tasks`에 둔다.

| 진입점 | 역할 |
|---|---|
| `run/unified_pick.py` | 통합 제어기의 고정 집기 실행. 공통 준비 절차를 호출하고, 준비·파일 해석 실패 시에도 작업 프로세스를 정리 |
| `run/navigation_trial_web.py` | 주행 시험의 웹 화면·센서·실행 연결. 집기 코드가 의존하는 공통 라이브러리가 아님 |
| `run/prepare_unified_controller.py` | 통합 제어기 점검·명시적 준비 진입점. 공통 관측 절차는 `home_robot_tasks/platform_preparation.py`에 유지 |
| `run/fixed_pick_demo.py` | 카메라 없는 고정 위치 촬영: 오른팔 실측 자세 저장·무동작 검사·명시적 재생. [사용 절차](../docs/history/FIXED_PICK_DEMO.md) |
| `run/verify_offline.py` | PC 회귀·C 빌드·모의 실행·반복 시험·소스/의존성 해시 보고; `--stm32 --ros --bundle` 선택 |
| `run/soak_mobile_core.py` | Python host→실제 C 코어→모의 STS 버스의 가속 반복 시험 |
| `run/check_nav2_action_port.py` | PC 내부 합성 ROS 액션 서버의 성공·접수 전 취소, 실제 Nav2 planner 시험은 아님 |
| `run/check_mobile_model.py` | 조립체 모델 확장·단일 TF root·좌우 팔의 리프트 위치 변환 |
| `diagnostics/s2_coordinates.py` | S2 설치 목록 조회·고정 카메라 대응점 보정/독립 오차 검사. 장치 접근 없음. [S2 절차](../docs/history/HOME_ASSISTANT_S2.md) |
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
측정한 도구로 해석하지 않는다. [로드맵](../docs/ROADMAP.md)의 전체 시간·실제 성공·개입 평가에 맞춰 기존 기록·재생 경계를
재사용하며, 아직 존재하지 않는 실행 명령이나 모델 다운로드 절차를 완료 기능처럼 안내하지 않는다.

평가 기록은 요청/모델·프롬프트 revision, 영상/시각/TF/지도 revision, 수동 정답, 후보/선택·보류 이유,
실행 결과·개입·구간 시간을 연결한다. 같은 입력과 시간 예산에서 한 요소씩 바꿔 재생·실물 비교하고,
모의 성공·실영상 인식 성공·물리적 배달 성공을 별도 집계한다. 상세 기준은 별도 메인 문서 없이 로드맵에 둔다.

### STM32 S1 연결 진단

`diagnostics/stm32_link_check.py`는 식별된 구형 텔레옵 펌웨어의 INFO 전용 검사다. 포트 독점·기존 backend 잠금을 사용하며 첫 실패에서 중단한다. [S1 안내](../docs/history/HOME_ASSISTANT_S1.md)를 참고한다.

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
상세 순서는 [S1 기록](../docs/history/HOME_ASSISTANT_S1.md)을 따른다.

`run/diagnose_lift_link.py`는 리프트 통신 오류와 복구 및 같은 버스 모터의 비교 응답을
읽기 전용으로 기록한다. 전체 높이 측정을 반복하기 전 연결 장애를 구분하는 용도이며
엔코더-높이 환산이나 모터 구동을 수행하지 않는다.

`run/teleop_lift.py`는 별도 `lift-teleop-v4` 펌웨어를 사용하는 ID 10 전용 하강 도구다.
기본 실행은 INFO 조회이며, `--prepare`는 설정 백업 후 속도 모드를 준비한다.
`--check-direction`은 S 입력에만 반응하는 짧은 방향 검사이고,
`--run --direction-confirmed`는 S 유지 하강 / Space 정지 / B 최저점 저장 / X 종료다.
정지 시 토크를 유지하며 원래 모드로 자동 복구하지 않는다. 전체 조작은
[S1 리프트 텔레옵 절차](../docs/history/HOME_ASSISTANT_S1.md#2026-10-29-리프트-단독-텔레옵으로-측정-방식-변경)를 따른다.
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


## 하드웨어 없는 실행 예시

아래 명령은 저장소 루트에서 실행한다. 합성 센서·모의 장치·기록 재생의 성공은 실물 작업 성공이 아니다.


저장소 루트에서 실행한다. 센서·보드·모터 없이 PC에서만 실행한다.
주행 안전 검사까지 연결한 전체 작업은 `python3 tools/run/run_fetch_stack.py --guarded-navigation`으로 확인한다. 영상·공간·바퀴 피드백은 합성이다.

```bash
python3 -m venv .venv-host
source .venv-host/bin/activate
python -m pip install -r requirements/offline.lock.txt
python tools/run/verify_offline.py --bundle
```

예제 요청의 계획 출력:

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python -m home_robot_tasks.cli \
  --world config/home.example.json \
  --request config/fetch_remote.example.json
```

결과는 `plan_only`, `executable=false`다. 예제는 소파 탐색과 침대의 지정 영역을 의미 이름으로 표현하며 실측 지도 좌표는 없다. 손에 든 채 운반하는 경로의 자세·적재 한계는 실물 검증이 필요하다.

가상 성공 기록으로 15단계 재생:

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.replay_cli \
  --world config/home.example.json --request config/fetch_remote.example.json \
  --recording config/fetch_remote.replay.example.json
```

`COMPLETED`는 재생 완료만 의미한다. `physical_task_completed=false`,
`motion_commands=0`이며 센서·모터에 연결하지 않는다. 기록 형식과 실패 사례는
[시스템 구조](../docs/ARCHITECTURE.md#오프라인-작업-재생-계약)에 설명한다.

PC에서 가짜 장치로 지도 지점 이동 실행:

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.simulate_cli \
  --world config/home.example.json --map config/navigation_map.simulation.json \
  --request config/navigate_to.simulation.json
```

`--request config/fetch_remote.example.json`으로 15단계 배달도 실행한다.
`--cancel-at-s 0.1` 또는 `--fail-skill pick`으로 취소/실패를 재현한다.
결과는 simulation이며 실제 하드웨어 명령은 0개다. 지도 이동은 XY/yaw 경로를
따르는 기구학 모형이고, 배달의 집기 등은 합성 skill 결과다. 입체 점유 지도 UI와 인증 HTTP 서버는 PC 시뮬레이션에서 사용할 수 있다. 이 모의 실행의 성공으로 실측 지도·시맨틱 처리의 완료를 판정하지 않는다.

### 소파 재탐색을 PC에서 실행

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.search_simulator --config config/search_sofa.simulation.json --scenario alternate_found
```

정면 미발견→미확인 영역이 많은 다른 시점→정지 후 새 RGB-D→3D 후보 발견을
합성 입력으로 실행한다. `hidden`, `invalid_depth`, `stale_transform`,
`cancel_pending`도 지원한다. 이 예제 자체에는 실제 영상 인식·주행·팔 동작이 없다. 좌표는 집기 자세/충돌 검사 전 후보이고,
미발견 결과는 물체 부재를 증명하지 않는다.

### 앱 연결 서버를 PC에서 실행

```bash
mkdir -p output/gateway
python3 -c "import secrets,pathlib; p=pathlib.Path('output/gateway/token'); p.touch(mode=0o600,exist_ok=False); p.write_text(secrets.token_urlsafe(32))"
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.gateway \
  --world config/home.example.json --map config/navigation_map.simulation.json \
  --token-file output/gateway/token --database output/gateway/requests.db
```

첫 생성 이후에는 기존 토큰 파일을 유지한다. 기본 주소 `http://127.0.0.1:8765`에서
bearer 인증으로 지도/permit/요청/상태/취소를 제공한다. LAN 연결에는 `--bind`와
TLS 인증서/키가 필요하다. [API와 재연결 규칙](../protocol/README.md#http-경로와-실행)을 따른다.
서버 재시작은 미완료 작업을 INTERRUPTED로 기록하며 자동 재개하지 않는다.

통합 검증 보고서와 선택한 소스 묶음은 `output/offline-verification/`에 생성한다.
`--stm32`는 ARM toolchain으로 두 프로파일을 빌드하며 flash하지 않는다.
`--ros`는 ROS Jazzy·nav2_msgs·xacro·URDF parser 환경에서 모델/액션 시험을 추가한다.
1시간 반복 시험은 **가속한 모의 시간**이며 실제 장치의 1시간 운용 시험이 아니다.

펌웨어 공통 코어 시험:

```bash
cmake -S firmware/stm32_actuator -B build/stm32_actuator-host
cmake --build build/stm32_actuator-host
ctest --test-dir build/stm32_actuator-host --output-on-failure
```


### 전체 작업 틀


```bash
python3 tools/run/run_fetch_stack.py
python3 tools/run/run_fetch_stack.py --cancel-step 7
python3 tools/run/run_fetch_stack.py --fault sensor_loss
```

등록 장소→관측 위치/리프트→RGB-D 좌표→계획/작업 실행을 조립한 15단계 모의 실행이다.
센서·모터·집기 결과는 합성이며 실제 성공을 뜻하지 않는다. 설정 생성·ROS 증거 입력·
재시작 복구와 남은 실물 연결은 [현재 상태](../docs/CURRENT_STATUS.md)를 따른다.

### 실제 ROS 계획·주행의 PC 검사

ROS Jazzy와 MoveIt/Nav2 의존성 및 새 증거 메시지를 빌드한 환경에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src --packages-select so101_interfaces so101_arm_bridge home_robot_tasks so101_description
source install/setup.bash
```

```bash
export PYTHONPATH=ros2_ws/src/home_robot_tasks:ros2_ws/src/so101_arm_bridge:$PYTHONPATH
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=173
python3 tools/run/verify_offline.py --ros --moveit --nav2 --stm32 --bundle
```

Nav2·AMCL의 합성 라이다 주행, 인증 HTTP 요청/취소·scan/TF 손실 정지, Python↔C 바퀴 피드백과 양팔
MoveIt IK/충돌/계획을 검사한다. 실제 장치는 열지 않는다. MoveIt 종료 문제의
프로세스 한정 우회를 포함하며 의존성과 범위는 [시스템 구조](../docs/ARCHITECTURE.md)를 따른다.
