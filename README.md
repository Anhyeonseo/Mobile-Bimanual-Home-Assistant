# Mobile Bimanual Home Assistant

ALOHA Mini 1 기반으로, **사람의 자연어 요청을 이해하고 집 안을 이동하며 물건을 집어 옮기는 홈 어시스턴트**를 개발한다.
예: “거실 탁자의 검은 마커펜을 침실 협탁에 놓아줘.”

**2026-11-05 현재 합의한 개발 범위의 S2 좌표·관측 연결과 정지 상태 실물 통합 검사를 완료했다. 공식 ALOHA Mini 1 + SO-ARM101 규격과 설치 보정을 분리하고 D415·양팔·리프트의 촬영 시각 좌표 계산을 확인했다. 실물 자율주행·집기 전체 과정은 아직 검증하지 않았다.**
기존 텔레옵·저장 자세 집기 촬영과 자율 작업 검증을 구분한다.

| 단계 | 현재 상태 |
|---|---|
| S0 설치 조사 | 기본 조사·저장소/배포 구조 정리 완료. LDS-01 거리 수신 확인, 간헐적 수신 공백과 장착·주행 검증은 남음 |
| S1 장치 제어 | 16개 모터 조회, 양팔 Home 2048 보정·유지, 관절 범위 결정, 리프트 약 90→26cm 측정 확보. 공통 동작·원점 복원·반복성 실물 검증은 남음 |
| **S2 좌표와 관측** | **개발 범위 통합 검사 완료.** 공식 기구 모델·저장한 양팔 Home·리프트 기준·고정 카메라 연결. 13축＋RGB-D 동시 관측과 촬영 시각 좌표 60회 검증. 카메라/TCP 작업 정확도는 S3에서 확인 |
| **S3 관측 기반 집기** | **2026-11-11 사용자 수용으로 완료.** 새 RGB-D 기반 리모컨 집기·들기·유지 성공1회, 접근 중 탁자 끌림 없음. 약8mm 접촉 위치 오차 기록; 반복 검증은 후속 범위. [완료 기록](docs/S3_STATIONARY_LAB.md#2026-11-11--s3-완료-판정) |
| S4a 주행·의미 지도 / S4b 모바일 조작 | 합성/기록 기반 기반 기능 구현. 실물 통합 미완료 |
| S5 자연어 전체 작업 | 의미 검증·앱·모의 작업 연결 구현. 외부 언어 API와 실물 전체 수행 미완료 |

리프트 하강 소음은 사용자 결정으로 최종 실물 점검 이슈에 남겨 S2 개발을 진행한다.

## 시스템 동작

```text
텍스트 요청 → 의미 추출·모호한 요청 확인 → 공간·물체 기억 조회
→ 관측 위치로 이동 → 물체 확인 → 베이스 정차 위치·리프트 높이 선택
→ 가까이서 다시 관측 → 집기·성공 확인 → 운반 → 놓기·결과 확인
```

첫 스캔 앱은 지도 생성과 검토를 돕는다. RGB-D에서 본 침대·소파·책상 등의 의미와
높이별 장애물을 2D LiDAR 지도에 연결한다. 라이다에 침대 다리만 보이더라도 RGB-D에서
확인한 몸체를 **지도 생성·저장 단계부터** 이동 불가 영역에 반영하는 방향이다.
초기 스캔을 텔레옵으로 할지 자동으로 할지는 S4a 개발 시 결정한다.

## 계층과 현재 상태

| 영역 | 역할 | 현재 확인 수준 |
|---|---|---|
| STM32·장치 통신 | 양팔·베이스·리프트 제어와 상태·정지 관리 | 기존 개별 시연 성공, 공통 제어 실물 통합 필요 |
| 위치·지도·인지 | 위치 추정, 가구·물체 기억, 높이 장애물 반영 | 합성/기록 자료로 지도 생성·저장·재로딩 검사, 라이브 연결 필요 |
| 이동·조작 계획 | 정차 위치·높이·팔 동작 후보와 충돌 검토 | 소프트웨어 경계와 후보 선택 구현, 실측 보정 필요 |
| 작업 실행 | 이동·집기·놓기 순서, 취소·실패·재관측 관리 | 공통 실행기 및 PC 회귀 검사 |
| 자연어·앱 | 요청 의미 추출, 질문·상태·스캔 검토 | 개발 앱과 의미 검증 구현. 외부 텍스트 API 클라이언트는 미연결 |

현재 장비는 Jetson Orin Nano Super, STM32G474, 양팔, 모바일 베이스·리프트,
RealSense D415 및 LDS-01이다. **LDS-01은 케이블이 없어 아직 연결 검사 전**이다.
DINO·SAM은 기존 인식/마스킹 출발점이며, 최종 인식 엔진은 실제 장면과 Jetson 부하를 비교해 결정한다.
자연어 모델이 모터 명령을 직접 만드는 구조는 사용하지 않는다.

## 먼저 실행하기

모바일 브랜치의 저장소 루트에서 Python 3.12로 실행한다. 아래 데모·앱은 모의 장치를 사용한다.

```bash
python3 -m venv .venv-host
source .venv-host/bin/activate
python -m pip install -r requirements/offline.lock.txt
python tools/run/home_assistant.py demo
python tools/run/home_assistant.py demo --fail-skill pick
python tools/run/home_assistant.py serve
```

앱: `http://127.0.0.1:18766/assistant`. 접속 키는 로컬 `output/home-assistant/access-key.txt`에 생성된다.
키·장치별 설정·촬영 자료·실측 로그는 Git에 넣지 않는다. 모델 다운로드나 모터 동작은 위 명령으로 시작되지 않는다.

## 개발 순서와 문서

**S0 설치 조사 → S1 공통 제어 → S2 좌표 보정 → S3 고정 베이스 관측 집기 → S4 지도·주행·모바일 집기 → S5 자연어 전체 작업**

- [시스템 요구사항·계층별 책임](docs/MOBILE_MANIPULATION_DESIGN.md)
- [구현 범위·검사 결과·남은 연결](docs/HOME_ASSISTANT_IMPLEMENTATION.md)
- [S0 설치 조사 요약](docs/HOME_ASSISTANT_S0.md)
- [S1 공통 제어 연결](docs/HOME_ASSISTANT_S1.md)
- [S2 좌표와 관측 연결](docs/HOME_ASSISTANT_S2.md)
- [S2 완료 결과·배포·남은 운용 과제](docs/S2_HANDOFF.md)
- [D415 방 스캔 촬영·파일 전달](docs/ROOM_SCAN_CAPTURE.md)
- [인지·지도 기술 선택 기준](docs/PERCEPTION_MAPPING_DECISIONS.md)
- [파일 구조·GitHub 업로드·젯슨 배포 관리](docs/REPOSITORY_MANAGEMENT.md)

아래 도구 안내와 기존 [아키텍처](docs/ARCHITECTURE.md)·[상태 기록](docs/CURRENT_STATUS.md)·
[초기 로드맵](docs/ROADMAP.md)은 기반 구현의 상세 자료다. 과거 하드웨어 구성이나 실기 성공을
현재 통합 시스템의 완료 상태로 해석하지 않는다.

## 하드웨어 없는 확인

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
[시스템 구조](docs/ARCHITECTURE.md#오프라인-작업-재생-계약)에 설명한다.

PC에서 가짜 장치로 지도 지점 이동 실행:

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.simulate_cli \
  --world config/home.example.json --map config/navigation_map.simulation.json \
  --request config/navigate_to.simulation.json
```

`--request config/fetch_remote.example.json`으로 15단계 배달도 실행한다.
`--cancel-at-s 0.1` 또는 `--fail-skill pick`으로 취소/실패를 재현한다.
결과는 simulation이며 실제 하드웨어 명령은 0개다. 지도 이동은 XY/yaw 경로를
따르는 기구학 모형이고, 배달의 집기 등은 합성 skill 결과다. 입체 점유 지도 UI와 인증 HTTP 서버는 PC 시뮬레이션에서 사용할 수 있다. 실측 지도 연결과 시맨틱 지도 자동 생성은 후속 개발이다.

### 소파 재탐색을 PC에서 실행

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.search_simulator --config config/search_sofa.simulation.json --scenario alternate_found
```

정면 미발견→미확인 영역이 많은 다른 시점→정지 후 새 RGB-D→3D 후보 발견을
합성 입력으로 실행한다. `hidden`, `invalid_depth`, `stale_transform`,
`cancel_pending`도 지원한다. 실제 영상 인식·주행·팔 동작은 없으며 전체 fetch
실행기에 아직 연결하지 않았다. 좌표는 집기 자세/충돌 검사 전 후보이고,
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
TLS 인증서/키가 필요하다. [API와 재연결 규칙](protocol/README.md#http-경로와-실행)을 따른다.
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

## 구성과 문서

| 경로 | 역할 |
|---|---|
| `firmware/` | 기존 STM32 제어기와 독립 C 코어 |
| `ros2_ws/src/so101_arm_bridge/` | 팔 명령·피드백·정지 브릿지 |
| `ros2_ws/src/so101_interfaces/` | 양팔 명령·피드백 ROS 메시지 |
| `ros2_ws/src/so101_description/` | SO101 팔 기구학과 형상 |
| `ros2_ws/src/home_robot_tasks/` | 가사 작업 요청·15단계 계획·오프라인 이벤트 재생 |
| `config/`, `hardware/` | 관절 한계, 예제 요청, 하드웨어 참조 |
| `protocol/`, `tools/`, `tests/` | 통신 규격, 검증 도구, 회귀 시험 |

[현재 상태](docs/CURRENT_STATUS.md) · [로드맵](docs/ROADMAP.md) · [시스템 구조](docs/ARCHITECTURE.md) · [팔 브릿지 사용](ros2_ws/src/so101_arm_bridge/README.md)

이전 수건 접기 개발은 [SO101-Towel-Folding](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80)에 보관한다.

## 전체 작업 틀을 하드웨어 없이 실행

```bash
python3 tools/run/run_fetch_stack.py
python3 tools/run/run_fetch_stack.py --cancel-step 7
python3 tools/run/run_fetch_stack.py --fault sensor_loss
```

등록 장소→관측 위치/리프트→RGB-D 좌표→계획/작업 실행을 조립한 15단계 모의 실행이다.
센서·모터·집기 결과는 합성이며 실제 성공을 뜻하지 않는다. 설정 생성·ROS 증거 입력·
재시작 복구와 남은 실물 연결은 [현재 상태](docs/CURRENT_STATUS.md)를 따른다.

## 실제 ROS 계획·주행의 PC 검사

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
프로세스 한정 우회를 포함하며 의존성과 범위는 [시스템 구조](docs/ARCHITECTURE.md)를 따른다.

## License

자체 작성 코드와 문서는 [Apache License 2.0](LICENSE)을 따른다. 로봇 모델과 STM32 구성 요소의 조건은 [제3자 고지](docs/THIRD_PARTY_NOTICES.md)에 정리했다.

### 휴대폰용 지도 화면 (PC 시뮬레이션)

`robot_gateway` 실행 후 같은 주소의 `/`를 열면 입체 점유 지도에서 바닥을 선택하고 경로 확인·이동·취소할 수 있다. Python 모듈로도 실행할 수 있다.

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks:ros2_ws/src/so101_arm_bridge python3 -m home_robot_tasks.gateway \
  --world config/home.example.json --map config/navigation_map.simulation.json \
  --token-file /절대/경로/token --database /절대/경로/requests.db
```

기본 주소는 `http://127.0.0.1:8765`이며 token 파일에는 32자 이상의 무작위 접속 키를 넣는다. 화면은 키를 저장하지 않는다. 휴대폰의 LAN 접근에는 `--bind`, `--cert`, `--key`, 정확한 `--public-origin https://호스트:포트`를 함께 지정한다. 현재 화면과 gateway는 시뮬레이션용이다.

## 2026-09-16 주행 구조 재점검

Mini 1의 공식 소스 리비전을 고정해 Mini 2 기본값과 분리했다. 원 시각을 보존하는 `TwistStamped → 바퀴 명령/실측 odometry` 재사용 브릿지, native depth의 3D 점유→2D 주행 격자, 관측되지 않은 정지 거리 차단, 비동기 정지 후 지도 교체와 카메라 3대의 후보 TF를 추가했다. 실제 DDS/C 모형 및 기존 회귀를 검증한다. 상세 연결·검증 한계는 [구조](docs/ARCHITECTURE.md#2026-09-16-주행센서지도-재점검), 남은 기구/실측 확인은 [로드맵](docs/ROADMAP.md)을 따른다.

한 대의 D415로 전신·전방 근거리·옆/뒤의 모든 공간을 볼 수 있다고 가정하지 않는다. `python3 tools/run/check_camera_coverage.py`로 장착 후보의 기하학적 사각을 먼저 확인한다. 예시 치수나 합성 성공은 실물 주행 승인 근거가 아니다.
