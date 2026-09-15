# ALOHA Mini Home Robot

ALOHA Mini 1 기반의 자취방·소형 주거공간용 모바일 매니퓰레이터다. 첫 목표는 **“거실 소파에서 리모컨을 찾아 침대에 내려놓기”**를 반복해서 수행하는 것이다.

기존 [Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)의 Classical P&P를 재사용한다. 해당 기반은 상단 카메라 인식부터 왼팔 집기, 오른팔 전달, 내려놓기까지 실기 완료한 시스템이다. 모바일 베이스와 리프트로 팔이 작업하기 좋은 상대 위치·높이를 재현하고, 새 장착 조건에 맞춰 통합한다.

## 개발 방향

- **우선 Classical 완성:** Nav2 이동, RGB-D 인식·미세 정렬, 리프트, 기존 P&P, 작업 상태 머신을 단계적으로 연결한다.
- **이후 VLA 확장·비교:** 리더암 시연으로 영상·관절 상태·동작 데이터를 수집하고 VLA를 모방학습한다. 같은 작업 조건에서 Classical과 성능을 비교한다.
- 초기 VLA의 제어 범위는 정지한 베이스·리프트 위의 팔과 그리퍼다. 주행과 리프트는 공통 실행 계층에 남긴다.
- 알려진 평탄한 실내와 가벼운 강체부터 시작한다. 여러 가사 작업과 변형체는 후속 확장이다.

## 대표 동작

```text
요청 → 소파 근처 주행 → 정지·리모컨 인식 → 베이스 미세 정렬·정지
     → 리프트 조절·정지·재인식 → 집기·집기 확인 → 운반 자세
     → 침대 근처 주행 → 정지·놓을 면 확인 → 필요 시 미세 정렬·정지
     → 리프트 조절·정지·재인식 → 놓기·배달 확인
```

Nav2 도착만으로 집기를 시작하지 않는다. 베이스·리프트·팔 조작은 순차 실행하며, 주행 중에는 팔과 리프트를 검증된 운반 상태로 유지한다. 양팔 플랫폼이지만 모든 물건을 양팔로 집거나 손 사이에서 전달할 필요는 없다.

## 현재 상태

현재 저장소에는 STM32 팔 펌웨어, 모바일 명령/응답·리프트·공유 버스·정지 C 코어, 미실측 조립체 모델과 작업 실행기가 있다. 인증 HTTP 서버에서 지도 지점 이동·상태·취소를 PC로 시험하고, 별도로 Python→C 코어→모의 STS 버스 통신을 반복 검증한다. 기존 P&P의 경로·그리퍼 방향 계산과 RGB-D 좌표 검사를 이식했다. 팔·모바일 host framing과 공유 UART 소유권을 통합했으며, 실제 Nav2·AMCL→공통 브릿지→C 바퀴 모형과 MoveIt 계획을 PC에서 검사했다. 모바일 주기 DMA 송신·모드/상태 수신기를 보드 루프에 연결하고 fake HAL로 시험했다. 실측 설정/모드 변경·전체 hold와 인식/집기 연결은 남아 있어 [로드맵](docs/ROADMAP.md)에 명시했다.

플랫폼은 ALOHA Mini 1로 확정했고 부품 출력·배송에 약 2주가 남아 있다(2026-09-15 기준). **LDS-01 2대는 Nav2 주행**, 리프트에 장착할 **D415는 정지 후 집기용 RGB-D 관측**에 사용한다. 기존 D435에서 D415로 변경하는 방향이며 실제 센서 수령·보정은 미확인이다. 보유한 **Jetson Orin Nano Devkit / Raspberry Pi 5 중 한 대**를 사용하며 로컬 영상·VLM 추론을 고려해 Jetson을 우선 검토한다. IMU는 보유 목록에 없다. [도착 전 준비 계획](docs/ROADMAP.md)에 소프트웨어 구현 범위, 보드 선택, 센서 시험과 도착 후 순서를 정리했다.

펌웨어의 통신 공통화, 왼쪽 10개·오른쪽 6개 모터 구성과 모바일 동작 범위 정책은
[펌웨어 점검](docs/ARCHITECTURE.md)에 정리했다. 바퀴·리프트 패킷 구성까지
구현했고 공유 버스 중재·명령/피드백 timeout·리프트·전체 정지 코어도 시험했다. 기존 팔 서비스/피드백/DMA를 공통 보드 전송 계층으로 옮기고 모바일 대기열과 연결했다. 모바일 주기 writer와 host parser handler는 구현했고 기본 설정은 미연결이다. 장치 READ→응답 검증→4축 feedback 경로도 연결했다. 실측 설정의 부팅 초기화·모드 변경·전체 정지/하중 유지 연결은 남아 있다.

## 하드웨어 없는 확인

저장소 루트에서 실행한다. 센서·보드·모터 없이 PC에서만 실행한다.

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
따르는 기구학 모형이고, 배달의 집기 등은 합성 skill 결과다. 앱 UI/3D 지도 생성은
후속 개발이며 인증 HTTP 서버는 지금 사용할 수 있다.

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

## 실제 ROS 계획·주행의 PC 검사

ROS Jazzy와 MoveIt/Nav2 의존성을 불러온 환경에서:

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
