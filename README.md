# Mobile Bimanual Home Assistant

[처음 실행하는 팀원을 위한 리모컨 집기 튜토리얼](docs/pick_station_tutorial.md) · 노트북 준비부터 Jetson 무장치 예행연습, 첫 실기 점검까지.

ALOHA Mini 1 기반으로 사용자의 요청에 따라 집 안에서 물건을 찾아 집고, 지정한 곳에 옮기는
홈 어시스턴트를 개발한다. **최종 목표는 온디바이스에서 전체 작업을 정확히 끝내고,
사용자의 대기 시간과 중간 개입을 줄이는 것**이다.

예: “책상 위 리모컨을 침대의 지정한 곳으로 옮겨줘.”

## 현재 방향

2026-11-24 기준, 자율주행·지도 갱신과 고정 위치의 흰색 지우개 집기는 실물 확인 이력이 있다.
Pick Station은 촬영→탐지·분할→GraspNet 후보→MoveIt→실행을 연결하며, 실물 성공은 사용자 확인으로 기록한다.
11월 20일 성공 설정과 이후 변경된 높이 모델·계획 코드는 [현재 상태](docs/CURRENT_STATUS.md)에서 구분한다.
개입 없는 이동→집기→운반→놓기 전체 작업과 최종 온디바이스 구성은 아직 미완료다.

개발은 **수납장 앞 고정 위치 자동 집기 → 좌우 이동 후 집기 → 지정 목적지까지 전체 작업**
순서로 진행한다. TF·캘리브레이션, 5DOF·고정 손가락 집기, 베이스 정차, 오른팔 통신을
전체 성공률·시간·개입 관점에서 개선한다. 실제 하드웨어의 측정 편차와 구동 반복성에 맞는
허용 오차를 사용하며 계산 정밀도를 실물 성능으로 표현하지 않는다.

| 읽을 문서 | 내용 |
|---|---|
| [개발 로드맵](docs/ROADMAP.md) | 합의한 목표, 해결할 문제, 현실적인 오차·평가 기준과 개발 순서 |
| [현재 상태](docs/CURRENT_STATUS.md) | 마지막 실물 결과, 미검증 범위, 로컬/GitHub·배포 상태 |
| [제품 설계](docs/MOBILE_MANIPULATION_DESIGN.md) | 사용자 흐름, 온디바이스 실행과 계층별 책임 |

## 문서 구조

README가 유일한 안내 입구다. 목표·현재 상태·설계는 위 세 문서에서 관리하고, 필요한 기술
참고는 아래에서 바로 연다. 과거 실험은 `docs/history/` 한 폴더에 모으며 하위 폴더를 더 나누지 않는다.

| 필요한 내용 | 바로가기 |
|---|---|
| 통신·상태·구성 | [시스템 구조](docs/ARCHITECTURE.md), [프로토콜](protocol/README.md), [하드웨어](hardware/README.md) |
| 센서·좌표·지도 | [카메라](docs/REALSENSE.md), [팔 Home](docs/ARM_HOME_CALIBRATION.md), [방 스캔](docs/ROOM_SCAN_CAPTURE.md), [생활 지도](docs/S4_LIVING_MAP_DESIGN.md) |
| 실행·진단·배포 | [도구](tools/README.md), [장치 조회](docs/DEVICE_INVENTORY_PROTOCOL.md), [텔레옵](docs/WHEEL_HOLD_TELEOP.md), [저장소 관리](docs/REPOSITORY_MANAGEMENT.md) |
| 기술 비교·의존성 | [인지·지도 참고](docs/PERCEPTION_MAPPING_DECISIONS.md), [설치 의존성](requirements/README.md) |
| 집기·주행 실험 이력 | [S3 집기](docs/history/S3_STATIONARY_LAB.md), [S4 주행](docs/history/S4_PREEXPERIMENT.md), [S4b 정렬·집기](docs/history/S4B_EXECUTION_PLAN.md) |
| 이전 판단·실행 근거 | [문제·보정 기록](docs/history/PROJECT_HISTORY_AND_DECISIONS_2026-11-15.md), [11월 16일 상태 원문](docs/history/EXECUTION_STATUS_THROUGH_2026-11-16.md), [이력 폴더](docs/history/) |

과거 문서의 승인·대기 상태는 현재 지시가 아니다. 긴 참고 문서는 첫 요약과 접을 수 있는
목차로 필요한 부분만 읽는다. 왼팔 시연 안내 세 개는 [기존 시연 문서](docs/history/FIXED_PICK_DEMO.md)에
합쳤으며, 오프라인 실행 예시는 [도구 안내](tools/README.md#하드웨어-없는-실행-예시)로 모았다.

## 실행 흐름

```text
요청 이해·필요한 확인 → 대상 위치로 이동 → 관측 → 정차·집기 결정
→ 집기·집힘 확인 → 운반·놓기 → 실제 결과 확인·응답
```

현재 장비는 Jetson Orin Nano Super, STM32G474, 3륜 베이스·리프트·양팔 SO101,
RealSense D415와 LDS-01이다. PC 로컬 언어 모델은 현재 개발 구성이며 최종 온디바이스
완료를 뜻하지 않는다. 사용 중인 인식·계획 기술은 전체 작업에 도움이 되는지 측정해 유지하거나 교체한다.

## 하드웨어 없이 시작하기

모바일 작업 폴더의 저장소 루트에서 Python 3.12로 실행한다. 새 clone에서는 clone한 저장소 루트에서 실행한다. 개발 PC의 기존 편집본은
`~/Mobile-Bimanual-Home-Assistant-edit`이며 실행 기록과 빌드는 Git에서 제외한다.

```bash
python3 -m venv .venv-host
source .venv-host/bin/activate
python -m pip install -r requirements/offline.lock.txt
python tools/run/home_assistant.py demo
python tools/run/home_assistant.py demo --fail-skill pick
python tools/run/home_assistant.py serve
```

앱은 `http://127.0.0.1:18766/assistant`, 접속 키는 로컬 `output/home-assistant/access-key.txt`에
생성된다. 위 명령은 모의 장치를 사용하며 모델 다운로드나 모터 동작을 시작하지 않는다.
상세 재생·ROS·펌웨어 검사는 [기존 도구 안내](tools/README.md#하드웨어-없는-실행-예시)를 따른다.

소스는 `firmware/`, `ros2_ws/src/`, `tools/`, 설정은 `config/`, 검사는 `tests/`에 둔다.
실내 영상·지도·접속 키·실측 원본은 Git에 넣지 않는다. [파일·배포 관리](docs/REPOSITORY_MANAGEMENT.md)를 따른다.

## License

자체 코드와 문서는 [Apache License 2.0](LICENSE), 로봇 모델·STM32 구성 요소는
[제3자 고지](docs/THIRD_PARTY_NOTICES.md)를 따른다.
이전 수건 접기 개발은 [SO101-Towel-Folding](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80)에 보관한다.
