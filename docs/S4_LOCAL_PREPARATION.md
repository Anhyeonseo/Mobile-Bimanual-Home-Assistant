# S4 로컬 준비와 현장 인수

2026-11-11. S3 완료 판정은 유지한다. 이번 작업은 **젯슨 접속과 실물 동작 없이** S4a 지도·주행과 S4b 작업 위치 선정·집기·운반·놓기의 로컬 구현 및 검증을 진행한 것이다. S4 실물 완료 판정은 아직 하지 않는다.

## 구현한 연결

| 단계 | 구현과 재사용 | 로컬에서 확인하는 결과 |
|---|---|---|
| 센서·좌표 | 기존 ROS depth/TF/odometry와 `NavigationHealth` 연결 | 원래 센서 시각, 라이다 연속 수신, 위치 추정 불확실성, 새 주행 세션 이후 관측 확인 |
| 지도 생성 | 공식 `slam_toolbox` + 기존 `ScanManager` / `DepthVolume` / 지도 융합 | 실제 SLAM 알고리즘으로 합성 LaserScan/TF 지도 생성, 그래프 저장·프로세스 재시작·복원 |
| 높이 장애물 | native-depth 프레임 입력, 보수적인 격자 변환 | SLAM 원점이 깊이 격자와 달라도 장애물이 밀려나지 않음. 겹치는 장애물은 유지하고 미관측 공간은 통과시키지 않음 |
| 지도 보존 | 기존 SQLite 저장소에 해시로 구분한 3D 관측 청크 저장 | 지도와 관측 청크를 같은 트랜잭션으로 저장. 변경·누락 검출, 재시작 뒤 과거 관측을 새 주행 허가로 사용하지 않음 |
| 지도 발행 | `saved_home_maps`, 지도 묶음 내보내기 | 위치 추정용 지도·몸체 높이 반영 주행 지도·원본 SLAM 지도·방/가구 기록·3D 자료 보존. Nav2 형식 PGM/YAML 및 ROS OccupancyGrid 발행 |
| 주행 | 기존 Nav2/`MobileClient`/C 코어/주행 안전 검사 | 실제 Nav2 목표 도착, 취소, 스캔 손실, TF 손실 후 정지 |
| 작업 위치 | 기존 `PlacementPlanner` → `TaskPlacementPolicy` → `MissionCatalog` | 베이스 위치·방향·리프트·집기 후보 선택, 도착 확인, 새 촬영 후 집기/놓기. ROS 런타임에도 같은 정책을 주입 가능 |
| 집기·운반 | S3의 공식 STL, 보정 모델, 집기 선호도, MoveIt 경로 사용 | 접근·접촉·들기·운반 경로 및 적재물 충돌 검사 |
| 놓기 | 같은 MoveIt/STL/적재물 검사기에 놓기 단계 추가 | 상판 위 접근 → 수직 하강 → 벌림 형상 검사 → 수직 후퇴 → 빈 팔 운반 자세. 실제 내려놓은 위치에 물체 형상을 남겨 후퇴 검사 |
| 실행·실패 처리 | 기존 15단계 작업 실행기, 하중 확인, 공통 정지 재사용 | 작업 전체, 여러 등록 위치, 단계별 취소, 센서 손실·재부팅·물체 손실·장면 변경 시 후속 동작 차단 |
| 배포 준비 | 기존 `package_stationary_pick.py`에 S4 모드 추가 | 소스·설정·검사·문서의 해시가 있는 보관용 릴리스. PC 라이브러리와 미검증 펌웨어 바이너리는 배포하지 않음 |

집기 후보를 골라주는 모델이나 별도 작업 실행기를 새로 만들지 않았다. 수동 베이스 텔레옵과 자율 주행기가 같은 직렬 포트를 각각 여는 구성도 추가하지 않았다.

## 검증 범위와 한계

실제 SLAM, Nav2, MoveIt 알고리즘을 PC에서 실행했다. 센서, 바퀴의 운동, 작업 대상, 놓는 상판은 합성 입력이다. 전체 fetch 시험은 실제 작업 조정기와 주행 안전 검사를 사용하지만 팔/리프트/감지 및 위치 후보의 도달성은 합성이다. **실제 센서 + Nav2 + MoveIt + MCU를 모두 묶은 실물 fetch 성공으로 합산하지 않는다.**

- SLAM 결과는 원본 평면 지도이고, 깊이 관측이 더해진 영역만 몸체 높이 지도에 반영한다. 방 전체의 관측률을 집 전체 스캔 완료율로 표시하지 않는다.
- 지도 재시작 복원 후 위치 추정과 새 깊이 관측이 필요하다. 저장 지도의 발행 시각은 센서 관측 시각이 아니다.
- SLAM 루프 보정으로 지도 버전이 바뀌면 기존 전역 좌표의 깊이 누적을 폐기한다. 과거 프레임을 자동으로 새 위치에 재투영했다고 주장하지 않는다. 저장 그래프와 등록된 프레임을 기준으로 다시 입력해야 한다.
- 지도 표현은 축에 평행한 최대 256×256 격자다. 회전된 지도는 명시적 변환 없이 거부한다. 더 큰 공간은 해상도·지도 범위를 정해야 한다.
- 가구/방 라벨은 기존 추정·사용자 확인 상태를 보존한다. 라벨이 없다는 이유로 관측된 장애물을 지우지 않는다.
- 놓기 경로의 상판 검사는 수평 직사각형 관측 범위에 한정된다. 물체 전체가 경계 안에 있고 바닥면이 상판에서 0~2mm 위인 후보를 검사한다. 기울어진 상판·연성 물체·미끄럼과 접촉 힘은 이 기하 검사로 증명하지 않는다.
- 전체 경로의 STL 검사는 샘플 기반이다. 연속 충돌 증명이나 실제 지지·파지력 증명이 아니다. 기존 실행기의 별도 하중/놓기 확인을 건너뛰지 않는다.
- `RobotApplication`의 미시운전 실물 모드 제한을 임의로 풀지 않았다. 로컬 테스트 성공만으로 모터가 켜지지 않는다.

## 재현

저장소 루트에서 실행한다. Python 의존성은 기존 `requirements/offline.lock.txt`와 `requirements/s2-observation.txt`를 사용한다. ROS 검증 환경은 Ubuntu 24.04 / Jazzy이며 설치할 Debian 패키지 목록은 `requirements/s4-ros-jazzy.txt`에 있다. 젯슨의 OS/ROS 버전은 이번 작업에서 확인하지 않았다. PC의 실행 파일을 ARM64 젯슨으로 복사하는 방식은 사용하지 않는다.

```bash
python -m pytest -c config/pytest.ini --rootdir=. -q
cmake -S firmware/stm32_actuator -B build/offline-core
cmake --build build/offline-core --parallel 4
ctest --test-dir build/offline-core --output-on-failure
python tools/run/run_fetch_stack.py --joint-placement --guarded-navigation --output output/s4-fetch.json
```

실제 ROS 알고리즘 검증도 장치 없이 실행한다. 아래의 ROS 환경에는 필요한 Nav2/SLAM/MoveIt 패키지가 설치되어 있어야 한다. 출력 폴더는 새 경로를 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=173
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$PWD/ros2_ws/src/home_robot_tasks:$PWD/ros2_ws/src/so101_arm_bridge:$PYTHONPATH"
python3 tools/run/check_slam_replay.py --output output/s4-slam-depth
python3 tools/run/check_nav2_closed_loop.py --output output/s4-nav2
python3 tools/run/check_nav2_closed_loop.py --fault scan_loss --output output/s4-nav2-scan-loss
python3 tools/run/check_nav2_closed_loop.py --fault tf_loss --output output/s4-nav2-tf-loss
python3 tools/setup/fetch_carry_meshes.py --output output/carry-model-meshes
python3 tools/diagnostics/check_carry_moveit.py --mobile-mesh-dir output/carry-model-meshes --include-place --output output/s4-carry-place
```

`check_slam_replay.py`는 실제 SLAM 지도 → 그래프 재시작 복원 → 합성 native-depth 융합 → 3D 청크 저장/복원 → 실제 ROS 지도 발행까지 검사한다. 재시작 시에는 이전 DDS 서비스 정보가 남는 것과 구분하려고 별도 로컬 네임스페이스를 쓴다. SLAM은 deactivate/cleanup 후 종료한다.

ROS 패키지 빌드와 센서 입력만 받는 SLAM 실행:

```bash
colcon build --base-paths ros2_ws/src --packages-select so101_interfaces so101_arm_bridge home_robot_tasks
source install/setup.bash
ros2 launch home_robot_tasks s4_mapping.launch.py --show-args
# 준비된 /offline_s4/scan과 TF를 입력받는다. 장치 드라이버·바퀴 제어기는 시작하지 않는다.
ros2 launch home_robot_tasks s4_mapping.launch.py
# 아래 지도 묶음도 위치 추정이나 주행을 자동으로 시작하지 않는다.
ros2 run home_robot_tasks saved_home_maps --bundle output/s4-slam-depth/height-map --ros-args -r __ns:=/saved_s4
```

실제 센서 설치 시 라이다 드라이버는 공식 `hls_lfcd_lds_driver`를 사용한다. 기존 관측 장치는 LDS-01, 230400 baud였으며 2026-11-05 기록의 최대 약 0.401초 수신 간격 문제는 미해결이다. 새 `NavigationHealth`의 기본 스캔 최대 간격은 0.35초, 연속 정상 관측은 3회다. 현장 기록 없이 임계값만 늘려 문제를 숨기지 않는다. 위치 추정 허용 표준편차 기본값은 평면 15cm, 방향 0.2rad이며 작업 정렬 허용치는 기존 정책에서 별도로 검사한다.

`build_guarded_navigation(..., health=health)`에 같은 `NavigationHealth`를 전달하고 `RosNavigationHealth`로 `/scan`과 AMCL 관측을 연결한다. `reset_observations`는 기존 depth/독립 몸체 공간 확인을 새 세션으로 바꾼다. 주행기는 같은 `MobileClient`와 공통 정지 소유자를 사용한다. `build_ros_runtime(..., placement_policy=policy)`에는 기존 `TaskPlacementPolicy`를 전달한다. 실물용 feasibility는 합성 도달 범위 검사 대신 현재 모델·장면의 전체 경로 검사 결과를 제공해야 한다.

### 기록된 깊이 프레임 입력

`replay_home_scan.py --manifest <등록기록.json> --output <새폴더>`는 이미 SLAM 지도 좌표로 등록된 native-depth만 받는다. RGB 정렬 영상이나 현재 로봇 자세를 과거 프레임에 대신 적용하지 않는다. 입력은 아래 구조다. 해시는 파일의 SHA-256, 깊이는 미터 단위 `.npy`이며 pickle은 허용하지 않는다.

```json
{
  "schema_version": 1,
  "mode": "replay",
  "session_id": "home-scan-1",
  "name": "집 지도",
  "map_file": "localization.json",
  "map_sha256": "<파일 SHA-256>",
  "context": {"map_id": "home", "map_revision": "slam-1", "calibration_id": "<보정 ID>", "clock_id": "<기록 시계 ID>", "frame_id": "map"},
  "depth_profile": {"frame": "camera_depth_optical_frame", "calibration": "<보정 ID>", "resolution_m": 0.05, "minimum_depth_m": 0.15, "maximum_depth_m": 3.5, "minimum_height_m": 0.0, "maximum_height_m": 1.0, "maximum_age_s": 0.3, "minimum_valid_fraction": 0.5, "world_frame": "map"},
  "envelope": {"geometry_id": "<실측 운반 자세 ID>", "radius_m": 0.3, "height_m": 1.0},
  "frames": [{"depth_file": "frame-1.npy", "sha256": "<파일 SHA-256>", "observed_s": 1.0, "intrinsics": [600, 600, 320, 240], "camera_to_map": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], "motion_qualified": true}]
}
```

위 숫자는 형식 설명용이며 설치 보정이나 실제 운반 외형 값이 아니다. 입력 경로는 기록 폴더 안으로 제한하고 지도·프레임 해시를 검증한다. SLAM 원점과 깊이 격자가 다르면 면적 겹침 기준으로 보수적으로 변환하며 원본 지도도 보존한다.

## 배포 묶음과 현장 재개 순서

```bash
python tools/setup/package_stationary_pick.py --stage s4 --output output/s4-source-release
# 생성된 릴리스 폴더 안에서:
sha256sum -c SHA256SUMS
```

릴리스에는 자동 SSH/업로드, 서비스 전환, 펌웨어 플래시, 전원 변경이 없다. 기존 젯슨 S3 릴리스는 유지한다. `.local.*` 설정·인증 정보·촬영 자료·집 지도·PC의 빌드 결과는 포함하지 않는다. 기존 사용자 Git 작성자 정보를 사용하고 도구 공동 작성자는 추가하지 않는다.

현장에서만 채울 항목은 `config/home_assistant/s4_commissioning.pending.json`의 `measurement_inputs`다. 빈 항목을 모의 수치로 채우지 않는다.

1. **젯슨 환경과 읽기 상태**: OS/아키텍처/ROS·의존성·여유 자원, 기존 릴리스와 모델 해시, 장치 연결 확인. 같은 포트 소유자는 하나만 둔다. 새 소스는 비활성 릴리스에 먼저 설치하고 기존 릴리스로 복귀할 수 있게 둔다.
2. **주행 기반**: 바퀴 배열·속도 환산·방향·피드백 odometry, 실제 바닥에서 감속·정지, 공통 제어의 버스 주기와 팔/리프트 유지 확인. 설치별 프로필은 기존 commissioning 생성기를 사용한다.
3. **센서와 운반 외형**: 라이다 장착 TF·거리·수신 간격, D415 노출 TF/깊이 및 주행 진동, 팔·리프트·적재물을 포함한 운반 외형과 보이지 않는 방향 확인.
4. **지도·위치 추정·빈손 주행**: 현장 스캔, 루프 연결, 저장/복원, 재위치 추정, 장애물 앞 정지와 목표 도착. 미관측 높이 공간이 많으면 먼저 추가 관측한다.
5. **S4b 실제 작업**: 작업 위치 선정 → 이동 → 새 촬영 → 집기 → 낮고 안정적인 운반 자세 → 주행 → 도착 후 새 상판 관측 → 놓기·후퇴 → 실제 배송 확인. 위치 선정만으로 예전 촬영의 집기 좌표를 재사용하지 않는다.
6. **반복과 실패 복구**: 적어도 서로 다른 출발·도착 위치에서 반복 기록을 남기고, 들고 가는 중 하중 손실·센서 단절·취소 때 정지와 복구 조건을 확인한 뒤 S4 완료 판정한다. 반복 횟수와 허용 도착/집기 오차는 실제 방 크기와 대상에 맞춰 기록한다.

S3의 높이 −9.1mm 보정과 5mm 닫힘 여유는 유지한다. 성공 실물은 4mm 설정에서 확인했으며 5mm 설정은 후속 실물 확인 항목이다. 약 8mm 접촉 오차는 관찰값으로 보존하고, 중력 원인이나 특정 좌표축의 상수 오차로 확정하지 않았다.

## 생활 공간 사용성 확장

[생활 공간 지도 설계](S4_LIVING_MAP_DESIGN.md)에 앱 자동 탐색, RGB-D 의미·형상 기록, 가구 이동 시 국소 지도 수정, 정지 후 재계획 원칙을 정리했다. 자동 탐색 UI는 가상 센서로 실행하며 실제 slam_toolbox 검증과 별개다. 현재 라이다 ray + native-depth 갱신 어댑터와 RGB/마스크 재생 경로를 추가했다.

## 결과 기록

이번 PC 실행의 세부 로그와 보고서는 `output/s4-local/`에 보관한다. 합성 지도·원본 로그·보관용 릴리스는 Git에 올리지 않는다. 공유 가능한 최종 통과 수와 릴리스 이름은 아래 완료 기록에 추가한다.

### 2026-11-11 로컬 검증 결과

- 확장 전 전체 Python 검사 **3,320개 통과**. 확장 전 최소 의존성 환경에서 **3,297개 통과, 5개 제외**(선택 의존성이 없는 기능). 개발 환경의 추가 패키지를 실물 준비 완료의 근거로 사용하지 않았다.
- C 공통 제어 코어 **17개 통과**, 프로토콜·관절 범위·서보 설정 생성 파일 일치 확인.
- ROS 패키지 **3개 빌드 통과**, 설치된 지도 실행 항목 확인.
- 실제 SLAM: 합성 라이다 **162프레임**, 지도 **77×77**, 저장 그래프를 새 프로세스에서 스캔 입력 없이 복원. 합성 native-depth **9프레임**, **32,046개** 관측 voxel 보존, 라이다 free **132칸**을 높이 장애물로 차단. 과거 깊이 자료의 주행 허가 재사용 차단과 실제 ROS 지도 발행 확인. 관련 프로세스 3개 모두 정상 종료.
- 실제 Nav2: 목표 도착/취소, 스캔 손실, TF 손실 세 시나리오 통과. 두 정보 손실 시 정지 **요청** 지연은 각각 약 **0.279초 / 0.275초**였으며 합성 C 바퀴 피드백의 정지를 확인했다. 실제 바닥 제동 시간 측정이 아니다.
- 실제 Nav2 costmap: 낮은 라이다가 높이 장애물을 지우지 않음, 깊이 장애물을 지운 뒤에도 별도 라이다 장애물이 유지됨을 확인.
- 실제 DDS 연결: native-depth/TF/격자/바퀴 odometry, 라이다·위치 추정 품질 확인, 중복 스캔 거부와 오래된 속도 명령 정지 통과. TF 성공값이 정수로 반환되는 경우도 명시적으로 처리.
- 실제 MoveIt: 공식 STL 포함 **2,168개** 상태 검사. 집기·운반 3개 경로와 하강·벌림·후퇴·빈 팔 복귀를 포함한 놓기 경로 통과. 큰 적재물 충돌과 환경 장애물 충돌은 거부.
- 작업 조정기: 전체 15단계, 주행 안전 검사 결합, 서로 다른 등록 위치 3조합, 15단계별 취소, 센서/하중/부팅/장면 변경 회귀 통과.

이 숫자들은 서로 다른 검사 층의 결과다. 합쳐서 실물 성공 횟수로 계산하지 않는다. `output/s4-local/report.json`에 상세 경로와 소스/릴리스 해시를 기록한다. 최종 배포 묶음은 `output/s4-source-release/`에 만들고 전체 파일 해시를 검증한다.

### 생활 지도 확장 후 검증

최소 의존성 환경과 별도 폴더에 푼 소스 묶음에서 각각 **3,309개 통과, 5개 제외**를 확인했다. 이후 관측 문맥 확인 보완까지 관련 검사 **15개**를 다시 통과했다. ROS 패키지 3개를 다시 빌드했다. 브라우저에서는 자동 탐색 시작, 정지 확인, 이어하기, 지도 검토, 방 이름 수정과 저장을 확인했다. 이 앱 시험은 가상 집이며 실물 자동 탐색 성공 기록이 아니다.
