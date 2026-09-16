# 하드웨어 등록

기준 플랫폼은 **ALOHA Mini 1의 3륜 옴니 베이스·수직 리프트·SO-ARM 계열 양팔**이다. 기존 SO101·STM32 팔 제어 기반을 재사용한다. 플랫폼 선택과 실물 구동·피드백 확인 완료는 구분한다.

## 구성 방향과 확인 항목

| 구성 | 방향 | 확인할 항목 |
|---|---|---|
| 베이스 | ALOHA Mini 1 | 실제 wheel protocol·구동 모드·속도/위치 피드백·주기·timeout·정지, 바퀴 반경·방향·배치 |
| 리프트 | 플랫폼 수직 리프트 | 구동 인터페이스·homing·위치 피드백·이동 한계·반복성·backlash·정지 시 하중 유지 |
| 팔·그리퍼 | SO101 P&P 기반 재사용 | 장착 변환·카메라 마운트 간섭·접근 높이·도달 범위·물체 크기/무게·운반 자세 |
| 작업 인식 | Intel RealSense D415 사용, 리프트 장착 | 시야·깊이 유효 범위·설치 위치·내외부 보정·USB/SDK 호환 |
| 주행 인식 | LDS-01 2대 보유, IMU 미확보 | 개별 장치 ID·장착·시각 동기·좌표축·위치 추정 연결 |
| 컴퓨팅·전원 | 보유 Jetson Orin Nano Devkit / Pi 5 중 한 대; Jetson 우선 후보 | RAM·JetPack/ROS 조합·드라이버·통신 지연·전원 분리·전체 전력·비상 정지 구성 |
| 후속 시연 수집 | 리더암 기반 | 팔 대응·그리퍼 매핑·카메라/상태/동작 동기화·수집 장치 |

기존 STM32 펌웨어에 베이스·리프트 제어 기능이 있다고 가정하지 않는다. 피드백이 없거나 필요한 제어 모드가 지원되지 않으면 해당 단계에서 구조와 대안을 판단한다. 임의의 피드백 API나 기구 치수로 연결을 완료한 것처럼 만들지 않는다.

## 공식 코드 조사: 2026-09-15 기록, 아래 2026-09-16 고정 리비전으로 보완

아래는 `liyiteng/lerobot_alohamini`의 `main`을 읽은 결과이며 주문 부품의 실기
검증이나 배포할 리비전 고정은 아니다. 실제 어댑터 구현 전에 사용할 커밋을
고정하고 장치 메타데이터·단위·정지 동작을 대조한다.

- [AlohaMini 클래스](https://github.com/liyiteng/lerobot_alohamini/blob/main/src/lerobot/robots/alohamini/alohamini.py):
  기본 왼쪽 Feetech 버스에 바퀴 ID 8/9/10과 리프트 ID 11을 구성한다.
  `get_observation()`은 바퀴 `Present_Velocity`를 읽어 본체 속도로 변환한다.
  `x.vel`, `y.vel`은 m/s, `theta.vel`은 deg/s다. ROS rad/s와 회전 단위가
  다르며 x/y 부호도 실제 본체 좌표축과 검증해야 한다. 명령 적분을 실측
  odometry로 사용하지 않는다.
- [LiftAxis](https://github.com/liyiteng/lerobot_alohamini/blob/main/src/lerobot/robots/alohamini/lift_axis.py):
  `lift_axis.height_mm` 목표·관측을 제공한다. 엔코더 누적 회전과 원점으로
  높이를 계산하고 속도 모드로 높이를 제어한다. homing 코드에는 내려가며
  정체/전류를 감지하고 토크를 해제하는 동작이 있어 자동 초기화에 포함하기 전
  실제 하중 유지·원점 검출·실패 처리를 검증해야 한다.
- [Host](https://github.com/liyiteng/lerobot_alohamini/blob/main/src/lerobot/robots/alohamini/alohamini_host.py)와
  [설정](https://github.com/liyiteng/lerobot_alohamini/blob/main/src/lerobot/robots/alohamini/config_alohamini.py):
  ZMQ 명령 5555·관측 5556, 선택 카메라 스트림 5557과 watchdog을 제공한다.
  `--no_follower`는 팔을 연결하지 않고 베이스·리프트를 구동하는 모드다.
  무동작 진단 모드가 아니므로 도착 전 준비 명령에 포함하지 않는다.

이번 구성은 **STM32 → 왼쪽 Waveshare의 공유 버스에 왼팔 6개와 바퀴 3개·리프트
1개**, 오른쪽 Waveshare에는 오른팔 6개를 연결하는 방식이다. 왼쪽 두 커넥터는
같은 버스이며 ID를 중복시키지 않는다. 그룹은 왼팔 `1–6`, 바퀴 `8/9/10`,
리프트 `11`, 오른팔은 별도 UART의 `1–6`이다. STM32와 공식 Host가 같은
데이터선을 동시에 소유하지 않는다. 공식 BOM의 12V STS3215-C018과 어댑터
전류 정격, 펌웨어 구현 범위는 [펌웨어 점검](../docs/ARCHITECTURE.md)을 따른다.
센서·컴퓨팅 선택과 도착 후 검증 순서는 [개발 로드맵](../docs/ROADMAP.md)에 정리했다.

## 기존 참조 파일

- `phase0_baseline.json`: 전원·서보 ID·방향·관절 범위·피드백을 기록하는 빈 양식.
- `left_wrist_camera_reference.yaml`: 이전 왼쪽 손목 카메라 변환. 자동 적용하지 않으며 새 장착 조건에서 재보정한다.
- `config/bimanual_operational_limits.json`(저장소 루트): 기존 양팔 관절 한계. 모바일 충돌 여유나 집기력 측정값이 아니다.

실물 조립체의 장착 위치·TF·전원·통신·정지 조건을 등록한다. 이전 고정 작업대의 카메라 보정과 팔 사이 거리를 새 플랫폼에 그대로 적용하지 않는다. [시스템 구조](../docs/ARCHITECTURE.md)와 [로드맵](../docs/ROADMAP.md)을 함께 따른다.

## 확정 장비와 선택 상태

| 구성 | 상태 | 초기 역할 |
|---|---|---|
| ALOHA Mini 1 | 플랫폼 확정, 부품 준비 중 | 3륜 옴니 베이스·리프트·양팔 |
| Intel RealSense D415 ×1 | D415 사용 확정; 장착/보정 미확인 | 리프트 장착, 정지 집기 좌표 + 주행 상부 장애물 관측 |
| LDS-01 ×2 | 보유 | 주행용 2D scan; 각각 독립 수집한 뒤 배치와 활용 결정 |
| Jetson Orin Nano Devkit | 보유, 주 컴퓨팅 우선 후보 | 로봇 실행·영상 추론, 가벼운 VLM 실험 |
| Raspberry Pi 5 | 보유, 대안 | 로봇 실행·센서 수집, 필요 시 외부 추론 |
| IMU | 현재 보유 목록에 없음 | 선정·추가 여부와 위치 추정 구성 결정 필요 |
| 리더암·추가 손목 카메라 | 이번 보유 목록에서 확인되지 않음 | 후속 시연 수집 및 가림 대응 시 확인 |

Jetson과 Pi를 동시에 쓰는 구성을 전제로 하지 않는다. 기본 방향은 보드 한 대이며,
Jetson의 RAM 용량·JetPack·저장장치·전력/냉각 조건은 실제 장치에서 확인한다.
Jetson은 GPU 기반 영상 추론과 VLM 실험을 고려한 우선 추천이며, VLA 전체를
온보드에서 실행할 수 있다는 결론은 아니다. NVIDIA의
[Orin Nano 자료](https://www.nvidia.com/en-eu/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)를
참고하되 모델별 지연·메모리는 센서와 제어를 함께 실행한 상태에서 측정한다.

### 보드 및 센서 연결 시 우선 확인

현재 저장소의 ROS 기준은 Jazzy다. Jetson에
[JetPack 6.2](https://developer.nvidia.com/embedded/jetpack-sdk-62)를 쓰면 Ubuntu
22.04 기반이므로 기존 Jazzy 환경을 그대로 설치한다고 가정하지 않는다.
[Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)은
22.04, [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)는
24.04용 공식 패키지를 제공한다. Jetson 실장 환경 확인 후 Humble 이식 또는
Jazzy 컨테이너의 GPU·USB 접근을 시험하고 하나로 고정한다. 이번에 ROS 배포판을
변경하거나 해당 조합의 호환성을 검증하지는 않았다.

1. D415(교체 모델 D435도 같은 입력 계약): RGB·depth·CameraInfo, 깊이 단위, RGB-depth 정렬, USB 3 연결,
   리프트 높이별 기울기·거리·시야·팔 가림과 유효 깊이 비율을 확인한다.
   `search_sofa.simulation.json`의 0.5–3 m·정지 대기 0.25 s·TF 시각 오차 20 ms는
   PC 시험 후보값이며 실측 허용치가 아니다. D415→D435 전환 시에도 해상도·보정은
   해당 CameraInfo와 다시 맞춘다. 기존 LiDAR 전용 Nav2 시험은 RGB-D를 주행 입력으로 사용하지 않는다. SDK는
   [공식 Jetson 설치 안내](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)에
   맞춰 실제 JetPack 조합으로 시험한다. 이전 카메라 보정을 복사하지 않는다.
2. LDS-01: 각각 별도 장치 ID·scan·frame을 사용한다. 역할 이름은 임시로
   `lidar_a`, `lidar_b`로 두고 전후 설치는 실물 배치 후 확정한다.
   [공식 규격](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/)은
   거리 0.12–3.5 m, 360°, 300±10 rpm이다. 실측 수신 주기와 유효 각도를 확인한다.
3. 위치 추정은 우선 한 대로 기준을 만들고 두 번째의 가림 보완 효과를 측정한다.
   두 scan을 그대로 이어 붙이지 않는다. 두 센서의 시각·TF를 맞추고 주행 중
   움직임을 반영하는 방식과 Nav2 입력을 검증해야 한다.
4. RGB-D·두 LiDAR만으로 wheel odometry나 IMU를 확보했다고 보지 않는다.
   IMU 미확보 상태의 추정 구성과 추가 여부를 결정한다. 2D scan이 팔 높이의
   충돌이나 소파/침대 위 집기 가능성을 보장하지 않는다.

## 2026-09-16 Mini 1 구조 확인과 배치 결정

기준은 **1세대**다. 하드웨어 저장소 최상위 README는 Mini 2를 안내하고,
소프트웨어 `AlohaMiniConfig.robot_model`도 Mini 2가 기본값이므로 그대로 실행하지 않는다.

- [1세대 BOM](https://github.com/liyiteng/alohamini/blob/17c6a98d79881a45ab869c1f392ed89c0723a298/AlohaMini1/docs/BOM.md), [조립](https://github.com/liyiteng/alohamini/blob/17c6a98d79881a45ab869c1f392ed89c0723a298/AlohaMini1/docs/hardware_assembly.md): 모바일 4축은 12 V STS3215-C018(1/345), 4인치 옴니휠 3개. 바퀴 축 베어링은 선택 품목이고 인쇄 조립체·축 결합·리프트 슬라이딩/베어링 상태가 흔들림에 영향을 준다. 주문한 모터 라벨과 대조해야 한다.
- [모델 구분](https://github.com/liyiteng/lerobot_alohamini/blob/7843e5888366eaa553630e2f9d5539505a62dddf/src/lerobot/robots/alohamini/model_specs.py): Mini 1의 명목 wheel radius 0.05 m, base radius 0.125 m, lift lead 84 mm/rev. Mini 2의 0.063/0.195 m, 131 mm/rev와 혼용하지 않는다. 명목값은 실제 축 배치·회전 방향·미끄러짐/리프트 변환 보정을 대체하지 않는다.
- [원본 제어](https://github.com/liyiteng/lerobot_alohamini/blob/7843e5888366eaa553630e2f9d5539505a62dddf/src/lerobot/robots/alohamini/alohamini.py): 왼쪽 버스 wheel 8/9/10, lift 11. 원본은 x/y 부호 변환과 theta deg/s를 사용한다. 우리 브릿지는 m/s·rad/s이며 모터 부호는 펌웨어 프로파일에서 한 번 적용한다. 원본 connect/configure의 자동 homing·gain 설정을 기존 STM32 세션에 겹쳐 실행하지 않는다.
- 확인한 고정 소스는 키보드/ZMQ 텔레옵 기반이다. ROS Nav2 실행·위치 추정·지도·cmd_vel/odom 드라이버가 완성돼 있는 것으로 취급하지 않는다. 기존 바퀴 코어 위에 우리 ROS 어댑터를 연결한다.

이번 카메라는 **앞 RGB 1 + 뒤 RGB 1 + 리프트 D415 1**이다. 원본 BOM 카메라 구성을 그대로 쓴다는 뜻이 아니다. 사용자가 팔 가림이 없다고 확인한 배치를 기준으로 하되 몸체/적재물·근거리·높이·측후방 가시성은 별도다. 카메라 pitch는 고정 마운트 파라미터이며 존재하지 않는 tilt 모터를 추가하지 않았다. LiDAR도 수평 고정 장착을 기준으로 한다. 기울이거나 리프트에 옮기면 별도의 scan 평면/TF 검증이 필요하다.

| 구간/구조 | 놓치기 쉬운 실패 | 조립·실험 판단 기준 |
|---|---|---|
| 3륜 옴니 + 높은 리프트 | 롤러 충격, 옆 미끄러짐, 지지 삼각형 밖 무게중심 | 축/허브/선택 베어링·프레임 유격부터 확인. 운반 높이/팔/물체별 급정지·횡이동 안정성과 제동 실측 |
| 리프트 위 D415 | 흔들림·backlash로 노출 중 기울기 변화, rolling shutter 왜곡 | 강성·케이블 strain relief 우선. 방진재로 장착각이 흔들리지 않게 하고 목표 시연 속도별 장애물 거리 오차·누락/지연을 측정 |
| 주행 자세 | 높은 상판과 근거리 바닥이 동시에 안 보임 | 실제 CameraInfo/장착 높이·pitch로 전체 운반 부피와 정지 거리의 시야를 검사. 불충족 시 마운트/관측 방향/추가 센서 판단 |
| 옆/뒤 이동·회전 | 옴니 구동은 가능해도 깊이 관측이 없음 | 앞뒤 RGB를 거리 안전 증거로 쓰지 않음. 처음 자세의 전신 여유 확인 + 새로 진입할 공간 관측이 없는 이동은 차단 |
| 문턱·계단·바닥 빨래 | 바닥 제거 필터가 낮은 물체를 지움; 무응답 depth가 낭떠러지일 수 있음 | 최소 장애물 높이·바닥 띠 필터 검증. 낙차 감지 성능 확인 전 계단/낭떠러지 접근 영역 제외; RGB 의미 인식만으로 통과 허용하지 않음 |
| 같은 왼쪽 버스 10축 | 전압강하/과전류·버스 포화로 팔/주행 동시 장애 | 실제 Waveshare 모델의 핀 연결/허용 전류 확인. 두 커넥터를 독립 버스로 계산하지 않으며 적절한 외부 전원 분배·공통 기준 전위·보호 구성 확인 |
| 리프트 케이블 | 전 높이에서 USB/서보선 당김·접점 이탈·걸림 | 양 끝 높이와 팔 운반 자세에서 굴곡/여유/고정점 확인. USB 3 대역폭을 RGB 2대와 동시 시험 |
| 전원/통신 상실 | wheel zero와 torque off만으로 lift/물체 지지 불가 | 기존 whole-stop 후속 증거 + 실제 독립 지지/기계 대책. 안전 정지와 전원 차단을 같은 동작으로 취급하지 않음 |

D415 사양의 좁은 깊이 화각/rolling shutter는 [제조사 자료](https://www.realsenseai.com/products/stereo-depth-camera-d415/)를 기준으로 한다. 배치 계산의 65×40°·minZ 약 0.45 m는 예시이며 실제 해상도·CameraInfo·노출 설정으로 바꾼다. 현재 IMU는 미확보다. 2D 위치 추정만으로 바퀴 충격에 의한 roll/pitch를 알고 있다고 가정하지 않는다. IMU를 추가하더라도 동기화·외부 보정 없이 rolling shutter 자체가 해결되지는 않는다.
