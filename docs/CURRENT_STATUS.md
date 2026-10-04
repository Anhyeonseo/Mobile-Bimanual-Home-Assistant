# 현재 진행 상황

기준일: 2026-10-04

## 현재 목표와 새 개발 범위

**VLM을 사용한 전체 모바일 매니퓰레이션 플랫폼 구축과 판단 근거 검증**이 현재 목표다. 대표 시나리오는 소파의 리모컨을 찾아 침대로 배달하는 것이다. VLA 학습·Classical 성능 비교는 현재 로드맵에서 제외했다. 기존 P&P/MoveIt/Nav2는 실행 기반으로 재사용한다.

| 항목 | 현재 근거 | 다음 개발 |
|---|---|---|
| 실행 기반 | 모의 전체 fetch·관측/정지 계약, 기존 팔 자산, v4 베이스 수동 텔레옵 실기 성공 | H2 리프트, H1 남은 실측, H3/H4/H5 실물 연결 |
| VLM/추가 학습 없는 인식 | 고정 클래스 ONNX/좌표 검증 계약은 있음; 실제 VLM 모델/평가 결과는 없음 | U1: 새 리모컨 인스턴스·유사 물체/없음/복수 후보, 영상 위치·3D 확인·보류 |
| 시맨틱 매핑 | 등록 장소 catalog·지도 revision/폐기·유한 시점 탐색 코어 | M1: RGB/RGB-D 관측 연결, 의미 항목·위치 오차·중복/이동·검색 평가 |
| 작업용 정차 선택 | 옴니 기구학·미세 정렬·도달/충돌·관측 수명 경계 | Q1: base/lift/팔·시야를 함께 본 후보 선택과 고정 접근점 비교 |
| 개발 판단 검증 | 모의 회귀·실행 로그·구간 시간 도구 | E1: 독립 평가 자료, 수동 정답, 인식/지도/정차/배달 지표·구성별 비교 |

위 U1/M1/Q1과 E1 확장은 **새로 해야 할 일**이다. 문서 개편이 해당 구현/성능 검증의 완료를 뜻하지 않는다. VLM 자료 수집은 H3부터 시작하며 실제 자동 배달은 검증된 H6 경로에 연결한다. 기존 메인 문서에서 [로드맵](ROADMAP.md)의 순서와 [구조](ARCHITECTURE.md)의 경계를 관리한다.

## 실물 bring-up 기록

**현재 실물 단계:** ALOHA Mini 1 조립 완료, v4 브라우저 베이스 텔레옵 동작을 사용자가 확인했다. Pi Ubuntu 24.04.3 → STM32(호스트 921600 baud) →
왼쪽 Waveshare 공유 버스(1 Mbaud)로 베이스·리프트 수동 시험을 진행한다. 바퀴 ID는 7/8/9,
리프트는 10, 사용자 확인 모터는 STS3215 12 V다.

- 기존 양팔 펌웨어 `0x00024809`의 HELLO를 확인하고 원본 flash 512 KiB를 백업/verify했다.
- 읽기 전용 점검 v1/v2를 실제 업로드했다. 무응답은 STM32 루프백 확인 후 Waveshare 점퍼를 A로
  바꾸면서 해결됐다. 모바일 재연결 시 ID 1/7/8/9/10 모두 model 777/baud code 0으로 응답했다.
- 후속 `mobile-wheel-setup-v1` SCAN에서 `all_reads_ok=true`: 바퀴 7/8/9는 **mode 1, torque 0**,
  리프트 10은 **mode 0, torque 0**, ID 1/7/8/9/10은 모두 model 777/baud code 0이다.
  9번 최초 MODE_WRITE 실패는 이력으로 보존하며 EEPROM 모드 설정을 재시도하지 않는다.
- 펄스 v1 실기에서 7번 엔코더 2724→2806(+82), 501 ms 시점 정지 요청,
  goal/torque/present speed 모두 0, `stop_confirmed=true`를 확인했다.
  당시에는 회전을 눈으로 구분하지 못했다. 이후 재시험으로 위치·방향을 확인했다.
- v2로 8번을 요청했으나 7번의 `WHEEL_MUST_BE_STOPPED` 사전 검사에서 차단됐다.
  `motion_attempted=false`, 모터 쓰기는 없었다. 당시 출력에 속도 원시값/읽기 상태가 없어
  비영 속도와 읽기 실패 중 어느 쪽인지 판단하지 않는다.
- 사용자가 단독 바퀴 시험 범위를 지적해 v3는 **선택한 바퀴만 검사·회전·정지 확인**한다.
  8번 선택 시 7/9번 상태를 읽거나 쓰지 않는다. raw speed ±200/3000 ms,
  한 부팅당 한 번 및 선택 모터의 mode/torque/lock/정지 조건은 유지한다.
  선택 모터 속도의 원시값과 HAL 상태도 출력해 읽기 실패를 별도 단계로 구분한다.
- v3 관련 pytest **77개 통과**, 펄스 이미지 빌드 통과. 사용자가 바퀴 위치를 확인했으며 v3 상세 결과 로그는 미수신이다.
  바퀴를 모두 띄우고 리프트를 지지하는 단독 시험이며 전체 베이스 정지/지면 주행 검증은 아니다.
  사용자 확인 배치는 **정면 9번 / 왼쪽 7번 / 오른쪽 8번**(로봇이 정면을 보는 기준)이다.
  사용자는 세 바퀴 모두 +200 명령에 바깥쪽에서 보아 시계 방향임을 확인했다.
- H1의 3륜 조합 시험을 진행했다. v4는 전/후진·좌/우 이동·좌/우 회전의 고정 3초 펄스를 제공한다.
  120° 명목 배치와 관측 부호를 기존 OmniKinematics와 대조했다. 단일 시험은 여전히 선택한 ID만,
  조합 시험은 참여하는 세 바퀴 모두 사전 검사하고 실패 시 전부 정지를 시도한다.
  관련 pytest **163개 통과**, 펄스/설정/점검/기존 resident **4개 이미지 빌드 통과**.
  사용자가 전진·왼쪽 이동·왼쪽 회전 조합이 잘 움직인다고 확인했다. 세 시험의 상세 JSON 로그는
  미수신이므로 정지 피드백 전체를 별도 인증한 것으로 취급하지 않는다.
- 사용자 요청으로 반복 펄스 시험을 마치고 텔레옵 v1을 실제 실행했다. ARM 전 검사에서 ID 7의
  speed raw 32818(부호 해석 -50), HAL_OK가 보고됐고 후속 정지 확인도 실패했다.
  session/sequence=0으로 이동 명령 전 차단이다. 사용자는 당시 실제 회전 여부를 보지 못했다.
- v2는 torque off 상태에서 목표 속도 0을 검증한 뒤 최대 1초 동안 각 축의 정지 증거를 반복 확인한다.
  연속 3회 goal 0/torque 0/present speed 0 확인 후에만 torque enable한다. -50을 허용하는 변경은 아니다.
  음수 0은 정규화하며 최초 실패 원인/ID와 축별 goal/torque/속도/위치를 출력한다.
- 사용자는 **키를 누르는 동안 계속 이동, 놓으면 정지**를 요청했다. SSH 반복 입력 대신 브라우저의
  실제 keydown/keyup과 100 ms 상태 전송을 사용한다. `--web --bind <Pi-IP>`로 실행하며 화면 버튼도 지원한다.
  브라우저 상태가 250 ms 없으면 호스트가 zero, STM32 명령이 400 ms 없으면 정지·토크 해제 시도다.
  한글 입력 상태도 물리 키 코드로 처리한다. 창 이탈/연결 상실 뒤 자동으로 이전 이동을 재개하지 않는다.
- 관련 pytest **217개 통과**, v2 STM32 빌드 통과. v1의 197개/5종 빌드는 이전 단계 검증 기록이다.
  v2 실기는 `STARTUP_NOT_QUIET_OR_READ_FAILED`(ID 7, session/sequence 0)이 반복됐다.
  cleanup 후 세 축 goal/torque/speed=0 및 quiet_samples=3, stop_confirmed=true이나 최초 이상 값이 덮여 원인은 미확정이다.
- v3는 정지 관측의 일시적 읽기 실패를 기존 1초 창 안에서 재관측한다. 무효 관측이 있으면 세 축의 연속 횟수를
  모두 초기화하며 새 정상 관측 3회가 필요하다. 지속 비영 속도·읽기 오류·시간 초과는 계속 차단한다.
  `first_observation`에 최초 이상 값/레지스터/HAL 상태/시점을 보존하고 최초 실패 대기 시간도 별도로 남긴다.
  fault 잠금이나 주행 쓰기 재시도 정책은 변경하지 않았다. v3 관련 pytest **225개**, STM32 빌드 통과.
  배포 파일은 `output/base-teleop-v3-20261003/`; 실물 기동/hold 조작 성공은 아직 미확인이다.
  실측 속도·Nav2 운용 프로파일·리프트 시험은 미완료다.

- v3 재검토에서 브라우저 lease가 만료된 뒤 늦게 도착한 새 sequence 이동 요청으로 재출발할 수 있는 버그와,
  같은 client의 claim이 sequence를 초기화하는 버그를 재현했다. Pi 실행 프로그램 v3.1에서 수정했다.
  만료 세션/이전 client 재사용을 차단하고, HTTP 요청을 하나씩 처리하며 200 ms 응답 기한을 적용한다.
  통신 실패 후 키 상태를 지우고 새로고침으로 새 client를 받아야 다시 조종할 수 있다.
  관련 pytest **230개**, 펌웨어 29개 시나리오의 ASan/UBSan 통과. 펌웨어 바이너리는 v3와 동일하다.
  최신 묶음은 `output/base-teleop-v3.1-20261003/`; 실물 검증은 대기다.

- 2026-10-04 v3.1 실기: 최초 STARTUP 관측 ID 7은 goal=0, torque=1, speed=0,
  failed_register=0/HAL_OK였다. 1050 ms 후 시작 차단, cleanup에서는 세 축 모두 torque=0/quiet_samples=3,
  stop_confirmed=true(174 ms)가 확인됐다. 단순 통신 실패로 추정했던 원인은 이번 로그로 좁혀졌다.
- v4는 각 바퀴의 목표 속도 0 쓰기/readback 직후 torque OFF 쓰기/readback을 추가한다.
  목표 쓰기 뒤에도 torque OFF가 유지된다는 기존 가정을 제거하고, 이후의 연속 정지 검사/명시적 enable을 유지한다.
  목표 쓰기에 의한 자동 토크 전환은 현재 로그에 부합하는 모의 조건이며 모터 내부 구현으로 확정하지 않는다.
  이 모의 조건에서 기존 코드 실패를 재현했고 수정 후 pytest **233개**, STM32 빌드,
  펌웨어 32개 시나리오 ASan/UBSan이 통과했다. 배포 `output/base-teleop-v4-20261004/`는
  firmware와 Pi 프로그램을 함께 교체한다. 이후 사용자가 “잘 되네”로 **v4 텔레옵 동작 성공**을 확인했다.
  성공 확인은 수동 텔레옵 범위다. 리프트 동작/원점, 실측 odometry·제동 거리, 실물 통신 단절 시험,
  팔 동시 구동·Nav2 자율주행은 아직 완료하지 않았다.

- 이번 실물 bring-up PR 범위 전체 검증: Python **1,002개**, CTest **17개**, STM32
  legacy/resident/inspection/setup/pulse/teleop **6종 빌드** 통과. 가속 1시간 모바일 시뮬레이션과
  조립된 fetch 스택의 정상/센서 상실/재부팅/하중 상실 사례도 통과했다. 실제 모터 장시간 운용 시험은 아니다.
  CI의 STM32 행렬에도 새 유지보수 이미지 4종을 추가했다.

ID 공통 설정 단계의 pytest 123개/CTest 17개, 점검 v2 단계의 pytest 91개는 이전 단계 검증 기록이다.
상세 실물 결과·절차·백업 해시는 [하드웨어 문서](../hardware/README.md#공통-버스-통신-복구-및-최초-5축-응답-확인)를 따른다.

아래 2026-09-16 PC 검증 수치는 당시 후보 기록이며, 이번 실물 준비 변경의 소스 해시를 포함하지 않는다.

**PC 마감 후보: 주행·관측·정렬·운반 전환까지 전체 작업에 연결해 재검증했다.** 사용자가 낮은 P에서 하중 추종 오차가
커졌던 이력을 알려줘 어깨 P/D=64/64, 팔꿈치=56/64로 복원했다. I=0이며 P16 후보는
철회했다. 양팔 runtime 설정 일치·출발/정지 곡선·MoveIt jerk 제한은 유지한다.
현재 PID는 기존 운용값이고, 떨림까지 최적화한 결과는 아니다. 실측 trace가 필요하다. 사용자가 보고한 MoveIt 출발/접힘 구간 떨림은 시연 시간 유지 조건의 실물 후속 항목으로 보류했다.

**2026-09 PC 개발 이력:** 당시 부품 대기 기간에 기존 P&P 기반 fetch 실행기를 준비했다. 현재는 조립·수동 베이스 텔레옵까지 확인했으며, 최종 방향은 위의 VLM 기반 전체 과정/판단 검증으로 갱신됐다. 이하 PC 마감 항목과 당시 시험 수치는 과거 범위의 결과다.

## 최종 PC 마감 범위

- 정렬도 팔 운반 자세 → 리프트 운반 높이 → 주행 순서로 실행한다. 물체를 발견한 시점의 접근점/높이를 유지하고, 이동 후에는 다시 촬영한다.
- `TravelSessionPort`는 주행마다 depth/clearance epoch를 새로 만들고 새 관측·바퀴 정지·리프트 hold를 기다린다. 후보 대기 한도는 2초다. 관측이 없으면 이동 없이 실패하며, 주행 중 관측 상실은 전체 정지다. 관측 복귀로 자동 재구동하지 않는다.
- 실제 단계 ID의 `/`·`:`를 베이스 제어권으로 허용하되 검증 실패 시 일부 소유권을 남기지 않는다. 베이스 이동은 카메라 pose를 무효화하지만, 별도로 측정한 리프트 hold를 초기화하지 않는다. 수직 드리프트·관측 공백은 계속 차단한다.
- `invalidate_place`로 가구 변경을 보고하면 기존 목표/표면 관측을 폐기하고 활성 작업을 정지시킨다. 정지 후 새 지도/장소 등록으로 runtime을 재구성한다. 자동 가구 이동 인식·semantic mapping이 구현됐다는 뜻은 아니다.
- payload 영상의 ROS `header.frame_id`를 보존하고 설정한 카메라만 받는다. 카메라 전환은 유지 판정의 누적 시간을 초기화한다. 영상 누락은 UNKNOWN이며 낙하 확정이나 성공으로 바꾸지 않는다. 실제 운반 자세에서 어느 카메라가 그리퍼를 볼 수 있는지는 H0/H3에서 확인해야 한다.
- `run_fetch_stack.py --guarded-navigation`은 실제 작업 조정기·wheel 변환/피드백 odometry·주행 안전 검사·관측 전환을 함께 실행한다. 바퀴 물리, full-height free 공간, 영상, 경로 추종기는 합성이다. D415 단독 커버리지나 실제 Nav2 전체 알고리즘 동시 실행의 증거가 아니다.

최신 검증 묶음은 `output/runtime-closeout-20260916/report.json`이다. **Python 793개, CTest 17개, STM32 두 빌드, ROS 4개 패키지**, 실제 DDS/C·Nav2 costmap·MoveIt 및 가속 모의 1시간을 포함한 33개 검사로 마감한다. 실제 Nav2/AMCL 전체 주행의 최근 기록은 2026-09-15이며, 이번 모의 전체 작업과 구분한다.

**범위 종료:** 발견된 PC 연결 결함과 정상/실패/취소 회귀를 마감한다. 다음은 H0의 실제 환경·시야·지지·도달 가능성 확인 및 그 결과에 따른 장치/scene 통합이다. 실측값만 채우면 실물 자율 배달이 바로 완성된다고 하지 않는다. 자동 ARM/homing·실물 구동·커밋·push는 수행하지 않았다.

## 구현과 연결 상태

| 구성 | 구현 상태 | 아직 남은 연결/검증 |
|---|---|---|
| 기존 Classical P&P | 원본 `b3d7a37` 고정·출처 기록; 관절 경로/반대팔 hold/그리퍼 방향 FK 이식, 비동기 RGB-D/계획→실행·표면 ROI·집기/놓기 11단계 조립 | 실제 모델/인식·접근/후퇴 보정·jaw/관측/충돌 장면 연결과 실기 검증 |
| STM32·팔 브릿지 | 기존 팔 실행 경로와 12축 v2·모바일 envelope·공통 transport/버전 검사 | 독립 interlock/하중 입력의 장비 연결과 실물 검증 필요 |
| 통신·모터 그룹 | STS3215 READ/WRITE/SYNC WRITE 공통화; 왼팔 1–6+바퀴 8/9/10+lift 11, 오른팔 별도 1–6 | 모드/상태 READ와 응답 검증 구현; 모델 식별/명시적 모드 변경 코어 추가; 명시적 torque-off provisioning·WRITE 직후 READ 확인 구현; 배선/전원·실측 설정 확인 |
| 공유 버스·보드 전송 | 기존 좌우 서비스/IT 피드백/DMA를 `servo_transport`에 편입; 새 router의 claim/완료/복구와 HAL 연결; 정지 차단을 기존 팔 경로에도 적용 | ISR epoch·주기 DMA TX/RX·fault/STOP·서비스 차단 구현; 버스 부하 실측·부팅 설정 필요 |
| 모바일 감시·통신 | v2 envelope/host handler·팔과 단일 transport·Python↔C parser 왕복; session/sequence·timeout·36B 명령/64B 응답·CRC·clock | 기본 설정은 비활성; 실측 단위/방향·실제 ROS cmd_vel/장치 연결 검증 |
| 전체 정지 | 왼쪽 모바일 zero→왼팔 hold, 오른팔 hold 병행; 정확한 TX 영수증·후속 12축 위치/리프트/하중 증거·AQ/AT 조회 연결 | AE/AF 시각 보존 입력·새 disabled boot 복구 구현; 실제 관측 제공자·제동/처짐·전원 상실 대응 |
| 리프트 제어 코어 | 명시적 homing·엔코더 wrap/점프·접촉 dwell·높이 제어·접근 감속·hold 확인·fault 후 원점 폐기 | AL/LS 높이 API·LiftPort 추가; 보드 interlock 공급·실측 변환·제동/하중 유지 |
| 관절 설정 | JSON 기준 C 표/ROS 사본 생성·빌드 검사; 엄격한 정수/좌표/wrap 검사 | 새 조립체의 기계 최대 범위·영점/방향·URDF 좌표 보정 |
| 로봇 형상 | 베이스·리프트·양팔·카메라·LiDAR 2대·바퀴의 조립체 32 links/단일 TF root; 양팔 lift FK 확인 | 모든 새 치수는 미실측; 전신 충돌 계획·물리 접촉/관성 모델·실제 joint_states |
| 작업 흐름 | 등록 장소 resolver·공유 runtime factory·실제 ROS action/service의 전체 15단계 모의 실행; 단계별 취소·센서/재부팅/낙하 장애 | 실제 관측 제공자와 실측 장치 설정, 집기/놓기 11단계 조립의 실물 jaw·장면·단계별 신선한 관측 연결 |
| 앱 연결 경계 | 인증 HTTP·단기 permit·SQLite 중복/재시작 처리·fault 차단·상태/취소; 지도 검사·fetch와 제어권 공유 | 휴대폰 입체 점유 지도·미리보기/이동/취소 구현; 실측 mesh·페어링·실물 연결 남음 |
| 베이스 수학 | 3륜 정/역기구학·공동 속도 포화·실측 속도 기반 SE(2) odometry·관측 공백 처리 | 실제 치수/모터 방향/servo 단위·가속 제한·실측 odometry/위치 추정 |
| 센서·주행 | 리프트 D415 정지/안정화·pose revision·노출 TF·거리/정합 검사; 유한 시점 재탐색·가림/미발견·확인된 취소 코어; XY/yaw 기구학 모의 주행·정지; 실제 Nav2/AMCL→C 모형 주행·앱 제어권 검사 | 이동/관측 포트의 fetch 조립 구현; detector·실측 센서/odometry와 전체 ROS 실행 연결 |
| 컴퓨팅·추론 | 현재 Pi 5/Ubuntu 24.04.3에서 베이스 텔레옵 실기 확인, Jetson 보유 | VLM 모델·실행 위치 미선정. 실제 제어/센서 동시 지연·메모리로 결정 |

모바일 주기 DMA TX/RX와 전체 정지를 실제 main loop/ISR/host stop에 연결했다.
`MobileBoard_Prepare`는 read-only 점검/부팅 ID/endpoint만 준비하며, 양팔 초기화가
성공한 뒤 수신기를 준비하고 새 정지/interlock 증거로 주기 출력을 연다.
모바일 설정에서는 일반 STOP/DISABLE이 자동 torque-off로 떨어지지 않는다.
기존 비모바일 후보의 동작은 유지한다. 기본 프로파일·독립 hold/하중 입력은 미설정이다.

## 관절 제한에 대한 정정과 현재 처리

변경해야 하는 것은 예전 작업에 맞춘 **관절별 가동 범위**다. `servo_joint_config`의
좁은 v1 점검 범위, canonical JSON의 과거 실측 v2 범위, URDF의 모델 범위가 따로
있다. 현재 펌웨어에 고정 Z 검사가 없다는 사실만으로 이 요구가 해결되는 것은 아니다.

이번에는 설정 중복·불일치를 막는 생성/검사를 구현했다. 실제 관절 한계 수치는
기존 그대로다. 새 장착에서 가능한 전체 기계 범위를 실측한 뒤 영점·방향·모델과
함께 갱신해야 한다. 새 수치의 검증을 과거 승인 기록으로 대신하지 않는다.

## 이번 탐색 확장

D415를 리프트에 장착해 정지 후 집기 관측에 쓰고, LDS-01과 Nav2가 주행을 맡도록
문서와 예제 설정을 맞췄다. 센서 수령·보정은 미확인이다. 탐색은 확인하지 못한
영역을 우선하며 총 시간·촬영 횟수와 미발견 이유를 기록한다. 합성 입력 시험이며
실제 카메라 인식 속도나 로봇의 총 작업 시간을 측정한 것은 아니다.

새 관측/재탐색 시험 39개와 공통 host framing 시험 9개, periodic 서비스 차단 시험을 추가했고 전체 검증을 통과했다.

## 이번 검증

2026-09-16 PC 관측·그리퍼 연결 마감 후보의 검사:

- Python **733개**, 공통 펌웨어 CTest **17개**, STM32 legacy/resident Release 두 빌드.
- ROS Jazzy 4개 패키지 빌드와 실제 localhost DDS의 3개 증거 메시지 검사.
- 전체 fetch: 합성 장치/센서로 15단계 완료, 단계별 취소 15개, 센서 상실·MCU 재부팅·물체 놓침에서 정지 미확인/제어권 유지.
- 실제 ROS 통신의 전체 fetch: 합성 NavigateToPose/GetMotionPlan 서버, 이동 5회·계획 6회, 집기/놓기 내부 11단계를 포함한 15단계 완료. 이것은 실제 Nav2 알고리즘과 MCU를 함께 구동한 시험은 아니다.
- 실제 MoveIt 2.12.4: 좌우 IK·리프트 높이별 FK/IK, 관절/Cartesian 목표 계획, 12축 resident 변환·반대팔 보존·월드 장애물 거부, 물체 부착/분리 geometry readback·부착 하중 충돌 유지. 기존 plugin lifetime 종료 우회를 사용한다.
- Python↔실제 C AE/AF codec 왕복, stale/미래/다른 boot/중복/CRC·예약 바이트·시계 wrap과 만료 차단. 보드 provisioning의 매 WRITE readback/torque-on 거부/실패 재시도 금지.
- 가속 모의 시간 **1시간/360,000개 명령**. 실물 운용·열·USB 안정성 검증은 아니다.
- 소스 해시·로그·설정 묶음: `output/prearrival-pc-completion-20260916/report.json`.

실제 Nav2/AMCL→C 바퀴 모형의 주행·취소·scan/TF 손실, 브라우저·새 venv·sanitizer
결과는 2026-09-15 기록이다. 이번에는 Nav2 서버 패키지가 설치되어 있지 않아 재실행하지
않았다. Nav2 메시지는 공식 deb를 임시 경로에 풀어 사용했으며 OS에 설치하지 않았다.
보드 플래시·모터 구동·실제 영상/추론 모델·원격 CI는 실행하지 않았다.

예제는 `plan_only`, `executable=false`다. synthetic 기록의 `COMPLETED`는 재생
완료이며 `physical_task_completed=false`, `motion_commands=0`,
`stop_command_sent=false`다. 센서나 물리 결과를 가정한 성공 신호를 실물 증거로
집계하지 않는다.

## 남은 실물 연결과 판단 기능 개발

[로드맵](ROADMAP.md)은 기존 H0–H6의 실물 단계에 U1(VLM 요청·대상 판단), M1(의미 메모리), Q1(관측/집기 정차 선택)을 핵심 개발로 연결했다. 다음 실물 작업은 H2 리프트이며 H3부터 영상/TF/수동 정답 수집과 U1 평가를 시작한다. H4 주행과 H5 정지 집기를 확보한 뒤 H6에서 단계적으로 통합하고 E1에서 구성별 효과와 전체 배달을 평가한다. 휴대폰 지도 이동 A1은 확장으로 유지한다.

현재 로드맵의 PC 완료 기준(같은 계약의 모의 장치, 정상/실패 재현, 빌드·실행 절차)을 통과했다. **실측만 넣으면 실물
fetch가 곧바로 완성되는 상태는 아니다.** 아래 연결을 구분해 유지한다.

1. **관측 입력 연결:** `EvidenceProducer`가 arm/base/localization/lift/scene의 원 시각·boot·profile·순서·시간차를 검사하고 `RobotObservation`을 만든다. `RosEvidencePublisher`→`RosEvidenceSource` 실제 DDS 왕복을 검사했다. 실제 장치 드라이버/TF의 출력·하중 지지·물체 유지 관측을 현장 profile에 매핑하는 통합과 실측은 남는다. 임의 true 또는 명령 echo를 입력으로 쓰지 않는다.
2. **인식·집기 연결:** `SurfaceSampler`/`SurfaceObserver`의 실제 depth ROI 평면·목적지 물체 관측 계산, `JawCalibration`/`SharedGripperPort`의 공통 팔 경로, `ManipulationBindings`의 단계별 신선한 pose/scene 조립을 구현했다. 실제 모델 파일·클래스/물체 식별 평가, 표면/접근/퇴각·jaw 보정, 신선한 TF와 scene 관측의 장비 연결은 미검증이다. 콜백 경계가 있다는 이유로 실물 배포 완료라고 하지 않는다.
3. **실측 설정:** 모터 모델/전압·방향·baud, 버스 시간 예산, 관절 범위, 리프트 원점·기구/하중 유지, 카메라 TF를 확보한다. 설정 생성기는 기록 해시와 형식을 검사하며 측정의 물리적 타당성을 인증하지 않는다.
4. **첫 구동:** 기구 지지 → torque-off 유지보수 → 명시적 모드 provisioning → disabled 새 boot → 신선한 관측 → 별도 homing/축별 시험 → 전체 정지 검증 순서다. 자동 mode 변경·자동 homing·기존 작업 재개는 없다.
5. **통합 승격:** 현재 physical 모드 차단을 유지한다. 실제 Nav2/MoveIt·카메라·장치 브릿지 동시 실행과 하중 유지/낙하 시험을 통과한 뒤 별도 승격한다.

휴대폰 지도 선택·미리보기·이동·취소는 기존 RobotApplication 경계와 같은 제어권을
사용한다. 실측 mesh·페어링·실물 연결은 이후 검증한다.

보드 통신 소유권은 실제 팔 경로에 적용했다. ISR은 DMA 완료/오류 사건만 기록하고
main loop가 소유권을 해제한다. 피드백 TX 완료만으로 UART를 내주지 않는다.
취소 시 RX 정리/짧은 quiet 확인을 수행하고 실패하면 소유권을 유지한다. 이 변경의
실제 타이밍·오류 복구는 아직 보드에서 시험하지 않았으며 과거 팔 실기 승인과 구분한다.

`tools/run/verify_offline.py`로 회귀·가속 반복·선택 STM32/ROS 검사와 소스 해시/의존성
보고서를 재생성한다. `--bundle` 소스 묶음은 **오프라인 개발 후보**이며 실물 배포 승인본이 아니다.

일정·의존성·완료 조건은 [로드맵](ROADMAP.md), 구현 계약과 재생 방법은
[시스템 구조](ARCHITECTURE.md), 실제 장비·출처·확인은 [하드웨어](../hardware/README.md)를
수정하며 관리한다. 별도 도착 전 준비/펌웨어 점검 메인 문서는 이 문서들에 통합했다.

이전 수건 접기 이력은 [보관본](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80)에 분리되어 있다.

## MoveIt 연결 추가 검증

실제 move_group/KDL/OMPL로 양팔·두 리프트 높이 IK, 목표 위치 오차, 팔 경로와
장애물 충돌 거절을 PC에서 확인했다. 합성 모델/상태이며 집기 실행은 미연결이다.
설치된 MoveIt 2.12.4의 종료 충돌은 자식 프로세스에만 plugin 라이브러리를 유지하는
옵션을 사용해 정상 종료까지 확인했다. 원래 환경의 종료 문제를 해결된 상위 패키지로
표시하지 않는다. 재현 방법과 제한은 시스템 구조의 MoveIt 절을 따른다.

## 실제 Nav2·앱·C 모형 통합 검증

앱 지도 요청→공통 작업 실행기→실제 Nav2/AMCL→공통 v2 브릿지→C 모터 모형→
합성 라이다/odometry를 한 경로로 연결했다. 장애물 우회·도착, 요청 중복/동시 실행
차단, 취소 시 하위 FollowPath 종료와 새 정지 feedback, 이후 제어권 반환을 검사했다.
도착 결과와 센서 증거의 수신 순서 차이를 처리하고, Nav2 시험은 다음 요청 전에
새 위치 추정을 공급한다. 실제 위치를 요청 목표로 대체하지 않는다.

단독 주행 사례는 약 12초였으나 합성 치수·속도·센서의 PC 사례이며 실물 성능이나
전체 물건 배달 시간은 아니다. 전체 검증은 `--moveit --nav2`로 함께 재현한다.
HTTP 인증/permit/지도 조회/목표/상태/취소도 같은 Nav2 경로에서 검사한다.
scan/TF 출력 손실은 입력 시각의 0.3초 후보 제한으로 감지하고 정지·하위 액션
종료·FAILED 결과·제어권 반환을 확인한다. 휴대폰 UI·실물 연결·탐색/집기 실행은 남아 있다.

## 전체 작업 시간 제한

작업 실행기는 기본 300초의 전체 예산을 공유한다(PC 운영 후보, 배달 성능 보장 아님).
단계별 준비 대기·실행 시간이 모두 포함되고 단계 전환으로 예산을 재설정하지 않는다.
만료되면 `task_timeout`으로 정지하며, 정지/하중 유지 확인이 없으면 제어권을 유지한다.
HTTP 상태에 전체 경과/남은 시간과 완료 단계의 대기/실행 시간을 기록한다.
운영자는 gateway의 `--task-timeout`으로 0초 초과 3600초 이하에서 설정할 수 있다.

## 모바일 주기 송신기 추가

`mobile_output`과 `MobileServoOutput`으로 supervisor→주기/대기열→실제 UART DMA를
연결했다. 펌웨어와 ROS 팔 브릿지 후보 식별자는 함께 `0x00024908`로 갱신했다.
관절 제한 수치는 그대로다. 새 fake HAL 14개 시나리오와 C 주기 시험이 추가됐다.
실제 main loop 연결은 있지만 미실측 기본값으로 자동 구동하지 않는다. boot ID/리프트 높이/전체 정지 경로는 아래 단계에서 연결했으며, mode 변경 진입점과 독립 hold/하중 입력 제공자는 남아 있다.

## 모바일 응답 수신·초기 확인 추가

팔 응답 parser를 `sts3215_response` 공통 코어로 옮기고 기존 팔 API를 유지했다.
완료 프레임 뒤 추가 바이트의 버퍼 초과를 막고 짧은 장치 오류 응답도 처리한다.
`mobile_feedback`은 IDs 8–11의 모드 READ와 15-byte 상태 READ를 순환하며
속도·전류의 부호, 전압/온도/전류 제한, 모든 축의 새 샘플과 가장 오래된 시각을 검사한다.
설정/송신 성공을 mode 확인으로 대신하지 않는다. lift 높이·원점 증거는 별도 입력이다.

`MobileServoFeedback`을 main loop/ISR과 기존 왼팔 circular DMA에 연결했다.
주기 경로는 기다리지 않으며 팔 예약 시간을 지키고 TX 완료·응답·quiet까지 UART를
소유한다. 정지 차단 중에도 단일 unicast READ만 허용하는 lease로 새 정지 증거를
수집한다. 오류 시 소유권 유지→명시적 abort/RX 복구→STOP 송신→읽기 재시작→
새 정지/전체 hold 증거→새 session 순서다. 반복 fault poll이 완료된 STOP을 다시
pending으로 만들어 재활성화를 막던 경계도 수정했다.

출력/수신의 속도 방향과 팔 예약 설정이 다르면 초기화를 거절한다. 새 fake HAL
24개 시나리오와 장치 응답→C endpoint→실제 Python ROS 브릿지 해독 1개,
공통 C 수신 시험을 추가했다. 실제 HAL/DMA 전기적 타이밍은 시험하지 않았다.
기본 부팅에서 Configure/endpoint 바인딩은 호출하지 않으며 모드 자동 쓰기,
EEPROM 변경, 원점 설정이나 하중 유지 성공을 만들지 않는다.

## 팔 초기화·전체 정지 연결 (2026-09-16)

- 부팅 점검이 팔 초기화보다 먼저 periodic mode를 잠그지 않도록 분리했다. 공유 RX는 finite 팔 동작 종료 시 유지한다. 모바일 UART가 바쁠 때 팔 피드백은 기다리며, 한쪽 응답 실패 시 반대쪽 응답까지 정리한 뒤 실패를 집계한다.
- `system_stop`은 성공한 정확한 패킷/종류/token의 영수증을 검사한다. 다른 STOP, 만료, 실패/복구된 TX는 hold 완료가 아니다. HAL ms와 독립 us 시간축을 따로 전달한다.
- 양팔 hold 후 새 12축 위치를 반복 READ하여 설정된 오차·dwell·나이를 검사한다. wheel/lift 표본 및 독립 lift hold·payload retention도 새로워야 한다. 반복 poll로 dwell을 만들지 않으며, stale/드리프트/하중 상실 시 CONFIRMED를 취소한다.
- AQ/AT 상태 조회는 같은 v2 transport/잠금을 사용한다. `SystemStopClient`와 `SystemStopBackend`는 MCU 재시작·요청 불일치·왕복 지연·노후 표본을 확인하며, 확인 전에는 작업 제어권을 반환하지 않는다. 팔 브릿지와 firmware ID는 `0x00024908`다.
- 신규 Python 26개와 CTest 1개 실행 파일을 추가했다. 실제 Board/양쪽 writer/core와 합성 센서의 통합, C 상태의 Python 해독, STOP 전용 writer 실패, paired-read 정리, 브릿지/실행기 실패를 검사한다. 실제 UART RX/전기적 타이밍 또는 하중 유지 시험으로 집계하지 않는다.

현재 시작 snapshot이 오래되면 모바일 zero만 시도하고 전체 stop은 미확인으로 남는다. 첫 finite 완료 이후의 지속 관측을 아래 단계에서 연결했다. 첫 동작 전 snapshot은 기존 준비 경로를 사용한다.


## 대기 관측·하중 판단·fetch 조립 (2026-09-16)

- `mobile_arm_observer`는 첫 finite 완료 후 6쌍 관절을 READ하여 기존 canonical 한계로 검증한다. 요청 시작 시각으로 snapshot을 갱신하며, 오래되거나 누락된 응답으로 최신성을 연장하지 않는다. 다음 팔 동작은 READ 소유권을 양쪽에서 반환한 뒤 실측 anchor/unwrap 기준을 다시 잡는다. 정리 실패·예상 밖 reader 소유권은 전체 정지를 요구한다.
- `PayloadMonitor`는 새 그리퍼 간격/effort와 새 영상 관측이 모두 있어야 HELD/RELEASED를 판정한다. 두 입력의 sequence·시각·시간차·유지 구간을 검사한다. 모터 전류 하나나 행동 성공 응답은 증거가 아니며, release는 같은 물체의 지정 목적지 관측까지 필요하다. 실측 임계값·detector·MCU `ObserveLoad` 연결은 아직 없다.
- `compose_fetch_adapter`는 운반 자세/높이→주행→탐색→정렬→재관측→집기→운반→놓기/확인을 연결한다. 준비 단계의 팔/리프트 조건을 분리해 리프트 이동 자체가 상위 정지 위반으로 오인되지 않는다. `CapturePort`는 이동 직후 요청에도 안정화 이후 노출을 기다리고, 그 사이 pose가 바뀌면 거절한다.
- 앱/상위 작업/중첩 탐색·장치가 하나의 `TaskLease`로 전체 stop을 공유한다. 하위 cancel 예외에도 전체 정지를 요청하며, 하위 액션이 끝나지 않으면 MCU stop 확인만으로 제어권을 반환하지 않는다. 종료한 작업의 늦은 stop 요청과 다른 작업의 요청은 거절한다.
- 이번 증가분은 Python **51개**다. idle 관측 10개, Board 장시간 idle/동작 전환 2개, 하중 11개, 조립 21개(15단계별 취소 포함), 장치 포트 3개, 중첩 stop/cancel 2개, 촬영 안정화 2개다. 기존 tracking 시험에는 READ 정리 실패 회귀를 보강했다.
- 전체 fetch 시험의 navigation/계획/모터/detector는 합성 leaf 포트이며 RGB-D와 하중 입력도 합성이다. 실제 ROS 서버와 MCU를 동시에 연결한 fetch 또는 실물 집기 시험은 아니다. 실행 모드는 계속 simulation이며 자동 실물 구동 진입점은 열지 않았다.

이번 보고서와 소스 묶음: `output/prearrival-idle-fetch-20260916/report.json`,
같은 폴더의 `offline-source-candidate.zip`. 기존 보고서는 보존한다.

## 전체 runtime 추가 (2026-09-16)

- `run_fetch_stack.py`는 기존 replay와 달리 실제 resolver/Search/Sequence/Capture/Manipulation/Payload/Stop 조정 코드를 조립한다. leaf 장치·영상·충돌 판정은 명시적 합성 입력이다.
- `RobotStateStore`는 관측 시각·sequence·boot·pose revision을 추적한다. scene 준비 여부만으로 reach/collision을 승인하지 않고 관측 capture/scene ID가 맞는 별도 판정을 요구한다.
- `build_ros_runtime`는 준비된 클라이언트만 주입받는다. ROS callback과 공유 직렬 클라이언트는 단일 제어 worker가 소유하며 fault 후에도 정지 관측 입력은 계속 처리한다.
- AE/AF는 독립 팔/리프트/하중 판단의 원시 관측 나이를 보존한다. 만료·반복 표본은 최신으로 갱신하지 않는다. 펌웨어/팔 브릿지 후보 ID는 `0x00024908`이다.
- `RecoveryCoordinator`는 이전 작업을 폐기하고 새 boot·출력 비활성·기구 지지·기존 클라이언트 종료·설정 hash를 확인한다. 실제 MCU reset/ARM/토크 해제 명령을 자동 전송하지 않는다.

## 팔 떨림·하중 추종 조정 상태 (2026-09-16)

- 사용자는 모터 기본 P32에서 하중 오차가 커져 제한에 걸렸고, P를 높여 오차를 줄였으나 떨림이 증가했던 것으로 기억한다. 이에 P16 일괄 변경을 철회하고 **어깨 P64/I0/D64, 팔꿈치 P56/I0/D64**를 복원했다. 나머지 네 관절은 기존 P16/I0/D32다. 이력의 정량적 원시 로그는 이번 저장소 검색에서 확보하지 못했으며 최적값 검증을 주장하지 않는다.
- 이전 LeRobot P16 후보의 보고서는 보존하지만 배포 기준에서 제외한다. 현재 firmware는 `0x00024908`, 설정 hash는 `0x2D90167E`다. 이전 hash가 같아도 firmware 버전까지 맞아야 실행된다.
- 양팔 acceleration/time 설정 일치와 readback, task waypoint의 5차 곡선, MoveIt TOTG→Ruckig→충돌 검증, 측정 로그 분석기는 유지한다. 관절 범위·torque·추종 오류 제한은 바꾸지 않았다.
- 다음 시험: 실제 PID/속도/acceleration/time readback → 문제 자세의 정지 hold 및 같은 경로 이동 기록 → P를 고정한 채 관절별 D를 작은 폭으로 비교 → 오차/진동/정착 시간/전류를 함께 평가. D 증가는 항상 개선되는 법칙이 아니며 현재 D64도 이미 기본 D32보다 크다.
- 정지 중의 목표가 일정한지 먼저 확인한다. 목표까지 떨리면 명령 생성/관측 보정 경로를, 목표는 일정한데 실측만 떨리면 servo loop·유격·하중을 분리해 조사한다. 이동 때만 떨리는 문제에 가감속 평활화를 우선 적용한다.
- 낮은 P에서 남는 정적 오차를 I로 줄이는 방법은 후속 후보지만, 실제 servo의 적분 제한/포화 동작과 안정성을 확인하지 않고 I를 켜지 않는다. 추종 오차를 숨기기 위해 안전 제한이나 측정값을 완화/필터링하지 않는다.

## 집기·놓기 세부 단계 추가 (2026-09-16)

- 기존 계획→팔 이동 포트를 `PickPlacePort`로 감싸 집기 6단계·놓기 5단계를 조립했다. 그리퍼 명령 성공과 실제 집힘/해제 관측은 별도로 요구한다. 후퇴는 부착/분리 확인 후 새 anchor/장면으로 계획한다.
- `SceneUpdatePort`는 실제 MoveIt Apply/Get 서비스로 부착·해제 결과의 ID·링크·치수·pose·touch links 및 world/attached 중복 부재를 확인한다. 불확실한 서비스 응답은 자동 재시도하지 않고 소유권/오류를 유지한다. 취소 시 자동 개방/분리/역방향 이동은 없다.
- MoveIt 시작 상태를 전체 교체하던 요청을 수정해 부착 물체가 다음 계획의 충돌 검사에서 사라지지 않게 했다. 실제 MoveIt Jazzy에서 큰 합성 하중의 충돌 거절·분리 후 계획 성공을 검사한다.
- 전체 모의 실행은 새 단계를 사용한다. 실제 ROS factory에도 주입 지점을 연결했지만, 실물 geometry/관측/그리퍼 제공자가 없으면 접근 계획 전용 경로이며 실물 집기 완료로 표시하지 않는다.
- 팔 떨림 조정은 실물 후속으로 보류했다. 기존 P와 동작 속도를 이번 변경으로 조정하지 않았다.

## PC 연결부 마감 (2026-09-16)

- 측정 소스 fusion은 가장 오래된 관측 시각을 사용한다. 누락·지연·재전송·다른 boot/profile·잘못된 boolean/벡터를 거부한다. 반복 poll로 새 증거를 만들지 않는다.
- 같은 `ResidentArmPort`를 쓰는 그리퍼 포트와 단조 jaw 각도/간격 보정, 측정 anchor·범위·속도·장면 검증을 거치는 route builder를 추가했다. 별도 UART writer는 없다.
- 표면 판정은 동결된 RGB-D 영상에서 지정 ROI의 실제 depth를 표본화하고 평면 residual·기울기·관측 폭을 확인한다. 변형/장애물/누락을 거부하며 소파/침대의 강성이나 전체 여유 공간은 증명하지 않는다.
- 계획 시작/실행 직전에는 target 노출 나이를 검사한다. 실행 중에는 최신 로봇 상태·동일 target ID/원시 노출 시각·pose/scene/calibration을 감시하여 정상적인 수초 경로가 최초 영상 만료만으로 중단되지 않게 했다.
- 기존 문서의 PC 범위를 기준으로 마감한다. 당시 PC 마감에는 VLM 실영상 기능, 네이티브 앱, 실측 3D mesh를 추가하지 않았다. 현재의 VLM/의미 지도/정차 판단 추가 개발은 위의 변경된 목표와 구분해 관리한다. 실물 성능/완료와 PC 소프트웨어 검증은 별도다.

## 2026-09-16 주행 구조 재점검

**이전 마감에 빠졌던 내용:** 공식 Mini 1/2 구분, 실제 재사용할 Nav2 속도/피드백 브릿지,
D415 주행 중 상판·미관측 공간, 움직인 가구의 지도 전환, 앞뒤 RGB 카메라 TF.

- `base_driver`/`navigation_binding`: 원 시각을 보존하는 TwistStamped, SI→3 wheel raw, lift 0, 공통 stop, measured sample odom/TF, 양자화 후 통과 검사, 액션 성공 이후 별도 정지 확인. 기존 runtime에 주입 가능.
- `navigation_depth`/`ros_navigation_depth`: native depth/CameraInfo·노출 TF·고정 보정·bounded worker, 높이 있는 occupancy와 full-height 2D projection, unknown/invalid/stale 차단. lower LiDAR는 upper obstacle을 삭제하지 못함.
- 지도 변경: whole-stop으로 owner 해제 후 revision 적용, 재위치 추정 및 MissionCatalog revision 일치 요구. 자동 장기 mapping/semantic mapping은 구현 완료로 세지 않음.
- 모델: 전방 RGB·후방 RGB·lift D415의 colour/native-depth 프레임과 마운트 pitch 파라미터. 치수/외부 보정은 합성값. 팔 가림 없음과 전신/근거리 시야 충분함은 별개.
- 새 검사: `check_navigation_bridge.py`의 실제 DDS/native C plant·depth grid·stale command 정지, `check_camera_coverage.py`의 후보 배치 사각, 단위 장애 회귀. 기존 전체 회귀와 별도 source hash 보고서는 `output/navigation-audit-20260916/report.json`.

**남은 경계:** 실제 sensor/quality·전체 부피 clearance/scene 제공자, 운반 부피·제동/전복·camera 진동과 USB 지연,
지속 map 수정/semantic 탐색의 현장 통합, 목표 보드 환경과 실기 배포. D415 하나로 사각을 해결했다고
주장하지 않는다. 카메라 배치가 통과 공간을 관측하지 못하면 기구/센서 보완이 필요하다.
상세 발견/대응은 [하드웨어](../hardware/README.md#2026-09-16-mini-1-구조-확인과-배치-결정)와
[로드맵](ROADMAP.md)의 H0/H4에 반영했다. 자동 ARM·homing·실물 명령·커밋·push는 하지 않았다.

이전 주행 재점검 후보 검증: Python **775개**, CTest **17개**, STM32 legacy/resident **2개 빌드**, ROS Jazzy **4개 패키지**, 실제 DDS/C 브릿지·Nav2 costmap·MoveIt와 가속 모의 1시간을 포함한 **32개 검사**를 실행한다. 합격 여부와 최종 source hash는 위 보고서가 기준이다. 실제 Nav2 costmap 계층 검사는 이번 기록이며, 전체 경로 계획/AMCL 주행의 최근 기록은 기존 2026-09-15 검사다.
