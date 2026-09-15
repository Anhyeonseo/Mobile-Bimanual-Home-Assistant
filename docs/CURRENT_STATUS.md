# 현재 진행 상황

기준일: 2026-09-15

**현재 작업은 오프라인 개발 후보로 마감한다.** 실물 구동 완료본은 아니다.
이번 마감에는 AL/LS 리프트 wire/브릿지, 모터 점검·부팅 ID 코어,
단계별 탐색·RGB-D/MoveIt/팔 실행 포트와 휴대폰 입체 지도 화면을 포함한다.
전체 보드 초기화·정지 피드백과 fetch 실행 연결의 남은 작업은 아래에 명시한다.

**ALOHA Mini 1 기반 Classical Fetch-and-Deliver를 우선 완성한다.** 대표 작업은 거실 소파의 리모컨을 집어 침대의 지정 영역에 내려놓기다. 이후 리더암 시연 기반 VLA와 비교한다. 플랫폼 부품은 출력·배송 중이며 약 2주를 소프트웨어 준비에 사용한다.

## 구현과 연결 상태

| 구성 | 구현 상태 | 아직 남은 연결/검증 |
|---|---|---|
| 기존 Classical P&P | 원본 `b3d7a37` 고정·출처 기록; 관절 경로/반대팔 hold/그리퍼 방향 FK 부분 이식 | 비동기 RGB-D/계획→실행 port 추가; 실제 모델·접근 방향·전체 실행 연결·실측 보정 |
| STM32·팔 브릿지 | 기존 팔 실행 경로와 12축 v2·모바일 envelope·공통 transport/버전 검사 | 부팅/초기화 조립 코드 추가; 팔 초기화와 주기 모드 전환·전체 hold 연결 미완 |
| 통신·모터 그룹 | STS3215 READ/WRITE/SYNC WRITE 공통화; 왼팔 1–6+바퀴 8/9/10+lift 11, 오른팔 별도 1–6 | 모드/상태 READ와 응답 검증 구현; 모델 식별/명시적 모드 변경 코어 추가; 실제 maintenance 진입점·배선/전원 확인 |
| 공유 버스·보드 전송 | 기존 좌우 서비스/IT 피드백/DMA를 `servo_transport`에 편입; 새 router의 claim/완료/복구와 HAL 연결; 정지 차단을 기존 팔 경로에도 적용 | ISR epoch·주기 DMA TX/RX·fault/STOP·서비스 차단 구현; 버스 부하 실측·부팅 설정 필요 |
| 모바일 감시·통신 | v2 envelope/host handler·팔과 단일 transport·Python↔C parser 왕복; session/sequence·timeout·36B 명령/64B 응답·CRC·clock | endpoint 기본 미연결; 부팅 ID·실측 단위/방향·ROS cmd_vel 연결 |
| 전체 정지 코어 | 모바일 zero→양팔 position hold 순서, 모든 전송 뒤 새 독립 정지/하중 증거 요구; 미확인 제어 차단 유지 | 실제 팔 fault/전체 로봇 stop 경로 연결·hold 구현·전원 상실 대책 |
| 리프트 제어 코어 | 명시적 homing·엔코더 wrap/점프·접촉 dwell·높이 제어·접근 감속·hold 확인·fault 후 원점 폐기 | AL/LS 높이 API·LiftPort 추가; 보드 interlock 공급·실측 변환·제동/하중 유지 |
| 관절 설정 | JSON 기준 C 표/ROS 사본 생성·빌드 검사; 엄격한 정수/좌표/wrap 검사 | 새 조립체의 기계 최대 범위·영점/방향·URDF 좌표 보정 |
| 로봇 형상 | 베이스·리프트·양팔·카메라·LiDAR 2대·바퀴의 조립체 27 links/단일 TF root; 양팔 lift FK 확인 | 모든 새 치수는 미실측; 전신 충돌 계획·물리 접촉/관성 모델·실제 joint_states |
| 작업 흐름 | 계획/재생 + 가짜 adapter의 15단계 실행·연속 조건 감시·확인된 취소·정지 미확인 시 제어권 유지 | 실제 adapter·유한 재시도·운영 복구 |
| 앱 연결 경계 | 인증 HTTP·단기 permit·SQLite 중복/재시작 처리·fault 차단·상태/취소; 지도 검사·fetch와 제어권 공유 | 휴대폰 입체 점유 지도·미리보기/이동/취소 구현; 실측 mesh·페어링·실물 연결 남음 |
| 베이스 수학 | 3륜 정/역기구학·공동 속도 포화·실측 속도 기반 SE(2) odometry·관측 공백 처리 | 실제 치수/모터 방향/servo 단위·가속 제한·실측 odometry/위치 추정 |
| 센서·주행 | 리프트 D415 정지/안정화·pose revision·노출 TF·거리/정합 검사; 유한 시점 재탐색·가림/미발견·확인된 취소 코어; XY/yaw 기구학 모의 주행·정지; 실제 Nav2/AMCL→C 모형 주행·앱 제어권 검사 | 재탐색의 이동/관측 adapter와 fetch 연결·detector·실측 센서/odometry |
| 컴퓨팅·학습 | Jetson Orin Nano Devkit 우선 검토, Pi 5 대안; 한 대 사용 | 실제 OS/ROS/GPU/USB 조합 확인. VLM/VLA 모델·데이터 미선정 |

모바일 주기 DMA 송신기를 실제 main loop/ISR/host stop에 연결하고 fake HAL로 시험했다.
장치 RX와 모드/상태 해독을 연결했으며 부팅 시 미설정이다. 모터 점검/부팅 ID·리프트 높이 명령·보드 조립 코드는 추가했다. 실측 프로파일과 독립 interlock 공급, 팔 초기화/전체 hold 연결은 미완이다. 이 펌웨어를 모바일 구동 완료본으로
보지 않는다. 기존 팔 정상 취소의 torque disable도 물체를 든 모바일 정상 정지를
대신하지 못한다.

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

- Python **496개** 통과: 기존 회귀에 보드 전송/dispatcher의 fake HAL 실행 15개 시나리오와 직접 송신 우회 검사 추가. 응답 대기 경합·양쪽 버스 예약·즉시/중복/늦은 callback·부분 송신 실패·abort 실패·router 만료/복구/정지 차단을 포함한다.
- 공통 펌웨어 CTest **16개 실행 파일** 통과: 공유 버스·모바일 감시·lift/정지 coordinator·stream parser 포함.
- STM32 Release legacy/resident v2 두 빌드 통과. protocol manifest/header와 관절 생성물 검사 통과.
- ROS 2 Jazzy 4개 패키지(`so101_interfaces`, `so101_arm_bridge`, `home_robot_tasks`, `so101_description`) 빌드 통과. 조립체 root와 좌우 lift FK 검사 통과. 실제 localhost DDS의 합성 NavigateToPose 서버에서 성공/접수 전 취소 통과; 실제 Nav2/AMCL planner/controller와 C 바퀴 모형의 주행·취소도 검사했다(아래 통합 절).
- 새 모바일 코어의 AddressSanitizer/UndefinedBehaviorSanitizer 검사 통과. 실행 환경 제약으로 LeakSanitizer는 제외했다.
- 고정 의존성의 새 Python 3.12 가상환경에서 전체 오프라인 검증 통과. 모의 시간 **1시간/속도 명령 360,000개**의 native C 왕복 시험 통과(가속 실행; 실제 1시간 운용/열/USB 안정성 시험 아님). 소스 해시·설정·의존성과 로그를 보고서에 기록한다.
- 보드 플래시·모터 움직임·제동/하중 유지·실제 센서 기록·원격 CI는 실행하지 않았다.

예제는 `plan_only`, `executable=false`다. synthetic 기록의 `COMPLETED`는 재생
완료이며 `physical_task_completed=false`, `motion_commands=0`,
`stop_command_sent=false`다. 센서나 물리 결과를 가정한 성공 신호를 실물 증거로
집계하지 않는다.

## 바로 다음 구현

**아직 실측만 남은 상태가 아니다.** 다음 작업을 보존한다.

- 모바일 주기 모드의 service 차단과 기존 팔 초기화 순서를 조정하고, mode provisioning의 명시적 진입점을 연결한다.
- 기존 양팔 torque-off stop을 하중 유지 stop 경로에 연결하고 정지 중 양팔/리프트/그리퍼의 독립 피드백을 공급한다.
- 새 Search/Sequence/Capture/MoveIt/Lift/ResidentArm 포트를 실제 실행 구성으로 묶고, 전체 fetch의 이동·집기·놓기·실패를 통합 시험한다. 새 포트의 존재만으로 실제 제어 연결 완료로 간주하지 않는다.
- RGB-D 리모컨 모델/설정, ROS cmd_vel/odometry·센서 진단, 실측 프로파일 생성/배포·복구 절차를 완성한다.

휴대폰 화면의 바닥 선택→경로 확인→모의 이동→취소 완료와 390px 폭을 브라우저에서 직접 확인했다. 실제 scene mesh 또는 물리 주행 시험은 아니다. 세부 의존성과 통과 조건은 로드맵의 「PC에서 다음에 닫아야 하는 통합 경계」를 따른다.

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
연결했다. 펌웨어와 ROS 팔 브릿지 후보 식별자는 함께 `0x00024903`로 갱신했다.
관절 제한 수치는 그대로다. 새 fake HAL 14개 시나리오와 C 주기 시험이 추가됐다.
실제 main loop 연결은 있지만 미실측 기본값으로 자동 구동하지 않는다. mode 변경/boot ID, 리프트 높이·hold 및 전체 정지 연결이 다음 순서다.

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
