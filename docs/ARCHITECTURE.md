# 시스템 구조

## 계층과 책임

```text
정형 요청 / 향후 자연어 해석
        ↓ 구조화된 작업
작업 상태 머신: 순서·취소·시간 제한·실패 복구·완료 확인
        ├─ Navigation: semantic waypoint → Nav2 → 베이스 드라이버
        ├─ Perception: RGB-D → 물체·작업면·유효 시각과 좌표계
        ├─ Fine alignment: 작업면 기준 베이스 미세 정렬
        ├─ Lift: 높이 목표·원점·위치 피드백·정지
        └─ Manipulation: Classical P&P / 후속 VLA
                    ↓ 공통 팔 명령·제한
              so101_arm_bridge → STM32 → 양팔
```

주행·인식·리프트·조작·작업 관리를 분리한다. 기존 [Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)의 실행 경로를 검토해 Classical 모듈에 이식한다. 팔 브릿지는 현재 양팔 명령·피드백·정지만 담당한다. 목표 하드웨어는 STM32 USART1의 왼쪽 Waveshare 공유 버스에 왼팔·바퀴·리프트를 연결하고 UART4에 오른팔을 연결한다. 바퀴·리프트 패킷, 공유 버스 중재와 모바일 명령 감시 코어를 구현했으며 실제 명령 라우트·UART·정지 실행기는 연결 전이다. 세부 구조와 범위 정책은 아래 펌웨어 항목을 따른다.

## 현재 코드와 목표 설계

| 구성 | 현재 코드 | 다음 연결 |
|---|---|---|
| `home_robot_tasks` | 15단계 계획/재생/실행·연속 감시·인증 gateway·기구학 모의 주행·비동기 port | 실제 skill·독립 센서 monitor 연결 |
| `so101_arm_bridge`, `so101_interfaces` | 양팔 v2 명령·12축 피드백·소유권·제한·정지 | Classical과 VLA가 공통 팔 실행 경계를 사용 |
| `so101_description` | 팔 mesh·미실측 ALOHA Mini 조립체 모델 | 실측 치수·좌표·전신 충돌 scene·실제 관절 상태 |
| STM32 | 기존 팔 펌웨어 보존 | 통신·관절 제한·정지 계약 재사용 |
| Navigation·Perception·Lift | 실제 Nav2/AMCL+C 모형 주행·RGB-D 좌표 검사·lift C 코어 | detector·실물 드라이버/feedback·탐색 하위 단계 연결 |
| Classical P&P | 원본 고정·관절 경로/그리퍼 방향 FK 부분 이식 | 전체 IK/충돌/실행·이동 후 재관측과 보정 |
| VLA | 후속 개발 | 리더암 시연 모방학습, 팔·그리퍼 조작 비교 |

미구현 영역은 아래 계약을 기준으로 개발한다. 실제/가짜 장치는 동일한 목표·피드백·결과·취소 인터페이스를 사용하고 가짜 장치는 지연·누락·고장까지 재현한다. 빈 패키지 생성만으로 구현 완료로 집계하지 않는다.

## 실행 순서와 작업 계약

```text
요청 해석 → 출발 waypoint 이동 → 베이스 정지 → 대상 탐색
→ 작업면 미세 정렬 → 베이스 정지 → 리프트 조절 → 리프트 정지
→ 대상 재인식 → Pick → 집기 확인 → 운반 자세
→ 도착 waypoint 이동 → 베이스 정지 → Place 면 확인
→ 필요 시 미세 정렬·베이스 정지 → 리프트 조절·정지
→ Place 면 재인식 → Place → 배달 확인
```

주행 중 manipulation을 하지 않는다. 미세 정렬과 리프트 조절 중에도 팔은 해당 이동에 적합한 고정 자세를 유지한다. Nav2와 미세 정렬기는 베이스 명령을 동시에 소유하지 않는다. Lift 이동 후에는 변경된 TF와 새로운 관측으로 작업을 재계산한다.

현재 요청은 `fetch_object`와 `object_id`, `source_room`, `destination_place`다. 예제는 출발 방의 탐색 후보를 `sofa`로 제한하고 `bedroom_drop_zone`을 침대 위 지정 영역으로 정의한다. 특정 가구를 직접 지정하는 요청 필드는 향후 확장 대상이다. 현재 15단계 계획은 미세 정렬·리프트·재인식을 분리하며, `plan_only`, `executable=false`, `physical_task_completed=false`다. 첫 주행 전 팔·리프트 운반 상태를 확인하고 집기와 놓기 전에는 이동 후 새 관측을 요구한다.

오프라인 재생기는 단계별 시작/완료 근거와 실행 ID·시각을 검사한다. 각 단계의 timeout에는 조건 대기가 포함되며, 실패·취소 뒤 자동 재시도하지 않는다. 실제 센서 값을 판정하거나 정지 명령을 보내는 기능은 없으며, `required_response`는 필요한 대응을 보고할 뿐이다. 상세 형식은 아래 오프라인 재생 항목을 따른다.

실행 skill은 목표·timeout·관측 시각·좌표계·실패 원인을 전달하고 긴 동작은 진행·취소를 지원하도록 설계한다. 공통 결과 후보는 `SUCCESS`, `NOT_FOUND`, `UNREACHABLE`, `TIMEOUT`, `PLANNING_FAILED`, `HARDWARE_ERROR`, `CANCELLED`다. 현재 ROS 메시지로 구현된 결과 코드가 아니다.

실패 시 원인에 따라 정지·후퇴·재인식·재계획을 수행하고 시도 횟수와 시간 예산을 제한한다. 통신·하드웨어 오류를 일반 집기 실패처럼 자동 반복하지 않는다. 정상 취소에서 물체를 든 팔의 토크를 무조건 끄지 않으며 구체적인 정지·하중 유지 정책은 실물에서 검증한다. 목표 관절 도달과 물체 집기·운반 유지·배달 완료를 각각 판정한다.

## 베이스·관측·TF 설계

목표 주행 경로는 `Nav2 → 베이스 속도 명령(vx, vy, wz) → 옴니 기구학 → 바퀴 제어`다. 바퀴 상태에서 odometry를 만들고 IMU 융합 및 LiDAR 위치 추정을 연결한다. 실제 바퀴 피드백 제공 여부·단위·주기는 확인 전이며, 명령 속도를 실측 속도로 취급하지 않는다. `base_driver.RosBaseIO`의 stamped 속도/odom 계약을 추가했다(아래 재점검 절). `/scan`·미보유 `/imu`와 실제 장치 연결은 별도다. ROS 배포판과 연결 노드에 맞춰 메시지 타입·명령 시각·timeout을 확정한다.

목표 TF는 `map → odom → base_link → lift_link → 팔 베이스`다. `base_link`와 `lift_link` 사이의 prismatic joint 위치를 실제 리프트 피드백으로 `/joint_states`와 TF에 반영한다. 각 TF의 발행자를 하나로 정하고 RGB-D는 리프트에 장착하는 방향으로 연결한다. 단위는 m·rad·s를 사용하고 센서 optical frame과 본체 좌표축을 구분해 등록한다. 조립체 치수·센서 변환·리프트 영점은 하드코딩하지 않는다.

RGB-D는 물체 XYZ, 작업면 높이, 상대 거리·방향, 놓을 영역을 추정한다. ROI의 유효 깊이·중앙 영역 또는 분할 결과를 사용하고 이상치를 제거한다. 바운딩 박스 중앙 한 픽셀의 깊이를 집기 좌표로 바로 쓰지 않는다. 리프트 목표는 같은 좌표계에서 팔의 검증된 작업 높이를 재현하도록 계산하고, 이동 후 정지·재인식을 확인한다.

## Classical 재사용과 VLA 비교 경계

기존 P&P에서 인식 출력→좌표 변환→계획→팔 실행 경로와 검증 기록을 먼저 확인한다. 새 RGB-D 관측과 리프트가 바꾼 팔 기준 좌표를 연결하고, 고정 작업대 보정·높이 offset·전달 자세 중 새 환경에 종속된 부분만 교체한다. 원본 리비전·재사용 범위·재검증 항목을 이식 시 기록한다.

Classical과 후속 VLA는 같은 조작 목표·관측·상태·완료 확인·제한을 사용한다. VLA는 리더암 시연 데이터로 학습하며 초기에는 베이스·리프트를 고정하고 팔·그리퍼만 제어한다. 자연어 요청 해석과 VLA 조작 정책은 별도 책임이다. 모델 출력도 팔 실행기의 제한·취소·정지 검사를 통과해야 한다.

Classical은 기준 구현이자 운영 복구 경로로 유지한다. VLA 실패 후 즉시 다른 경로를 덮어쓰지 않고 정지·물체/팔 상태 확인·필요한 복구를 거쳐 전환한다. 비교 평가에서는 정책별 결과와 fallback으로 복구한 결과를 분리한다.

## 펌웨어 구성과 공유 버스

### 연결 구조

[AlohaMini 1 BOM](https://github.com/liyiteng/AlohaMini/blob/main/AlohaMini1/docs/BOM.md)은
베이스/리프트 4개와 follower 팔 12개를 모두 Feetech STS3215-C018,
12V, 1/345 감속비로 지정한다. 해당 세대
[README](https://github.com/liyiteng/AlohaMini/blob/main/AlohaMini1/README.md)는
Waveshare Bus Servo Adapter (A) 2개를 명시한다. 이는 공식 부품 기준이며,
기존 보유 팔의 실제 모델·전압까지 조회한 것은 아니다.

```text
Jetson 또는 Pi → STM32 (모터 통신의 단일 소유자)
                 ├─ USART1 → 왼쪽 Waveshare Adapter (A)
                 │            ├─ 커넥터 1 → 왼팔 6개: ID 1–6
                 │            └─ 커넥터 2 → 바퀴 3개: ID 8/9/10, 리프트: ID 11
                 └─ UART4  → 오른쪽 Waveshare Adapter (A)
                              └─ 오른팔 6개: ID 1–6
```

커넥터 두 개는 같은 물리 버스를 공유한다.
[Waveshare FAQ](https://docs.waveshare.com/Bus_Servo_Adapter_A/FAQ)는 동일 인터페이스와
동일 어댑터 내 ID 중복 금지를 명시한다. 좌우의 ID 1–6 재사용은 UART가 분리돼
가능하며 왼쪽 커넥터 사이에는 ID를 중복시키지 않는다. ID 7은 비워 둔다.
8/9/10은 왼쪽/뒤쪽/오른쪽 바퀴 순서로 공식 소프트웨어 구성과 맞춘다.

어댑터는 전압 조정 기능이 없고 입력 전압을 그대로 공급하며, 공식 허용 전류는
5A다. 10개 모터가 사용하는 전류를 실부하에서 측정하고 배선·커넥터·전원 분배를
검증한다. 부하 전원은 정격에 맞는 분배 경로를 설계하고 데이터·접지는 공통으로
유지한다. 구체적인 전원 주입 방식은 실제 보드와 배선을 확인한 뒤 정하며,
어댑터에 통신 연결이 가능하다는 이유로 전체 전류를 보드에 통과시키지 않는다.

STM32가 이 버스를 소유하는 구성에서는 같은 어댑터의 USB에 LeRobot Host 등
두 번째 명령 송신자를 동시에 연결하지 않는다. 바퀴·리프트도 STM32 경로를
확장해 제어한다. 기존 별도 Host와 팔 STM32를 같은 데이터선에 병렬 연결하지 않는다.

### 구조 점검 결과

| 위치 | 확인한 상태 | 처리 |
|---|---|---|
| `binary_control.c` | 약 4,480줄, 프레임 응답·세션·여러 검증 세대·동작·정지 처리 혼재 | 이번에는 wire/session 의미를 유지. 다음 분리는 v1 진단과 v2 실행·정지 핸들러 |
| `servo_bus.c` | 변경 전 2,376줄, UART 복구·패킷 구성·팔 설정·순차 실행 혼재 | 패킷 조립과 팔 설정을 분리, HAL 수신·복구 시간 동작 유지 |
| `right_servo_bus.c` | 변경 전 962줄, 왼쪽과 READ/WRITE/checksum 중복 | 공통 패킷 모듈 사용 |
| `sts3215_packet.c` | 6축 위치 SYNC WRITE만 공통화 | READ·WRITE·16-bit register SYNC WRITE 공통화 |
| `servo_joints` | 팔 서비스 설정과 UART 구현이 같은 파일 | `servo_joint_config.h/.c`로 분리 |
| `bimanual_operational_limits.c` | 실측 12축 raw/urad 범위와 raw wrap 변환 | 값 유지, 잘못된 arm 선택값 방어 명시 |
| 기존 정지 경로 | 팔 ID 1–6의 토크 해제·readback만 처리 | 모바일 전체 정지로 사용 불가; 아래 별도 정지 계약 필요 |

기존 UART/DMA 복구·4 ms telemetry timeout·5 ms 출력 주기는 실기 이력이 있는
동작이므로 공통화 과정에서 임의로 재작성하지 않았다. 새 공통 패킷 함수는
HAL을 사용하지 않으며 원래의 정상 프레임 바이트를 유지한다. READ/WRITE의
잘못된 ID, broadcast ID, 길이·주소 범위 초과는 전송 전에 거절한다.

### 통신·모터 그룹 모듈

- `firmware/stm32_actuator/src/sts3215_packet.c`: 패킷 구성과 checksum.
  왼쪽·오른쪽 동기 읽기, 비동기 telemetry 요청, register write에 적용했다.
- `firmware/stm32_g474_single_arm/Core/Src/servo_joint_config.c`: 기존 팔
  서비스 설정·PID·토크 제한 표. 바퀴·리프트 항목을 이 표에 추가하지 않는다.
- `firmware/stm32_actuator/src/motor_groups.c`: 왼팔/오른팔/바퀴/리프트의
  물리 버스, 대상 ID, 기대 모드와 그룹별 패킷을 정의한다.
- 양팔 DMA dispatcher는 `motor_groups`의 팔 그룹으로 패킷을 만든다.
  각 팔 위치 패킷에는 해당 버스의 ID 1–6만 들어간다.

| 그룹 | 버스 | 수 | 모드 | 명령 의미 |
|---|---|---:|---|---|
| 왼팔 | LEFT_SHARED | 6 | Position 0 | 6개 관절 목표 위치 |
| 오른팔 | RIGHT_ARM | 6 | Position 0 | 6개 관절 목표 위치 |
| 바퀴 | LEFT_SHARED | 3 | Velocity 1 | 바퀴 3개 raw 속도 |
| 리프트 | LEFT_SHARED | 1 | Velocity 1 | 리프트 raw 속도; 높이 코어 구현, 장치/wire 연결 전 |

바퀴·리프트에 팔 위치 패킷을 만들거나 팔에 모바일 속도 패킷을 만드는 요청은
거절한다. STS3215 속도는 16-bit two's complement가 아니라 bit 15 방향 비트를
쓰는 sign-magnitude 표현이다. 음수·0·범위 초과를 시험했다.
[LeRobot Feetech 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py)을
참조했다. `motor_groups`는 **기대 모드와 데이터 형식을 구분**할 뿐 실제
서보의 Operating Mode를 읽거나 설정하지 않는다. 실제 출력 전 mode readback이 필요하다.

그룹 모듈은 패킷만 구성한다. 속도·watchdog 검사는 아래 별도 코어에서 담당하며,
실제 UART 연결·기구학 단위 변환·리프트 제어기의 장치 연결은 남아 있다. zero-speed 패킷
생성은 실제 정지나 리프트 하중 유지의 증거가 아니다.

### 공유 버스 중재와 모바일 감시 코어

`firmware/stm32_actuator/src/shared_bus.c`는 동적 할당 없이 한 UART의 진행 중
거래를 하나로 제한한다. 소유권은 TX뿐 아니라 RX/응답 정리까지 포함한다.
STOP 대기 중 일반 요청을 거절하고, 일반 거래는 다음 팔 슬롯까지 남은 시간에
전체 처리 예산이 들어갈 때만 허용한다. 이미 진행 중인 거래는 STOP도 덮어쓰지
않는다. timeout/전송 오류는 소유권과 fault를 유지하며 HAL abort·RX 정리 후
idle을 확인한 명시적 복구만 허용한다. 거래 token은 늦은 callback이 새 거래를
해제하는 것을 막는다. token을 다 쓰면 재사용하지 않고 fault한다.

이 코어 자체는 주기 스케줄러나 HAL 드라이버가 아니다. 호출자가 남은 arm slot,
최악 TX/RX 예산, 완료 이벤트와 실제 idle을 공급해야 한다. 보드 연결은 다음의
`servo_transport`가 담당하며 주기 송신은 아래 `mobile_output`/보드 writer가 연결한다.

#### 보드 전송 계층

`Core/Src/servo_transport.c`는 물리 UART당 소유자 한 개를 둔다. 좌우 서비스
read/write·IT feedback·동기 위치 송신·양팔 DMA가 모두 공통 송신 함수를 사용한다.
기존 응답 parser/복구 코드는 `Owned` 내부 함수에 유지하고, 공개 함수가 거래의
시작부터 응답 처리/정리까지 소유권을 잡는다. TX가 끝나도 응답 대기 중이면 다른
writer는 들어갈 수 없다. 등록을 다시 호출해도 진행 중 소유권/token을 초기화하지 않는다.

양팔 DMA는 **두 UART를 모두 예약한 다음** 왼팔 송신을 시작한다. 한쪽이 사용 중이면
어느 팔에도 패킷을 보내지 않는다. ISR은 완료/오류 사건을 기록하고 main loop의
`BimanualServoDispatch_Poll`이 처리한다. HAL 호출 안에서 즉시 도착한 완료 사건도
양팔 dispatch 상태 설정 뒤 처리한다. 중단할 때는 자신의 거래만 abort한다.

새 대기열 연결은 `ClaimQueued → RX 준비/Transmit* → 응답·quiet 확인 → CompleteQueued`다.
기존 팔이 사용 중이면 모바일 대기열에서 항목을 꺼내지 않는다. 만료/오류/STOP은
보드 계층에도 차단을 걸어 기존 팔 서비스가 router의 정지 상태를 우회하지 못한다.
복구는 RX 정리 확인과 실제 TX abort 성공을 요구하며, 정지 전송 뒤 독립 정지 확인을
받은 `ResumeQueued`에서만 정상 송신을 다시 허용한다. 수신 정리 전용 lease는
송신할 수 없으며 다른 소유자를 선점하지 않는다.

통신 선로 BUSY 또는 abort 실패를 단순 idle로 덮어쓰지 않는다. feedback 종료의
RX abort와 짧은 quiet 대기는 정지/오류 경로에 있고 정상 5 ms 위치 송신에 넣지 않았다.
실제 보드의 IRQ/버스 타이밍은 후속 검증 대상이다. 최대 100 ms짜리 기존 동기 서비스
함수도 남아 있으며 `ServoTransport_SetPeriodicMode`가 모바일 운용 중 이를 차단한다.

`mobile_supervisor.c`는 wheel 3개+lift 1개의 전체 속도 목표를 검사한다.

| 상태/입력 | 동작 |
|---|---|
| 부팅 또는 잘못된 설정 | DISABLED; 출력 허용 없음 |
| 명시적 arm | 새 증가 session, 4축의 최신 건강한 피드백·velocity mode·lift 원점·실측 정지 요구 |
| 명령 | session 일치, 증가 sequence, 설정 속도 한계, lift 경계 확인 |
| timeout/피드백 상실/mode·원점 이상/이동 중 lift 경계 | STOP_LATCHED, 4축 목표 0, 첫 원인 보존 |
| 복구된 피드백·새 명령 | 자동 재출발 없음; 새 session으로 명시적 재활성화 |
| measured_stopped | 최신 실제 속도와 허용 정지 속도로 별도 판정 |

시간은 STM32 단조 증가 ms이며 32-bit wrap을 처리한다. timeout은 경계에서
만료한다. 수신 callback에서도 먼저 만료를 검사해 늦은 명령/피드백이 단절을
숨기지 않도록 한다. 같은/과거/미래 피드백은 거절한다. 합성 feedback의 시각은
네 축 중 **가장 오래된 실제 샘플 시각**이어야 하며 수신 시각으로 덮어쓰지 않는다.

물리값 기본값은 없다. 속도·관측/명령 timeout·높이 한계는 설정으로 공급한다.
리프트 속도 양수는 상승을 뜻하며 실제 모터 방향을 확인해 어댑터에서 변환한다.
코어에 넣는 현재 높이는 이미 원점 보정된 µm이고 목표 속도는 서보 단위다.
이 경계 검사는 감속·제동 거리·높이 추정·하중 유지 제어를 대신하지 않는다.
wire에서 지연된 host 명령의 절대 만료 검사도 별도 프로토콜 계층의 책임이다.

두 코어는 단일 main-loop 소유자를 전제로 하며 ISR에서 동시에 호출하지 않는다.
현 resident 팔 펌웨어에 실제 모바일 모션 경로가 생긴 것은 아니다. 팔 tracking
fault를 모바일 정지로 연결하는 전체 정지 실행기와 물체/lift 유지 정책도 필요하다.

### 관절 가동 범위와 설정 생성

수정 대상은 이전 작업의 **관절별 제한**이다. 목표는 새 조립체에서 SO-ARM의
기구적으로 가능한 전체 범위를 사용하는 것이며 팔 베이스 아래의 접근도 포함한다.
현재 코드에 고정 Cartesian Z 검사가 없다는 사실로 이 작업을 완료했다고 보지 않는다.

| 위치 | 의미 | 모바일 전환 |
|---|---|---|
| `servo_joint_config.c` | 예전 v1 점검/서비스의 좁은 raw 범위·PID·토크 | 정비용 프로파일로 구분; v2 전체 범위로 오인하지 않음 |
| `config/bimanual_operational_limits.json` | 과거 양팔에서 수동으로 확인한 v2 raw/urad 범위 | 현재 기본값; 새 장착 기계 최대 범위를 증명하지 않음 |
| `so101_arm_macro.xacro` | 팔 참조 모델의 각도/그리퍼 형상 범위 | 실제 영점/방향/그리퍼 gap과의 변환 보정 후 정합 |

canonical JSON 한 곳을 기준으로 `generate_joint_limits.py`가 firmware C 표와
ROS 패키지 설정 사본을 생성/검사한다. STM32 CMake 빌드도 검사에 의존하므로
두 설정이 다르면 컴파일 전에 거절한다. 생성 영역 밖의 wrap/goal mapping과
과거 shadow 표는 보존한다. CubeIDE 직접 빌드는 별도로 검사 명령을 실행한다.

```bash
python3 tools/setup/firmware/generate_joint_limits.py --check
# canonical JSON을 검증된 새 보정으로 갱신한 뒤:
python3 tools/setup/firmware/generate_joint_limits.py --write
```

host loader와 생성기는 같은 validator로 12축 순서·정수형·int32 범위·raw/urad
정합·coordinate·wrap의 유일성을 검사한다. float/string/bool을 정수로 자동 변환하지
않고 JSON 중복 키도 거절한다. schema 1의 zero=2048과 방향은 고정 계약이다.
영점이 바뀌면 schema/calibration과 host handshake도 함께 갱신해야 한다.
JSON의 과거 source 경로/hash는 이력 근거이며 모바일 보정 검증을 뜻하지 않는다.

**이번에는 실제 관절 한계를 넓히지 않았다.** 기계 끝점·케이블·자기 충돌의 새
실측값이 없기 때문이다. 그 값을 임의로 0–4095로 대체하면 shoulder wrap과
그리퍼 의미도 달라진다. 지금 준비한 것은 설정의 단일화와 잘못된 변경 차단이다.
도착 후 좌우 끝점/방향/영점을 측정하고 JSON·firmware·host·로봇 모델 좌표를
함께 갱신한다. URDF는 영점과 jaw geometry가 달라 그대로 자동 생성하지 않는다.

범위는 기계/관절 보호, 조립체 자기 충돌, 현재 환경 충돌로 나누어 적용한다.
작업대에 종속된 관절 제한을 기계 한계로 영구 고정하지 않는다. 속도·변화량·추종
오차·통신 정지와 케이블/기계 끝점은 작업 범위 확대와 별도로 유지한다.

### 남은 펌웨어 연결과 관측성

- 모든 USART1 writer/read를 중재에 연결하고 팔 tick·feedback 최대 나이를 유지한다.
- 모바일 wire 메시지·capability·명령 절대 만료와 mode readback을 연결한다.
- 정상 취소의 wheel 실측 정지·lift/팔 하중 유지와 고장/비상 경로를 구분한다.
- homing·엔코더 누적·높이 제어, 속도/가속/감속 제한, 전류/온도/전압 진단을 추가한다.
- 진단은 명령 session/sequence, 실제 feedback 나이, 버스 owner/token, 지연·누락·복구
  수, 정지 원인과 원점 유효성을 포함한다. UART 전송 성공과 물리 결과를 구분한다.

실행 순서와 통과 기준은 [로드맵 F1–F3](ROADMAP.md#작업별-구현-범위와-검증)에 둔다.

## 오프라인 작업 재생 계약

### 로봇 없이 실행

저장소 루트에서 Python 3.10 이상으로 실행한다. 아래 두 경로는 Python 표준
라이브러리만 사용하며 ROS·센서·모터·VLM·네트워크에 연결하지 않는다.

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.cli \
  --world config/home.example.json --request config/fetch_remote.example.json

PYTHONPATH=ros2_ws/src/home_robot_tasks python3 -m home_robot_tasks.replay_cli \
  --world config/home.example.json --request config/fetch_remote.example.json \
  --recording config/fetch_remote.replay.example.json
```

예제 기록은 모든 조건을 만족하도록 만든 `synthetic` 자료다. 실제 센서나 로봇
성능 자료가 아니다. 다른 결과를 시험하려면 복사본의 조건을 `false`로 바꾸거나
실패·취소 이벤트로 이후 기록을 대체한다. 종료 이후 이벤트는 거절한다.
명령 종료 코드는 재생 완료 `0`, 실패·취소·미완료 `1`, 잘못된 입력 `2`다.

### 기록 형식과 시간

`schema_version=1`, `run_id`, `evidence_source`, `events`를 저장한다.
`evidence_source`는 `synthetic` 또는 `recorded_assertions`이며 실제 이미지 경로나
ROS bag을 직접 읽는 기능은 아직 없다. 각 이벤트는 고유 `event_id`와 동일한
`run_id`, `at_s`, `kind`를 갖는다. 단계 이벤트는 계획에 나온 `step_id`를 참조한다.

- `start`/`success`: `evidence_at_s`, `evidence`가 필요하다. 계획의 해당
  `start_requires`/`success_requires`를 모두 boolean `true`로 충족해야 한다.
- `failure`: `reason`을 기록한다. 준비 중 장치 오류도 즉시 종료할 수 있다.
- `cancel`/`tick`: 단계 ID나 근거를 갖지 않는 전역 이벤트다.
- 모든 시간은 **실행 시작 이후 경과 초**다. 장치 시간이나 UTC와 혼합하지 않는다.
- 단계 시간 예산은 이전 단계 완료부터 조건 대기와 실행을 모두 포함한다.
  `tick`으로 결과가 오지 않는 상황을 시험하며 예산 경계에서 시간 초과한다.
- 근거는 해당 단계의 준비/시작 시각 이후이고 이벤트 시각보다 미래가 아니어야 한다.
  기본 최대 나이는 1초다. 시간 예산과 1초는 소프트웨어 시험 후보값이며
  실물 운용 허용값은 센서·제어 지연 측정 후 별도로 정한다.
- 파일이 중간에 끝나면 `WAITING`/`RUNNING`을 보고한다. 자동 성공이나
  자동 시간 초과로 꾸미지 않는다.

실제 연결 시 각 어댑터가 관측 시각·보정 ID·TF·대상 ID와 시작/완료 조건을
계산하고 주행 중 하중·위치 추정·정지 상태를 계속 감시해야 한다.
`required_response`는 필요한 대응을 기록할 뿐 실제 정지를 보낸 것이 아니다.

## 펌웨어–ROS 브릿지 연결 보강

현재 ROS 실행기는 `so101_arm_bridge/bimanual_stream_node`이며 예전
`single_arm_bridge` 실행기는 제거됐다. 보드 폴더의 single_arm 이름은 기존
HAL 프로젝트 경로로 남아 있고, resident 후보는 protocol v2/12축이다.
현재 후보 firmware ID와 ROS 기대값은 **0x00024908**으로 함께 갱신했다.
이전 0x00024809 실기 승인 기록과 구분하며 보정/관절 숫자는 변경하지 않았다.

`mobile_framed`는 v2 메시지 64/65로 AM 36B 요청과 AS 64B 응답을 운반한다.
기존 binary_control switch에 handler를 등록했으며 모바일 endpoint는 기본
미연결이다. 새 보드 코드에서 실측 설정/feedback을 붙이기 전까지 명시 거절한다.
`MobileV2Exchange`는 팔 transport 객체를 재사용하고 전체 요청/응답 구간을
공통 lock으로 보호한다. 외부 frame sequence와 내부 mobile sequence를 각각
검사하고 부분 송신·재부팅·CRC/지연 오류를 검출한다. 자세한 바이트 계약은
[프로토콜](../protocol/README.md)에 기록했다.

`bus_schedule`은 실제 팔 출력 clock에 맞출 epoch와 주기를 받아 arm 예약 구간,
다음 arm slot 전 guard를 제외한 여유 시간, wheels/lift/feedback 갱신 시점을
계산한다. 짧은 시간 차이를 누적해 uint32 wrap을 통과하고 poll 공백이 허용치를
넘으면 stop_required를 유지한다. 밀린 명령을 몰아서 보내지 않는다. due는 새
입력을 enqueue할 기회이며 enqueue 실패·freshness·장치 RX 처리 책임은 보드
driver에 있다. `MobileServoOutput`이 실제 ISR epoch와 main loop에 연결하며 기본 부팅은 미설정이다.

`ServoTransport_SetPeriodicMode`는 독립적인 전체 정지 확인과 두 UART idle을
요구한다. periodic 모드에서는 기존 blocking read/write/smoothstep/설정 경로와
blocking HAL TX를 차단한다. 비동기 IT/DMA와 오류 정리는 유지한다. 기본은 기존
maintenance 모드이며 자동 전환은 없다. 실제 모바일 driver가 초기 설정을 끝낸
뒤 전환해야 한다. 정상 운용 중 서비스 허용을 위한 자동 모드 해제는 금지한다.

## 정지 후 집기 관측과 시점 재탐색

사용자 결정: **LDS-01 → Nav2 2D 주행**, **리프트 장착 D415 → 집기용 RGB-D →
좌표 변환 → MoveIt**. 이는 기존 집기 전용 기준이다. 2026-09-16 재점검부터 native depth를 상부 장애물/통과 공간 검사에도 사용하며 아래 재점검 절이 최신 기준이다.

`stationary_rgbd.StationaryRGBD`는 정합·보정된 RGB 픽셀과 동일 optical frame의
깊이(m), 해당 영상의 CameraInfo, 노출 시각의 root_from_camera TF를 받는다.
기존 `perception.depth_target`의 ROI 이상치/강체 변환 검사에 다음 조건을 더한다.

- 베이스·리프트의 새 정지 증거와 안정화 이후, 촬영 요청 이후의 영상만 허용한다.
- base/lift가 움직일 때마다 바뀌는 pose_revision이 현재 상태와 같아야 한다.
  움직였다가 같은 위치로 돌아온 경우에도 revision을 바꿔야 한다.
- 카메라 frame·root frame·보정 ID·해상도·정합/왜곡 보정 여부를 확인한다.
- TF는 노출 시각과 허용 오차 이내이고 마지막 정지 이전 것이면 거절한다.
- 거리 범위 밖 깊이는 제외하며 최소 유효 ROI와 이미지 내부 픽셀만 사용한다.
- 결과는 XYZ 후보다. `grasp_pose_resolved=false`, `motion_authorized=false`이며
  접근 방향·IK·충돌 검사·이동 이후 재관측을 대신하지 않는다.

`search.SearchSession`은 위치 이름(sofa)과 유한 관측 시점 목록을 받는다.
`MOVE_TO_VIEW → SETTLING → CAPTURE_RGBD → FOUND/다음 시점`으로 진행한다.
미확인 영역 예상 수가 많은 시점을 우선하지만 실제 관측 범위는 관측자가 준
visible/occluded 증거만 누적한다. 예상 시야만으로 확인 완료라 하지 않는다.
빈 영상은 다음 시점으로, 불확실한 후보/잘못된 영상은 제한 재시도로 처리한다.
후보 소진은 `NOT_FOUND_UNCONFIRMED`이며 `absence_proven=false`다. 쿠션을
옮기는 가림 제거 조작은 포함하지 않는다.

명령 ID로 늦은 응답을 거절한다. timeout·취소·scene revision 변경은 STOPPING으로
들어가고, 이전 이동 backend 종료와 새로운 베이스/리프트 정지·팔 hold 증거가
모여야 종료한다. 정지 응답이 없으면 제어가 종료됐다고 보고하지 않는다.

현재 카탈로그의 view ID는 의미 이름이며 실측 pose나 자동 시점 생성기가 아니다.
`search_simulator`는 합성 도착·가림·깊이를 공급한다. 실제 detector·FOV 가시성
계산·Nav2/리프트/MoveIt 어댑터 및 fetch 연결은 남아 있다. 기존 TaskExecutor의
`search`는 연속 정지 조건이므로 이동을 허용하도록 단순 완화하지 않는다. 통합 시
하위 이동/리프트/촬영 각각의 조건을 적용해야 한다. 보정값·시간·confidence·거리
한계는 `config/search_sofa.simulation.json`의 PC 후보이며 실물 승인 설정이 아니다.

## 하드웨어 없는 실행기와 앱 연결 경계

`execution.TaskExecutor`는 재생 파일을 읽는 대신 adapter의 observe/start/poll/
request_stop/poll_stop을 호출한다. 현재 simulation 모드만 허용한다. `FakeRobot`은
합성 조건·지연·실패·오래된/다른 goal feedback·정지 응답 누락을 제공하며 물리
운동이나 센서 성능을 시뮬레이션하지 않는다. 기존 `FetchReplay`는 기록 검증용으로 유지한다.

WAITING → RUNNING → 다음 단계 또는 SUCCEEDED로 진행한다. 각 단계는 고유
goal ID와 시작/완료 조건을 가지며 관측 시각·준비/시작 이후 증거·timeout을 검사한다.
주행 중 위치 추정·운반 자세·lift 정지, 조작 중 base/lift 정지, 운반 중 load_retained를
계속 확인한다. 실패/취소는 STOPPING으로 들어가 정지 확인을 기다린다.

정지 응답이 없으면 STOP_UNCONFIRMED가 되고 **제어권을 유지**한다. 이후 유효한
정지 증거를 받으면 종료한다. stop 전송 오류도 제어권을 해제하지 않는다. 집기
도중의 취소에서도 gripper 하중 보존을 요청한다. 현재 boolean 증거는 가짜 장치의
합성값이며 실제 장치 판정은 후속 어댑터의 책임이다. 유한 재시도·운영 복구는 남아 있다.

`application.RobotApplication`은 fetch와 navigate_to를 같은 `TaskLease`와
실행기에 연결한다. 한 작업 중 다른 작업이 베이스 명령을 덮어쓰지 않는다.
동일 request_id 재전송은 중복 실행하지 않고 내용이 다르면 거절한다. 취소/실패나
semantic fetch 이후에는 위치를 임의 추정하지 않고 재위치 추정을 요구한다.
상태/계획 사본을 반환하므로 클라이언트가 내부 계획을 변경하지 못한다.

### 향후 휴대폰의 3D 지도 선택

```text
휴대폰 3D 지도에서 바닥 선택 + 최종 방향
 → map ID/revision·floor·frame·point(x,y,z)·yaw를 보낸다
 → 지도/바닥/장애물/로봇 여유/경로 검사
 → 공통 작업 실행기와 베이스 제어권
 → Nav2 NavigateToPose adapter (PC 통합 구현)
 → 상태·진행·취소 결과를 앱에 반환
```

3D 시각화 지도와 주행용 바닥 지도를 분리한다. 현재 `navigation.NavigationMap`은
고정 격자 A* 경로 가능성 검사이며 Nav2/SLAM이 아니다. 바닥 높이와 맞지 않는
소파/벽 위 클릭, 다른 floor/frame/revision, 지도 밖·unknown·장애물·로봇 여유가
부족한 지점·경로 단절을 거절한다. 점유 셀과 지도 경계에 로봇 반경+여유를 보수적으로
팽창시킨다. 경로는 축에 평행한 셀 중심 경로이며 곡선/동적 장애물/전신 충돌은 미지원이다.

Nav2에는 지도 좌표의 목표 pose를 전달한다. [공식 NavigateToPose 계약](https://api.nav2.org/actions/jazzy/navigatetopose.html)의
목표·진행·결과를 `Nav2Port`에서 변환한다. 합성 ROS 액션 서버와 통신/취소를
검사했고 실제 Nav2/AMCL과 C 모터 모형의 주행도 연결했다(아래 PC 통합 절). 별도의 `KinematicRobot`은 정적 격자 경로를 XY/yaw로 따라가며 속도
감소 후 정지를 확인한다. 접촉·미끄럼·전신 충돌 물리 시뮬레이터는 아니다.

앱 UI·3D mesh 생성·WebSocket은 아직 없다. 인증 HTTP gateway에서 동일 요청/
상태/취소 계약을 사용한다. 기본 loopback이며 LAN은 명시적인 TLS 설정을 요구한다.
세부 필드·예제는 [프로토콜](../protocol/README.md)의 앱 계약을 따른다.

### 베이스 수학·모바일 명령 코덱

`base_motion`은 각 wheel 중심·구름 방향·반경을 입력받아 정/역기구학과 공동 비율
속도 포화를 계산한다. 입력은 base vx/vy [m/s], wz [rad/s], wheel [rad/s]다.
실측 치수·방향·servo speed scale은 가정하지 않는다. `WheelOdometry`는 이전
실측 속도를 유지하는 SE(2) 적분을 하며 시간 역행과 큰 관측 공백을 거절한다.
미관측 시간의 이동을 만들어내지 않는다. wheel slip·추정 공분산·ROS 연결은 남아 있다.

C/Python `mobile_wire`는 ARM/VELOCITY/STOP의 동일 36-byte 초안 프레임과
CRC32C를 교환하고 `mobile_supervisor`에 admission을 연결한다. 짧은 절대 명령
만료도 적용하며 수신 시점부터 최대 watchdog을 다시 부여하지 않는다.
`mobile_endpoint`·`MobileClient`에 capability·bounded RTT 동기화·64-byte
응답/feedback·stream parser를 구현했다. resident v2 host parser에도 MOBILE_REQUEST/RESPONSE handler를 등록했다. 실제 보드 endpoint는 기본 미연결이며, 이 상태에서는 명시적으로 거절한다.
[프레임 규격](../protocol/README.md)을 따른다.

### 추가한 실행·검증 모듈

- `bus_router`: 종류별 고정 슬롯에 명령을 복사하고, 진행 중인 전송 버퍼를
  보존한다. STOP은 일반 대기열을 폐기하고 전송 뒤에도 차단을 유지한다.
  `resume(stop_confirmed)`는 정지 확인 뒤 호출자가 명시적으로 요청해야 한다.
- `system_stop`: 왼쪽 모바일 4축 zero와 오른팔 hold를 전송한 뒤 왼팔 hold를
  전송한다. 모든 전송 이후의 새 base/lift/arm/필요한 load 증거가 모여야
  CONFIRMED다. timeout 뒤 UNCONFIRMED에서도 차단을 유지한다. hold 목표는
  호출자가 검증한 현재 관절값이며 torque off를 정상 정지로 사용하지 않는다.
- `lift_controller`: UNHOMED/HOMING/READY/MOVING/HOLD_PENDING/HOLDING/FAULT.
  새로운 encoder/current 샘플만 접촉 dwell을 증가시키며 wrap/점프/공백/과전류를
  검사한다. hold 증거와 높이 허용 오차를 별도 확인한다. 원점 초기화는 명시적이며
  재부팅/리셋 뒤 자동 homing하지 않는다. 실물 current 임계·속도/높이 변환과
  제동 모델은 미확정이다. 높이 제어기는 아직 모바일 wire에 연결하지 않았다.
- `RoutedSkillAdapter`: 작업 상태를 비동기 port에서, 성공 조건을 독립 monitor에서
  얻는다. 액션의 성공 신호가 집기/위치 추정/정지 증거를 생성하지 않는다.
  취소 중 접수 대기 goal이 남으면 제어권을 해제하지 않는다.
- `classical`·`grasp_yaw`: 원본 리비전·해시는 `config/pnp_source.json`으로 고정.
  반대팔 현재 hold를 포함한 12축 유한 경로와 URDF 손가락 방향 계산을 재사용한다.
  기존 작업대 offset/고정 자세는 복사하지 않았다. 이동식 mount 변환에 lift 위치가
  없으면 거절한다. 전체 IK/충돌 계획/실행 이식은 남아 있다.
- `perception`: 기록/합성 ROI 깊이의 유효 샘플·이상치·내부 파라미터·관측/TF
  시각·frame/calibration ID를 검사해 root 좌표를 계산한다. 실제 detector와
  이미지 입력 드라이버는 별도다. 출력만으로 motion을 허용하지 않는다.

`pc_mobile_simulator.c`는 실제 endpoint/supervisor/router/STS 패킷 코드를
Python client와 연결한다. feedback은 직전 목표를 이상적으로 따르는 가짜 값이다.
기본 gateway의 기구학 모형과 C 모형은 별도 구현이고, HTTP/Nav2 통합 시험에서는
C 모형을 같은 실행 경로로 연결한다. 보드 주기 writer/HAL은 별도 fake HAL 시험을
수행하며 실제 feedback/hold·초기화 연결은 남아 있다.

### 개발 후보 재현과 복구

`requirements/offline.lock.txt`는 Python 3.12에서 확인한 패키지 버전을 고정한다.
`verify_offline.py`는 현재 dirty/untracked 소스까지 해시로 기록하고, 실행 중
소스가 바뀌면 후보 생성을 실패시킨다. host 검사·C build/CTest·1시간 가속
모의 통신·fetch/navigation 예제를 실행하며, 옵션으로 두 STM32 build·ROS 모델/
액션 검사를 추가한다. 결과와 선택 소스 묶음은 `output/offline-verification/`다.
이 묶음은 소스 후보이며 검증된 Jetson 이미지나 flash 가능한 모바일 완성본이 아니다.

gateway는 SQLite로 요청을 보존한다. 재시작 시 미완료 작업을 INTERRUPTED로
바꾸고 실행하지 않는다. 이전 후보로 돌아갈 때는 서버를 종료하고 기록 DB를
보존한 채 해당 소스/의존성을 복원한다. 새 후보에서도 명시적으로 새 요청을
보내기 전에는 작업을 시작하지 않는다. OS/ROS 이미지·실측 설정의 배포 고정은
보드 환경 확인 후 진행한다.

### 실제 MoveIt 계획의 PC 검증

`moveit_config.simulation_parameters`가 이동식 조립체·SRDF·KDL position-only IK·
OMPL 설정을 만든다. 로봇 모델과 planner 모두 기존 canonical 관절 범위와 URDF
범위의 교집합을 사용한다. 기본 속도/가속 제한은 PC 후보이며, 원본 관절 수치는
바꾸지 않았다. 베이스와 리프트는 arm 그룹 밖에 두고 관측한 상태로 고정한다.

`check_moveit_planning.py`는 격리된 ROS namespace에서 실제 move_group과
robot_state_publisher를 실행한다. 합성 joint_states로 두 리프트 높이의 양팔 IK,
다른 초기값에서 목표 위치 오차, 팔만 포함한 OMPL 경로, 월드 장애물의 충돌/계획
거절을 검사한다. 실행 액션 capability는 제외하며 컨트롤러나 모터를 시작하지 않는다.
이 시험은 실제 IK/충돌 라이브러리 시험이지만 영상 인식이나 집기 성공 시험은 아니다.
5축 position-only IK의 위치 해만으로 그립퍼 접근 방향까지 해결됐다고 보지 않는다.

설치된 Jazzy MoveIt 2.12.4에서 서비스 시험 후 종료 시 callback 객체 해제 충돌이
재현됐다. `--retain-moveit-plugin`은 **이 자식 프로세스에만** capability 라이브러리를
LD_PRELOAD로 유지해 정상 종료시킨다. 경로·라이브러리 SHA와 종료 코드를 보고서에
기록하며 시스템 파일은 바꾸지 않는다. 상위 프로젝트의 유사한
[종료 객체 수명 문제](https://github.com/moveit/moveit2/issues/1597)는 참고 자료이며
이 환경의 동일 원인/수정 버전이 확정됐다는 뜻은 아니다. 배포 환경에서는 해당
우회가 필요한지 다시 시험한다. 검증 도구는 종료 오류도 실패로 처리한다.

### 실제 Nav2·AMCL과 앱/C 브릿지의 PC 통합

`tools/run/check_nav2_closed_loop.py`는 다음 경로를 한 실행에서 검사한다.

```text
인증 HTTP 지도 지점 요청 → Gateway → RobotApplication/TaskLease → RoutedSkillAdapter → Nav2Port
 → 실제 Navfn A* / DWB 옴니 제어 / NavigateToPose BT
 → cmd_vel → 옴니 기구학 → MobileClient / 공통 v2 transport
 → 실제 C parser·endpoint·supervisor·bus router / 합성 STS feedback
 → wheel odometry + 합성 2D scan → 실제 AMCL → map/odom TF → Nav2
```

지도는 6×6 m 합성 방이며 장애물 우회·도착 위치·원형 베이스 여유를 검사한다.
HTTP 인증·짧은 요청 permit·중복 실행/동시 작업 차단을 검사하고, 상태 조회와
취소 후 NavigateToPose와 하위 FollowPath
종료, STOP 이후 새 C 속도 feedback을 모두 확인한 뒤 제어권을 반환한다.
`RoutedSkillAdapter`는 backend 성공보다 센서 증거가 늦게 오면 완료를 기다린다.
대기 중에도 TaskExecutor의 전체 제한 시간과 연속 조건을 적용한다.

이 시험은 `exact_goal_simulation=False`로 앱을 만들고, 도착 후 다음 요청 전에
AMCL/TF 위치를 명시적으로 공급한다. `FakeRobot`용 기본 설정의 정확한 목표 도착
가정을 실제 Nav2 시험에 재사용하지 않는다. 초기화 때 AMCL의 미래 시각 TF가
새 navigator의 odom cache와 겹칠 때까지 준비를 기다린다.

설정은 `config/nav2.simulation.yaml`과 `navigate.simulation.xml`이다. 현재 BT는
경로 생성→추종이며 동적 재계획·복구는 후속이다. DWB 방향 전환 잠금은
목표 근처 정체에서 1초 후 해제하도록 후보 설정했고, 별도의 10초 진행 감시는 유지한다.
설정 의미는 [Jazzy OscillationCritic 소스](https://api.nav2.org/nav2-jazzy/html/oscillation_8cpp_source.html)를 따른다. 센서·지도·바퀴 치수/속도 단위·
팔/리프트 운반 조건은 합성이며 미끄럼·하중·제동 거리를 검증하지 않는다.
2D 라이다만 주행 입력으로 사용한다. HTTP→실제 Nav2 전 경로를 loopback에서
시험한다. 실제 휴대폰 UI/페어링과 fetch/search/manipulation 통합은 남아 있다.

`--fault scan_loss`/`--fault tf_loss`는 두 번째 주행 중 해당 출력을 중단한다.
수신 scan과 조회한 TF의 시각이 0.3초 후보 제한을 넘으면 연속 조건 상실로 정지한다.
응답을 방금 받았더라도 MCU 샘플 시각·상태·fault·mode/homing/hardware flags·소유
session을 따로 검사한다. MCU 시각 비교는 uint32 wrap을 처리한다. C STOP 이후
새 속도와 하위 액션 종료를 기다린 뒤 FAILED 결과를 HTTP로 반환한다. HTTP 취소와
ROS 타이머는 프레임뿐 아니라 전체 모바일 명령 주기/세션 전환을 같은 lock으로 보호한다.

필요한 Jazzy 패키지는 `nav2_amcl`, `nav2_planner`, `nav2_navfn_planner`,
`nav2_controller`, `nav2_dwb_controller`, `nav2_bt_navigator`, `nav2_msgs`,
`moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`,
`moveit_simple_controller_manager`, `robot_state_publisher`, `xacro` 및 전이 의존성이다.
Python host 의존성은 기존 lock 파일을 따른다. 현재 PC는 Nav2 1.3.13을 임시 prefix에
풀어 사용했으며 시스템 ROS 패키지를 교체하지 않았다. CI에는 동일 통합 작업을
추가했지만 원격 실행 결과는 아직 없다.

### 전체 작업 시간과 구간 기록

`TaskExecutor`의 기본 전체 작업 예산은 300초다. 각 단계의 기존 timeout과 전체
deadline 중 먼저 도달한 제한을 적용한다. 준비 대기도 포함하며 새 단계 진입으로
전체 예산을 갱신하지 않는다. `RobotApplication.task_timeout_s`와 gateway
`--task-timeout`은 같은 설정을 전달한다. 허용 범위는 (0, 3600]초다.

`task_timeout`도 일반 정지 경로로 들어가며 하중 유지·새 정지 증거를 요구한다.
정지 확인을 빨리 끝내려고 제어권을 강제로 반환하지 않는다. 상태의 `elapsed_s`,
`task_remaining_s`, `task_timeout_s`와 완료 이력의 `wait_s`/`execution_s`를 사용해
전체 시간과 구간 병목을 비교한다. 종료 후 elapsed는 고정된다. 실제 배달 소요
시간이나 탐색 성공을 이 제한값으로 대신하지 않는다.

### 모바일 주기 송신기와 실제 보드 연결

`mobile_output`은 기존 supervisor의 승인된 target과 `bus_schedule`/`bus_router`를
묶는다. 바퀴 IDs 8/9/10과 lift ID 11을 각 주기로 송신하고 STOP은 네 축 zero를
한 프레임으로 보낸다. 팔 ID 1–6을 모바일 패킷에 포함하지 않는다. **zero는 정지나
리프트 하중 유지 증거가 아니다.** `system_stop`은 이 zero 이후 왼팔 hold, 병행 오른팔 hold를 별도 패킷으로 보낸다.

`MobileServoOutput`은 실제 `ServoTransport_ClaimQueued`→DMA→완료 사건→quiet→
`CompleteQueued` 경로를 사용한다. ISR은 사건만 기록하고 main loop가 완료한다.
기존 `ControlTick` 사건은 소비하지 않고 최신 실제 ISR epoch를 읽어 arm 예약 구간을
피한다. claim 후 시각도 재검사하며 일정 지연/송신 실패/응답 누락/대기열 만료는
정상 대기열을 폐기하고 STOP을 고정한다. UART 소유자가 막고 있어도 대기열 만료를
검사한다. 새 주기가 지난 명령의 기한을 숨기거나 연장하지 않는다.

설정 시 실제 UART baud의 8N1 최소 송신 시간과 quiet를 계산해 budget보다 짧은지
검사한다. main loop의 `MobileServoOutput_Poll`, TX/error callback, 기존 host stop
latch까지 연결했다. faulted TX는 UART 소유권을 유지하고 RX 정리·abort 확인 후에만
복구한다. 재활성화에는 STOP 송신 완료, 새 측정 정지, 전체 정지 확인과 더 큰 session이
필요하다. 상위 host의 ARM만으로 출력 fault를 지울 수 없다. host handler도 같은 supervisor에
연결된 output의 가용성을 검사하며 ARM/VELOCITY를 명시 거절한다(status 4).
출력 미연결 상태에서 명령을 수락한 것처럼 응답하지 않는다.

기본 부팅에서는 **미설정**이며 `MobileServoOutput_Configure`를 호출하지 않는다.
외부 통합자가 실측 supervisor/feedback, 시간 budget, whole-stop 증거를 준비해야 한다.
새 송신기는 mode 설정·부팅 ID 생성·리프트 homing/height/hold와 endpoint
등록을 대신하지 않는다. 두 servo UART는 기존 코드의 8N1 설정을 전제로 한다.

검증은 실제 보드 C 코드의 fake HAL 14개 시나리오와 portable C의 합계 120초 가속
주기 시험이다. baud 부족, 기존 팔 소유권, quiet 구간, ISR 즉시/중복/누락, UART 오류,
기한/시계 손실, 복구 실패, 명시 재활성화와 두 시계 wrap을 포함한다. 기존 HTTP/Nav2
시험의 C 모터 모형과 이 HAL 송신 시험은 구분하며, 실물 버스 운용을 입증하지 않는다.


### 모바일 응답 수신과 정지 중 관측

`sts3215_response`가 팔/모바일의 공통 transaction parser다. 기존
`ServoResponseParser`는 호환 wrapper로 남겨 팔 호출부를 보존했다. 체크섬·ID·길이,
헤더 재동기화와 오류 응답을 검사하고 READY/STATUS_ERROR는 init까지 고정해
완료 후 버퍼 초과를 방지한다. 모바일 reader는 완료 후 추가 바이트를 모호한
응답으로 거절한다. READ echo와 분할 응답은 처리하지만 동일 ID·동일 길이의 아주
늦은 과거 응답은 프로토콜에 sequence가 없어 구별할 수 없다. 따라서 실제 RX 정리,
응답 최악 시간과 quiet budget 측정이 필요하다.

`mobile_feedback`은 8–11번 모터를 순환하며 mode register 33의 값 1을 READ로
확인하고 telemetry 56–70을 읽는다. mode는 만료 전에 재확인하며 전원/통신 복구 시
모든 mode/샘플을 폐기한다. 주소와 속도/전류 sign-magnitude 해석은
[Feetech SDK 헤더](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.h)와
[해석 구현](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.cpp)을 확인했다.
장치 모델/펌웨어 일치와 전류 환산은 별도 확인하며 `current_raw`를 mA로 표시하지 않는다.

한 번의 publish에는 네 축 모두의 새 telemetry가 필요하다. 가장 오래된 요청 시작
시각을 feedback 시각으로 사용하고 축 간 시각 차이도 제한한다. mode가 오래됐거나
한 축이 빠져도 다른 축의 갱신으로 건강 상태를 연장하지 않는다. 리프트의 원점/높이/
건강 증거는 독립 입력이며 NULL·만료·비정상 증거로 ARM을 허용하지 않는다. 부호가
있는 모터 위치 raw만으로 다회전 높이나 load hold를 추정하지 않는다.

`MobileServoFeedback`은 기존 `servo_bus`의 circular DMA를 빌려 사용한다.
Configure는 유지보수 모드·전체 정지 확인 아래 RX를 준비하며 blocking HAL 정리는
이 초기화와 명시적 RecoverTransport에만 있다. 주기 Poll은 팔 ISR 예약 구간을 피하고
시작 전 quiet→단일 READ DMA→응답 파싱→TX 완료와 RX quiet 확인→commit 순서다.
TX 완료만으로 물리 UART를 반환하지 않는다. overflow·기한 초과·UART/장치 오류는
feedback을 무효화하고 출력 정지를 요구하며 실패한 거래의 소유권을 유지한다.

정지 후 측정을 막는 순환 의존성을 피하기 위해 `BeginReadOnly`는 motion 차단 중에도
허용된다. 전송 함수가 유효한 8-byte unicast READ 한 번만 검사/허용한다. WRITE,
broadcast, 잘못된 checksum 또는 같은 lease의 재송신은 거절한다. 준비 중인 STOP은
새 READ보다 먼저 처리된다. fault의 명시적 복구는 RX/TX 정리 후 STOP 송신이 가능하게
버스를 반환하고, Reset은 motion을 계속 차단한 채 관측만 재개한다. 재활성화에는
별도로 새 정지 feedback·전체 정지 증거와 새 session이 필요하다.

출력 `velocity_direction[4]`와 수신 방향은 같은 실측 부호를 사용한다(리프트 논리 +는 UP).
reader가 바인딩되면 supervisor·방향·팔 예약 시간이 output 설정과 일치해야 한다.
기본 부팅에서는 reader/output Configure와 endpoint 초기화를 호출하지 않는다.
장치 모델 readback·boot ID·AL/LS 높이·전체 hold는 보드 조립 코드에 연결했다.
모드 WRITE 진입점·실측 프로파일·독립 interlock/하중 제공자와 명시적 복구는 남아 있다. 이번 fake HAL 통합은 실제 reader/writer/gate/core를 실행하며 RX ring과
모터 응답만 합성했다. 별도의 Nav2 C plant가 이 HAL 경로를 실행한다고 집계하지 않는다.

## 리프트·탐색·앱의 추가 실행 계약

- `AL` v1 명령은 36B, `LS` v1 응답은 64B이며 기존 v2 MOBILE_REQUEST/RESPONSE와 같은 전송 잠금을 쓴다. HOME/MOVE/CANCEL/STATUS/KEEPALIVE/RESET은 리프트 전용 session/sequence/MCU 기한/boot ID로 구분한다. STATUS는 임대를 갱신하지 않는다.
- lift HOLD_PENDING은 실제 zero-speed DMA 완료보다 **뒤의 새 표본**에서 정지와 독립 hold 증거를 받아야 HOLDING이 된다. normal completion은 다음 작업을 허용하고 오류·취소는 정지를 잠근다.
- 부팅 ID는 마지막 8 KiB 예약 영역의 두 슬롯에 CRC/반전 counter와 함께 저장한다. 512 KiB G474RE의 dual-bank 구성만 허용한다. 쓰기/readback 실패 또는 손상된 슬롯은 자동 복구·counter 재사용을 하지 않는다. 실제 flash 전원 차단 시험은 미실시다.
- SequencePort는 운반 자세→운반 높이→Nav2→관측 높이를 별도 단계로 검사한다. SearchSkillPort의 이동 중에는 단계 조건, 촬영 중에는 정지/pose revision 조건을 검사한다.
- CapturePort는 RGB/depth 동기화·등록·rectification·노출 시각 TF를 요구한다. 이미지 버퍼를 복사하고 비동기 추론 결과를 받은 뒤 pose revision과 최신성을 다시 검사한다. 실제 모델의 리모컨 인식 성능은 별도 평가 대상이다.
- ManipulationPort는 계획 완료 뒤 scene/calibration/pose revision과 12축 시작 상태를 재확인한다. 팔 실행 성공은 grasp/release 증거를 대신하지 않는다. RobotRuntime에는 작업 사이에도 팔 heartbeat를 유지하는 maintenance 호출을 등록한다.
- 휴대폰 화면은 gateway `/`에서 제공한다. 점유 지도를 입체로 표현하고 렌더링된 바닥 polygon만 선택한다. 실제 scene mesh는 아직 포함하지 않는다. `/v1/navigation/preview`는 제어권을 얻지 않고 경로만 검사한다. 브라우저 POST는 설정된 정확한 origin만 허용하며 bearer 키는 저장하지 않는다.


### 모바일 보드의 초기화와 전체 정지

`MobileBoard_Prepare(left,right,profile,secured)`는 양쪽 UART와 commissioned 설정을
받는다. 모델/ID/baud/mode readback, 부팅 ID와 endpoint를 준비하며 주기 RX/TX를
시작하지 않는다. 기존 양팔 configure/torque-at-present가 성공해야 `ArmsPrepared`가
공유 수신기와 오른팔 STOP writer를 준비한다. 새 정지/interlock 뒤에만 periodic
mode를 설정한다. shared reader의 수명은 한 번의 finite 팔 실행보다 길다.

정지 시 새 일반 출력 차단→신선한 관절 snapshot을 기존 한계로 검증→dispatcher/read
정리→왼쪽 IDs 8–11 zero + 오른팔 현재 위치 hold→zero 영수증 뒤 왼팔 hold 순서다.
오른팔 writer는 주소 42, ID 1–6, 26-byte position SYNC_WRITE만 받는다.
`bus_router`의 완료 영수증은 성공 TX와 quiet가 끝난 정확한 bytes/token/kind다.
보드 시간은 `Timebase_NowUs`와 `HAL_GetTick`을 독립 전달하고 ms×1000으로 대체하지 않는다.

모든 hold 전송 뒤 새 양팔 READ의 요청 시각을 기록한다. `arm_hold_monitor`는 12축의
목표 오차·새 표본의 dwell·노후화를 검사하며 같은 표본을 반복 poll해 안정성을 만들지
않는다. `arm_read_period_ms`는 모바일 READ에 시간을 남기도록 설정하고, 6쌍 관측이
stop age 제한 안에 들어야 한다. 기준 raw tolerance/dwell/전송 budget에는 자동 실물
기본값이 없다. 팔 초기화 이전 STOP 또는 오류/오래된 anchor는 UNCONFIRMED로 남는다.

wheel/lift/양팔·독립 lift hold·payload retention 중 가장 오래된 관측 시각이 정지
증거의 시각이다. `ObserveInterlocks`/`ObserveLoad`는 단조로운 실제 측정 입력을
요구하며 명령 성공을 넣지 않는다. 이 제공자는 아직 미연결이다. CONFIRMED 후에도
노후화/드리프트/하중 상실은 UNCONFIRMED로 돌아간다. 전체 stop에서는 자동 rearm,
CLEAR_FAULT 또는 일반 DISABLE의 torque-off를 허용하지 않는다. 물리적으로 기구를
지지한 유지보수/복구 경로는 후속 구현이며 재부팅만으로 안전을 증명하지 않는다.

상위 `SystemStopClient`는 같은 resident v2 transport에서 SAFE_STOP 접수와 AQ/AT
실측 확인을 분리한다. `SystemStopBackend`는 실행기 stop 계약으로 이를 변환한다.
외부 Nav2/팔 action의 종료 확인은 기존 RoutedSkillAdapter가 별도로 수행한다.


### idle 관측과 전체 fetch 조립

`MobileArmObserver`는 첫 finite 완료 후 시작하는 READ 전용 모듈이다. snapshot의
요청 시각/기존 canonical 관절 한계를 유지하고 6쌍을 순환한다. 초기 torque-off
준비 관측을 대체하지 않는다. active trajectory와 STOP은 observer를 Suspend한
뒤 paired reader를 인계받는다. End가 실패해 양쪽 READ lease가 남으면 active를
거짓으로 지우지 않으며 새 동작을 허용하지 않는다. 다음 동작의 시작 anchor와
unwrap 기준은 idle 실측으로 갱신한다. 관측 공백/범위 이탈/인계 실패는 fail-closed다.

PC 조립은 `compose_fetch_adapter`에 arm/lift/navigation/alignment/search/capture/
manipulation 포트, semantic resolver, 환경/단계 monitor, stop backend, clock,
capture sink를 명시적으로 주입한다. resolver는 준비 동작을 `{arm, height_um}`으로,
나머지 작업을 해당 포트의 파라미터로 변환한다. 장소 이름을 임의 좌표로 바꾸는
기본값은 없다. capture sink는 검증된 frame/깊이 관측으로 다음 계획 context를 갱신한다.
준비 단계는 정지한 플랫폼에서 팔 이동→안전한 팔 자세에서 lift 이동을 구분하며,
완료 조건은 실제 monitor에서 별도로 받아야 한다. 조립 자체는 simulation 경계다.

`PayloadMonitor`는 commissioning 시 제공할 간격/effort/시간 임계값과 독립
`GripMeasurement`/`PayloadObservation`을 받는다. 두 표본 모두 sequence가
증가하고 충분한 관측 구간을 충족해야 증거가 된다. stale·재전송·잘못된 물체·관측
공백은 증거를 폐기하며 carrying 의무는 지우지 않는다. release는 새 빈 그리퍼와
같은 물체의 지정 목적지 관측이 필요하다. `PayloadTaskMonitor`는 환경 monitor의
하중 관련 boolean을 이 증거로 덮어쓰고 가장 오래된 시각을 유지한다. 조립의
`before_start`에는 그 monitor의 동일 이름 메서드를 연결해 pick/place 의도만 설정한다.
이 모듈은 영상 인식기·전류 환산·MCU 메시지나 실제 하중 유지 제어기가 아니다.

앱 구성 시 하나의 `TaskLease`를 `RobotApplication(..., lease=lease)`와
`SystemStopBackend(client, lease)`에 함께 전달한다. stop backend는 root 작업과
`root/step/...` 하위 작업을 같은 정지 사건으로 처리하되 응답 goal ID는 호출자 것을
보존한다. 다른 작업, 비슷한 이름의 접두사, lease 반환 뒤 늦은 호출은 거절한다.
하위 액션 cancel 실패에도 전체 정지는 요청하고, 장치 포트는 접수 실패를 재시도한다.
정지된 MCU의 재활성화는 이 조립 API에 포함하지 않는다.

`CapturePort`는 요청 시 정지/pose를 기록하고 source에 안정화 이후 노출 시각을
요구한다. 안정화를 기다리는 동안에도 움직임/pose 변경·timeout을 검사한다. 이로써
리프트 도착 직후 재촬영이 이전 frame을 받아 잘못 실패하거나 좌표를 재사용하지 않는다.


## 전체 runtime·관측 증거·설정 경계 (2026-09-16)

`build_runtime`는 `RobotStateStore`·`PayloadMonitor`·`MissionCatalog`와 장치 port를
하나의 `TaskLease`/`SystemStopBackend`에 연결한다. `MissionCatalog`는 등록된 지도
revision, 장소 접근점, 유한 관측 위치, 리프트 높이, 명시적 grasp offset/quaternion을
해석한다. 플랫폼 이동 또는 collision scene 변경 후 이전 robot-frame 좌표는 재사용하지 않는다.

`RobotStateEvidence`, `GripperObservation`, `PayloadVisualObservation`은 측정 sequence,
가장 오래된 원 관측 시각, profile SHA-256을 전달한다. ROS 고정 배열은 경계에서 Python
수치로 변환하며, 미래/오래된 시각·다른 profile/frame/boot·sequence 회귀는 fault다.
`collision_scene_ready`와 `reachable`/`collision_checked`는 별개다. 후자는 유효한
capture/scene ID와 연결된 독립 계획 검증 결과여야 한다.

`RosEvidenceSource`로 초기의 신선한 관측을 받은 뒤 `build_ros_runtime`에 준비된
resident arm/lift/stop/evidence 클라이언트와 route builder, TF buffer, 카메라 topic,
검출기·표면 관측 함수를 주입한다. 단일 ROS executor에서 callback→runtime tick을
진행하며 공유 직렬 통신을 다른 worker에서 사용하지 않는다. inference만 별도 worker다.
런타임 종료 시 신규 작업을 차단하고 정지 증거를 확인한 뒤 inference/ROS/클라이언트를
닫는다. 정지 미확인을 임의로 성공 처리하지 않는다. factory는 장치 검색/ARM/home을 수행하지 않는다.

`OnnxDetector`는 명시적 YOLOv8 axis-aligned detection 출력 `[1,4+classes,anchors]`,
고정 크기 NCHW float32 RGB, 모델 SHA-256·클래스 순서를 요구한다. 자동 다운로드는
없다. letterbox/NMS 어댑터 시험만 했고 실제 모델·영상 정확도/Jetson 성능은 미측정이다.
`MoveItBridge`는 측정 12축 anchor + lift state로 단일 팔 Cartesian 접근 계획을 요청하고
50ms resident 목표로 변환한다. 시간/속도/관절 범위/처음 anchor/반대팔/그리퍼 보존을
검사한다. `start_state.is_diff=true`로 실측 관절을 갱신하면서 기존 attached object를 보존한다.
집기/놓기 세부 조정 코드는 아래 `PickPlacePort`로 연결했다. 실제 그리퍼 보정·scene
geometry·단계별 관측 제공자는 별도이며 합성 held 전환은 물리적 집힘의 증거가 아니다.

`EvidencePublisher`는 로봇/하중 중 가장 오래된 관측 시각을 AE/AF로 전달한다. UNKNOWN
하중은 payload_safe=false다. fault/STOPPING 중에도 관측 단계를 계속 실행해 정지
증거를 모으며, 동작 유지보수는 중단한다. evidence 채널은 신뢰된 host 판단의 입력이며
물리 센서 또는 안전 인증된 회로를 대체하지 않는다.

설정 생성:

```bash
python3 tools/setup/firmware/prepare_mobile_profile.py \
  --profile config/mobile_board.simulation.json --output output/profile-review-01
```

새 폴더에 JSON/C header/manifest를 생성한다. 모의 header는 명시적 host 시험 define
없이는 컴파일이 실패한다. measured 설정은 모터/관절/버스/lift 기구/lift hold/카메라
측정 기록 6종의 경로·hash를 요구한다. 기록 내용의 정확성은 별도 검토한다. 기본
`MOBILE_BOARD_PROFILE_HEADER`는 비어 있으며 자동으로 생성물을 적용하지 않는다.
`MobileBoard_ProvisionModes`는 secured maintenance에서만 명시적으로 호출하고 torque-off
READ 후 unlock/mode/lock 각각을 WRITE→READ 검증한다. 실패 후 동일 부팅에서
재시도하지 않는다. `MobileBoard_Boot`는 mode provisioning을 호출하지 않는다.

`RecoveryCoordinator`는 오래된 runtime을 계속 fault 상태로 둔다. fresh secured 증거,
새 disabled boot, 이전 클라이언트 종료, 원점 폐기, 장치 식별·동일 profile hash 후
새 idle runtime을 한 번만 bootstrap한다. factory의 부분 실패도 자동 재시도하지 않는다.
이전 작업/명령을 복구하지 않으며 실제 reset/ARM/homing은 이 API의 자동 동작이 아니다.

재현:

```bash
python3 tools/run/run_fetch_stack.py
python3 tools/run/run_fetch_stack.py --cancel-step 7
python3 tools/run/run_fetch_stack.py --fault reboot
# ROS Jazzy와 빌드된 so101_interfaces, nav2_msgs, moveit_msgs를 source한 환경:
python3 tools/run/check_fetch_ros_ports.py
```

`verify_offline.py`는 전체 모의 fetch/장애/profile 생성도 포함한다. `--ros`는 3개 증거
메시지와 합성 ROS 서버의 전체 fetch를 검사하고, `--moveit`는 실제 planner를 따로
검사한다. ROS 전체 모의 실행, 실제 알고리즘 개별 시험, 실제 하드웨어 통합은 구별한다.


## SO-101 모션 평활화와 PID 시험 후보

**근거:** [LeRobot SO follower 설정(고정 커밋)](https://github.com/huggingface/lerobot/blob/7d615acf9aef770e66054ce1d38d256e2922448f/src/lerobot/robots/so_follower/config_so_follower.py)은 P=16, I=0, D=32다.
[SO follower 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)은 모터 내부 위치 제어기에 이를 설정한다.
[MoveIt 시간 계획 문서](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html)는 TOTG 뒤 Ruckig jerk smoothing을 배치하고 명시적 jerk 한계를 지정한다.
[SO-ARM101 공식 모델](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.xml)의 시뮬레이터 gain은 서보 gain과 일대일 대응하지 않으므로 그대로 이식하지 않았다.

이번 변경은 바깥에 PID를 하나 더 추가하는 방식이 아니다. `servo_joint_config.c`의
어깨/팔꿈치 P/D는 사용자의 하중 추종 이력을 반영해 각각 64/64·56/64로 복원했다. P16 후보는 철회했고 I=0, torque limit,
raw/model 범위·영점·방향·피드백/정지 판정은 보존했다. LeRobot 기본값은 실제 이
조립체의 최적값이라는 의미가 아니다. 정지 진동과 부하 유지/추종 성능을 함께 평가한다.

기존 좌측 준비는 acceleration=0·goal time=0·speed=800을 쓰는데, 우측은 speed와
torque만 쓰고 acceleration/time은 이전 값을 남겼다. 우측도 두 필드를 별도로 쓰고
확인한다. Goal_Position은 건드리지 않으며 configuration-only torque-off 계약을
유지한다. 좌측은 mode와 전체 acceleration/target/time/speed 블록도 readback한다.
이 경로에서 EEPROM unlock/재시도 또는 모바일 모터 PID 변경을 추가하지 않았다.

`route_for_arm`의 task waypoint 사이에서 `s(u)=10u³−15u⁴+6u⁵`를 사용한다. 양 끝
속도/가속도는 0이고 각 관절 이동량 d에 대해 `T >= max(1.875|d|/v,
sqrt((10/sqrt(3))|d|/a), cbrt(60|d|/j))`를 만족하는 최소 50ms 격자 시간을 선택한다.
기본 a=1, j=8은 미실측 후보이며 관절별 vector로 주입할 수 있다. 충돌 계획이 아니라
기존 두 endpoint 사이 선분의 시간만 바꾸므로 upstream 충돌 검증을 생략할 수 없다.
arm→gripper 같은 의미 있는 task 경계에서만 정지하며 MoveIt 경로점마다 이 함수를
호출해 반복 정지시키지 않는다. MoveIt은 TOTG→Ruckig→ValidateSolution을 사용한다.
Ruckig 플러그인 부재/실패는 계획 실패이며 무평활화 자동 fallback은 없다.

서보에 도달하는 것은 기존 50ms position 샘플/5ms MCU 선형 보간이다. 따라서 위
연속 곡선 및 Ruckig의 jerk 제한과 실제 전송·양자화된 목표의 jerk는 같은 보장이
아니다. 두 팔의 송신 주기를 무작정 올려 공유 UART를 포화시키거나, 필터 지연으로
계획 목표를 뒤따르게 만드는 방식은 추가하지 않았다.

PID 변경 뒤 브릿지 계약 확인:

```bash
python3 tools/setup/firmware/check_servo_profile.py --check
# 의도적으로 servo_joint_config.c를 재조정한 경우에만 host hash를 맞춘다.
python3 tools/setup/firmware/check_servo_profile.py --write
```

실제 C 테이블과 `Host_CalibrationHash`를 host compiler로 실행한다. 현재 hash는
`0x2D90167E`, 철회한 P16 후보는 `0xA56DC10B`다. --write는 host 기대 hash만 갱신하며
서보/펌웨어를 쓰지 않는다. 이후 전체 회귀와 firmware/ROS 동시 배포가 필요하다.

실측 로그 비교:

```bash
python3 tools/run/analyze_arm_motion.py --trace output/arm-trace.json \
  --output output/arm-motion-quality.json
```

JSON 최상위는 `joint_names`(12개 고유 이름), `samples`다. 각 sample에는 원 관측
`time_s`, `target_rad` 12개, `measured_rad` 12개를 기록한다. 도구는 추종 RMS/최대
오차, 최종 목표 정지 구간의 마지막 0.5초 peak-to-peak/RMS, 정착 시간을 출력한다.
관측 간격이 0.1초를 넘거나 충분한 hold가 없으면 해당 지표를 null로 남긴다. 0.02rad
정착 오차는 분석 기본값이며 `--settling-tolerance-rad`로 변경 가능하다. 목표를
측정으로 복제한 기록은 실측 비교에 사용할 수 없다. 자동 PID tuning이나 원인 확정은
하지 않으며 센서 샘플링보다 빠른 진동은 이 위치 로그만으로 평가할 수 없다.


### 높은 P에서의 진동과 하중 오차를 분리하는 시험

기존 gain 복원은 회귀 수정이며 새 PID tuning 완료가 아니다. 사용자 이력은 낮은 P의
하중 오차와 높은 P의 진동을 시사하지만, 정지 오차/이동 추종 오차/관절 범위 오류 중
무엇이 정지를 일으켰는지는 raw fault/목표/실측을 대조해야 한다. 해당 제한은 유지한다.

1. 같은 자세/하중/목표에서 P64·56/I0/D64를 기준으로 기록한다. 현재 모터 register의
   실제 값과 명령 target이 고정되어 있는지도 확인한다.
2. 정지 중 목표가 고정인데 위치만 진동하면 P를 고정하고 관절별 D를 소폭 양방향으로
   비교한다. 진동/추종 RMS/정착 시간/전류가 모두 허용되는 조합을 선택한다. D를
   무조건 올리거나 모든 관절에 같은 gain을 배포하지 않는다.
3. 가감속/방향 전환에서만 진동하면 upstream smooth timing과 가속/jerk 한계를 먼저
   평가한다. 이는 고정 목표에서의 내부 servo 진동을 해결했다는 증거가 아니다.
4. I는 정적 중력 오차의 후속 후보다. 다만 integral windup/포화/접촉 시 반응을
   검증하기 전에는 0을 유지한다. 현 위치 명령 인터페이스의 Torque_Limit은 토크
   feedforward 명령이 아니므로 단순히 값을 더해 중력 보상을 구현할 수 없다.

[Feetech 공식 설명](https://www.feetech.cn/Data/feetechrc/upload/file/20220618/%E5%85%88%E7%9C%8B%E8%BF%99%E9%87%8C-%E5%85%A5%E6%89%8B%E6%95%99%E7%A8%8B2-1.pdf)은 P의 정지 유지/운동 강성, D의 제동, I의 정적 오차 감소와 진동 위험을 구분한다.
[Modern Robotics의 중력하 PD/PID 설명](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-4-motion-control-with-torque-or-force-inputs-part-2-of-3/)은 P가 하중을 버티려면 오차가 남을 수 있고 I가 이를 줄이는 대신 과도 응답/안정성을 악화할 수 있음을 설명한다. 이 이론의 Kp/Kd 수치를 STS raw register와 동일시하지 않는다.

## 집기·놓기 단계와 물체 장면 갱신

`PickPlacePort`는 기존 `SequencePort`/`ResolvedPort`를 사용한다. 새 제어 스레드나
서보 PID를 추가하지 않는다. `build_runtime`과 `build_ros_runtime`의 선택적
`manipulation_factory(motion, arm, monitor, clock, catalog)`로 주입한다. factory 미제공
경로는 종전 Cartesian 계획/실행만 수행한다. 물리 모드는 계속 차단된다.

| 작업 | 순서 | 다음 단계의 필수 증거 |
|---|---|---|
| 집기 | open → pregrasp → approach → close → attach → retreat | 새 빈/열린 그리퍼 관측 → 집힘 확인 → 부착 readback → 하중 유지/팔 이격 |
| 놓기 | preplace → approach → open → detach → retreat | 하중/부착 유지 → 목적지에서 해제 확인 → world 복귀 readback → 팔 이격 |

단계 이름과 원래 task parameters를 `resolve_stage(stage, parameters, now)`에 **시작 시점에**
전달한다. motion 단계는 기존 `ManipulationPort`용 Cartesian task, jaw 단계는 주입한
그리퍼 port의 보정된 명령, scene 단계는 아래 명시적 scene change를 반환한다.
후퇴를 미리 계획하거나 최초 영상 시각을 갱신해 재사용하지 않는다. 새로운 실측 anchor,
scene revision, 원 관측 시각을 사용하며 오래된 target은 기존 `ManipulationPort`가 거절한다.
관측 유효 시간이 만료되면 새 관측을 공급해야 하며 timeout을 늘려 신선함을 만들지 않는다.

`monitor`는 기존 `PayloadTaskMonitor`의 `grasp_verified`, `load_retained`,
`release_verified`, 새 `gripper_empty`/`gripper_open` 및 독립 환경 조건을 사용한다.
scene 제공자는 **해당 object ID**의 `object_attached`/`object_detached`를 함께 제공한다.
실제 배포는 scene의 단일 writer 소유권·revision/신선함을 보장해야 하며, bool을 상수로
넣어서는 안 된다. 그리퍼 전환 중에는 전환 완료 조건을 연속 조건으로 요구하지 않지만,
완료 관측 전에는 부착/분리/후퇴로 넘어가지 않는다. 최상위 `RoutedSkillAdapter`가
전체 정지를 요청하며 내부 child 종료와 별도의 전체 정지 증거 전에는 작업을 놓지 않는다.

`SceneUpdatePort` 입력은 operation(attach/detach), object_id, arm, link_name, frame_id,
position_m, quaternion_xyzw, size_m(box 3축), touch_links, observed_s다. attach의 pose는
부착 링크 기준, detach는 설정된 world frame 기준의 신선한 관측/TF에서 얻는다.
생성자에 좌우 gripper link 허용 목록과 관측 나이를 명시한다. 다른 팔/베이스로 touch
권한을 넓히지 않으며 전체 allowed-collision matrix는 수정하지 않는다.

[MoveIt Planning Scene API](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)를
사용하되 실제 Jazzy 동작을 검증했다. 부착 ADD가 world object를 자동 제거하므로
동일 diff에 world REMOVE를 중복 전송하지 않는다(중복 제거는 false ACK를 만든다).
분리는 attached REMOVE와 명시적 world ADD를 함께 보낸다. Apply의 success만으로
완료하지 않고 Get 결과의 geometry/pose/링크/부착·world 배타성을 확인한다. 링크 상대
object pose와 primitive pose를 합성해 비교하고 q와 -q를 동일 회전으로 취급한다.

응답 유실/예외는 서버 처리 여부를 알 수 없으므로 RUNNING+fault와 소유권을 유지한다.
readback 불일치는 FAILED+fault이며 재사용하지 않는다. cancel도 진행 중 서비스가 끝나고
readback이 올 때까지 대기한다. 자동 재송신이나 보상 detach는 없다. 새 scene/client
구성으로 명시적 복구할 때 실제 world/attachment와 잡힌 물체를 먼저 대조한다.

`OfflineStack`은 동일 조정 코드를 쓰고 jaw/scene/센서/geometry만 합성한다. 기존
팔 이동 성공에서 held를 바꾸던 fixture를 제거했고, jaw 동작 뒤 독립 모의 센서의
dwell을 기다린다. 전체 출력에 내부 11단계의 이름·시간이 포함된다. 실제 ROS 통신
시험은 6회 계획/3회 jaw/2회 scene을 모의 실행하고, 실제 MoveIt 시험은 별도로
부착/분리 readback과 부착된 큰 물체의 충돌 거절을 검사한다. 두 시험을 합쳐 실물
그리퍼·scene 동기화가 검증됐다고 주장하지 않는다.

## 측정 관측·그리퍼·표면 연결

`EvidenceProducer`는 arm/base/localization/lift/scene별 `SourceSample`을 받는다.
각 입력은 sequence, 원 관측 시각, boot ID, profile SHA-256과 정해진 측정 필드만
허용한다. 가장 오래된 원 시각을 `RobotObservation.observed_s`로 보존하고 동일
시각의 반복 poll은 출력하지 않는다. 누락 시 미준비, stale/skew는 거절, 순서/identity
위반은 fault다. 건강·lift 지지·reach/collision은 독립 입력이며 명령 성공에서 만들지 않는다.
`RosEvidencePublisher`는 이 관측과 `GripMeasurement`/`PayloadObservation`을 ROS
시각으로 변환해 기존 typed topic으로 보낸다. 상대 원 관측 나이를 보존한다. 장치
드라이버 callback은 원 취득 시각/boot/profile을 넣어 observe한 뒤 produce/publish한다.
상태 fusion 오류는 기존 runtime fault/stop 경로로 전달해야 한다. serial polling은
기존 단일 제어 worker가 담당하며 새 publisher가 UART를 열지 않는다.

`JawCalibration`은 측정 각도→jaw 간격의 단조 표를 선형 보간하고 범위 밖 외삽을
거부한다. open/close 방향을 raw 값의 대소로 추측하지 않는다. `SharedGripperPort`는
팔 실행기와 **같은** `ResidentArmPort`에 보정된 gripper-only step을 보낸다.
`ResidentRouteBuilder`는 최신 snapshot, 12축 범위/속도 및 필수 current-scene route
validator를 확인한다. 그리퍼 동작에서는 나머지 11축을 보존한다. validator는 전체
경로를 해당 scene에서 검사해야 하며 단순 true 콜백을 실물에서 사용하지 않는다.

`ManipulationBindings.factory`를 ROS factory의 `manipulation_bindings`에 주입하면
공유 그리퍼와 기존 motion/scene 포트가 연결된다. 모든 6개 motion stage의 명시적
frame 기준 offset, fresh stage observation과 object-specific scene evidence, 좌우 jaw
보정을 요구한다. stage 관측은 object/arm/stage/capture/pose/scene/calibration/frame과
원 시각에 묶인다. attach/detach는 `SceneUpdatePort` 형식의 fresh object pose를 사용한다.
대기 중 오래된 값을 새 시각으로 바꾸지 않으며 보정 전에는 geometry를 임의 생성하지 않는다.

`PlanningContext.observed_s`는 로봇 관측 시각이고 `target_observed_s`는 원 target
노출 시각이다. 입장·계획→실행 handoff는 둘 다 신선해야 한다. 실행 중에는 state가
신선하고 target의 identity/노출 시각·platform pose·scene이 그대로여야 한다. 계획된
경로를 실행하는 도중 영상 나이만으로 중단하지 않으며, scene/상태 변화는 계속 중단한다.
기존 `target_observed_s=None` 호출은 기존 단일 시각 계약을 유지한다.

`CapturePort.surface_sampler`는 detector에 넘긴 것과 **같은 동결 image pair**에서
목적지 ROI를 표본화한다. `SurfaceObserver`는 root frame으로 투영한 depth의 평면
residual, 위쪽 normal 기울기, 두 방향 관측 폭을 검사한다. 군집 외 표본을 버려서
장애물을 지우지 않는다. 배달 확인에서는 동일 capture의 단일 대상 detection 위치와
지정 plane 내 반경/높이를 비교한다. 그 결과는 전체 장애물 여유, 물체 동일성의
실물 인식 정확도, 매트리스 강성을 보장하지 않는다. ROI/임계값은 명시적 보정 입력이다.

전체 OfflineStack은 동일 EvidenceProducer와 SurfaceObserver 계산을 쓰고 입력만
합성한다. 실제 DDS 시험은 source timestamp 보존도 검사한다. 센서/모델 자산과
현장 topic·TF mapping을 선택/측정해야 실제 노드 구성에 연결할 수 있으며, 현재
기본 모드에서 장치 검색/모터 자동 활성화/집기 물리 완료를 허용하지 않는다.

## 2026-09-16 주행·센서·지도 재점검

### 텔레옵에서 Nav2로 연결

```text
Nav2 NavigateToPose → GuardedNavigationPort(기존 task/stop 소유권)
Nav2 controller TwistStamped → RosBaseIO → MobileBaseDriver
  → m/s·rad/s 옴니 변환·wheel 가속/속도 제한·원 명령 기한
  → TravelGuard(실측 운반 자세 + localization + 신선한 전체 정지 거리 관측)
  → 기존 MobileClient/v2/단일 UART → STM32 공유 버스 → wheel 8/9/10 (lift 속도 0)
MCU 실측 wheel sample → MCU 취득 시각/host RTT 검사 → odom + odom→base TF
```

`build_guarded_navigation`은 준비된 기존 client를 주입받아 포트/IO/maintenance를 반환한다.
`build_ros_runtime(navigation_port=port, navigation_maintenance=(maintain,),
navigation_observations=(depth_source.poll,))`로 기존 단일 executor에 넣는다.
`request_stop`은 같은 `SystemStopClient.request`에 연결한다. 별도 UART writer,
자동 ARM/homing/reconnect나 원본 LeRobot Host를 만들지 않는다. 아직 실물 launch나
보드 설정을 완성했다는 뜻은 아니며 physical application 차단은 유지한다.

- 입력은 `TwistStamped`만 받는다. 원 발행 시각·body frame·owner epoch를 확인하며 수신 시각으로 오래된 명령을 새로 만들지 않는다. 명령 토픽의 유일 publisher는 선택한 Nav2/중재기여야 한다. teleop 우회 writer를 붙이지 않는다.
- 한 비율로 wheel 속도/가속을 제한하고 raw 양자화 후 실제 보낼 속도로 공간을 검사한다. 제어·깊이 계산 시간이 기한을 넘으면 이동 명령을 보내지 않는다. firmware deadman은 별도로 유지한다.
- 반복 MCU sample은 같은 odom 시각을 유지하고 재발행하지 않는다. 공백/boot 변경/회귀·늦은 응답은 정지다. covariance는 명시 입력이며 wheel 적분을 완벽한 실제 위치로 주장하지 않는다.
- 성공 액션 후에도 별도의 **성공 이후 취득한 바퀴 정지 증거**까지 기다린다. 취소·fault는 공통 whole-stop을 요청하고 자동 새 owner/세션을 허용하지 않는다.
- 기존 `check_nav2_closed_loop.py`는 2D scan 전용의 과거 알고리즘 시험이다. 시험 내부 unstamped/수신 시각 기반 plant를 실물 driver로 복사하지 않는다. 새 브릿지는 `check_navigation_bridge.py`의 DDS→실제 C parser/mobile plant→odom, native depth→노출 TF→grid 경로로 별도 검증한다. 이 plant의 정지는 모바일 축만 증명하며 공통 whole-stop 요청 binding과 팔/하중 정지는 별도 factory·HAL/stop 회귀로 검사한다.

### native depth와 통과 공간

집기용 registered RGB-D는 기존 정지 캡처 경로를 유지한다. 주행은 RGB 정합/탐지 결과를 기다리지 않는 `RosNavigationDepth`다. native rectified depth와 같은 CameraInfo,
명시 depth scale, **노출 시각 odom→depth TF**, 보정 identity/고정 intrinsics가 필요하다.
이미지 1개 대기·worker 1개로 적체를 막고 ray/map/work budget을 넘으면 관측 실패로 처리한다.
제어 executor에서는 완료된 결과만 반영한다. 중간 리프트 자세·새 mode epoch 이전 이미지·TF 부재·장착 품질 미검증·깊이 부족은 free로 바뀌지 않는다.

`DepthVolume`은 높이가 있는 voxel의 hit/free/unknown을 보존한다. hit는 신선한 깊이 ray가
다시 통과해야 지울 수 있고, 2D laser나 시간 경과는 상판을 지우지 못한다. 2D projection은
어느 높이에라도 hit가 있으면 occupied, 전체 높이를 확인한 경우만 free, 나머지는 unknown이다.
이는 분해능과 센서 성능에 한계가 있는 모델이다. 한 ray가 실제 모든 재질·작은 물체·voxel 내부를 확인했다는 증명은 아니다.

`nav2.depth.simulation.yaml`은 기존 simulation YAML 뒤에 적용하는 후보 overlay다.
controller stamped 출력을 켜고 **rolling local costmap**에 odom 기준 depth 격자를 별도
StaticLayer로 추가한다. LiDAR layer와 분리하고 maximum 결합/footprint 자동 삭제 금지를
설정한다. 이 격자를 map 기준 전역 지도에 그대로 넣지 않는다. depth global projection이
필요하면 map→odom 변경까지 반영하는 별도 동기화가 필요하다.
[Nav2 StaticLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/)의 지도 수신 기능을 이용한다.
Jazzy StaticLayer의 `use_maximum`은 plugin 하위가 아닌 costmap 수준 파라미터다.
rolling max 결합에서 255(unknown)가 254(lethal)를 가리지 않도록 local costmap의
`track_unknown_space=false`를 사용한다. 원 depth volume의 unknown은 유지하며 최종
`TravelGuard`가 미관측 공간 진입을 차단한다. 이 guard 없이 이 overlay를 구동하지 않는다.
실제 Nav2 costmap 시험에서 낮은 laser의 상판 삭제 방지·새 depth의 상판 삭제·depth free가
LiDAR hit를 지우지 않는 양방향 합성을 확인한다. 전체 controller 경로/실기 성능 검증과는 별개다.

운반 반경은 팔/그리퍼/물체까지 포함한 원형 부피로 보수적으로 잡고, 측정 속도와 명령 속도
각각의 반응·관측 나이·제동 거리를 검사한다. yaw 변화 불확실성에 대한 여유도 포함한다.
처음 로봇이 차지한 부피가 비어 있다는 독립 관측, 실측 주행 lift 높이·정지/hold, 팔 운반 자세,
localization이 필요하다. 거짓 관측을 `True`로 채우는 배포 기본값은 없다.

**중요한 한계:** 이 엄격한 검사는 D415 단독에서 관측하지 못한 영역 때문에 실제 주행을
차단할 수 있다. 장애물 색/재질·진동·외부 물체 접근·몸체 근접 사각은 소프트웨어로 보장되지 않는다.
하드웨어에서 커버되지 않는다면 마운트/운용 방향/보완 센서를 변경해야 한다. 수치만 완화해
unknown을 free로 바꾸지 않는다. `check_camera_coverage.py`는 사전에 기하학적 사각을 보여주는
도구이며, 기본 높이 .6 m/pitch 15°에서 예시 54점 중 21점만 화각 안이다. 실제 배치 결과가 아니다.

### 이동한 가구와 세 종류의 지도

| 정보 | 업데이트 | 기존 위치 삭제 조건 |
|---|---|---|
| 주행 local obstacle volume/costmap | 신선한 2D scan + 위 native depth; 새 위치 B는 즉시 표시 | 상판 A는 해당 높이를 다시 본 깊이의 free 증거 필요; 누락/가림/낮은 LiDAR 통과로 삭제하지 않음 |
| 위치 추정용 2D map | 기본은 고정 기준 지도 + localization. 지속 변경 확인 뒤 별도 mapping 세션/수정 revision | AMCL은 지도 작성기가 아님. transient 장애물을 영구 벽으로 매 프레임 저장하지 않음 |
| semantic object/place memory | 물체/가구의 관측 위치·시각·신뢰도·지도 revision을 갱신할 단계 | 미관측은 삭제가 아니라 오래된/불확실 상태. 재접근 시 재관측 필요 |

`RobotApplication.replace_map`은 active task를 취소하고 whole-stop으로 lease가 해제될 때까지
기존 revision을 유지한다. 그 뒤 새 revision을 적용하고 재위치 추정을 요구한다. 통합 runtime은
MissionCatalog의 map revision이 다르면 새 요청을 막으므로 장소/시점 catalog도 재검증·재구성한다.
외부 AMCL/map server 갱신도 같은 정지된 전환 안에 넣어야 한다. 자동 장기 SLAM map 편집이나
semantic mapping 전체가 이번 구현으로 완성된 것은 아니다.

### 이번 검증의 경계

`verify_offline.py --ros --moveit --stm32 --nav2-costmap --bundle`에 새 주행 DDS/C 시험과 화각 보고를 넣었다.
단위 회귀는 상판 이동/unknown/invalid depth/epoch·목표 도달 이후 정지·stale cmd/반복 피드백·
boot/제어 기한·지도 정지 후 전환을 포함한다. 3카메라 후보 TF와 lift FK를 검사한다.
모든 값/피드백/영상은 PC 합성이다. 실제 D415 노출·Jetson 지연·모터 제동·운반 안정성을 시험한
결과가 아니며, 이 통과만으로 physical gate를 해제하지 않는다.

### 전체 작업의 주행 전환과 관측 수명

`build_guarded_navigation`은 `TravelSessionPort → GuardedNavigationPort → MobileBaseDriver`를 반환한다.
`reset_observations(epoch)`를 반드시 주입해 `RosNavigationDepth.reset(epoch)`와 독립 clearance 생산자를 함께 초기화하고,
`depth.poll`은 runtime observations에, 반환 maintenance는 공유 제어 worker에 등록한다.
새 epoch의 정지 피드백·posture/hold·depth·clearance가 모두 준비된 뒤에만 Nav2와 바퀴 소유권을 연다.
정지 상태에서 관측을 다시 기다리는 한도는 후보 2초이며, timeout은 작업 실패/공통 stop이다.
활성 주행의 입력 상실은 fault를 유지하며 자동 재시도·회전·후진·ARM은 없다.

탐색 후 정렬도 운반 자세/높이 준비를 거친다. 복합 단계의 `alignment_phase_safe`는 실제 현재 하위 단계에서 계산하며
리프트 이동을 바퀴 이동과 혼동하지 않는다. 집기 정렬/높이는 선택된 관측 시점을 사용하고 이동 뒤 재촬영한다.
베이스 움직임은 카메라 pose revision만 바꾸며, 리프트 hold dwell은 독립 수직 관측으로 유지/폐기한다.

등록 장소 변경은 `MissionCatalog.invalidate_place(name, reason)`로 전달한다. 목표/표면 캐시 폐기와 활성 작업 정지를
유발하며 신규 요청도 막는다. 이동한 가구의 새로운 위치를 자동 추정하는 API가 아니다. 정지 이후 외부 지도/장소 관측을
재검증하고 새 runtime을 구성한다. 등록 시점들의 경로는 탐색 시작 시와 실제 이동 요청 시 다시 검사한다.

ROS payload 관측은 명시적인 `PayloadMonitor(camera_frames=[...])` 설정이 필요하다. 발행/수신 시 영상의
`header.frame_id`를 보존하며, 잘못된 카메라·만료·unknown은 유지 증거가 아니다. 카메라 변경 시 dwell을 다시 쌓는다.
허용 프레임 목록 자체가 실제 가시성을 증명하지 않는다. 물체/그리퍼 관계 판정은 해당 영상에서 독립적으로 생산해야 한다.

전체 합성 회귀는 `python3 tools/run/run_fetch_stack.py --guarded-navigation`이다. grid waypoint 추종기와 이상적인
wheel plant/free 공간을 사용하므로 Nav2 알고리즘, 진동·미끄러짐, 실제 D415 전체 높이 관측의 통과로 집계하지 않는다.
