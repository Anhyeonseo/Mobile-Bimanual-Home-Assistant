# Pi–STM32 팔 제어 프로토콜

현재 ROS 실행 경로는 `so101_arm_bridge`의 **protocol v2 양팔 스트림**이다. 기존 12축 팔 payload는 유지하고 모바일 endpoint용 v2 envelope를 추가했다. 가사 작업 의미는 상위 실행기의 책임이다.

## 버전과 소스

| 범위 | 정의 |
|---|---|
| v2 frame·payload·진단 | `ros2_ws/src/so101_arm_bridge/so101_arm_bridge/stream_protocol_v2.py` |
| COBS·CRC-32C | 같은 패키지의 `wire.py` |
| v2 통신 | 같은 패키지의 `stream_transport_v2.py` |
| 명령 소유권·관절 한계·종료 확인 | 같은 패키지의 `bimanual_stream_adapter.py` |
| MCU v2 계약·실행 | `firmware/stm32_actuator/include/actuator_core/`, `src/stream_*_v2.c` |
| 기존 v1 message ID | [message_ids.json](message_ids.json), 생성된 MCU `message_ids.h` |
| v1 검사 codec | `tools/lib/actuator_protocol.py` |

`message_ids.json`은 보존된 v1 manifest다. v2 전체 메시지 목록으로 해석하지 않는다. 이전 v1 ROS 동작 실행기는 제거했으며 v1의 자세한 과거 운용 기록은 [수건 보관본](https://github.com/Anhyeonseo/SO101-Towel-Folding/tree/5b16fff82e400e4cca8cdcff96a6d1548058ef80/protocol)에 남아 있다.

## 모바일 envelope와 팔 브릿지의 공통 소유권

- `MOBILE_REQUEST=64`: v2 frame payload에 정확히 36B AM v1 요청을 넣는다.
- `MOBILE_RESPONSE=65`: 같은 외부 sequence를 돌려준다. payload 첫 바이트는
  0=정상 envelope, 1=endpoint 미연결, 2=형식 오류, 3=host fault로 동작 차단,
  4=모바일 output 미설정/다른 supervisor 연결/출력 fault로 동작 차단이다.
  0일 때만 뒤에 64B AS v1 응답이 온다. 내부 동작 거절은 AS opcode의 bit 7이다.
- 실제 호스트 UART는 기존 COBS/CRC/v2 파서 하나로 읽는다. AM/AS 원시 바이트를
  팔 프레임 사이에 직접 넣지 않는다. v1 manifest의 예약 범위는 그대로이며 이 ID는 v2 전용이다.
- `MobileV2Exchange(existing_transport)`를 `MobileClient`에 주입한다. 기존 팔
  transport의 포트·sequence·응답 버퍼·transaction lock을 공유한다. 별도 모바일
  프로세스가 같은 USB 포트를 열지 않는다. task 제어권과 통신 lock은 별개다.
- 발신 시계가 아닌 MCU 처리 시각으로 내부 deadline을 검사한다. 조각난 수신이나
  대기 중 지연으로 지난 명령의 유효기간을 연장하지 않는다.
- 보드 handler는 등록했지만 endpoint는 부팅 시 NULL이다. 검증된 설정/boot ID/
  feedback을 준비한 보드 실행기가 binary mode 이전에 한 번만 연결해야 한다.
  현재 자동 연결·자동 ARM·ROS cmd_vel 구독은 없다. 미연결은 즉시 명시 거절한다.
- 팔/host 오류의 stop latch는 연결된 모바일 supervisor도 정지시킨다. 팔 latch가
  남아 있으면 모바일 ARM/velocity를 거절한다. 기존 팔 torque disable과 모바일
  zero 응답이 전체 로봇의 정지/하중 유지 증거를 대신하지 않는다.

## 공통 프레임

ST-LINK VCP를 통해 little-endian 바이트를 전송한다. frame은 COBS 인코딩 후 `0x00`으로 구분하며, header와 payload를 CRC-32C로 검사한다. 현재 양팔 브릿지의 host baud는 921600이다.

| offset | 필드 | 형식 |
|---|---|---|
| 0 | magic `0xA55A` | uint16 |
| 2 | version | uint8 |
| 3 | message type | uint8 |
| 4 | flags | uint16 |
| 6 | payload length | uint16, 최대 512 byte |
| 8 | sequence | uint32 |
| 12 | sender time | uint32, ms |
| 16 | payload | 메시지별 정의 |
| 16 + N | CRC-32C | uint32 |

MCU와 호스트의 절대 시각이 같다고 가정하지 않는다. 적용 시각은 MCU control tick을 기준으로 계산하며 응답의 sequence와 시간 echo를 검사한다.

## 양팔 명령과 피드백

양팔은 왼팔 6축·오른팔 6축의 순서로 하나의 목표 스트림을 사용한다. 한 batch는 최대 9개 표본을 담으며 각 표본에는 12축 목표가 모두 필요하다. ROS 입력은 radian, wire 목표는 부호 있는 micro-radian이다.

세션 준비는 펌웨어 버전 `0x00024908`, capabilities `0xEFFFFFFF`, 양팔 보정 hash `0x2D90167E` 등 식별 계약을 확인한다. `0x00024908`은 이번 소프트웨어 후보이며 이전 `0x00024809` 실기 기록과 구분한다. 설치된 펌웨어와 맞지 않으면 실행을 거절한다. 이 값의 존재는 새 모바일 플랫폼의 실물 검증을 의미하지 않는다.

`START_FINITE`, `START_OPEN`, `APPEND`, `SPLICE`, `STOP`은 ROS `BimanualStreamCommand`의 연산이다. 브릿지는 이를 wire 세션·batch·stop 명령으로 변환한다. 상위 작업 계층은 UART를 직접 사용하지 않는다.

`BimanualJointFeedback`은 실제 12축 위치, 축별 측정 나이, 유효 축 mask와 MCU 시각을 전달한다. 완료 응답 이후에도 관측의 유효성을 확인해야 하며, 관절 목표 도달만으로 물체의 집기나 배달을 판정하지 않는다.

## 검증과 실행 범위

```bash
python tools/run/validate_protocol_manifest.py
python tools/setup/firmware/generate_protocol_header.py --check
python -m pytest -c config/pytest.ini --rootdir=. -q tests/test_stream_protocol_v2_contract.py tests/test_bimanual_stream_adapter.py
```

위 명령은 통신 계약과 소프트웨어 동작을 검사하며 실제 모터를 실행하지 않는다. 실제 장치 연결·토크·동작은 별도 승인과 초기화·관절 제한·정지 조건 확인이 필요하다.

## 모바일 명령 프레임 초안 v1 — PC/C 상호 시험 단계

`mobile_wire.py`와 `actuator_core/mobile_wire.c`는 아래 고정 길이 프레임을
encode/decode한다. **기존 팔 message ID에 등록하거나 live UART parser에
연결하지 않았다.** 팔 v2 capability로 모바일 지원을 추정하지 않는다.
`mobile_endpoint`와 `MobileClient`에 capability·stream resync·응답·진단·시간
동기화를 구현했고 native C 모형으로 상호 시험했다. CRC는 전송 오류 검사이며 인증 기능이 아니다.

| byte | 형식 | 의미 |
|---|---|---|
| 0–1 | ASCII `AM` | magic |
| 2 | u8 = 1 | version |
| 3 | u8 | 1 ARM, 2 VELOCITY, 3 STOP |
| 4–7 | u32 little endian | nonzero session |
| 8–11 | u32 little endian | sequence; ARM은 0, 활성 session의 이동/정지는 증가 |
| 12–15 | u32 little endian | **동기화된 MCU 시각 기준** valid_until_ms |
| 16–31 | i32 ×4 little endian | wheel 8/9/10, lift 11 속도. ARM/STOP은 모두 0 |
| 32–35 | u32 little endian | 앞 32 bytes의 CRC32C |

host↔MCU 속도는 signed int32 ±32767이다. MCU→STS3215 단계의 sign-magnitude
인코딩과 구분한다. 초안 STOP은 session에 속하는 정상 정지 요청이며 전역 비상
차단을 대신하지 않는다. 정상 stop 재전송은 새로운 sequence를 사용한다.

`mobile_wire_apply`는 현재 feedback/mode/원점이 검증된 supervisor에서만
활성화하며 만료·너무 먼 미래·다른 session·재전송 명령을 거절한다. 명령의 남은
유효 시간은 watchdog 최대값보다 짧으면 그대로 적용한다. 잘못된 프레임이 계속
도착해도 watchdog은 갱신되지 않는다. 32-bit 시각 wrap을 시험했다.

### 상태 응답·handshake·동기화

동일 36-byte 요청의 opcode 4/5/6은 HELLO/STATUS/TIME_SYNC다. session 자리에는
0이 아닌 query nonce, sequence에는 요청 상관 번호를 넣고 속도 필드는 모두 0이다.
조회는 모터를 활성화하지 않는다. stream parser는 쓰레기/CRC 오류를 밀어내며
100 ms 이상 끊긴 부분 프레임을 폐기한다.

응답은 아래 **64 bytes little endian**, magic `AS`, version 1이다.

| byte | 값 |
|---|---|
| 0–3 | `AS`, version, 원래 opcode (거절 시 bit 7) |
| 4–15 | 요청 sequence, MCU tick ms, boot ID (각 u32) |
| 16–27 | capabilities, 현재 session, 현재 명령 sequence (각 u32) |
| 28–31 | state u8, reason u8, flags u16 |
| 32–51 | feedback tick u32, wheel 8/9/10·lift 11 속도 i32×4 |
| 52–59 | lift 높이 µm i32, 거절 프레임 누적 u32 |
| 60–63 | 앞 60 bytes CRC32C |

capabilities=7은 명령/feedback/clock만 지원한다. 높이 제어기의 wire API를
지원한다는 뜻이 아니다. flags bit 0..4는 sample 있음/mode 확인/homed/실측
정지/hardware_ok다. sample 시각과 timeout도 확인해야 하며 과거 mode flag만
보고 현재 상태가 건강하다고 판단하지 않는다.

`MobileClock`은 RTT≤20 ms인 상관 응답을 사용하고 최대 1초짜리 동기 표본에
RTT 여유를 빼서 보수적 MCU 만료 시각을 계산한다. `MobileClient`는 500 ms마다
필요 시 새 동기를 얻는다. boot ID 변경·응답 불일치·통신 예외는 소유 session을
폐기한다. 명시적 synchronize/arm 없이 재활성화하지 않는다. 실물 endpoint는
매 부팅 다른 boot ID와 실제 UART의 시간 예산을 공급해야 한다.

## 앱의 지도 지점 이동 계약 v1

앱/UI → gateway → `RobotApplication` → `TaskExecutor` → navigation adapter 순서다.
`home_robot_tasks.gateway`는 simulation 전용 HTTP 서버다. bearer 인증·SQLite
request ID 보존·단기 permit을 제공한다. 앱 UI/WebSocket은 없으며 상태를 조회한다.
모바일 앱에 STM32 포트나 wheel velocity를 노출하지 않는다.

지도 descriptor: `NavigationMap.descriptor()`로 map_id/revision/floor_id/frame_id,
바닥 높이·격자 해상도·원점·점유 상태를 제공한다. 현재 3D mesh는 없고
`scene_mesh_available=false`다. rows는 map y가 증가하는 순서이며 이미지 상하
방향과 다를 수 있다. display 좌표→map 좌표 변환은 UI가 명시적으로 수행한다.

이동 요청 예제는 `config/navigate_to.simulation.json`이다. strict 필드는
schema_version=1, operation=navigate_to, request_id, map_id, map_revision,
floor_id, frame_id, point{x,y,z} [m], yaw_rad [-π,π]다. 앱에서 선택한 바닥 지점과
마지막 바라볼 방향을 전송한다. 지도/footprint·여유·센서 유효성 설정은 서버 소유다.

- `submit(request, now_s)` → request_id·상태·검사된 모의 계획. 동일 ID/내용은 같은
  작업 상태를 반환하고 다른 내용의 ID 재사용은 거절한다. 실행 중 새 작업은 busy다.
- `status(request_id)` → WAITING/RUNNING/STOPPING/STOP_UNCONFIRMED 또는
  SUCCEEDED/FAILED/CANCELLED, 실패 원인·단계·제어권·모의 실행 표시.
- `cancel(request_id, now_s)` → 정지 요청. 정지 확인 전에는 취소 완료가 아니다.
- `tick(now_s)` → adapter 진행·시간 초과·연속 상태를 갱신한다. 호출 측이 단조
  증가 시계로 주기 실행해야 한다. 현재 메모리 안의 요청 기록은 최대 128개다.

앱 이동과 fetch는 같은 제어권을 쓴다. map revision 변경은 작업을 취소하고 새
위치 추정을 요구한다. 위치 추정 주입은 현재 simulation 전용 내부 함수이며
클라이언트 명령이 아니다. 앱 통신이 끊겨도 이미 접수한 자율 작업은 서버에서
진행한다. 다시 연결한 앱은 같은 ID로 상태를 조회한다. 서버 자체가 재시작하면
미완료 기록은 INTERRUPTED가 되고 실행되지 않는다. 새 실행에는 새 ID가 필요하다.

### HTTP 경로와 실행

모든 경로에 `Authorization: Bearer <token>`을 요구한다. 토큰은 최소 32자의
ASCII 비공백 난수로 별도 파일에 보관한다. POST는 JSON, 본문 최대 16 KiB,
중복 JSON 필드·Origin·Transfer-Encoding은 거절한다. 기본 주소는 127.0.0.1,
LAN bind는 인증서/키와 TLS≥1.2를 요구한다. 외부 포트를 자동으로 열지 않는다.

| 요청 | 역할 |
|---|---|
| GET `/v1/health` | simulation/장치 비활성/작업/서버 fault |
| GET `/v1/map` | 서버 소유 지도 descriptor |
| GET `/v1/permit` | 3초 유효 단회 permit |
| POST `/v1/requests` | 예제 JSON 접수, 새 ID는 `X-Request-Permit` 필요 |
| GET `/v1/requests/{id}` | 상태 조회 |
| POST `/v1/requests/{id}/cancel` | 본문 `{}`, 확인된 정지까지 STOPPING |

동일 ID/내용 재시도는 permit 없이 기존 결과를 반환한다. 변경된 내용은 409,
만료/없는 permit은 409다. 서버 내부 fault는 새 접수를 503으로 차단하고 정지를
계속 확인한다. 인증 자체의 기기 페어링·앱 UI·인증서 배포는 후속 앱 작업이다.

### Lift AL/LS v1

기존 v2 MOBILE_REQUEST(64)/MOBILE_RESPONSE(65)에 탑재한다. 숫자는 little-endian이며 길이·예약 바이트·CRC32C를 확인한다.

| AL 36B offset | 필드 |
|---|---|
| 0..3 | `AL`, version 1, opcode(1 HOME / 2 MOVE / 3 CANCEL / 4 STATUS / 5 KEEPALIVE / 6 RESET) |
| 4,8,12 | session, sequence, MCU valid_until_ms (u32) |
| 16,20 | target_um (i32, MOVE만 사용), boot_id(u32) |
| 24..31,32 | 예약 0, CRC32C(앞 32B) |

LS 64B: magic/version/opcode(거절은 bit7), request sequence(4), MCU tick(8), boot ID(12), session(16), accepted sequence(20), state/fault/flags(24/25/26), 예약0(27), observed_ms(28), height/target/command/current(32/36/40/44, i32), position_raw(48), rejected_count(52), lease deadline(56), CRC32C(60). Flags: homed=1, active=2, interlocked=4, sample=8, holding=16. 상태는 lift_controller enum과 동일하다. 부팅 뒤 자동 HOME/MOVE는 없다.


### AQ/AT 전체 정지 조회 (후보 0x00024908)

기존 v2 MOBILE_REQUEST/RESPONSE envelope와 팔/리프트의 단일 전송 잠금을 사용한다.
AQ 요청은 36B: `AQ`, version 1, opcode 1, u32 request sequence(offset 4, 0 금지),
reserved 8–31=0, CRC32C(offset 32). 상태 조회는 정지 요청·임대 갱신·재활성화를 하지 않는다.
정지 요청은 기존 SAFE_STOP이며 접수 응답은 물리 완료를 의미하지 않는다.

AT 응답은 64B: `AT`, version/opcode 1, 요청 sequence(4), MCU tick(8), boot ID(12),
stop 시작(16), 모든 전송 완료 시각(20), state byte(24: IDLE=0/PENDING=1/CONFIRMED=2/
UNCONFIRMED=3), flags(25), reserved26–27=0, 증거 시각(28), 증거 나이(32),
최대 나이(36), reserved40–59=0, CRC32C(60). 다중 바이트 정수는 little endian이다.
flags bit0..7은 configured/runtime/output/actions/base/lift/arms/load 순서다.
상위 완료 조건은 state=2, flags=255, 유효한 새 증거 및 boot/sequence 일치다.
호스트는 전체 왕복 시간을 보수적으로 빼서 표본 시각을 환산하고 전송 중 만료도 거절한다.
MCU 미설정/예약 필드/CRC 오류에는 기존 1B envelope 오류를 반환한다.


### AE/AF 독립 관측 입력 (후보 0x00024908)

기존 v2 MOBILE envelope 안의 36B AE 요청 / 64B AF 응답. 모든 정수는 little-endian.
응답 envelope는 기존 status+64B 규칙이며 오류 시 기존 1B status다.

| offset | AE 요청 | AF 응답 |
|---|---|---|
| 0–3 | `AE`, version=1, op=1 | `AF`, version=1, op=1 |
| 4–7 | 증가하는 nonzero sequence | sequence echo |
| 8–11 | MCU boot ID | boot ID |
| 12–15 | 원 관측 MCU ms | 수신 처리 MCU ms |
| 16–19 | 관측 만료 MCU ms | 원 관측 MCU ms echo |
| 20 | bit0 arms_safe, bit1 lift_hold, bit2 payload_safe | flags echo |
| 21–31 / 21–59 | 예약 0 | 예약 0 |
| 32–35 / 60–63 | CRC32C 앞 32B | CRC32C 앞 60B |

수신 age 한계는 stop feedback/lift feedback 중 더 짧은 값(1–1000ms). 원 관측과
sequence가 반드시 증가하고 boot가 일치해야 한다. 미래/만료/너무 긴 expiry/예약
bit·byte/CRC 오류를 거부한다. uint32 ms wrap은 허용하지만 sequence wrap은 새 boot가
필요하다. 첫 성공 이후 만료는 interlock/load 입력을 무효화하고 출력 준비 상태라면
전체 stop을 요구한다. 0인 flags도 유효한 관측이며 안전 승인을 뜻하지 않는다.

`RobotEvidenceClient`는 host→MCU clock 동기화 왕복 불확실성과 센서 나이를 보수적으로
반영하고, 동일 원 관측을 재전송해 수명을 늘리지 않는다. AF는 수신 확인일 뿐 물리적
정지/하중 유지 완료 신호가 아니다. 확인은 AQ/AT와 독립 후속 관측을 사용한다.
