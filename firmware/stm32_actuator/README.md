# STM32 액추에이터 공통 코어

STM32 HAL과 분리한 C11 모듈이다. PC에서 프로토콜·동작 상태·패킷을 시험하고
보드의 UART/DMA 계층에서 재사용한다. 자세한 구조 점검과 모바일 확장 범위는
[펌웨어 점검](../../docs/ARCHITECTURE.md)을 따른다.

## 모듈

| 모듈 | 역할 |
|---|---|
| `protocol`, `cobs`, `crc32c` | host 프레임 구성·검증 |
| `safety` | arm/enable, heartbeat, fault/estop 상태 |
| `setpoint_queue`, `buffered_*` | 기존 6축 경로와 유한 버퍼 실행 |
| `stream_*_v2` | 12축 세션·stream·피드백 기반 실행 |
| `bimanual_goal_map`, `joint_unwrap`, `calibration` | 관절↔raw 좌표와 wrap 처리 |
| `bimanual_dispatch` | 양팔 송신 상태·실패 처리 |
| `sts3215_response` | 팔/모바일 공통 응답 parser·terminal/오류/길이/체크섬 |
| `mobile_feedback` | 모드 readback·4축 raw 상태·시각/건강 집계·독립 lift 증거 |
| `sts3215_packet` | READ/WRITE/SYNC WRITE/checksum의 HAL 독립 구현 |
| `shared_bus` | 단일 UART 거래 소유권·시간 예산·STOP 우선 대기·fault/복구 |
| `bus_router` | 고정 대기열·stable TX buffer·STOP 후 지속 차단·명시적 재개 |
| `mobile_endpoint` | stream resync·64-byte 상태·HELLO/STATUS/TIME_SYNC |
| `lift_controller` | wrap·명시적 homing·높이 접근·독립 hold 확인·fault |
| `system_stop` | 모바일 zero/양팔 hold 순서와 전송 이후 독립 정지/하중 증거 |
| `mobile_wire` | C/Python 호환 36-byte 모바일 명령 초안·CRC·절대 만료 admission; v2 host handler 등록, endpoint 기본 미연결 |
| `mobile_output` | 승인 target→주기/대기열→바퀴/lift/STOP 패킷, 만료·fault 고정 |
| `mobile_supervisor` | 모바일 session/sequence·watchdog·feedback/속도/lift 경계·정지 latch |
| `motor_groups` | 왼팔·오른팔·바퀴·리프트의 버스/ID/모드와 그룹별 패킷 |

`motor_groups`의 모바일 속도 패킷은 바이트 구성 기능이다. 실제 장치 discovery,
mode 전환·readback, wheel odometry, lift 높이 코어의 장치 연결과 UART scheduler는 별도다.
팔 12축 stream을 바퀴·리프트까지 포함한 위치 배열로 확장하지 않는다.

## 보드 계층

- host: LPUART1/STLINK VCP
- 왼쪽: USART1 → Waveshare → 왼팔 + 바퀴 + 리프트의 공유 물리 버스
- 오른쪽: UART4 → Waveshare → 오른팔 버스
- 기존 resident 출력 주기: 5 ms; 현재 telemetry 스케줄은 양팔용
- 팔 서비스 설정: `Core/Inc/servo_joint_config.h`, `Core/Src/servo_joint_config.c`
- 관절 제한: canonical JSON에서 `bimanual_operational_limits`와 ROS 사본 생성; 과거 실측 수치 유지, 새 조립체 범위 실측 필요

중재 코어는 거래 timeout과 소유권을 검사하고 실제 UART 수신·abort·복구는 보드 계층이 수행한다.
`servo_transport`가 기존 팔 서비스/IT/DMA와 router의 보드 HAL 소유권을 연결한다.
`mobile_output`/`MobileServoOutput`의 주기 DMA 출력과 host parser handler는 등록했다.
기본 부팅에서는 미설정이며 자동으로 모바일 장치를 활성화하지 않는다.
PC 모형은 endpoint→supervisor→router→STS 패킷을 연결한다. 모바일 출력 연결 전
장치 RX·mode readback은 `MobileServoFeedback`에 연결했다. 실측 설정·모드 변경·부팅 ID·리프트 높이/hold와 전체 정지 증거의 연결이 필요하다.

## PC 시험

저장소 루트에서 실행한다.

```bash
cmake -S firmware/stm32_actuator -B build/stm32_actuator-host
cmake --build build/stm32_actuator-host
ctest --test-dir build/stm32_actuator-host --output-on-failure
python3 tools/setup/firmware/generate_protocol_header.py --check
python3 tools/setup/firmware/generate_joint_limits.py --check
```

protocol header는 `protocol/message_ids.json`에서 생성한다. 사람이 직접 고치지 않는다.
