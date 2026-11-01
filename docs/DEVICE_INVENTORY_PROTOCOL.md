# S1 장치 조회 계약

이 조회는 L1(STM32)의 고정 레지스터 읽기를 L2(호스트)가 해석하는 유지보수 기능이다.
자연어·인지·동작 계획 계층에서 모터 레지스터에 직접 접근하지 않는다.
`home-assistant-s1-inventory` preset의 펌웨어 식별값은 `0x00024909`다.
모바일 실행 프로필은 아직 연결하지 않는다.

## 사용

오른팔 UART4는 STM32 PC10(TX)/PC11(RX), NUCLEO-G474RE의 **CN7 1/2번**이다.
CN10 1/2번(PC9/PC8)과 혼동하지 않는다.
Waveshare Bus Servo Adapter (A)는 일반 UART의 교차 배선 설명을 적용하지 않고,
제조사 표기대로 **어댑터 TX → CN7 1번(TX), RX → CN7 2번(RX), GND → GND**로 연결한다.
점퍼는 A 위치다. 배선 변경은 전원을 끈 상태에서 한다.
근거: [Waveshare 공식 배선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example),
[ST UM2505 핀 배치](https://www.st.com/resource/en/user_manual/um2505-getting-started-with-stm32g4-series-nucleo64-board-stmicroelectronics.pdf),
`Core/Src/stm32g4xx_hal_msp.c`의 UART4 설정.

```bash
python3 tools/diagnostics/stm32_device_inventory.py \
  --port /dev/serial/by-id/<확인한-STM32-포트> \
  --output output/home-assistant/s1/host-inspection.json
```

기본 실행은 HELLO와 제어기 상태만 읽는다. 모터 전원·지지 상태를 확인한 후
`--read-motors`를 추가하면 왼쪽 버스 1–10, 오른쪽 버스 1–6을 순서대로 읽는다.
왼쪽 7–9는 바퀴, 10은 리프트의 예상 ID이며 실제 응답 확인 전에는 선언값이다.
결과는 개인 장치 정보이므로 `output/`에 보관하고 Git에 올리지 않는다.
공유 backend 잠금과 시리얼 독점을 사용한다.

## 전송과 허용 조건

기존 v2 COBS/CRC/sequence 프레임의 요청 66, 응답 67을 사용한다. v1에는 추가하지 않는다.
요청 payload는 `[schema=1, bus, motor_id, reserved=0]` 4바이트다.
bus 0은 왼쪽, 1은 오른쪽이다. 위 ID 범위 이외 및 broadcast 요청은 거절한다.
주소·길이·쓰기 값을 호스트가 지정할 수 없다.

STOP 상태, 팔 설정/출력 활성, legacy/buffered 실행, stream 실행 또는 모바일 설정 중에는
읽기 전에 거절한다. 실행기는 준비된 CLOSED 상태여야 한다. 이 조건은 호스트 검사와
별개로 STM32가 적용한다. 요청은 토크·운전 모드·목표값·EEPROM을 쓰지 않는다.
진단 성공도 이동 허가나 물리 보정 완료를 의미하지 않는다.

## 응답 56바이트

| 바이트 | 의미 |
|---|---|
| 0–3 | schema, status, bus, 요청 motor ID |
| 4 | 아래 5개 블록 유효 비트 |
| 5–7 | 첫 실패 블록(없음=255), 원래 읽기 오류, 오류 영역(왼쪽 HAL=1/오른쪽 read status=2) |
| 8–11 | 조회 시작 tick, little-endian uint32 |
| 12–18 | 레지스터 0–6: 펌웨어·모델·응답 ID·baud |
| 19–22 | 레지스터 9–12: 최소/최대 위치 |
| 23–25 | 레지스터 31–33: offset 원시값·운전 모드 |
| 26–35 | 레지스터 40–49: 토크·목표 위치/속도·토크 한계 |
| 36–51 | 레지스터 55–70: lock·현재 위치/속도/부하·전압/온도·이동 상태·전류 |
| 52–55 | 예약, 0 |

status: 0=전체 읽기 성공, 1=잘못된 요청, 2=유지보수 불가, 3=일부/전체 읽기 실패.
블록들은 순서대로 읽으므로 동시 측정값이 아니다. 첫 identity 읽기가 실패하면 해당 모터의
나머지 블록은 생략한다. 이후 블록 실패 시 해당 영역은 0으로 지우고 유효 비트를 내린다.
호스트에서는 무효 영역을 `null`로 표현하여 0 위치/정상값과 구분한다.
모든 값은 모터 원시 단위다. offset 부호 해석, 관절 각도·바퀴 거리·리프트 높이 환산은
검증된 장치 프로필을 적용할 다음 단계의 책임이다.

호스트는 부분 응답, 요청 ID와 실제 ID 불일치, 연결 중단을 통과로 처리하지 않는다.
정지 상태로 조회가 차단된 경우에도 HELLO와 차단 당시 상태를 보고서에 보존한다.
연결이 끊어지면 이미 얻은 결과를 보존하고 재시도 없이 중단한다.
이름과 ID만으로 장치가 미설치인지, 전원이 꺼졌는지, 통신 문제가 있는지 단정하지 않는다.
