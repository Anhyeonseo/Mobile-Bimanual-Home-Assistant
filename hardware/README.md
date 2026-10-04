# 하드웨어 등록

기준 플랫폼은 **ALOHA Mini 1의 3륜 옴니 베이스·수직 리프트·SO-ARM 계열 양팔**이다. 기존 SO101·STM32 팔 제어 기반을 재사용한다. 플랫폼 선택과 실물 구동·피드백 확인 완료는 구분한다.

## 2026-10-02 조립 완료 및 첫 텔레옵 준비

사용자 확인: ALOHA Mini 1 실물 조립 완료. 우선 Raspberry Pi를 사용하며, 바퀴 3개와
리프트는 모두 STS3215 12 V, 30 kg급(사용자 표기)이다. 전원/모델 레지스터 실측은 아직 하지 않았다.
통신은 **Pi → STM32 → 왼쪽 Waveshare 공유 버스**를 유지한다.

| 대상 | 실제 설정 ID | 남은 확인 |
|---|---|---|
| 왼팔 | 기존 1–6 | 실물 응답/펌웨어 버전 |
| 바퀴 | **7, 8, 9** | 각 ID의 왼쪽/뒤쪽/오른쪽 대응과 회전 부호 |
| 리프트 | **10** | 원점·상하 방향·스트로크·정지/하중 유지 |
| 오른팔 | 별도 버스의 기존 1–6 | 실물 응답 |

아래 과거 기록의 8/9/10+11은 공식 예제/기존 모의 구성이다. 새 실물에 적용할 값은 7/8/9+10이다.
`ACTUATOR_MOBILE_FIRST_ID=7` 빌드 설정을 추가해 시작 점검·피드백·정상 명령·모바일 zero·전체 stop이
같은 배치를 사용하게 했다. 기본값 8은 기존 모의 회귀의 기준으로 남긴다. 모터 ID를 EEPROM에 다시 쓰는 기능은 아니다.

공식 [wheels.py](https://github.com/liyiteng/lerobot_alohamini/blob/main/examples/debug/wheels.py)와
[axis.py](https://github.com/liyiteng/lerobot_alohamini/blob/main/examples/debug/axis.py)는 직접 USB 서보 버스용이다.
이번 STM32 포트에 실행하지 않는다. 키 배치/기구학은 참고하되 통신은 기존 STM32 프로토콜을 사용한다.
특히 공식 axis 도구는 시작/과전류 처리 중 torque-off를 수행하므로 조립된 리프트에 그대로 이식하지 않는다.

첫 단계는 Pi의 OS·STM32 포트/baud·펌웨어를 확인하는 것이다. `tools/run/inspect_mobile.py`는
명시적으로 고른 STM32 포트를 독점 사용하고 HELLO와 모바일 조회만 보낸다. ARM/ENABLE/모드 변경/원점 이동/속도 명령은 없다.
호스트 통신을 binary 모드로 전환하므로 기존 ROS 브릿지를 먼저 정상 종료하고, 텍스트 콘솔 복귀에는 MCU reset이 필요하다.

```bash
# Pi에서 실행: 장치 목록 확인만 수행
python3 --version
cat /etc/os-release
ls -l /dev/serial/by-id/

# 포트와 현재 펌웨어의 host baud를 확인한 뒤 저장소 루트에서 실행
python3 tools/run/inspect_mobile.py --port /dev/serial/by-id/<STM32장치> --baud <115200또는921600>
```

`mobile_available=false`는 기존 펌웨어/미설정 endpoint일 수 있다. `true`도 실제 모터 ID·모드·원점·하중 지지가
확인됐다는 뜻은 아니다. 현재 ID 선택 빌드는 컴파일 검사이며 실물 설정/header가 없는 기본 빌드는 모바일 구동을 열지 않는다.
실물 텔레옵은 아직 실행하지 않았고, 조회 결과·바퀴 위치 대응·리프트 시험 조건 확인 후 H1/H2를 진행한다.

### 첫 실물 통신 확인 — 2026-10-02

- Pi: Ubuntu 24.04.3, pyserial 3.5. ST-LINK V3 VCP의 안정 경로는
  `/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02`.
- 기존 YAML은 115200이나 실물 조회는 해당 속도에서 HELLO timeout.
  **921600에서 HELLO 성공**: protocol 2, joints 12, firmware `0x00024809`,
  capabilities `0xEFFFFFFF`, 양쪽 calibration hash `764417662`.
- 조회 시 `stop_latched=true`, `rejected_frame_count=1`. 원인은 이 결과만으로 단정하지 않는다.
  정지 잠금 해제/모터 구동 명령은 보내지 않았다.
- 모바일 HELLO는 `MOBILE_RESPONSE sequence=2` timeout. 호스트 통신은 확인했지만,
  현재 이미지에서 모바일 기능 가용성 및 바퀴/리프트의 실제 ID 응답은 확인되지 않았다.
- 다음 순서: Pi의 ST-LINK 업로드 도구 확인 → 현재 flash 백업 → 모터 구동 없는
  버스 점검 경로 준비/검증 → ID 7/8/9/10 응답과 모드 확인 → 제한된 개별 바퀴 시험.
  실물 profile이 없는 일반 빌드를 업로드하는 것만으로 텔레옵이 준비됐다고 판단하지 않는다.

### 원본 백업 및 읽기 전용 점검 이미지

사용자 실행 결과로 OpenOCD 0.12.0/ST-LINK V3의 flash dump + verify가 성공했다.
원본은 Pi의 `/home/pi/firmware_updates/before_mobile_ayg40G/flash.bin`, **524288 bytes**,
SHA-256 `93dd991524cd279d307cf73e4c60f6cbfef57896c41ba5e7030363b1e44684c4`다.
백업 때 CPU를 halt했으며 모터 전원을 끈 상태에서 이후 업로드를 진행한다.

`MOBILE_BUS_INSPECTION_ONLY=ON`, `ACTUATOR_MOBILE_FIRST_ID=7`로 만든 별도 점검 이미지는
`mobile-bus-inspection-v2` ASCII 인터페이스다. 일반 바이너리 `inspect_mobile.py`와 혼용하지 않는다.
`tools/run/probe_mobile_bus.py --port <STM32포트>`는 INFO만 요청하고,
**이미지 식별 후에만** 명시적 `--scan`으로 SCAN을 보낸다. Python+pyserial만 필요하다.

- 부팅 시 버스 읽기/쓰기 없음. 기존 팔 앱과 모바일 runtime boot/poll을 실행하지 않는다.
- INFO는 12 V를 끈 상태로 실행한다. 기대값은 firmware 이름, 921600 baud, IDs 7/8/9/10,
  `motion_enabled=false`다. 이를 확인한 뒤 지지된 리프트/베이스에서 별도로 12 V 인가 후 SCAN한다.
- SCAN은 왼팔 ID 1의 identity를 비교용으로 먼저 읽고, ID 7..10의 model/ID/baud(주소 3..6), mode(33), torque(40), position(56..57)를 읽는다.
  실패한 필드는 -1이며 실제로 읽은 값으로 취급하지 않는다. 정상 응답도 움직임/하중 안전의 증거는 아니다.
- UART 공통 송신 단계가 모든 WRITE, broadcast, 허용되지 않은 READ를 거절한다.
  팔은 ID 1의 identity READ(주소 3, 길이 4)만 허용한다.
  일반 ARM/토크/위치/속도/홈 명령 처리기는 점검 이미지의 실행 경로에 없다.
- 잘못된 요청과 읽기 실패를 포함한 관련 pytest **91개 통과**, 점검 이미지 및 기존 resident 후보 빌드 성공.
  v1 업로드/INFO는 실물 성공했으며 첫 SCAN은 ID 7..10 모두 실패했다.
  v2는 아직 실물 업로드 전이다. 텔레옵 동작 펌웨어가 아니다.

빌드 예시:

```bash
cmake -S firmware/stm32_g474_single_arm -B build/mobile-readonly -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release -DMOBILE_BUS_INSPECTION_ONLY=ON -DACTUATOR_MOBILE_FIRST_ID=7
cmake --build build/mobile-readonly --parallel 2
```

### 첫 SCAN 실패의 진단 기록과 v2 비교 조회

사용자는 ID 7/8/9/10을 실제 저장했으며 왼팔도 같은 드라이버에 연결돼 있다고 확인했다.
v1 SCAN에서 identity가 모두 실패했다. 해당 **v1 ELF에 한정된 주소**의 RAM dump를 판독하면:

- transaction/success/failure = 4/0/4, timeout 4, lazy RX arm 4, receiver resync 4.
- 마지막 진단 packed word `199170 = 0x00030A02`: reason 2(RX_TIMEOUT), ID 10, HAL status 3(TIMEOUT).
  ARM 빌드 enum 배치를 반영해 해석해야 하며, 32-bit 숫자 하나를 reason으로 해석하면 안 된다.
- 마지막 UART error 0, ISR `0x006000C0`, DMA error 0, 받은 바이트 0.
- STM32 소프트웨어상 송신 이후 RX timeout이다. 커넥터/모터까지 전기 신호가 도달했거나
  모터 전원/baud/배선이 맞다는 증거는 아니다. 원인은 아직 미확정이다.

v2는 SCAN에 왼팔 ID 1 identity 비교와 각 identity의 HAL 상태/실패 이름/RX 바이트/UART·DMA 오류를
직접 출력한다. bus service 획득 전에 실패하면 이전 모터의 진단을 재사용하지 않고
`SERVICE_NOT_STARTED`로 표시한다. ID 1만 성공하면 모바일 연결/전원/ID·baud 쪽을 우선 점검하고,
ID 1도 실패하면 공통 왼쪽 버스 경로부터 점검한다. v1의 고정 RAM 주소를 다른 빌드에 재사용하지 않는다.

### 공통 버스 통신 복구 및 최초 5축 응답 확인

1. 모바일 분리 후에도 왼팔 ID 1은 무응답이었다.
2. 드라이버를 분리한 STM32 D1–D0 루프백에서 각 요청의 RX 8 bytes 및 LENGTH 거절을 확인했다.
   보낸 8-byte READ를 받았으며, 모터 응답 형식과 달라 거절되는 예상된 결과였다.
3. 사용자가 Waveshare 점퍼가 A가 아니었음을 확인하고 A로 수정했다. 왼팔 ID 1 조회가 성공했다.
4. 모바일 버스를 다시 연결한 v2 SCAN에서 **ID 1/7/8/9/10 모두 identity 성공**, `all_reads_ok=true`.
   model_raw=777, baud_code=0(현재 STM32 서보 UART 1 Mbaud에서 응답), 통신 오류 없음.
   바퀴 7/8/9 및 리프트 10은 모두 **mode=0, torque=0**였다.

| ID | 최초 position_raw | mode | torque |
|---|---:|---:|---:|
| 7 | 2639 | 0 | 0 |
| 8 | 3584 | 0 | 0 |
| 9 | 2712 | 0 | 0 |
| 10 | 1438 | 0 | 0 |

이 값은 조회 시점의 모터 레지스터다. 바퀴 위치/부호와 리프트 높이/원점/스트로크는 아직 미확인이다.

### 바퀴 모드 설정 후보 — 움직임 없음

`MOBILE_WHEEL_SETUP_ONLY=ON`, `ACTUATOR_MOBILE_FIRST_ID=7`은 별도 `mobile-wheel-setup-v1` 이미지다.
기존 점검 후보와 동시에 선택할 수 없다. 부팅/INFO/SCAN은 쓰기를 수행하지 않는다.
`tools/run/prepare_mobile_wheels.py --port <STM32포트>`는 INFO만 확인하며, **명시적 `--prepare`**일 때만
`PREPARE_WHEELS`를 보낸다. 이는 EEPROM의 바퀴 모드를 변경하는 명령이며 읽기 전용 조회와 구분한다.

- 세 바퀴 전체의 model 777, ID, baud 0, torque 0, mode 0/1, lock 0/1을 읽은 뒤에만 설정을 시작한다.
- ID 7/8/9의 목표 속도를 0으로 쓰고 검증한다. mode 0인 모터만 unlock → mode 1 → relock 및 readback한다.
  mode 변경 후 20 ms를 기다린다. mode 1이면 불필요한 EEPROM 재기록을 생략한다.
- 불확실한 unlock/mode write 뒤에도 relock을 시도하고 실패 단계/관측값을 보고한다.
  실패 시 다음 바퀴를 설정하지 않고, 같은 MCU 부팅에서 재시도하지 않는다.
- UART 전송 단계는 팔/리프트 쓰기, torque=1, 모든 0이 아닌 속도, 위치/ID/PID 쓰기를 거부한다.
  완료 기대값은 바퀴 세 개 모두 **mode=1, torque=0, lock=1**이다.
- 모터 전원을 끄고 이미지를 업로드/검증한다. 바퀴를 띄우고 리프트를 지지한 상태에서 12 V 인가 후
  `--prepare`를 실행한다. 결과를 확인한 다음 개별 바퀴 펄스 시험으로 진행한다.
- 관련 pytest 102개 통과. 설정용/읽기 전용/기존 resident 이미지 빌드도 통과했다.
  설정 후보는 실물 업로드/실행했다. 다음 결과처럼 부분 실패를 확인했으며 실제 바퀴 회전은 아직 수행하지 않았다.

레지스터와 mode 1/EPROM lock 절차 근거:
[Feetech 공식 Python SDK](https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py),
[ALOHA Mini 공식 wheels 예제](https://github.com/liyiteng/lerobot_alohamini/blob/main/examples/debug/wheels.py).
직접 USB 제어 예제 대신 기존 STM32 공유 버스를 통해 동일 레지스터를 사용한다.

### 최초 바퀴 설정 결과 — 재쓰기 전 독립 조회

사용자 실행 결과: 7/8번은 READY_TORQUE_OFF, mode 1/torque 0/lock 1이다. 9번은 MODE_WRITE 실패이나
실패 후 별도 조회가 mode 1/torque 0/lock 1을 반환했다. 전체 `setup_ok=false`를 유지한다.
현재 결과만으로 전송 실패인지 즉시 readback 실패인지 구분할 수 없다. 모드 변경은 적용됐을 수 있으므로
`--prepare`를 자동 재시도하거나 상태를 임의로 성공 처리하지 않는다.

현재 이미지의 SCAN을 그대로 사용할 수 있도록 읽기 전용 `probe_mobile_bus.py`가
`mobile-wheel-setup-v1`도 식별하도록 했다. INFO와 SCAN만 보내며 PREPARE_WHEELS는 보내지 않는다.
모터 전원을 껐다 켠 뒤 7/8/9의 mode 1 유지 및 torque 0, 리프트 10의 mode 0/torque 0을 확인한다.
이 조회에는 펌웨어 재업로드가 필요 없다. 이후 검증이 통과해도 최초 MODE_WRITE 실패 기록은 보존한다.


### 바퀴 모드 후속 조회 및 단일 바퀴 펄스 후보 — 2026-10-03

사용자가 전달한 후속 SCAN은 `all_reads_ok=true`다. 모든 ID/모델/baud 읽기가 정상이고,
바퀴 7/8/9는 mode 1/torque 0, 리프트 10은 mode 0/torque 0이다.
위치는 각각 2725/3669/2797/1438이다. SCAN은 lock을 읽지 않으므로 이번 결과를 lock 재검증으로
표현하지 않는다. 9번 최초 설정 실패는 보존하고 EEPROM 재쓰기를 생략한다.

`MOBILE_WHEEL_PULSE_ONLY=ON`, `ACTUATOR_MOBILE_FIRST_ID=7`의 별도 `mobile-wheel-pulse-v1`은
**바퀴를 모두 띄운 상태의 ID/방향 확인 도구**다. 부팅/INFO/SCAN에는 모터 쓰기가 없다.
`tools/run/pulse_mobile_wheel.py --port <STM32포트>`는 INFO만 읽고,
`--pulse --wheel 7`을 추가하면 7번에 고정된 +200 raw 속도를 최대 500 ms 창으로 요청한다.
`--direction negative`는 -200이다. 실제 각도·속도 보정 및 지면 주행 기능은 아직 아니다.

- 모든 바퀴의 model/ID/baud, mode 1, torque 0, lock 1, 현재 속도 0을 먼저 읽는다.
- 선택한 바퀴만 목표 속도 0 확인 → torque 1 확인 → 지정 속도 쓰기/확인을 수행한다.
  모터 설정 실패도 한 번의 시도로 소비하며, 같은 MCU 부팅에서 두 번째 펄스를 허용하지 않는다.
- 속도 쓰기 시작 시각부터 500 ms가 되면 STM32가 zero speed와 torque off를 순서대로 시도한다.
  불확실한 쓰기/검증 실패 시 즉시 정지 절차로 들어간다. 첫 정지 쓰기 실패도 두 번째 시도를 생략하지 않는다.
  이 구간에는 호스트 입력/출력을 기다리지 않는다. 버스 전송·정지 완료 시간이 별도로 필요하므로
  500 ms를 실제 기계 정지 시간의 보증으로 보지 않는다.
- 정지 이후 goal speed 0/torque 0/present speed 0을 읽어 `stop_confirmed`에 표시하고,
  전후 위치를 기록한다. 실제 바퀴 위치·방향·회전 여부는 사람이 확인한다.
- 독립 TX 제한은 모든 전송 API에서 wheel 7/8/9의 torque 0/1 및 speed 0/±200만 쓰게 한다.
  팔/리프트, EEPROM, PID, 임의 속도, broadcast는 차단한다. 기존 설정 후보와 운용 이미지는 별도다.
- 모터 12 V를 끄고 flash/verify/reset한다. 바퀴를 안정적으로 모두 띄우고 리프트를 지지한 뒤
  12 V를 켜고 2초 이상 기다려 **7번만 한 번** 시험한다. 출력과 어느 바퀴/방향인지 기록한다.
- 무응답, `stop_confirmed=false`, 계속 회전하면 모터 12 V를 차단한다. MCU reset/USB 전원 상실·
  버스 고장 시 서보의 기존 속도 명령이 남을 수 있다. 재시험은 모터 전원 차단 → MCU reset →
  모터 전원 인가 순서로 한다. 재플래시는 매번 필요하지 않다. 리프트 지지는 계속 유지한다.

PC 검증: 관련 pytest 73개, STM32 펄스/설정/읽기 점검/기존 resident 4개 빌드 통과.
고장 주입은 속도 쓰기 적용 후 timeout, torque/readback 실패, 정지 쓰기 실패, 정지 후 잔류 속도,
잘못된 모드/이미지/명령, 반복 명령, tick wrap을 포함한다. 실물 펄스 결과는 아직 대기다.
배포 묶음: `output/wheel-pulse-20261003/` (소스 해시·빌드 옵션·파일 SHA256 포함).


### 펄스 v1 실기 결과 및 가시성 개선 v2

7번 v1 실기: `pulse_ok=true`, position 2724→2806(+82), stop_requested_ms 501,
goal_after/torque_after/speed_after 모두 0, `stop_confirmed=true`다. 사용자 눈에는 회전이
잘 보이지 않아 물리적 바퀴 위치·방향은 아직 미확인이다. 엔코더 변화만으로 바퀴 결합/회전 방향을 확정하지 않는다.

사용자 요청에 따라 `mobile-wheel-pulse-v2`는 **속도 ±200 raw 유지, 시간 500→3000 ms**로 변경했다.
TX 속도 허용 범위, 한 부팅당 한 번, 전 축 사전 점검, 정지 쓰기/readback은 유지한다.
호스트 응답 대기는 8초이며 v1 이미지에는 펄스 명령을 보내지 않는다. 모터 전원을 끄고 재업로드 후
바퀴를 모두 띄우고 리프트를 지지한 채 7번부터 다시 확인한다. MCU/버스 고장 시 전원 차단 필요 조건도 동일하다.
3초는 속도 쓰기 시작부터 정지 요청까지의 창이며 실제 회전량이나 기계 정지 시간을 보증하지 않는다.

v2 관련 pytest 74개와 펄스 펌웨어 빌드가 통과했다. 실물 결과는 대기다.
배포 묶음은 `output/wheel-pulse-v2-20261003/`, 실행 도구는 `pulse.pyz`다.


### v3 — 선택 바퀴만 사전 점검

v2의 8번 요청은 7번 사전 정지 검사에서 `WHEEL_MUST_BE_STOPPED`로 차단됐다.
`motion_attempted=false`, `cleanup_attempted=false`이며 이 실행에는 모터 쓰기가 없었다.
v2는 속도 읽기 실패(-1)도 같은 단계로 표시했으므로 이 결과만으로 7번이 회전 중이었다고 판단하지 않는다.

사용자가 요구한 바퀴를 띄운 개별 ID 확인 범위에 맞춰 **v3는 요청한 ID만 읽고 쓴다**.
8번 요청 시 7/9번은 점검 대상이 아니다. 선택한 모터의 신원/모드/토크/lock/속도 사전 검사,
±200 raw/3초, 한 부팅당 한 번, zero/torque-off와 정지 readback은 유지한다.
속도 READ 실패는 `SPEED_READ_FAILED`, 비영 원시값은 `WHEEL_MUST_BE_STOPPED`로 구분하고
`speed_check_raw`와 `speed_check_hal_status`를 출력한다. 정지 허용 오차를 늘리지는 않았다.
전체 주행의 다축 정지 검증과 이 단독 시험은 구분한다. 모든 바퀴를 띄우고 리프트를 지지한다.

pytest 77개와 펄스 이미지 빌드 통과. 테스트는 선택하지 않은 바퀴에 비영 속도/다른 모드가
있어도 해당 ID를 읽거나 쓰지 않는 것과 선택한 모터의 읽기 실패/비영 속도 차단을 확인한다.
배포 묶음: `output/wheel-pulse-v3-20261003/`. 실물 결과 대기.


### 사용자 확인 바퀴 ID 배치 — 2026-10-03

좌우는 로봇이 정면을 바라보는 기준이다.

| 바퀴 위치 | 모터 ID | +200 명령, 바깥쪽에서 본 회전 |
|---|---|---|
| 정면 중앙 | 9 | 시계 |
| 왼쪽 | 7 | 시계 |
| 오른쪽 | 8 | 시계 |

사용자가 개별 재시험으로 위 배치와 회전 방향을 확인했다. 실제 설치 각도/치수와
본체 이동 방향은 별도 검증이 필요하다. 실측 운용 프로파일은 아직 활성화하지 않는다.


### v4 — 공중 3륜 조합 시험

`mobile-wheel-pulse-v4`/`base.pyz`는 단일 ID 시험과 별도로 `BASE F/B/L/R/A/D`를 지원한다.
기본 실행은 INFO만 읽는다. 명시적 `--motion forward` 등의 옵션이 있어야 구동한다.
단일 바퀴/조합 시험은 한 MCU 부팅당 한 번의 제한을 공유하며 자동 재시도하지 않는다.

| 동작 후보 | 7 왼쪽 | 8 오른쪽 | 9 정면 |
|---|---:|---:|---:|
| forward | -200 | +200 | 0 |
| backward | +200 | -200 | 0 |
| left | -100 | -100 | +200 |
| right | +100 | +100 | -200 |
| rotate-left | +200 | +200 | +200 |
| rotate-right | -200 | -200 | -200 |

수치는 서보 raw 목표 속도이며 m/s가 아니다. +는 바깥쪽에서 시계 방향이다.
전진 x/왼쪽 y/반시계 yaw 기준, 바퀴 중심의 명목 방위각은 7=120°, 8=240°, 9=0°다.
양의 회전에 대응하는 굴림 방향 `(-sinθ, cosθ)`으로 계산한 비율을 기존 `OmniKinematics`와 대조했다.
[공식 wheels 예제](https://github.com/liyiteng/lerobot_alohamini/blob/main/examples/debug/wheels.py)의
120° 기구학을 참고하되 원본의 left/back/right 이름·ID·입력 부호를 그대로 복사하지 않았다.
실제 120° 설치와 치수/속도는 아직 미실측이므로 일반 주행 profile로 승격하지 않는다.

- 조합 시험에는 세 바퀴가 참여한다. 세 바퀴의 신원/mode 1/torque 0/lock 1/속도 0과 위치를
  읽은 다음 모두 목표 속도 0 검증 → 모두 torque 1 검증 → 조합 목표 속도를 쓴다.
  첫 속도 쓰기부터 3초 뒤 전체 정지 절차로 들어가며, 시작 중 실패하면 즉시 전체 정지를 시도한다.
- 쓰기는 공유 UART의 순차 유니캐스트다. 완전히 동시 시작하거나 실제 주행 속도를 보장하지 않는다.
  전진 시 9번도 목표 속도 0의 참여 축이다. 한 모터 실패로 다른 축 정지 시도가 생략되지 않는다.
- 전체 zero 쓰기와 torque-off 쓰기를 먼저 시도하고 각 축의 goal/torque/present speed=0을 확인한다.
  전후 위치와 각 축·그룹 `stop_confirmed`를 보고한다. 실패/무응답 시 모터 12 V를 차단한다.
  MCU 자체/USB 전원/버스 고장에 대한 독립 정지 장치는 아니다.
- 단일 ID 시험은 선택한 ID만 검사한다. UART gate는 wheel 7/8/9의 torque 0/1과 speed 0/±100/±200만
  허용한다. 팔/리프트/EEPROM/임의 속도/방송 쓰기는 차단한다.
- 모터 전원 off 상태로 flash 후 바퀴를 모두 띄우고 리프트를 지지한다. 12 V 인가 후 2초 기다려
  `base.pyz --port <STM32포트> --motion forward`를 먼저 한 번 실행한다.
  7 반시계/8 시계/9 정지와 결과를 확인한다. 다음 조합 전에는 12 V off → MCU reset → 12 V on 순서다.

검증: 관련 pytest 163개, 4종 이미지 빌드 통과. 순차 시작 중 불확실한 쓰기/읽기 실패,
부분 torque enable, 한 축 정지 실패, 잔류 속도, 6개 조합 및 기구학 대조, tick wrap,
단일/그룹 재실행 차단을 포함한다. 실물 조합 결과는 아직 대기다.
배포: `output/base-pulse-20261003/`의 `base-pulse.hex`, `base.pyz`, `pulse.pyz` 및 SHA256/소스 manifest.


### 수동 베이스 텔레옵 — mobile-base-teleop-v1

사용자가 전진·왼쪽 이동·왼쪽 회전 조합의 움직임을 확인하고 반복 시험 종료/텔레옵 진행을 요청했다.
상세 JSON 정지 결과는 이번 세 회차에서 미수신이다. 독립 실물 정지 성능 검증 완료로 확대하지 않는다.

별도 빌드 `MOBILE_BASE_TELEOP_ONLY=ON`, `ACTUATOR_MOBILE_FIRST_ID=7`을 사용한다.
`teleop.pyz --port <STM32포트>`는 INFO만 읽는다. **`--run`**이 있어야 사전 검사와 명시적 ARM 후
키보드를 받는다. Pi 로컬/SSH의 대화형 터미널에서 실행하며 ROS나 pygame이 필요하지 않다.
STM32 포트를 다른 bridge/프로세스가 동시에 열면 안 된다.

| 키 | 동작 |
|---|---|
| W / S | 전진 / 후진 |
| A / D | 왼쪽 / 오른쪽 횡이동 |
| Q / E | 왼쪽 / 오른쪽 제자리 회전 |
| Space | 세 바퀴 목표 속도 0; 다음 이동 입력 가능 |
| X / Esc / Ctrl-C | zero → torque off → 정지 readback 후 종료 |

- 키를 누르고 유지할 때 터미널의 키 반복 입력을 사용한다. 입력이 300 ms 없으면 호스트가 zero를 보낸다.
  터미널은 key-up을 전송하지 않으므로 즉시 손을 뗀 시각을 아는 방식이 아니다. OS 반복 시작 지연이 길면
  첫 이동이 잠깐 멈췄다가 반복 입력 때 이어질 수 있다. Space는 다음 제어 주기에 zero를 보낸다.
- 호스트는 요청/응답 순서로 최대 20 Hz 전송한다. CRC16-CCITT(초기 0xffff), 세션 nonce, 엄격한 증가
  sequence로 손상/재전송/다른 세션 명령을 거부한다. 펌웨어는 수신 시각 기준 400 ms 공백에서
  모든 wheel zero/torque-off를 시도하고 fault를 유지한다. INFO/불완전 프레임은 lease를 갱신하지 않는다.
- 통신 종료, malformed frame, goal write/readback 실패, torque 상실에서도 전체 정지를 시도한다.
  정지는 모든 zero와 torque-off를 먼저 시도하고 goal/torque/present speed=0을 후속 조회한다.
  버스 함수는 유한 timeout을 갖지만 400 ms는 정지 절차 시작 기준이지 실제 기계 정지 완료 보증이 아니다.
  MCU 정지/USB 전원 상실/버스 고장은 소프트웨어만으로 정지를 보장하지 못하므로 12 V 차단 수단을 유지한다.
- 정상 종료 후에는 재부팅 없이 새 세션으로 재실행 가능하다. fault는 자동으로 해제하지 않는다.
  원인을 확인하고 모터 12 V off → MCU reset → 12 V on으로 복구한다. 이전 이동 명령은 재사용하지 않는다.
- 확인한 조합 표를 펄스/텔레옵이 `mobile_bench_patterns.h`에서 공유한다. 최대 raw 200이며 횡이동의
  좌우 휠은 100이다. 임의 속도·팔·리프트·EEPROM 쓰기는 UART gate로 차단한다. 부팅에는 모터 쓰기가 없다.
- 이는 수동 베이스 운전 후보다. 지면 속도/치수·odom·Nav2·충돌 회피·리프트 제어를 완료했다는 뜻이 아니다.
  팔·리프트는 낮고 안정적으로 지지된 상태, 주변이 비어 있는 공간에서 시작한다.

배포: `output/base-teleop-20261003/base-teleop.hex`, `teleop.pyz`, SHA256/소스 manifest.
모터 12 V off 상태로 flash/verify/reset → 12 V on 후 2초 대기 → `teleop.pyz --port <포트> --run`.
PC 검증: pytest 197개, 텔레옵/펄스/설정/점검/기존 resident 5종 빌드 통과.
고장 주입은 CRC/세션/중복 sequence, heartbeat 상실, 입력 공백, partial frame, tick wrap,
시작 중 실패, 늦은 버스 응답, 정지 실패, fault 재ARM 차단, 터미널 정상/오류 종료 cleanup을 포함한다.
실물 텔레옵 결과는 대기다.


### 텔레옵 v2 — 시작 정지 재관측과 실제 키 누름/해제

v1 실기 ARM 실패: ID 7, raw speed 32818(0x8032), HAL_OK, session/sequence 0,
`STOP_UNCONFIRMED_CUT_MOTOR_POWER`. 사용자는 당시 실제 바퀴 움직임을 보지 못했다.
[Feetech SDK](https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py)의
15번 비트 부호 해석으로 -50 raw다. 회전량/물리 속도나 잡음이라고 단정하지 않는다.

v2는 한 번의 즉시 속도 비교를 다음과 같이 바꿨다.

- 신원/mode 1/torque 0/lock 1 확인 → torque off 상태에서 각 wheel 목표 속도 0 쓰기/readback →
  최대 1초의 반복 관측에서 각 축 goal 0/torque 0/present speed 0을 연속 3회 확인 → torque enable.
- 0x8000의 음수 0만 0으로 정규화한다. ±50 같은 비영 속도는 허용하지 않는다. 읽기 실패나 지속적인
  비영 값은 여전히 ARM을 차단한다. 종료 정지도 같은 연속 관측을 사용하며 모든 zero/torque-off 시도는 유지한다.
- 최초 실패 원인 `first_failure`/`first_failed_id`를 후속 STOP 결과와 분리한다. 실패/기동/종료 응답은
  각 축의 목표 속도, 토크, 속도 원시값/부호 해석/유효 여부, 위치, 연속 정지 표본 수를 포함한다.
  1초는 관측 창이며 개별 통신 timeout과 정지 쓰기 시간이 추가될 수 있다. ARM/STOP 호스트 대기는 6초다.

**현재 권장 입력은 `--web`**이다. Pi에 GUI 설치 없이 PC/휴대폰 브라우저에서 조종한다.
`teleop.pyz --port <STM32포트> --web --bind <Pi-IP>`를 실행하고 터미널의 전체 조종 URL(`#토큰` 포함)을 연다.
토큰은 매 실행 생성되며 로그에 기록하지 않는다. 네트워크 공개 범위는 명시한 bind 주소이고 기본은 localhost다.
W/S 전후, A/D 횡이동, Q/E 회전, Space 정지. 키/화면 버튼을 누르는 동안만 움직인다.
한 번 누른 키를 반복 입력할 필요 없으며 keyup/pointerup/cancel/창 이탈에서 즉시 zero 요청을 보낸다.
한글 입력 중에도 물리적인 W/A/S/D/Q/E 키 위치로 동작한다. Pi 터미널 Ctrl+C로 정지·토크 해제 후 종료한다.

- 브라우저는 누른 상태를 100 ms마다 전송한다. 250 ms 끊기면 호스트가 zero를 전송한다.
  호스트→STM32는 기존 400 ms watchdog을 유지한다. 웹 연결 복구는 누르고 있던 동작을 자동 재개하지 않는다.
- 한 조종창이 제어하며 클라이언트 ID/증가 sequence로 뒤늦은 이동 메시지가 keyup 정지를 덮어쓰지 못한다.
  토큰 검증·same-origin 검사·짧은 입력 제한을 사용한다. 조종 URL을 타인에게 공유하지 않는다.
- 원래 `--run` 터미널 입력은 호환용으로 남아 있지만 key-up을 받지 못하므로 실제 hold 조작은 `--web`을 사용한다.
- raw 최대 200, 팔/리프트 쓰기 차단, 부팅 무동작, fault 후 재ARM 차단은 유지한다.
  계속 움직이거나 정지 확인 실패 시 모터 12 V를 차단한다. MCU/버스 자체 고장은 독립 정지 장치가 필요하다.

검증: pytest 217개와 v2 STM32 빌드 통과. -50 일시/지속 피드백, 음수 0, READ 실패,
정지 지연, fault 원인 보존, 웹 소유권/인증·sequence·연결 상실·key-up·blur·touch·한글 키 입력을 포함한다.
배포: `output/base-teleop-v2-20261003/`. 후속 실기에서는 ID 7의
`STARTUP_NOT_QUIET_OR_READ_FAILED`가 반복됐다. session/sequence=0이며 cleanup 후 모든 축은
목표/토크/속도 0, quiet_samples=3, stop_confirmed=true였다. 최초 이상 값은 cleanup에 덮여 미확정이다.

### 텔레옵 v3 — 제한된 읽기 회복과 최초 이상 기록

현재 배포: `output/base-teleop-v3-20261003/`의 firmware와 `teleop.pyz`를 함께 사용한다.
정지 검사 읽기가 일시적으로 실패하면 기존 1초 관측 창 안에서 재관측하며, 무효 sweep이 있으면 모든 축의
연속 정지 횟수를 초기화한다. 유효한 새 정지 표본 3회가 있어야 ARM을 허용한다. 관측 창이 끝난 뒤
완료된 표본도 시작 허용에 사용하지 않는다. 계속되는 읽기 실패나 비영 속도를 허용하는 변경은 아니다.
주행 쓰기 자동 재시도·fault 자동 해제·팔/리프트 구동은 추가하지 않았다.

`first_observation`은 최초 이상 관측을 후속 정지 처리/STOP에도 보존한다. 회복된 일시 오류도 포함하므로
이 필드의 존재 자체가 fault라는 뜻은 아니다. `phase`, `id`, goal/torque/speed/position 원시값,
`failed_register`, `hal_status`, `elapsed_ms`를 기록한다. failed_register=0이면 읽기 자체는 성공했고
목표/토크/속도 값이 정지 조건과 달랐다는 뜻이다. 주소 46=목표 속도, 40=토크, 58=현재 속도,
56=위치다. `first_failure_wait_ms`는 최초 fault 당시 관측 대기 시간이며 cleanup 후 시간과 구분한다.

검증: 관련 pytest **225개** 및 STM32 빌드 통과. 각 레지스터 일시 오류 후 새 관측으로 회복,
연속 횟수 중간의 오류 초기화, 지속 오류/비영 속도/기한 초과 차단, cleanup 성공 뒤에도 최초 이상 보존을 검증했다.
실물에서 원인이 확인되거나 텔레옵 성공이 확인된 상태는 아니다. 업로드 시 모터 12 V를 끄고 USB 연결을 유지한다.
업로드 후 모터 전원을 켜고 2초 뒤 `--web --bind <Pi-IP>`로 실행한다. 다시 실패하면 반복 리셋 대신
`first_observation`을 포함한 전체 출력을 확인한다.


### Pi 조종 프로그램 v3.1 — 만료 세션과 지연 요청 차단

재검토에서 v3 웹 호스트의 두 문제를 재현했다. 브라우저 상태가 250 ms 끊겨 zero로 바뀌어도,
더 큰 sequence의 지연 이동 요청을 받아 다시 출발할 수 있었다. 같은 client의 claim 재요청도
sequence를 0으로 초기화해 이전 요청을 다시 허용했다.

v3.1은 만료된 client의 제어 요청을 계속 거부하고 새 페이지의 새 client가 claim해야 한다.
같은 client나 이미 교체된 client의 claim은 순서 번호를 초기화하지 못한다. 교체 이력은 최대 256개이며,
한 실행에서 그 수를 넘기면 Pi 프로그램을 재시작해야 한다. 웹 요청은 한 번에 하나만 보내며,
대기 중 키를 놓으면 이전 응답 완료 직후 현재 zero 상태를 보낸다. 요청이 200 ms 안에 완료되지 않으면
입력을 지우고 더 전송하지 않는다. 서버는 250 ms lease 만료 시 zero를 요청하며 MCU watchdog은 400 ms다.
이는 물리적 제동 완료 시간의 보장이 아니다. 통신 실패 후에는 키를 놓고 페이지를 새로고침한다.

관련 pytest **230개**와 펌웨어 29개 시나리오의 ASan/UBSan 검사 통과.
LeakSanitizer는 검사 환경의 ptrace 제약으로 비활성화했으며 누수 검사 통과를 의미하지 않는다.
배포는 `output/base-teleop-v3.1-20261003/`. 펌웨어/HEX는 v3와 해시가 같아서 v3를 이미 업로드했다면
Pi의 새 `teleop.pyz`만 사용하면 된다. 이전 v2 펌웨어라면 모터 전원을 끄고 이 묶음의 HEX를 업로드한다.
실물 시작 오류 원인은 여전히 `first_observation` 결과 확인이 필요하다.


### 텔레옵 v4 — 목표 0 쓰기 후 토크 OFF 명시

2026-10-04 사용자 실기 로그에서 `first_observation`은 STARTUP, ID 7, goal_raw=0,
torque_raw=1, speed_raw=0, position_raw=1354, failed_register=0, hal_status=0,
elapsed_ms=8이었다. 시작 검사는 1050 ms 뒤 차단됐으며 cleanup 후에는 ID 7/8/9 모두
목표/토크/속도 0, quiet_samples=3, stop_confirmed=true(174 ms)가 확인됐다.
시작 preflight의 torque=0 검사를 통과하고 목표 속도 0을 쓴 뒤에 torque=1이 관측된 것이다.
정지 실패 원인을 단순 읽기 오류나 비영 속도로 취급할 수 없으며, 모터 내부 자동 enable 동작인지는 아직 추론이다.
토크 주소 40과 목표 속도 주소 46은 [Feetech 공식 SDK](https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py)와 일치한다.

v4는 각 wheel에 목표 0 쓰기/검증 → torque OFF 쓰기/검증을 수행한다. 그 뒤에는 목표를 더 쓰지 않고
세 축의 목표 0/토크 0/속도 0을 연속 관측한 뒤 명시적으로 torque enable한다. OFF 쓰기나 readback이
실패하면 `TORQUE_OFF_BEFORE_SETTLE`로 차단하고 전 축 정지 처리를 시도한다.
부팅 무동작·초기 신원/mode/torque/lock 검사·지속 비영 속도 차단·fault 잠금·웹 v3.1 세션 처리는 유지한다.

목표 쓰기 시 토크가 켜지는 모의 조건에서 수정 전 실패를 재현했다. 수정 후 관련 pytest **233개**,
STM32 빌드, 펌웨어 **32개** 시나리오 ASan/UBSan 통과. 모의 장치 검증이며 실물 성공 증거는 아니다.
배포 `output/base-teleop-v4-20261004/`: 이번에는 HEX와 Pi `teleop.pyz`를 함께 교체한다.
모터 12 V를 끄고 USB를 유지해 업로드/검증한 뒤, 모터 전원을 켜고 2초 후 `--web --bind <Pi-IP>`를 실행한다.

**2026-10-04 실물 결과:** 사용자가 v4 배포 후 브라우저 베이스 텔레옵이 잘 동작한다고 확인했다.
이는 수동 베이스 조작 성공이며 리프트/팔 동시 제어, odometry·제동 거리 측정, 통신 단절의 실물 검증,
Nav2 자율주행 완료를 뜻하지 않는다. 앞선 버전의 실패/대기 표기는 당시 이력으로 보존한다.

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
