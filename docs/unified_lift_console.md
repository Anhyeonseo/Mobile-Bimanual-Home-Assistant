# 통합 펌웨어 리프트 현장 조작

`tools/run/unified_lift.py`는 STM32 `0x0002492d`, 설치 프로필 CRC32 `2430209922`용 현장 감독 조작 도구다. 기존 `unified_pick.py`와 `platform_preparation.move_observed_lift`의 최하단 기준 1cm 제한은 변경하지 않았다. 새 도구는 그 1cm 검사를 먼저 실행한 뒤 별도 사용자 키 입력으로 더 높은 목표를 요청한다.

최초 단일 파일 배포는 Jetson 구형 릴리스에 `home_robot_tasks.platform_preparation`이 없어 import 단계에서 중단됐다. 제어기 연결·토크 변경 전의 실패다. 이를 수정한 묶음 배포와 장치 모의 실행을 포함한 관련 테스트 146개가 통과했다. 이후 실제 첫 상승 시험에서는 아래의 유지/정지 확인 실패가 발생했다. 모의 테스트 통과가 실제 유지·정지 성능을 검증한 것은 아니다. 기존 촬영 자료의 0~0.55m 상승 샘플 검사에서는 충돌이 없었으나, 이것은 현재 양팔 자세와 보이지 않는 배선·받침·장착물의 충돌 증명은 아니다.

## 실제 첫 상승 시험 실패 — 2026-11-17

한국 시간 19:31:28에 시작한 `unified-lift-20261117T103128_184409Z` 실행에서 원점 등록 뒤 10mm 목표로 상승했고, 표시 높이 약 8.1mm에서 `WAITING_UP_CHECK`로 전환했다. 도착 판정의 허용오차 이내라는 사실과 이후 유지 성공은 별개다. 다음 구간에서 `controller stop or runtime fault`가 발생했고, 통합 STOP 확인은 4초 시간 초과로 `stop_confirmed=false`였다. 콘솔에 `RISING_TO_TARGET`이 없으므로 550mm 상승은 시작되지 않았다.

19:36:19에 수집한 진단에서 최초 원인은 보드 source 2, 리프트 `LIFT_BAD_FEEDBACK`(fault 1)로 확인됐다. 속도 목표 0·토크 ON 읽기 상태에서 raw가 직전 표본의 3268에서 3269로 변했고 speed -50, moving 1, hold_verified 0이 기록됐다. 모터 통신 실패는 기록되지 않았다. 로컬 C 재현에서는 위치 1카운트 변화와 해당 speed/moving 보고의 조합이 유지 근거를 해제했다. 다만 실제 미세한 내려앉음과 보고 흔들림은 구분되지 않았고, 초기화 전 quiet anchor도 기록되지 않아 정확한 조건 확정에는 연속 표본이 더 필요하다.

STOP ACK는 왔지만 독립 증명의 lift holding·load retained 비트가 빠져 전체 정지는 확인되지 않았다. 실제 정지/지지는 아직 현장 확인이 필요하다. `tools/diagnostics/inspect_lift_failure.py`는 기록 파일, cached runtime/read-failure, commissioning query, STOP proof query, lift status만 읽으며 구동·토크·원점·STOP·리셋 명령을 보내지 않는다. 상세 분석과 재현은 `output/jetson-remote-test-20261117/analysis/LIFT_FAILURE.md` 및 `replay_lift_hold_failure.py`에 보존했다. 실행·원점 재등록·리셋을 반복하거나 보호 조건을 완화하지 않고, 유지·정지 복구 절차를 검증한 뒤 구동 재개를 별도로 정한다.

## Jetson 배포

### 실패한 boot 87의 지지 상태 복구

`tools/run/recover_supported_lift.py`는 이번에 확인한 boot 87 / 최초 오류 tick 7137307에 한정한 복구 도구다. 기본은 조회만 수행한다. 사용자가 양팔·빈 리프트를 독립적인 기계적 받침으로 지지한 뒤 `--execute --supported-empty`를 명시하면 기존 OpenOCD 경로로 MCU를 한 번 재시작하고, 기존 펌웨어의 supported boot release를 한 번 호출한다. 펌웨어 설치·구동·토크 ON·원점 등록은 하지 않는다.

재시작 전에 펌웨어/보정/프로필/최초 오류를 대조하고, 현재 리프트의 신선한 연속 위치 관측을 400ms 이상 확인한다. 기록된 위치와 2카운트를 초과하는 차이가 나거나 구동·부팅 상태가 달라지면 쓰기 전에 중단한다. 재시작 후에는 깨끗한 미등록 부팅만 받아들이며, 모든 모터의 토크 OFF와 보정 보존을 독립 조회로 확인한다. 해제 ACK 오류가 발생하면 재전송 없이 조회만 수행한다. 이미 받은 정상 ACK 이후의 검증 실패는 새 관측 창으로 재시도하지 않는다.

최종 `ALL_16_TORQUE_OFF_VERIFIED` 뒤에도 받침은 유지해야 한다. 실제 최하단 복귀는 현장에서 수행하고 별도로 확인한다. 이 도구는 올라간 위치를 최하단으로 등록하거나 기존 유지 오류를 해결했다고 간주하지 않는다. 실패 시 같은 명령을 반복하지 않는다. 재시작된 부팅에서는 boot 87 조건을 통과할 수 없다.

복구 도구의 모의 검사 20개와 기존 commissioning/진단 검사를 합쳐 113개가 통과했고, 배포된 묶음 32개 모듈로 독립 import도 확인했다. 이 검사는 실제 토크 해제 성공을 대신하지 않는다.

### 상승 콘솔 배포

`python3 tools/diagnostics/package_unified_lift.py`로 필요한 Python 모듈 32개를 포함하는 `output/jetson-remote-test-20261117/deployment/unified-lift-<해시>.tar.gz`를 만든다. 압축 파일을 Jetson의 `/home/hyper/robot/shared`에 복사하고 같은 위치에 풀면 독립된 실행 폴더가 생긴다. 실행 파일만 복사하는 방식은 구형 릴리스의 모듈 누락을 해결하지 못한다.

묶음은 펌웨어나 설정 파일을 포함하지 않는다. `--root`는 기존 배포본의 baseline/캘리브레이션 설정에 사용되며 기본값은 `/home/hyper/robot/releases/s4-unified-pick-20261116`이다. Python 실행 코드는 묶음 내부 파일을 사용하고 manifest 해시를 검사한다. 기존 배포본과의 모듈 혼용은 거부한다.

같은 Python/실행 파일로 먼저 `--check-environment`를 실행한다. 이 모드는 첫 상승 시점에 지연 로드되던 모듈까지 가져오고 설정 파일을 읽으며, 시리얼 포트를 열지 않는다. `ENVIRONMENT_OK`가 확인된 다음 `--execute` 실행으로 연결한다. 단일 셸에서 `set -e`를 사용하면 환경 검사 실패 시 구동 단계로 진행하지 않는다.

복구 도구의 공개 실행본은 특정 보드 고유번호를 내장하지 않는다. `--execute`에는 실제 보드의 `--stlink-serial`을 전달하고, 시리얼 통신 장치는 `--port`로 지정한다.

## 실행 조건

- 양쪽 집게가 비어 있고 실제 리프트가 최하단이다.
- 양팔과 리프트를 구동 준비 중 지지할 수 있으며, 지지물·팔·케이블의 전체 상승 공간이 확보되어 있다.
- 준비된 제어기는 STOP이 걸리지 않은 토크 OFF 상태다. 원점은 미등록 상태여야 한다.
- 새 조회의 16개 장치와 관절 범위, 기존 baseline, 설치 프로필이 일치해야 한다. 불일치는 자동 수정하지 않는다.

`--execute` 없이는 기존 조회만 실행한다. 실행에는 `--supported-empty-bottom`과 `--swept-clearance-confirmed`가 필요하며, SSH의 대화형 터미널(`ssh -t`)을 사용한다. stdin으로 코드를 전달하는 `python -` 방식은 구동에 사용하지 않는다.

## 동작

1. 실시간 조회와 관절 범위를 검사한다.
2. 양팔은 새로 읽은 현재 자세에서 유지한다. 바퀴 이동은 명령하지 않는다.
3. 실제 최하단 확인을 바탕으로 원점을 등록한다. 아래로 자동 원점 탐색하지 않는다.
4. 기존 1cm 상승·정지 검사를 수행한다.
5. `WAITING_UP_CHECK`에서 실제 상승 방향·정지·주변 공간을 확인한 사용자가 **새로 g를 누른 경우에만** 목표 높이로 상승한다. 미리 입력한 g는 무시한다.
6. `TARGET_HOLDING`에서는 같은 제어 연결과 기준점을 유지한다. 다음 촬영을 준비할 때도 창을 열어 둔다. 이 도구는 자동 파지나 다른 실행기에 제어를 넘기는 기능을 포함하지 않는다.

목표 단위는 **최하단 대비 mm**다. `--target-mm 550`은 총 55cm 상승이며, 바닥에서 상판까지의 절대 높이가 아니다. 첫 1cm에 추가로 55cm를 더하는 것도 아니다. 범위는 10~550mm이고 기존 MCU 높이·전류·속도·인터록 제한은 그대로 적용된다.

Space, x, q, Ctrl-C, 터미널 EOF, 연결 종료 또는 오류는 통합 STOP을 요청한다. 정지 확인은 ACK와 별도로 조회하며 토크 해제·리셋·자동 재시작은 하지 않는다. 목표에서 정상 유지 중인 상태와 전체 STOP은 다르다. STOP 뒤의 높이 기준 재사용을 보장하지 않으므로 올라간 상태에서 재실행하거나 다시 최하단으로 등록하면 안 된다.

로그는 Jetson `/home/hyper/robot/shared/diagnostics/unified-lift-<시각>/state.json`에 남는다. 최신 관절·리프트 상태, 원점/첫 상승/목표 상승 결과, 중단 단계, 정지 확인을 포함한다.

## 검증

```bash
PYTHONPATH=.:tools/run:ros2_ws/src/so101_arm_bridge:ros2_ws/src/home_robot_tasks \
python3 -m pytest -q tests/test_unified_lift_package.py tests/test_unified_lift_console.py \
  tests/test_supported_lift_rise.py tests/test_lift_wire.py \
  tests/test_unified_arm_adapter.py
```

범위 오류, 오래된/다른 부팅의 원점, 첫 1cm 검사 누락, ACK 손실, 하강 감지, 높이 초과, 관측 시간 초과, 인터록 해제, 사용자 중단과 정지 확인 실패를 검사한다. 모의 전체 세션에서 조기 g 무시, 새 g 이후 상승, 토크 활성화 ACK 손실 뒤에도 STOP 요청, 읽기 전용 모드의 무구동을 확인했다.

배포 회귀 검사는 실제 압축 파일을 별도 폴더에 풀고 `python -I` 독립 인터프리터로 실행한다. 기존 릴리스에 새 모듈이 없고 패키지를 읽으면 실패하도록 구성한 환경에서도 묶음만으로 환경 검사가 통과한다. 누락/변조 파일은 시리얼 접근 전 거부하며, 검사 중 시리얼 포트를 열려 하면 테스트가 실패하도록 했다.
