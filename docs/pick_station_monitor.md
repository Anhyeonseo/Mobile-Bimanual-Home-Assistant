# 상주 집기 실행기와 로컬 모니터

진입점은 `tools/run/pick_service.py`, 화면은 `http://127.0.0.1:18768`이다.
준비 후 요청을 기다리고, 요청마다 촬영 → 탐지 → 후보 → 계획/검증 → 실행 → 결과 확인을 호출한다.
웹은 요청·중단·조회만 처리하고 장치는 실행기 스레드 하나가 소유한다. 모델·MoveIt·파일 처리의
느린 작업은 별도 상주 프로세스로 보낸다. 사용자는 한 번 실행하며 내부 계산 작업자와 MoveIt도 유지된다.

## 실행 모드

| 모드 | 실제 계산/관측 | 장치 동작 |
| --- | --- | --- |
| `demo` | 모의 단계 전환 | 없음 |
| `watch` | 기존 `unified_pick.py`의 `state.json` 읽기 | 없음. 웹 제어 버튼 비활성 |
| `replay` | 저장 RGB-D + 같은 촬영의 관절 보고서 → 실제 탐지·후보·MoveIt·실행 전 검증 | 없음 |
| `live --plan-only` | 새 RGB-D/실측 관절 → 실제 탐지·후보·MoveIt·실행 전 검증 | 초기 준비·현재 자세 유지·지정 범위 리프트 준비는 실제 수행. 파지 경로 전송 없음 |
| `live` | 위 계산 + 검증 번들 실행 + 새 RGB-D 파지 확인 | 기존 통합 컨트롤러의 고정 위치 왼팔 파지 |

실제 하드웨어 분기까지 코드가 연결됐지만 Jetson 실기 검증은 아직 하지 않았다.
기존 `unified_pick.py`의 지지된 바닥 원점 준비와 **0~10mm 리프트 준비 범위**를 유지한다.
별도로 개발 중인 550mm 리프트 콘솔을 자동 편입하거나 다른 프로세스의 컨트롤러를 인수하지 않는다.
고정 위치 왼팔 집기 범위이며 베이스 이동·오른팔 조작·놓기·적재 상태 복귀는 포함하지 않는다.

## 지금 노트북에서 실행

```bash
cd /home/an-hyeonseo/Mobile-Bimanual-Home-Assistant-edit
python3 tools/run/pick_service.py --mode demo
```

브라우저에서 `http://127.0.0.1:18768`을 열고 집을 물체(예: `지우개를 집어줘`)를 입력하고 **집기 요청**을 누른다. 종료는 터미널 `Ctrl+C`다.
`--demo-image output/jetson-remote-test-20261117/cam/color.png`로 저장 참고 이미지를 표시할 수 있다.
참고 이미지를 새 촬영이라고 표시하지 않는다. `--demo-fail-stage plan`으로 실패 화면을 볼 수 있다.

현재 노트북의 기존 모델·Python·ROS 설치를 사용하는 환경 점검 명령:

```bash
bash tools/run/run_pick_station_local.sh --mode live --check
```

장치·모델·웹 서버를 시작하지 않고 패키지와 파일 존재만 확인한다.
`DEPENDENCIES_PRESENT`는 실기 구동 인증이 아니다. 설치 위치가 다르면 `GRASP_SOURCE_REPO`,
`PICK_PYTHON` 또는 직접 실행의 `--artifacts-dir`, `--mobile-mesh-dir`를 지정한다.
모델은 고정 revision의 로컬 snapshot에서 읽으며 자동 다운로드하지 않는다.

## 저장 자료로 실제 계산

아래 경로는 실제 촬영 쌍으로 바꾼다. **나중에 읽은 컨트롤러 로그를 원본 촬영 보고서 대신 쓰면 안 된다.**
현재 설치 보정과 호환되지 않는 과거 촬영도 거절된다.

```bash
bash tools/run/run_pick_station_local.sh --mode replay \
  --snapshot /실제/촬영폴더/rgbd-123 \
  --robot-report /실제/같은촬영의관절보고서.json \
  --camera-mode near
```

웹 요청으로 모델을 유지한 채 탐지·분할·SO101 후보·경로 계획·실행 전 검증을 수행하고
`PLANNED_ONLY`로 끝난다. `near`는 640×480 RGB/640×360 depth, `wide`는 1280×720,
`legacy`는 기존 프로필이다. SDK와 보정 일치도 별도로 검사한다.
GraspNet 비교는 `--candidate-source graspnet` 또는 `graspnet-first`를 사용한다.
기본 `so101` 생성기가 그 자체로 물리 성공이 입증된 모델이라는 뜻은 아니다.

## Jetson 실제 장치 실행

실기 연결 전 첫 점검은 노트북 터미널에서 다음 한 명령으로 수행한다.

```bash
bash tools/run/check_pick_hardware.sh
```

SSH로 Jetson의 설치 파일·Python 의존성·현재 제어 프로세스·장치 점유 상태를 조사한다.
장치가 비어 있을 때만 배타적 소유권을 잡아 컨트롤러의 캐시된 관절·리프트·STOP 상태를 조회하고
RealSense 장치를 열거한다. 촬영 스트림·토크·이동·원점 등록·STOP·모드 준비는 시작하지 않는다.
확인된 제어 프로그램 또는 해당 장치 사용자가 있으면 조회를 건너뛴다. 일부 PID의 상세 정보가
보이지 않아도 컨트롤러는 기존 백엔드 잠금과 배타적 시리얼 열기가 성공해야 조회한다.
새 촬영도 함께 점검하려면 `bash tools/run/check_pick_hardware.sh --capture`를 사용한다.
같은 실행의 `cam/`에 RGB·정렬 깊이·촬영 메타데이터가 저장된다. 이 촬영에는 관절 동기화 증명이 없다.
결과와 로컬 보정/집기 파일 해시 비교는 `output/pick-hardware-check/check-*/comparison.json`에 저장된다.
`CACHED_STATUS_READ`는 통신 확인이며 실시간 촬영/관절 쌍이나 집기 실행 허가가 아니다.
기본 SSH 주소는 `hyper@192.168.35.2`이며 `JETSON_HOST`로 변경할 수 있다.
프로토콜 설치 경로가 다르면 `--root /실제/설치경로`를 붙인다. 파일 변경이나 배포는 하지 않는다.

ROS Jazzy 환경을 읽고 카메라·PyTorch/Transformers·SciPy·MoveIt 의존성이 함께 보이는 Python을 사용한다.
작업자는 같은 Python으로 생성된다. 아래는 **실제 준비·유지 명령을 보내는 실행 형식**이다.
기존 절차의 지지된 빈 그리퍼/바닥 원점, 올바른 baseline, 원본 메시, 현재 보정이 전제다.
`/실제/...`는 설치 경로로 바꾼다.

```bash
python3 tools/run/pick_service.py --mode live \
  --artifacts-dir /실제/모델/artifacts \
  --mobile-mesh-dir /실제/원본/meshes \
  --baseline /실제/컨트롤러/baseline.json \
  --serial-port /dev/ttyACM0 \
  --camera-mode wide \
  --lift-target-um 0 \
  --execute --supported-empty-bottom --plan-only
```

`--plan-only`는 초기 준비와 유지까지 수행하지만 파지 경로는 전송하지 않는다.
파지 경로까지 실행하는 형식은 위 명령에서 `--plan-only`를 제외한 것이다.
준비가 끝난 뒤 **웹의 집기 요청을 누르면** 새 촬영부터 파지까지 진행한다.
외부 `execute.json` 작성은 필요 없고 baseline·보정·현재 자세·장면·번들 검사를 유지한다.
다른 리프트/팔 프로그램과 같은 컨트롤러를 동시에 열지 않는다.
종료·중단·실행 오류는 통합 STOP과 정지 증명 조회를 사용하고 토크 해제를 보내지 않는다.
STOP 뒤 자동 재무장은 없다.

Jetson 화면을 노트북에서 보려면 노트북에서 아래 터널을 열고 localhost 주소를 방문한다.
주소는 현재 Jetson 주소에 맞춘다.

```bash
ssh -N -L 18768:127.0.0.1:18768 hyper@192.168.35.2
```

기존 실행기 관찰은 `--mode watch --watch /실제/unified_pick출력폴더`를 사용한다.
이 모드의 모니터 종료는 로봇 정지 명령이 아니다. 상태 파일이 5초 이상 갱신되지 않으면 마지막 상태임을 표시한다.

## 연결한 처리

| 단계 | 구현 |
| --- | --- |
| 준비 | DINO/SAM을 한 번 로드. 선택 시 GraspNet도 한 번 로드. live는 MoveIt도 요청 대기 전에 준비 |
| 촬영 | 요청 이후 노출 + 정지 관절 창 + 통합 베이스/리프트 증명을 `unified_held_capture`로 저장 |
| 탐지·후보 | 기존 탐지기와 SO101/GraspNet 재사용. RGB·depth·metadata·mask 해시 유지 |
| 계획 | 첫 완전 경로에서 종료. MoveIt 유지, 매 작업 시작에 이전 월드·부착 물체·충돌 허용 행렬·관절 상태 초기화 |
| 실행 전 검증 | 정수 명령으로 반올림된 경로 native 검사, 독립 장애물 음성 대조, 실행 리허설, 번들 해시 검사 |
| 실행 | 요청/촬영/컨트롤러 세션 일치와 설치 보정·현재 앵커·리프트·장면 확인 후 기존 CarrySession 실행 |
| 확인 | 새 대상 점군이 측정 TCP를 따라왔는지 + 그리퍼 접촉 범위. 서로 다른 노출 3회의 일관성과 마지막 최신 프레임 확인 |

MCU 관절 스캔에 대해 50ms 동시 촬영 기록을 꾸미지 않는다. 기존 고정 카메라·일정 자세 유지
규칙을 사용하고 원래 노출/관절 측정 구간을 보존한다. 요청·세션·보정·파일 해시가 다르면 실행하지 않는다.

RGB-D 초기 판정값은 양방향 점군 일치 65% 이상, 반대 가설 일치 15% 이하,
거리 허용치 8mm, 가설 분리 이동 20mm 이상이다. 설치 정확도 검증값은 아니므로 실험으로 평가해야 한다.
미검출·가림·깊이 부족·복수 대상·근거 충돌은 `UNVERIFIED`다.
파지 후 정지 자세에서의 별도 확인이며 연속 추적이나 힘 센서 측정을 뜻하지 않는다.
추가할 좌우 팔 RGB 카메라의 미측정 장착 보정값은 생성하지 않는다.

## 다음 요청과 실패

- 탐지 실패·후보 없음·경로 없음: 움직이지 않았고 유지 상태가 정상일 때 다음 요청을 받는다. 새 촬영부터 수행한다.
- replay/plan-only 완료: 모델·플래너를 유지하고 다음 요청을 받는다.
- 실제 파지 후: 결과와 함께 자세 유지. 물체 보유 또는 보유 여부 미확인이면 새 집기 차단.
- 실행 오류·중단·시간 초과: STOP을 확인하고 새 요청 차단. 정지 확인 실패를 성공으로 바꾸지 않는다.
- 물체 놓기·빈손 복귀·물리적 실패 뒤 자동 재시도는 아직 연결하지 않았다. 기존 경로 역재생이나 자동 rearm으로 대체하지 않는다.

최상단 감독 시험은 동작 완료 후 **다음 시험 준비**를 누른다. 물체 제거와 받침 확인을 받으면 현재 팔 자세·리프트 높이·모터 힘을 유지하면서 이전 요청의 사진·후보·경로·결과와 실행 세션을 비우고 새 명령을 받는다. 다음 요청은 새 사진 한 장부터 시작한다. 실제 정지·고장은 별도 초기화 대상이며, 자동 놓기나 빈손 복귀 동작은 포함하지 않는다.

## 코드와 검증

- `pick_backend.py`: 실제 단계 연결과 다음 요청 조건
- `pick_hardware.py`: 단일 장치 소유, 촬영·실행·유지·STOP
- `pick_capture.py`: 통합 컨트롤러 촬영 증거
- `pick_worker.py`, `tools/lib/pick_compute.py`: 상주 계산 작업자와 모델·계획·검증
- `pick_rgbd_verification.py`: 깊이 기반 파지 후 확인
- `pick_service.py`, `pick_web.py`: 상태 루프와 로컬 화면

관련 회귀 테스트 367개 통과, 3개 환경 의존 검사 생략. 실제 저장 리모컨 RGB-D로 CPU에서
동일 DINO/SAM 객체를 유지하며 두 번 처리해 각각 SO101 후보 24개를 생성했다.
준비 1.86초, 탐지·분할·후보 생성 3.46초/2.84초. MoveIt·로봇 이동·새 촬영 시간은 포함하지 않는다.
실측: `output/pick-station-model-check-local-snapshots/report.json`.
재현 도구: `tools/diagnostics/check_pick_models.py --help`.

이 작업 환경은 소켓 생성이 제한되어 실제 localhost 접속과 native ROS/MoveIt 서비스 연결을 검증하지 못했다.
Jetson 카메라/모터는 실행하지 않았으며 실제 성공률·전체 지연 감소율은 미확인이다.

```bash
python3 -m pytest -q -c config/pytest.ini --rootdir=. \
  tests/test_pick_service.py tests/test_pick_monitor.py \
  tests/test_pick_pipeline.py tests/test_pick_worker.py
```

최상단의 들어 올리기는 관측한 상판의 수직 방향을 우선하고 기존 베이스 수직 방향을 대안으로 검사한다. 기존 2 mm 측방 이동 범위, 전체 충돌 검사, 반올림한 실행 명령 검사는 유지한다. 특정 촬영의 후보 번호나 좌표를 고정하지 않는다.

집기 수정 후에는 최신 실패 장면과 기존 통과 장면을 함께 재검증한다. 아래 도구는 저장된 후보부터 경로·실행 명령·고장 시뮬레이션까지 확인하며 카메라·모터를 열지 않는다. 물리적 집기 성공 여부는 별도 현장 기록으로 남긴다.

```bash
python3 tools/diagnostics/check_pick_regressions.py \
  --prepared /path/to/previous-pass/prepared.json \
  --prepared /path/to/latest-failure/prepared.json \
  --mobile-mesh-dir /path/to/mobile_meshes \
  --output /path/to/regression-results
```
