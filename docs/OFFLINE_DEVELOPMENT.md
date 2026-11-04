# 실물 없이 개발하기

원룸의 등록된 가구 사이에서 물체를 배달하는 기능을 개발한다. Qwen은 요청/물체를 해석하고,
작업 순서·실행·실패 처리는 FSM이 맡는다. 첫 범위는 수동 가구 등록·정차 위치와 RGB-D 정렬이다.
자동 시맨틱 매핑·동적 정차 최적화는 후속 확장이다.

## 실제 가구 등록 DB — 좌표는 측정 전까지 공란

실제 장소 목록은 `config/home_places.json`, 로컬 SQLite DB는 `data/home.sqlite3`다.
큰 책상(`desk`), 탁상(`coffee_table`), 미니 테이블(`mini_table`), 소파(`sofa`),
싱크대(`sink`), 쓰레기통(`trash_bin`)을 등록했다.
지도/리비전·방 ID·가구 위치·로봇 정차 위치·작업면 높이는 모두 `NULL`로 시작한다.
좌표 0은 미등록 표시로 사용하지 않는다.

```bash
.venv-offline/bin/python tools/run/places_db.py init
.venv-offline/bin/python tools/run/places_db.py list
.venv-offline/bin/python tools/run/places_db.py show '큰 책상'
```

`책상`·`큰책상`은 큰 책상, `미니테이블`은 미니 테이블로 조회된다.
조회 결과의 `furniture_pose`는 가구 자체 위치, `stop_pose`는 로봇이 설 위치로 별도 관리한다.
좌표가 없으면 `stop_status=UNREGISTERED`이며 `PlaceRegistry.require_stop()`은 정차 위치를 반환하지 않는다.
이름이 모호하면 임의의 장소를 선택하지 않는다.

SLAM 지도와 정차 위치를 측정한 뒤에는 `set-stop --help`에서 입력 항목을 확인한다.
정차 위치를 등록할 때 지도 ID·리비전·좌표 프레임과 x/y(m)·yaw(rad)를 함께 입력해야 한다.
다른 지도 리비전에서 기존 좌표를 사용하려 하면 거절한다.
등록 상태 `REGISTERED`는 수치가 입력됐다는 뜻이며 실제 주행/팔 도달성 검증 완료를 뜻하지 않는다.

`init`을 다시 실행해도 DB에 입력한 정차 좌표를 지우지 않는다.
실측 DB는 Git에서 제외하고, 좌표가 빈 초기 목록과 생성 도구를 버전 관리한다.
기존 `studio.simulation.json`의 모의 좌표를 실제 DB로 복사하지 않는다.
이 DB는 장소 저장/조회 단계이며 실제 Nav2/FSM 자동 실행 연결은 별도다.

## 개발 환경

저장소 루트에서 실행한다. CPU 개발 환경과 GPU 모델 환경을 분리한다.

```bash
python3 -m venv .venv-offline
.venv-offline/bin/python -m pip install -r requirements/offline.lock.txt
.venv-offline/bin/python tools/run/dev_lab.py doctor
```

GPU 환경은 [Qwen 안내](QWEN3_VL.md)의 `.venv-qwen3-vl`을 사용한다.
샌드박스에서는 GPU와 로컬 소켓이 가려질 수 있다. 아래 GPU/API 명령은 호스트 터미널에서 실행한다.

```bash
.venv-qwen3-vl/bin/python tools/run/dev_lab.py doctor --gpu
```

`doctor`는 실행 환경을 검사한다. GPU 환경에 pytest/serial이 없는 것은 CPU 검증 환경의 누락과 구분한다.
실제 모델/캐시의 완전성은 GPU 파이프라인 실행으로 확인한다.

## 1. 원룸 FSM을 한 번에 검증

```bash
.venv-offline/bin/python tools/run/dev_lab.py suite --output output/my-first-lab
```

20개 사례를 기존 조립형 실행기 `OfflineStack`으로 실행한다.

- 책상 → 소파 리모컨 배달, 집기/놓기의 내부 11단계.
- 대상 미발견의 유한 종료.
- 운반 시 센서 상실, MCU 재부팅, 물체 놓침.
- 전체 15개 작업 단계 각각의 취소.

장애 사례의 PASS는 **예상한 실패/정지 상태가 관측됐다는 뜻**이다. 배달 성공률이 아니다.
`STOP_UNCONFIRMED`에서는 제어권이 유지되어야 통과한다.
시간은 가속한 모의 시계이며 실제 로봇/모델 지연이 아니다.

하나의 사례만 실행:

```bash
.venv-offline/bin/python tools/run/dev_lab.py run \
  --scenario sensor_loss --output output/my-sensor-loss
```

`success`, `target_missing`, `sensor_loss`, `reboot`, `load_loss`, `cancel_00`–`cancel_14`를 지원한다.
출력 디렉터리는 새 경로여야 한다. 기존 기록을 덮어쓰지 않는다.

| 산출물 | 내용 |
|---|---|
| `report.json` | 사례별 기대 상태와 통과 여부 |
| `manifest.json` | 코드/설정 해시, Git revision, dirty 상태, Python/패키지 버전 |
| `profile.resolved.json` | 상속한 모의 설정을 포함한 실행 입력 전체 |
| `<scenario>/events.jsonl` | 작업 단계·상태·제어권·장애 변경 기록 |
| `<scenario>/result.json` | 최종 상태와 조작 내부 단계 결과 |

설정은 [`config/studio.simulation.json`](../config/studio.simulation.json)이다.
4.5×4.5m 방과 책상·탁상·소파의 이름/정차 위치를 정의한다.
크기·좌표·리프트 높이는 **미실측 fixture**이며 지도에는 현재 외벽만 있다.
가구의 실제 충돌 형상이나 SO-ARM101 도달성을 검증한 결과가 아니다.
센서/시간/하중 정책은 `fetch_stack.simulation.json`을 상속한다.

요청의 선택 필드 `source_place`로 출발 가구를 한정한다.
예: `source_room=living_room, source_place=desk, destination_place=sofa`.
생략하면 기존처럼 방의 유한한 `search_locations` 전체를 탐색한다.
등록되지 않은 출발 가구를 임의의 방 탐색으로 바꾸지 않는다.

## 2. 스마트폰/클라이언트용 API 개발

```bash
.venv-offline/bin/python tools/run/dev_lab.py serve --state output/dev-lab-api
```

기존 인증 Gateway를 `http://127.0.0.1:8765`에서 실행한다. 토큰은
`output/dev-lab-api/token`에 권한 600으로 저장하며 화면/로그에 값을 출력하지 않는다.
요청 예시는 같은 디렉터리의 `request.json`에 생성한다. Ctrl+C로 종료한다.

| API | 용도 |
|---|---|
| `GET /v1/health` | 실행 모드·활성 작업 |
| `GET /v1/map` | 지도/리비전 |
| `GET /v1/permit` | 새 요청을 위한 3초 유효 permit |
| `POST /v1/requests` | JSON 작업 접수; `X-Request-Permit` 필요 |
| `GET /v1/requests/<id>` | 상태 조회 |
| `POST /v1/requests/<id>/cancel` | 본문 `{}`로 취소 요청 |

API 요청에는 `Authorization: Bearer <토큰 파일의 값>`을 사용한다.
동일 ID·본문 재전송은 중복 실행하지 않고, 다른 본문은 충돌로 처리한다.
SQLite에 저장하며 프로세스 재시작 시 미완료 작업은 `INTERRUPTED`로 남긴다. 자동 재개하지 않는다.
같은 state 디렉터리에서 world/map을 변경하면 차단한다. 새 설정은 새 state 디렉터리를 사용한다.

이 API 서버는 **KinematicRobot 기반 클라이언트 개발용**이다.
위 suite는 관측·하중·조작 포트를 연결한 OfflineStack을 사용한다.
GPU 인식 결과나 실제 ROS/MCU에 API가 연결된 상태는 아직 아니다.

## 3. 팀원과 주고받을 RGB-D 기록

카메라 없이 예제 기록을 생성하고 검사할 수 있다. NumPy만 필요하다.

```bash
.venv-offline/bin/python tools/run/rgbd_recording.py demo --output output/recordings/toy
.venv-offline/bin/python tools/run/rgbd_recording.py validate output/recordings/toy/manifest.json
```

실제 장면을 보내기 전 아래 형식으로 저장한다.
현재 도구는 파일 검사/변환을 제공하며 D415의 라이브 수집기는 아직 연결하지 않는다.

| 항목 | 계약 |
|---|---|
| `schema_version` | 정수 `1` |
| `recording_id`, `clock_domain` | 기록 ID와 시계 구분; 각 프레임 시각은 같은 시계 사용 |
| `evidence_source` | `synthetic`, `public_dataset`, `measured`를 구분 |
| `camera` | 해상도, rectified 3×3 intrinsics, depth scale(m/raw), frame/calibration ID |
| RGB | `uint8` H×W×3 `.npy`, RGB 채널 순서 |
| Depth | RGB에 정렬된 원본 `uint16` H×W `.npy`; 0은 무효 |
| workspace | 선택적인 이진 H×W `.npy`; 개별 물체 마스크와 구분 |
| 프레임 | 고유 `capture_id`, 증가하는 `observed_s`, 파일별 상대 경로·SHA256 |
| `camera_to_base` | 선택적인 4×4 강체 변환; 카메라 optical → base_link, 단위 m |

변환을 모르면 `null`로 둔다. identity를 실제 보정값으로 만들어 넣지 않는다.
카메라 장착이나 리프트/로봇 자세가 바뀌면 해당 촬영 시점의 변환을 사용한다.
검사 도구는 변환 행렬의 수학적 유효성을 검사하며 실측 정확도를 인증하지 않는다.
해시 불일치·단위/형식 오류·잘못된 회전·중복/역순 시각·기록 밖 경로는 거절한다.

현재 GraspNet 공식 샘플을 표준 기록으로 가져오기:

```bash
.venv-qwen3-vl/bin/python tools/run/rgbd_recording.py import-graspnet \
  --output output/recordings/graspnet
```

이미 내려받은 `artifacts/datasets/graspnet-example`을 사용하며 다운로드하지 않는다.
`export-graspnet`은 검증한 기록의 한 프레임을 기존 추론 스크립트가 받는 PNG/MAT 형식으로 변환한다.
이 변환은 Pillow/SciPy가 있는 GPU 환경을 사용하며, 명시적인 workspace 마스크가 필요하다.

## 4. 호스트 GPU로 전체 인식 파이프라인 실행

```bash
.venv-qwen3-vl/bin/python tools/run/run_perception_pipeline.py \
  --recording output/recordings/graspnet/manifest.json \
  --prompt '바나나를 찾아줘.' --output output/my-perception-run
```

기존 PNG/MAT 샘플은 `--recording` 대신 `--sample <폴더>`로 입력한다.
둘 다 생략하면 로컬 GraspNet 공식 샘플을 사용한다.
`--plan-only`는 GPU/모델 로딩이나 파일 생성 없이 명령을 보여준다.

실행 순서:

1. 입력 검증 및 원본/모델 해시 기록.
2. Qwen으로 bbox 생성, 원문·검출 이미지 저장.
3. 선택한 bbox를 SAM에 전달해 물체 마스크·점군 생성.
4. 작업 영역 점군으로 GraspNet 추론, 대상 연관·장면 충돌 필터·NMS.

모델을 별도 프로세스에서 순차 실행해 단계 종료 시 GPU 메모리를 반환한다.
Hugging Face 오프라인 모드를 강제한다. Qwen 가중치, SAM 캐시, GraspNet 소스/체크포인트가
없으면 다운로드 없이 실패하며 해당 단계 로그를 남긴다. 모델 설치는 [Qwen 안내](QWEN3_VL.md)를 따른다.

여러 검출 중에는 명시적인 `--index`를 사용한다(기본 0).
기록에서 선택할 프레임은 `--frame-index`다(기본 0).
현재 자연어 요청을 FSM 작업 요청으로 바꾸는 기능과는 별개인 **물체 검출용 prompt**다.

| 최종 상태 | 의미 |
|---|---|
| `CANDIDATES_READY` | 필터를 통과한 GraspNet 후보 있음 |
| `NO_DETECTIONS` | 대상 검출 없음; SAM/GraspNet 실행 안 함 |
| `MASK_REJECTED` | 선택한 대상의 마스크/깊이 품질 미달 |
| `NO_GRASP_CANDIDATES` | 최종 파지 후보 없음 |
| `FAILED` | 입력/의존성/추론 오류 또는 단계 timeout |
| `CANCELLED` | 사용자가 실행을 중단함; 종료 코드 130 |

`FAILED`는 종료 코드 1, `CANCELLED`는 130, 나머지는 기록 가능한 처리 결과로 0을 반환한다.
호출자는 배달 가능 여부를 판단할 때 **종료 코드와 report.status를 함께 확인**해야 한다.
`report.json`에 코드·입력/모델 해시·패키지 버전·단계 시간과 후보 개수를 남긴다.
`qwen/`, `objects/`, `grasps/`에는 기존 상세 결과, `<stage>.log`에는 추론 로그가 저장된다.

2026-10-31 RTX 5070 Ti에서 기록 가져오기 → 전체 GPU 추론을 확인했다.
바나나 후보: 전체 317 → 대상 연관 11 → 충돌/빈 파지 필터 2 → 최종 1.
이는 공개 샘플에 대한 결과이며 실제 리모컨·로봇 파지 성공률이 아니다.

## 검증 및 다음 연결 지점

### 바나나와 SO-ARM101을 같은 3D 화면에서 보기

```bash
.venv-qwen3-vl/bin/python tools/run/preview_grasp_scene.py
```

앞서 생성한 `output/perception-pipeline-gpu/objects`와 `grasps/grasps.json`을 기본 입력으로 사용한다.
다른 결과는 `--objects <폴더> --grasps <grasps.json>`으로 지정한다.
추론이나 장치 연결 없이 저장한 점군과 레포의 SO-ARM101 STL 메시를 표시한다.

- 노란색: 실제 레포 형상의 팔. 초록색: 바나나 점군.
- 파지 후보: 검사 전 주황색, IK 통과 초록색, IK 거절 빨간색.
- 회색: 작업 영역 점군. 좌표축: X 빨강 / Y 초록 / Z 파랑.
- 마우스로 회전/이동/확대하고, 오른쪽 슬라이더로 5개 팔 관절·시각화용 턱 각도와 가상 카메라 위치/회전을 조절한다.
- `Fit view`로 팔/대상을 중심에 맞추고 `Reset layout`으로 초기 배치를 복구한다.
- `Save layout + geometry`로 현재 배치를 `output/grasp-preview/layout.json`과 PLY 파일에 저장한다.
  원본 데이터와 기본 설정 파일은 변경하지 않는다.

초기 가상 배치는 `config/grasp_preview.virtual.json`이다. 길이는 m, 각도는 degree다.
변환 방향은 카메라 optical → 독립 팔의 `arm_mount_frame`이다.
이 좌표는 실제 로봇 보정값이 아니며, 수동 TCP–대상 거리는 IK 가능 여부를 뜻하지 않는다.
주황색 후보는 SO-ARM101의 실제 TCP/턱 형상에 맞춘 결과가 아니다.
팔 5축은 URDF/등록 관절 범위의 교집합을 쓰며, 턱 슬라이더는 URDF 시각화 각도다.

화면 없이 기하만 내보내거나 PNG 미리보기를 만들 수 있다:

```bash
.venv-qwen3-vl/bin/python tools/run/preview_grasp_scene.py --export-only
.venv-qwen3-vl/bin/python tools/run/preview_grasp_scene.py --render
```

`--render`는 호스트 그래픽 접근이 필요하며 `output/grasp-preview/preview.png`를 저장하고 종료한다.
다시 실행하면 출력 폴더의 저장된 `layout.json`을 우선 사용한다. 특정 배치는 `--layout <경로>`로 선택한다.
레포의 초기 배치로 열려면 `--layout config/grasp_preview.virtual.json`을 사용한다.
Open3D와 `xacro`는 `requirements/qwen3-vl.txt`에 포함한다.

### 파지 후보의 5축 위치·방향 IK 검사

```bash
.venv-qwen3-vl/bin/python tools/run/preview_grasp_scene.py \
  --show-ik --output output/grasp-ik-preview
```

저장된 GraspNet 후보를 팔 좌표로 변환하고, **5개 관절만** 최적화한다.
위치의 3개 성분과 회전의 3개 성분을 모두 잔차로 사용하며, 회전축 하나를 버리지 않는다.
정해진 횟수의 초기 자세에서 bounded least-squares를 실행하고, 얻은 관절값을 FK로 다시 검사한다.
기본 허용치는 위치 5mm·전체 회전 오차 10°이며 실측으로 정한 운용 허용치는 아니다.
위치와 방향이 각각 허용치 안에 있을 때만 `IK_FEASIBLE`로 기록한다.

화면의 `Check candidate IK`로 재검사하고 `Show selected IK pose`로 계산한 자세를 표시한다.
검사는 백그라운드에서 실행한다. 카메라 위치/회전이 바뀌면 결과를 무효화하고 재검사를 요구한다.
`--check-ik`는 검사만, `--show-ik`는 검사 후 통과한 관절 자세를 화면에 적용한다.
`--render` 또는 `--export-only`와 함께 사용할 수도 있다.
화면상의 자세 변경은 궤적 실행이나 물리 시뮬레이션이 아니다.

`config/grasp_ik.virtual.json`에 다음 가정을 명시한다.

- GraspNet +X 접근축 → TCP +Z, GraspNet +Y 닫힘축 → TCP -X.
- GraspNet 원점에서 TCP 원점으로의 오프셋은 가상 기본값 0m.
- 범용 평행 그리퍼의 접근축 주위 180° 대칭을 허용한다. 실제 SO-ARM101의 비대칭 턱/충돌 형상에 대한 검증은 별도다.

TCP 원점/축과 실제 핑거 접촉점의 관계는 실측 보정이 필요하다.
그리퍼 폭을 관절 각도로 변환하거나 충돌·접근/후퇴 경로를 검사하지 않는다.
따라서 **IK 통과는 실제로 집을 수 있다는 판정이 아니다.**

| 상태 | 의미 |
|---|---|
| `IK_FEASIBLE` | 한 관절 해가 위치·방향 허용치를 모두 통과 |
| `ORIENTATION_NOT_MATCHED` | 위치만 맞출 수 있는 해는 찾았지만 위치·방향을 함께 만족하는 해를 못 찾음 |
| `NO_POSITION_SOLUTION_FOUND` | 제한된 수치 탐색에서 위치 해를 못 찾음; 전역적인 불가능 증명은 아님 |
| `INVALID_CANDIDATE` | 비정상 좌표/회전/폭/깊이/점수 입력 |

`ik_report.json`에는 후보별 오차·관절 해·거절 이유·180° 대칭 사용 여부·탐색 예산,
로봇 URDF 해시·관절 범위·가상 변환과 허용치를 저장한다.
통과한 후보 중 정규화된 자세 오차, 현재 자세와의 관절 거리, GraspNet 점수 순서로 선택한다.
실패한 최적화의 근사 해나 위치 전용 진단 해를 적용하지 않는다.

2026-10-31 기본 가상 배치의 바나나 후보 1개에서 위치 오차 약 0.047mm·방향 오차 약 5.84°를 확인했다.
이 값은 모델 안에서 계산한 잔차이며 실제 로봇 정확도를 뜻하지 않는다.

### 회귀 검사

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-offline/bin/python -m pytest -c config/pytest.ini --rootdir=. -q
```

CI에서도 원룸 suite를 실행하고 산출물을 보존한다. 모델 다운로드/GPU는 CPU CI에 요구하지 않는다.
기존 C/STM32/ROS/MoveIt 확인 도구는 `tools/run/verify_offline.py`에 유지한다.

| 연결 지점 | 현재 상태 | 다음 작업 |
|---|---|---|
| 원룸 가구·정차·FSM | 모의 입력으로 실행 | 실측 지도/가구 설정과 교체 |
| 사용자 요청 API/DB | 기존 모의 API 재사용 | 조립형 runtime과 연결, 클라이언트 상태 동기화 |
| RGB-D 기록 | 저장 형식·검사·샘플 변환 | 실제 D415 기록 수집, 관절/TF 동시 기록 |
| Qwen/SAM/GraspNet | 로컬 GPU 전체 추론 | 실제 방 데이터 평가, 모델 서비스 포트 연결 |
| 6DoF → 5DoF | 가상 TCP 축 매핑·관절 제한·전체 자세 IK·후보 선택/시각화 | 실측 TCP·그리퍼 폭 보정, 환경/자기 충돌·접근/후퇴 경로 검사 |
| RGB-D 미세 정렬 | 기존 모의 alignment 기반 | 작업면·도달 범위 기반 오차 추정과 재관측 |
| Jetson 배치 | 미검증 | 메모리·동시 부하·지연 측정 후 실행 위치 결정 |

실제 인식 결과가 모의 FSM의 성공 증거로 자동 사용되지는 않는다.
실측 TF·그리퍼/TCP·팔 도달성 없이 파지 후보를 실행 명령으로 승격하지 않는다.
