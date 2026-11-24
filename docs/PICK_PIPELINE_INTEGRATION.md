# 기존 파지 코드의 웹 연결과 베이스 재배치 확장

현재 우선순위는 고정된 베이스에서 기존 탐지·후보·계획·실행 코드를 한 요청으로 연결하는 것이다. 실제 베이스 좌우 자동 이동은 아직 연결하지 않는다.

## 현재 호출 경로

| 단계 | 기존 모듈과 웹 연결 |
|---|---|
| 웹 요청 및 장치 서비스 유지 | `PickStation` → `PipelineBackend` / `WebPickBackend` |
| RGB-D·현재 관절 관측 | `UnifiedPickHardware.capture`, `pick_capture` |
| 탐지·분할·GraspNet | 별도 `PickCompute` 프로세스의 `detect`, `candidates` |
| 후보 변환 및 경로 검사 | `carry_binding.bind_grasp` → `plan_capture_carry.plan` → MoveIt |
| 실행 전 검증 | `pick_plan_validation.validate_pick_plan`: 기존 `load_inputs`, `validate_native`, `carry_stream_sim` |
| 실행 자료 묶기·최신 상태 확인 | `bundle_carry_execution`, `carry_bundle`, `prepare_execution_inputs` |
| 실제 파지 실행 | `UnifiedPickHardware.execute` → 기존 `build_carry_session` → `UnifiedArmAdapter` |

웹과 `check_pick_regressions.py`는 동일한 `station_plan_options`와 `validate_pick_plan`을 사용한다. 저장 촬영 재검증은 카메라·시리얼·모터에 연결하지 않는다. 검사 성공이 물리적인 파지 성공을 뜻하지 않는다. 현재 `top-pick`의 결과는 사용자가 확인하는 `UNVERIFIED`다.

## 경로 실패의 처리

`pick_recovery.planning_recovery`가 보고서의 실제 거절 사유를 분류한다. 분류 결과는 계획 보고서의 `recovery`, 요청 결과의 `recovery`, 웹 화면에 보존된다.

| 확인된 상태 | 다음 판단 |
|---|---|
| 전체 경로 존재 | 기존 실행 전 검증 후 기존 파지 실행기 사용 |
| 아직 검사하지 않은 후보 존재 | 현재 위치에서 후보 탐색 계속 필요 |
| 관측·보정·좌표계 오류, 계획기 오류, 알 수 없는 오류 | 진단 필요; 베이스 재배치 대상으로 취급하지 않음 |
| 시작 상태 충돌 또는 시작 상태 검증 없음 | 시작 자세·충돌 모델 확인 |
| 물체 영역·그리퍼 폭·닫힘 방향만 거절 | 파지 형상 검토; 이동으로 해결된다고 가정하지 않음 |
| 알려진 IK/접근 경로 실패, 모든 연결 후보에 검사 기록 존재 | `REPOSITION_REVIEW_REQUIRED`: 베이스 재배치 가능성 검토 |

경로 실패는 제한된 후보·시간 내에서 해를 못 찾았다는 뜻이다. 물리적으로 가능한 모든 해가 없다는 증명은 아니다. 재배치 검토는 이동 명령이나 이동 가능 판정이 아니다. 정지 확인 실패는 재배치 표시보다 우선하여 `STOP_UNCONFIRMED`로 남는다.

## 다음 확장: 좌우 재배치 후 같은 파지 흐름 재사용

1. 위의 재배치 검토 결과를 입력받아 실제 이동 가능 영역과 팔·리프트의 이동 자세를 확인한다. 통신/STOP/보정 문제에는 진입하지 않는다.
2. 현재 관측의 좌표와 실제 베이스 이동 모델로 좌우 배치 후보를 만들고, 팔 도달 가능성 및 이동 경로를 평가한다. 이동 거리·속도는 실측 프로필을 사용하며 임의의 고정 펄스로 결정하지 않는다.
3. 검증된 베이스 실행기를 동일 장치 소유자 아래 연결한다. 시리얼 포트를 별도 프로세스에서 중복으로 열지 않는다.
4. 이동 완료와 정지를 관측으로 확인한다. 이전 촬영·후보·경로는 실행에 재사용하지 않는다.
5. 새 RGB-D 및 로봇 자세로 `capture → detect → candidates → plan → execute → verify`를 다시 호출한다. 대상의 재식별이 모호하면 중단한다.
6. 시도 횟수와 총 이동량을 제한한다. 파지 실행이 이미 시작됐거나 물체를 들었을 가능성이 있으면 이 빈손 재배치 흐름을 사용하지 않는다.

현재 API는 `automatic_base_motion=false`, `motion_authorized=false`, `requires_new_capture_after_move=true`, `reuse_previous_plan_after_move=false`를 기록한다. 좌우 이동 실행기를 연결하기 전까지 웹에 자동 이동 버튼을 노출하지 않는다.

## 실행 및 확인

노트북에서 `bash tools/run/open_pick_station.sh`로 최신 패키지를 배포하고 웹을 연다. 현재 실제 리프트 위치에 맞는 작업과 자연어 대상을 선택한다. 실행 요청은 실제 팔 동작을 포함한다.

오프라인 재검증 예:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p output/pick-replay-ros
export ROS_LOG_DIR="$PWD/output/pick-replay-ros"
python3 tools/diagnostics/check_pick_regressions.py \
  --prepared output/pick-no-route-20261123/prepared.json \
  --output output/pick-pipeline-review \
  --mobile-mesh-dir output/pick-kit/current/assets/mobile_meshes
```

출력 경로는 검사마다 새 이름을 사용한다. 보고서의 `hardware_commands: 0`과 계획·명령 변환 검증 결과를 확인한다.
