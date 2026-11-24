# 좌우 팔 RGB 카메라로 파지 확인하기

## 이번 구현 범위

ArUco 마커를 사용하지 않는다. 기존 GroundingSAMDetector와 같은 검출/분할
출력을 받아 **원래 자리에 남은 물체**와 **그리퍼를 따라온 물체**라는 두 가설을
비교한다. `RGBGraspVerifier.observe_rgb()`는 실제 RGB 이미지에서 주입된 검출기를
호출한다. 새 모델이나 학습 가중치를 다운로드하지 않는다.

구현한 것은 판정기, 기존 왼팔 snapshot 실행기에 결과를 전달하는 연결부,
재시도 허용 검사, 저장된 관측의 재생 도구이다. 실제 카메라 드라이버/토픽,
실측 장착 보정, 가림 마스크 생산기, 복귀 경로 실행기는 자동 생성하지 않았다.
기존 `tools/run/stationary_pick.py` CLI에 카메라가 자동 연결되거나 하드웨어
자동 재시도가 활성화되는 변경은 **없다**. 이 실행기는 현재 왼팔 전용이다.
판정기 자체는 좌/우를 각각 생성해서 사용할 수 있다.

## 왜 '사라졌다'만으로 성공 처리하지 않는가

물체가 가려졌거나 밀렸거나 떨어져도 검출되지 않을 수 있다. 검출 결과가
없는 프레임은 항상 UNKNOWN이다. RGB 한 장으로 새로운 깊이나 지지면과의
간격을 측정했다고 간주하지 않는다. 결과는 시각적 파지 추정이며 실제 부하
계측을 대체하지 않는다.

| 출력 | 필요한 증거 |
|---|---|
| SUCCESS | 대상 마스크가 그리퍼를 따라온 예상 위치와 일치, 원래 위치와 불일치, 실제 그리퍼 측정이 접촉 범위, 시간에 걸쳐 유지 |
| FAILURE | 대상 마스크가 원래 위치와 일치, 따라온 위치와 불일치, 실제 그리퍼 측정이 빈손 닫힘 범위, 시간에 걸쳐 유지 |
| UNKNOWN | 대상 미검출/복수 검출, 가림, 두 예상 위치가 겹침, 시야 이탈, 센서 불일치, 시간 초과, 관측 부족 |

성공 확인 뒤 관측을 잃으면 같은 시도에서 단순 빈손 실패로 바꾸지 않는다.
낙하 가능성이 있으므로 검토 대상이다. 연결된 snapshot 입력도 성공 이후
관측 유실/만료를 기존 정지 경로로 전달한다. 성공 전 UNKNOWN은 기존
snapshot 모드의 미확인 상태를 유지하며, 새로운 이동을 허용하는 증거가 아니다.

## 좌표와 설치

각 팔에 **별도의** camera_frame, parent_frame, 내/외부 보정값을 사용한다.

```
base_from_optical(t) = base_from_parent(t) @ parent_from_optical
stayed_points_base = pregrasp_target_points_base
held_points_base(t) = base_from_tcp(t) @ inverse(reference_base_from_tcp) @ stayed_points_base
```

두 점군을 현재 optical frame으로 바꾼 뒤 현재 RGB 내참수로 투영한다.
고정 픽셀 ROI나 단순 영상 차분을 사용하지 않는다. 카메라 움직임 보정은
영상에서 추측하는 대신 노출 시점의 측정 FK/TF로 수행한다.

- 카메라가 손목 회전부에 붙었으면 실제 해당 링크를 parent로 설정한다.
- 카메라가 팔 고정 베이스에 붙었으면 실제 베이스 링크를 parent로 설정한다.
  이 경우 손목이 돌아도 카메라 좌표계는 돌아가지 않는다.
- 외부 보정은 실측해야 한다. 코드의 단위행렬 테스트는 합성 테스트 좌표계이며
  하드웨어 보정값으로 사용할 수 없다.
- 움직이는 리프트도 같은 stationary base frame까지의 측정 변환에 포함해야 한다.
  기존 stationary 실행은 별도로 리프트 정지 조건을 계속 검사한다.
- RGB는 보정된 rectified 영상이어야 한다. 렌즈 왜곡이 남은 영상에 이
  pinhole 투영식을 그대로 적용하지 않는다.
- reference_base_from_tcp는 **물체가 지지면에 있는 파지 기준 자세**여야 한다.
  임의의 홈 자세와 물체 위치를 묶으면 잘못된 따라오기 가설이 된다.
- target_points_base는 같은 capture의 대상 RGB-D 점군이다. 물체/지지면이
  움직였으면 기존 reference를 재사용하지 않고 새 capture로 새 시도를 만든다.

## 입력과 기본값

`observe_rgb(rgb, detector=..., visibility=..., **measurements)`를 카메라 작업
스레드에서 호출한다. inference가 모터 heartbeat를 막지 않도록 한다.

필수 입력:

1. 실제 카메라 노출 sequence/time, 같은 시점의 측정 FK와 그리퍼 측정 시간.
2. 현재 RGB 영상의 대상 인스턴스 분할 마스크. 원본 촬영의 마스크를 재사용하지 않는다.
3. 현재 영상과 정렬된 로봇/전경 장애물/무효 픽셀 가림 마스크.
   가림 관측이 없을 때 모두 0인 마스크를 임의로 넣지 않는다.
4. `jaw_state=contact/empty/unknown`. empty는 **닫힌 빈손**의 실측 범위다.
   닫기 명령 완료만으로 empty 또는 contact를 생성하지 않는다.
5. 명시적인 `verification_phase=True` — 닫기 이후 확인 단계에서만 사용한다.

기본값은 실제 장착 후 조정할 초기값이다:

- 측정 TCP 이동 10 mm 이상. 기존 4 mm probe에서는 UNKNOWN이 정상일 수 있다.
  이 판정은 우선 20 mm lift 이후 확인용이며 probe 안전 검사를 대체하지 않는다.
- 대상/예상 영역의 양방향 겹침 65% 이상, 반대 가설과의 겹침 15% 이하.
- 두 가설 영역 간 겹침 15% 이하. 위아래 이동이 영상에서 구분 안 되면 보류한다.
- 각 영역의 가림 15% 이하. 원래 자리와 그리퍼를 함께 볼 수 있는 설치가 유리하다.
- 최소 3개의 새 프레임, 최소 0.3초 연속 유지. 0.2초 초과 프레임 공백은 연속성 초기화.
- 입력 최대 나이 0.2초, 영상/FK/그리퍼 시간 차이 최대 0.05초.

`target_id`는 연결 시 `program.candidate_id`와 같아야 한다. 검출기에서 나온
의미 클래스명을 무조건 candidate_id로 바꾸면 같은 종류의 다른 물체가 섞일
수 있다. 대상 연결/추적 단계에서 인스턴스를 확인하고, 복수 후보는 UNKNOWN으로
남긴다. 기존 `GroundingSAMDetector`의 object_id/label_resolver는 이 경계에서
설정한다. 겹침 방식은 외형 변화, 반사/무늬 없는 물체, 큰 회전, 부분 가림에
약하므로 실제 물체별 오탐률을 반드시 측정해야 한다.

## 기존 실행기 연결

입력기가 IDLE이고 snapshot admission 이전일 때 한 번 묶는다:

```python
inputs.bind_grasp_verifier(
    arm="left", attempt_id=attempt_id, camera_frame="left_grasp_optical")
# 별도의 카메라 worker: 새 영상 + 동기화된 FK + 분할/가림 결과
result = verifier.observe_rgb(rgb, detector=detector, visibility=visibility,
                             **measurements)
# admission 후 probe/lift/carry/hold 단계에서만 전달
inputs.add_grasp_verification(result)
```

실제 프로그램/시도/카메라가 다르거나 오래된 결과는 거부한다. SUCCESS는
기존 CarryFeedback의 held로, FAILURE는 empty로 전달한다. 기존 jaw/scene/
platform 검사도 계속 적용된다. `CarrySession.holding_reason`은 실제 fresh
held 증거가 있을 때만 `HOLDING_VISUALLY_VERIFIED`가 된다. 확인된 FAILURE는
후속 미검출로 지우지 않고 기존 정지 경로로 전달한다.

기존 stationary 실행기의 오른팔 계획/실행을 이번 코드가 추가한 것은 아니다.
오른팔 카메라 판정기는 만들 수 있지만 왼팔 프로그램 입력에는 연결할 수 없다.

## 재시도 정책과 남은 연결 작업

`GraspRetryGate`는 팔별 최대 2회(설정 가능)의 재시도를 허용한다. 아래 모든
조건을 확인해야 `REPLAN_FIRST_FEASIBLE_FROM_CURRENT_STATE`를 반환한다:

1. 해당 시도가 TARGET_REMAINED_AND_JAW_EMPTY로 확정 실패했다.
2. 실행이 실제로 정지했고 컨트롤러 정상 및 실제 빈손 상태를 확인했다.
3. 현재 측정 자세에서 충돌 검사를 거친 빈손 후퇴/복귀가 완료됐다.
4. 실패 이후의 새 RGB-D capture와 현재 관절 상태를 얻었다.
5. 동일 실패를 이미 재시도에 사용하지 않았고 재시도 예산이 남았다.

그 후 기존 `plan_pick_carry(stop_after_first=True)`로 새 관측/현재 관절에서
첫 유효 경로를 구한다. 재시도 때문에 모든 후보의 경로를 미리 비교할 필요는 없다.
기존 경로 역재생, 무조건 홈 복귀, 같은 물체 좌표 재사용은 하지 않는다.

**Gate는 허용 판정이며 복귀/재촬영/재실행을 직접 수행하지 않는다.** 이 호출은
기존 runtime의 recovery 및 capture/planner 포트에서 연결해야 한다. 현재
정지기는 fault/torque-off를 동반할 수 있으므로 새 카메라 실패가 발생했다고
자동 rearm하는 코드를 넣지 않았다. 실측 복귀 포트가 연결되기 전에는 확정
실패 시 정지/보고까지가 실제 실행 경로의 동작이다.

## 검증

```bash
PYTHONPATH=ros2_ws/src/home_robot_tasks:ros2_ws/src/so101_arm_bridge:. \
  python3 -m pytest -q tests/test_rgb_grasp_verification.py \
  tests/test_carry_snapshot_execution.py tests/test_carry_live_inputs.py \
  tests/test_carry_execution.py
```

저장한 실제 프레임의 마스크와 측정 변환은 모터 없이 재생 가능하다:

```bash
python3 tools/diagnostics/replay_rgb_grasp.py \
  --manifest /path/to/recording.json --output /path/to/result.json
```

manifest의 `calibration`은 RGBGraspVerifier 생성자 인자이고, `frames`는
observe 인자 목록이다. `target_masks`에는 해당 프레임 bool .npy 경로 목록,
`occluded_mask`에는 가림 bool .npy 경로 또는 null을 쓴다. 경로는 manifest
위치 기준이다. 테스트의 replay 사례가 최소 실행 예제다.

실물 검증에서는 성공 파지/빈손/밀림/가림/낙하/손목 회전/리프트 이동을
각각 기록하고 성공 오탐, 실패 오탐, UNKNOWN 비율, 판정 지연을 구분한다.
합성 테스트 통과는 카메라 장착 정확도나 실제 파지 인식률 검증이 아니다.
