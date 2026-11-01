# 인식 인터페이스

기준 코드는 [home_perception/contracts.py](../home_perception/contracts.py)다. 다른 계층은 이 양식에 맞춰 연결한다. 좌표는 m, 시각은 UTC ISO 8601이다. 모르는 값은 `null`로 두고 추정해 채우지 않는다.

## 흐름

```text
TaskRequest → 인식 실행 → ObjectObservation → (S2 보정) → WorldEntity
```

`ObjectObservation`은 사진 1장에서 본 물체 1개의 근거이고, `WorldEntity`는 시간이 지나도 유지되는 기억이다.

## TaskRequest

L5가 만들고 L4가 검증한다. 좌표와 관절값을 담지 않는다.

| 항목 | 형식 | 설명 |
|---|---|---|
| `request_id`, `user_id`, `text` | 문자열, 필수 | 요청 ID, 사용자, 원문 |
| `action`, `target`, `source`, `destination` | 문자열 또는 null | 의미 슬롯 |
| `unresolved` | 목록 | 채우지 못한 슬롯. `action`이 null이면 반드시 포함 |

## ObjectObservation

| 항목 | 형식 | 설명 |
|---|---|---|
| `capture_id`, `captured_at_utc`, `frame_id` | 문자열, 필수 | 촬영 ID, 시각, 좌표계 |
| `label`, `box_xyxy` | 문자열, 숫자 4개 | 검출 라벨과 픽셀 박스 |
| `detector`, `detector_revision`, `detector_score` | 문자열, 문자열, 숫자 | 모델과 버전. 점수는 확률이 아니며 Qwen은 null |
| `source_sha256` | 사전 | 입력 파일 해시 |
| `status` | `accepted`, `held`, `rejected` | 판정 |
| `failure`, `reason` | 문자열 | 실패 코드와 상세 이유. 통과면 null |
| `mask_quality`, `mask_pixels` | 숫자 | SAM 결과. 인식 확률이 아님 |
| `valid_depth_fraction`, `point_count`, `cluster_fraction` | 숫자 | 깊이 품질 |
| `surface_xyz_m` | 숫자 3개 | 측정된 표면 픽셀 1개 |
| `center_xyz_m`, `obb` | 숫자 3개, 사전 | 보이는 표면의 중심과 상자. 무게중심이나 파지 위치가 아님 |
| `support_height_m` | 숫자 | 지지면 위 높이 |
| `calibration_id`, `clock_domain` | 문자열 | 보정 ID, 시계 영역 |
| `robot_transform` | 목록 또는 null | 보정 전에는 null. 단위 행렬로 채우지 않음 |

규칙:
- `rejected`는 좌표를 가질 수 없다.
- `held`와 `rejected`는 `reason`이 필수다.
- 재확인에 실패한 검출은 `failure: NOT_FOUND`, `reason: verification_failed`다.

## WorldEntity

L3B만 갱신한다.

| 항목 | 형식 | 설명 |
|---|---|---|
| `entity_id`, `kind`, `frame_id` | 문자열, 필수 | ID, 종류, 좌표계 |
| `state` | `observed`, `unconfirmed`, `stale`, `occluded`, `carried`, `released_unconfirmed`, `placed` | 상태 |
| `source` | `observed`, `inferred`, `user_confirmed` | 값의 출처 |
| `position_m`, `extent_m` | 숫자 3개 또는 null | 위치와 크기 |
| `geometry_kind` | `none`, `surface_point`, `visible_surface_center` | 위치의 의미. 위치가 없으면 `none` |
| `room_id`, `support_id` | 문자열 또는 null | 소속 방, 받침 |
| `aliases`, `attributes`, `relations` | 목록, 사전, 목록 | 별칭, 속성, 관계 |
| `evidence` | 목록 | 근거 `capture_id`. `source`가 `observed`면 필수 |
| `last_seen_utc`, `position_uncertainty_m` | 문자열, 숫자 | 마지막 관측, 위치 불확실성 |
| `map_version`, `revision` | 문자열, 정수 | 지도 버전, 항목 버전 |

## ScanSession

| 항목 | 형식 | 설명 |
|---|---|---|
| `session_id`, `sensor_id`, `calibration_id` | 문자열, 필수 | 세션, 센서, 보정 ID |
| `status` | `running`, `paused`, `stopped`, `saved`, `failed` | 진행 상태 |
| `coverage_fraction`, `map_revision` | 숫자, 문자열 | 관측 범위, 지도 버전 |

## 실패 코드

`NOT_FOUND`, `AMBIGUOUS_TARGET`, `INVALID_DEPTH`, `STALE_OBSERVATION`

## `home_robot_tasks` 쪽에서 맞출 곳

| 현재 코드 | 맞출 내용 |
|---|---|
| `assistant/contracts.py`의 `entity` | `WorldEntity` 항목으로 대응. 차이: `source`의 `sensor`/`user`/`simulation` 값, `stamp` 구조, `category`, `label` |
| `assistant/perception.py`의 `report_entities` | Grounding 보고서 대신 `ObjectObservation`을 입력으로 받음. 표면점만이 아니라 `center_xyz_m`, `obb`도 사용 |
| `assistant/perception.py`의 `TransformSample` | `ObjectObservation` 좌표 변환에 그대로 사용 |
| `assistant/scanning.py`의 `ScanManager` | 외부 표시 상태를 `ScanSession`으로 제공 |
| `assistant/meaning.py`, `fetch.py`의 `FetchRequest` | L5 출력을 `TaskRequest`로 통일 |

`stamp`의 지도 ID와 시계 ID는 `WorldEntity`에 아직 없다. 위 표의 `source`, `stamp` 대응은 연결할 때 함께 정한다.
