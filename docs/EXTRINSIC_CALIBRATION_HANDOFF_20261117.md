# 카메라·팔·리프트 보정 인계 (2026-11-16~17)

대상: 집기·좌표 코드 담당 · 작성: 여은 · 브랜치 `yeoeun/extrinsic-calibration`
측정 경과: [결과 기록](CALIBRATION_RESULTS_20261117.md) · 절차: [현장 가이드](CALIBRATION_FIELD_GUIDE_20261116.md) · 이전: [11/13 인계](EXTRINSIC_CALIBRATION_HANDOFF.md)

## 1. 결론

- 토크 ON 자동 투어로 리프트 2개 높이에서 120장을 촬영했다(재투영 RMS 1.1–1.6px, 11/13은 4–5px). 카메라 6자유도, 양팔 관절 영점, 오른팔 장착, **리프트 배율·영점**을 동시에 추정했다.
- 계통 오차의 주원인은 리프트 환산이다. 런타임은 `h = (raw − origin) × meters_per_count`(84mm/rev)로 계산하는데, 실제 리프트는 **+3.4~5.1% 더 이동**하고 **영점이 −6~−16mm**다. 팔이 프로필보다 낮게 위치하므로, S3에서 관찰한 "목표보다 약 8mm 아래"와 방향이 일치한다.
- 카메라 x·y·yaw는 이번에 식별됐다. 모델 간 편차가 x·y 약 5mm, yaw 0.5°다(11/14는 x가 10–29mm 범위). 기준은 왼팔 CAD 장착과 왼팔 base 영점이며, 바퀴 기준 절대값은 측정 불가다.
- 런타임 필드에 그대로 들어가는 모델(영점 + 장착 + 리프트, 처짐 없음)을 **이 브랜치의 프로필에 반영했다**(커밋 `bc105c4`). 새 `calibration_id`는 `s2-dev-4683680db9f74ce58184eced`이고, 이전 프로필은 `config/home_assistant/historical/pre-extrinsic-20261117/`에 보존했다. 런타임 식으로 재계산한 TCP는 보정 모델과 ≤0.24mm 일치한다.
- 마커 기준 5겹 교차 검증 중앙값은 리프트 미보정 15.8mm → **적용 모델 10.5mm** → 처짐 포함 5.1mm(런타임 미지원)다. 화면 평면(lateral) 오차는 0.7–3mm다.
- 전체 테스트: 4,411 통과. 기존부터 실패하던 `test_s2_lift_contacts` 4개는 그대로다.
- Jetson 반영: 2026-11-18 확인 결과 `s4-unified-pick-20261116-cal1117`, `s4-unified-6624e5730336`, `s4-unified-8237cc790447`이 새 프로필(`s2-dev-4683680…`)이다. 원래 설치본 `s4-unified-pick-20261116`만 이전 프로필이다. **실물 집기 검증은 아직이다**(1-2, 4절).

### 1-1. 검증 범위: 이 보정이 보장하는 것과 보장하지 않는 것

위 오차 수치(10.5mm, lateral 0.7–3mm)는 아래 **검증된 조건**에서만 성립한다. 조건 밖에서 쓰면 오차가 이보다 커질 수 있고, 그 크기는 측정하지 않았다.

| 검증된 조건 | 내용 |
|---|---|
| 리프트 | 5cm, 30cm 두 높이. 원점은 **최하단**에서 등록 |
| 팔 끝 기준점 | 고정 손가락 바깥면에 붙인 **팔 마커**. 마커 위치 오차로 평가 |
| 카메라 관측 | 마커 자세, 바닥·탁자 **평면** 적합 |

| 검증하지 않은 것 | 이유 | 영향 |
|---|---|---|
| 리프트 30cm 초과, 상단 등록 | `lift.travel_m`(0.6438m)은 CAD 값 그대로다. 배율 추정은 모델에 따라 +3.4~5.1%로 갈린다 | 64cm에서 배율 차이 1.7%는 약 11mm다. 상단에서 CAD 높이로 등록하면 최하단 기준 보정과 기준이 달라진다 |
| 손끝(TCP) 위치 | 마커가 붙은 자세는 6자유도 미지수로 같이 추정했다. 손끝은 마커로 관측하지 않았고 CAD `gripper_frame_link` 오프셋으로 계산한다. **손목 회전 영점은 마커 자세에 흡수돼 구해지지 않는다**(`calib_model.py` 머리말) | 손끝 오프셋이나 손목 회전 영점이 틀리면 손목 자세에 따라 높이와 앞뒤가 함께 틀린다. 손끝 접촉 시험은 판정하지 못했다(잔차 12mm, 5절 한계 5) |
| 한 점의 깊이 | 평면과 마커만 검증했다. 어두운 표면(검은 테이프 등)의 한 픽셀 깊이는 다루지 않았다 | 카메라가 비스듬히 보므로 깊이 오차는 높이와 앞뒤 오차로 함께 나타난다 |
| 중력 처짐 | 측정은 했지만 런타임 필드가 없어 빠졌다 | 처짐을 넣으면 10.5mm에서 5.1mm로 줄어드는 차이만큼 남는다 |

"카메라 6자유도 식별"과 "교차 검증 10.5mm"를 위 조건 밖의 팔 끝 정확도로 옮겨 쓰지 않는다.

### 1-2. 2026-11-18 점 이동 시험 (현서 측 보고)

| 항목 | 내용 |
|---|---|
| 프로필 | 새 프로필 확인(Jetson `s4-unified-6624e5730336`, `s2-dev-4683680…`) |
| 조건 | 높은 수납장(약 80cm), 리프트 상단 리셋 후 CAD 약 64.4cm로 등록, 목표는 검은 테이프 한 점, 끝점은 CAD 고정 손가락 끝 |
| 결과 | 좌우는 맞음. 손끝이 수납장 쪽으로 약 1cm 내려가고 앞뒤도 어긋남(사진·육안, 정량 측정 아님) |
| 배제된 원인 | 명령 관절각 대 엔코더 차이로 계산한 높이 변화 약 0.2mm. 서보 추종은 주원인이 아니다 |
| 의심 원인 | 위 1-1의 미검증 항목 세 가지(리프트 상단 등록, 손끝, 테이프 깊이)가 모두 이 시험 조건에 해당한다. 테이프 목표 높이는 약 79.38cm로, 같은 장면의 수납장 평면(80.13cm)보다 약 7.5mm 낮다 |

판별 순서:
1. (로봇 불필요) 저장된 RGB-D에서 테이프 픽셀의 시선과 수납장 평면의 교점으로 목표를 다시 계산해 79.38cm와 비교한다.
2. (로봇 불필요) 리프트 raw 값을 상단 등록 시점과 팔 도착 시점에서 비교한다. 다르면 리프트 하강이다.
3. (실물) 최하단 원점으로 등록하고 낮은 책상(검증 범위 안)에서 같은 시험을 한다. 오차가 줄면 상단 등록·외삽이 원인이다.
4. (실물) 같은 표적에 손목 자세를 3–4가지로 바꿔 손끝을 댄다. 자세에 따라 오차가 바뀌면 손끝·손목 회전 영점이 원인이다. 리프트 높이를 함께 기록하면 손끝 오프셋까지 구할 수 있다.

## 2. 반영 구조와 변경 내용

**원본**: `config/home_assistant/measured_calibration.development.json`(새 파일). 실측값과, 보정할 때 기준으로 삼은 명목값이 들어 있다.

**빌드**: `s2_development.build(root)`
1. 기존 명목 프로필을 만든다(`build_nominal`).
2. 보정 파일의 기준 명목값과 수치로 비교한다(허용 1e-9). id가 아니라 수치로 비교하는 이유는, 재빌드하면 1e-16 차이로 id가 바뀌기 때문이다.
3. 기준이 일치할 때만 `apply_measured_calibration`을 적용하고, 다르면 오류를 낸다.
- `build(root, measured=False)`는 기존 명목 경로를 그대로 쓴다.

**배포 파일 생성**: `tools/diagnostics/extrinsic_capture/apply_measured_calibration.py`
1. 보존된 명목 프로필에 보정을 적용한다.
2. 광각 프로필을 만든다(`camera_evidence.wide_color_profile`).
3. 높이 모델을 만든다(같은 카메라의 z행).
4. 집게 기록의 바인딩을 갱신한다(집게 매핑 변화 없음).
5. 런타임 로더로 확인한다.

| 필드 | 이전(명목) | 현재(실측) |
|---|---|---|
| `base_from_optical` pitch / roll / yaw | 45.00 / 0.00 / −0.68° | 45.78 / −1.31 / +0.30° |
| `base_from_optical` t (mm) | 100.7, 33.2, 1007.1 | 121.9, 34.8, 1008.0 |
| `camera_height_adjustment` (−9.1mm) | 있음 | 없음(카메라 실측에 포함) |
| left `home_model_rad` Δ (shoulder/elbow/wrist_flex) | 0 | +5.37 / +0.91 / −0.03° |
| right `home_model_rad` Δ | 0 | +7.69 / +3.18 / −4.64° |
| `reference_model_rad` | 0 | 위 Δ와 같음(Home 자세의 모델각 = 측정 영점) |
| right `base_from_arm_at_lift_zero` | CAD | `CAD @ right_mount_correction`, t ≈ (10.2, 1.2, −1.2)mm, 회전 ≈ 3° |
| 양팔 `base_from_arm_at_lift_zero` t | CAD | `+= axis_base × (−5.74mm)`(리프트 영점) |
| `lift.meters_per_count` | −2.0508e-05 | −2.1310e-05 (×1.0391) |
| `calibration_id` | `s2-dev-5bfc28c62078dfb0377bfdb7` | `s2-dev-4683680db9f74ce58184eced` |
| wide `calibration_id` | `s2-dev-28078929a9113a0c7b4a4559` | `s2-dev-d705d579ede27011cb5c0785` |
| 높이 모델 `model_id` | `height-2b4512f39d52caae4cae1a10` | `height-accbcb85329e6564ef266208` |
| `left_pad_gap.observed.json` | 이전 id | 새 id, `measured_calibration_profile_rebind` 기록 추가(측정값 동일) |

- base 영점은 게이지라 바꾸지 않았다(오른팔 base는 `right_mount_correction`에 포함). 집게 매핑도 그대로다.
- 승인 플래그(`coordinate_use_allowed`, `motion_authorized` 등)는 `false`다.

**테스트 변경**
- 명목 경로를 검사하는 테스트(`test_s2_development`, `test_s2_acquisition`)는 `build(ROOT, measured=False)`를 쓰게 했다.
- 명목값을 하드코딩한 기대값(높이 행, Home 자세 0°, 카메라 높이 보정 재바인딩)은 실측 기록이나 보존본을 기준으로 바꿨다.
- 가상 장면 회귀 테스트(stationary lab 계열)는 명목 카메라 기준으로 설계된 장면이다. 그래서 `tests/conftest.py`에서 보존된 명목 프로필과 집게 기록으로 고정했다. `run_s3_stationary_lab.py`에 `PROFILE`, `PAD_RECORD`, `--profile`, `--pad-record`를 추가했다.
- 실측 보정 전용 테스트: `tests/test_measured_calibration.py`
  - 배포본 재현
  - 항목별 반영 위치
  - 기준이 다른 명목값 거부(8가지)
  - 높이 모델·광각 일치
  - 집게 측정값 보존

## 3. 영향 범위

프로필을 읽는 모든 모듈에 자동 반영된다. 코드와 문서에 값을 옮겨 적을 필요는 없다.

| 영역 | 사용처(예) | 영향 |
|---|---|---|
| 집기 좌표·계획 | `unified_pick.py`, `carry_live_inputs.py`, `carry_scene*.py`, `stationary_pick.py` | 물체 → base 변환, 팔 FK·IK, 리프트 높이 |
| 높이 판정 | `stationary_rgbd.load_surface_height_model` | 탁자·물체 높이 |
| 센서 TF | `s4_sensor_frames.py` | 주행·지도의 RGB-D 투영 |
| 정렬 후보 | `assistant/object_alignment.py` | 리프트 높이별 도달성 |

- **이전 id에 묶인 산출물은 거부된다.** 저장된 집기 bundle(`validate_installed_binding`이 프로필 digest를 비교), 미션 대상, 이동 조작 프로그램은 새 관측·계획으로 다시 만든다.
- 기존 지도의 RGB-D 층은 이전 카메라 TF로 만들어졌다. 카메라 앞뒤 약 2cm, roll 1.3° 차이만큼 cm 단위로 어긋날 수 있다. 라이다 2D 지도는 상관없다.
- 리프트 원점은 지금처럼 최하단에서 등록한다(리프트 영점과 배율은 "최하단 = 원점"이 전제).
- 이력 문서(`CALIBRATION_GUIDE.md` 12.3, `history/*`)는 그 시점의 기록이라 수정하지 않았다. `CURRENT_STATUS.md`의 좌표·높이 항목만 갱신했다.

## 4. Jetson 반영과 실물 집기 시험

### 4-1. 설치본 복사본에 반영 (기존 설치본은 그대로)

노트북(브랜치 체크아웃 상태)에서:

```bash
R=/home/hyper/robot/releases/s4-unified-pick-20261116
ssh hyper@192.168.35.2 "cp -a $R ${R}-cal1117"
cd ~/Mobile-Bimanual-Home-Assistant/config/home_assistant
scp s2_coordinates.development.json s2_coordinates.wide.development.json camera_height.calibrated.json \
    left_pad_gap.observed.json measured_calibration.development.json hyper@192.168.35.2:${R}-cal1117/config/home_assistant/
```

Jetson에서 로더 확인(모터 접근 없음):

```bash
cd /home/hyper/robot/releases/s4-unified-pick-20261116-cal1117
/home/hyper/robot/shared/venvs/s4-jazzy/bin/python - <<'PY'
import sys, json; from pathlib import Path
sys.path[:0]=['ros2_ws/src/home_robot_tasks','ros2_ws/src/so101_arm_bridge']
from home_robot_tasks import s2_development as dev, s2_geometry as s2, camera_evidence as ce
from home_robot_tasks.stationary_rgbd import load_surface_height_model
from home_robot_tasks.installed_jaw import InstalledLeftJaw
g=s2.load_geometry(Path('config/home_assistant/official_geometry.json')); p=dev.released_profile(Path('.'),g)
w,_=ce.load_capture_profile(Path('.'),'wide'); h=load_surface_height_model(Path('.'))
InstalledLeftJaw(json.loads(Path('config/home_assistant/left_pad_gap.observed.json').read_text()),p)
print('OK', p['calibration_id'], p['provenance'], w['calibration_id'], h.model_id)
PY
```

`OK s2-dev-4683680db9f74ce58184eced measured_joint_camera_arm_lift_calibration ...`이 나오면 된다.

### 4-2. 시험 순서

| 단계 | 내용 |
|---|---|
| 1. 기준(이전 프로필) | 기존 설치본(`s4-unified-pick-20261116`)으로 평소 절차대로 수납장 흰 리모컨 정지 집기. 새 촬영 → 계획 → bundle → 실행 |
| 2. 새 프로필 | 리모컨, 베이스, 리프트를 그대로 두고, **복사본(`-cal1117`)에서** 같은 절차를 처음부터 진행. 새 촬영과 재계획이 필요하다(이전 bundle은 거부됨) |
| 3. 반복 | 2를 2–3회 |
| 4. (선택) 리프트 상승 | 80cm 책상에서 1·2 비교. 리프트 배율 보정 효과 확인 |

각 실행은 **닫힌 직후, 들기 전**에 기록한다.
- 사진: 정면, 옆면, 위. 자를 같이 대고 찍는다.
- 측정 4가지:
  - 고정 손가락 접촉점의 높이 차이(목표 단면 대비, 위 +, 아래 −, mm)
  - 좌우 차이
  - 길이 방향 차이
  - 닫힘 선 기울기
- 실행 로그 폴더 이름
- 판정 기준: 이전 대비 **아래쪽 치우침이 줄었는지**가 핵심이다. S3에서는 약 8mm 아래였다.

### 4-3. 되돌리기

복사본을 쓰지 않으면 된다. 기존 설치본은 바꾸지 않았다. 저장소에서는 `historical/pre-extrinsic-20261117/`의 파일 4개를 되돌리고 `measured_calibration.development.json`을 지우면 이전 상태가 된다.

## 5. 근거와 한계

**모델 비교** (리프트 높이 = 엔코더, 5겹 교차 검증, 마커 위치 오차 중앙값 mm)

| 모델 | 전체 | 95% | L 5cm | L 30cm | R 5cm | R 30cm |
|---|---|---|---|---|---|---|
| 영점 | 15.8 | 47.5 | 12.2 | 2.0 | 34.5 | 12.5 |
| **영점 + 리프트 (후보)** | **10.5** | 30.9 | 9.8 | 2.0 | 19.5 | 7.9 |
| 영점 + 처짐 + 리프트 | 5.1 | 18.1 | 5.7 | 2.2 | 9.3 | 4.4 |
| + 리프트 기울기 | 6.1 | 29.9 | 7.2 | 2.3 | 18.4 | 4.0 |

방향별 오차(처짐 모델): lateral 0.7–3.0mm, along-ray −8.5 ~ +3.3mm. 바닥·탁자 평면 오차 ≤4mm. near fx 618.1 vs 공장값 617.6(0.08%).

**한계**
1. **중력 처짐(`c_*`)은 런타임 필드가 없어 후보에서 빠졌다.** 처짐 모델로 가면 5.1mm까지 줄어든다.
2. **서보 추종 오차는 보정 범위 밖이다.** 보정은 측정 엔코더 기준이다. 실측한 명령 대비 오차는 어깨 10–14 raw(≈1°, 팔 끝 ≈5mm), 손목 굽힘 최대 15 raw이고 주로 중력 방향이다. 투어 각 샷의 `joints.json`에 `commanded_raw`, `settle_error_raw`로 남아 있다.
3. 리프트 배율·영점은 처짐 모델 포함 여부에 따라 +3.4~5.1%, −6~−16mm로 갈린다. 부호는 모든 분석에서 같다. 후보는 처짐 없는 모델 값(+3.91%, −5.74mm)이다.
4. **오른팔은 두 높이를 단일 장착 보정으로 설명하지 못한다**(5cm 19.5mm). 장착 강성, 촬영 중 fault로 팔이 낙하한 구간, 마커 변위가 후보다.
5. TCP(손가락 끝) 독립 검증이 미완이다. 집게 끝 접촉 시험은 리프트 회전 수 기록 문제(기록 0.29m, 영상상 약 0.55m)와 pivot 잔차 12mm로 판정하지 못했다.
6. 카메라 받침, 팔 장착, 리프트를 기계적으로 변경하면 재측정이 필요하다.

## 6. 후속 작업

1. Jetson 복사본 반영 → 집기 전후 비교(4절)
2. 런타임 FK(`MeasuredFK`, 계획 모델)에 처짐 항 추가 → 처짐 모델 적용
3. 명령 보정: `settle_error_raw`를 관절별 중력 토크에 회귀시켜 목표각에 오프셋 적용
5. 리프트 배율 확정: 높은 높이 추가 측정(55cm는 카메라와 약 6cm 근접 → 해당 높이 훑기 필요)
6. 오른팔 원인 조사, 광각 평면(`W2-*`) 반영, 렌즈 왜곡(모서리 데이터 보강)

## 7. 현장 발견 사항

| 항목 | 내용 |
|---|---|
| 투어 중 안전 정지 2회 | 호스트 메시지는 contract 27(`SAMPLE_DISCONTINUITY`)이지만, 이는 `binary_control.c`에서 실행기가 구체 결과 없이 거부할 때의 대체 코드다. 실제 원인은 dispatch `failure_count=1`(서보 명령 송신 중 UART 오류) → `BinaryControl_LatchStop`. 읽기 실패 0, 두 번 모두 오른팔 이동 중. 계획 경로 최대 1,745µrad/tick(한도 9,000). 진단에 버스 구분이 없어 좌·우를 단정할 수 없음 |
| 오른팔 버스 전체 무응답 반복 | 재연결로 복구(C01과 동일 증상) |
| 리프트 자연 하강 | S3 펌웨어에서 리프트 토크 없음. 투어 후 raw 1689 → 2058(≈7.6mm) |
| 베이스 수리 후 바퀴 7–9 무응답 | 배선 분리. S3 inventory와 정지 확인에 바퀴가 필요 |
| 2.4/5GHz 대역 간 격리 | Jetson 2.4GHz. 노트북을 같은 대역으로 |

## 8. 펌웨어·로봇 상태

S3(0x24911)로 임시 교체 후 **0x2492D 복구 설치 완료**(`install.sh` exit 0). 백업은 Jetson에 있다.

| 백업 | 경로 | SHA256 |
|---|---|---|
| S4 원본 | `diagnostics/s3/stationary/s3-stationary-9d2fd662e718/flash/preflash-full.bin` | `8dae4b9e…` |
| S3 상태 | `diagnostics/s4-unified/s4-unified-e004bef4f622/install/` | `380e78a7…` |

이전 설치 기록은 `*-before-20261116`으로 보존했다. 최종 식별 확인과 S4 준비는 기존 절차대로 한다.

## 9. 기존 문제 목록과의 대응

기준 문서: `history/PROJECT_HISTORY_AND_DECISIONS_2026-11-15.md`(TalkFile)의 항목 번호. "적용 시"는 이 브랜치의 실측 프로필이 Jetson에 반영됐을 때를 뜻한다.

| 항목 | 문제 | 이번 결과 | 상태 |
|---|---|---|---|
| H01 | 80.91 vs 80cm 한 점으로 Z −9.1mm 보정 | −9.1mm는 약 1° pitch 오차를 한 거리에서 상쇄한 값으로 확인. 카메라 6자유도 실측으로 대체, 후보에서 `camera_height_adjustment` 삭제 | 해결(적용 시) |
| H03 | 높이 성분만 사용, 카메라 XY·yaw·팔 보정 미적용 | 카메라·영점·장착·리프트 단일 묶음 + 같은 카메라에서 만든 높이 모델 | 해결(적용 시) |
| §1 S2 | 외부 보정 실측 정확도 미보증 | x·y·yaw 식별(모델 간 5mm·0.5°), 교차 검증 수치화(lateral 0.7–3mm) | 대부분 해결 |
| G06 | S3 약 8mm 아래 접촉, 원인 미확정 | 리프트 환산 오차(+3.9%, −5.7mm, 팔이 프로필보다 낮음)와 서보 추종 오차(어깨 약 1°)를 실측. 둘 다 아래 방향 | 원인 확인, 실물 검증 대기 |
| G01 | Home 영점 정확도 미검증 | 양팔 영점 보정값 실측. 처짐 포함 여부에 따라 수 도 차이 | 부분 |
| G05 | 손가락이 탁자를 끎, 모델·실물 높이 차이 | 높이 차이의 주요인(리프트) 확인 | 관련 원인 해소 |
| H04 | 다른 바닥 영역 −10.7mm 잔차 | 전체 카메라 자세 보정 후 near 평면 4mm 이내 | 개선(광각 재확인 필요) |
| H06 | 광각 실측 미완 | 광각 프로필도 같은 실측 카메라로 재생성. 광각 렌즈 왜곡은 촬영만 하고 미분석 | 부분 |
| §11 | 자세·하중에 따른 처짐 평가 | 처짐 계수, 서보 추종 오차 실측(투어 전 샷 기록) | 측정 완료, 런타임 미반영 |
| C01 | 오른팔 응답 누락 원인 미확정 | 투어 중 정지 2회가 명령 송신 중 UART 오류(dispatch failure), 모두 오른팔 이동 중 | 단서 추가, 미해결 |
| B01–B03 | 정차 후보 정확도·시간 | 리프트 높이 계산이 맞아져 도달성 평가 정확도 향상. 시간 문제는 그대로 | 간접 |
| G02·G03·G04 | 고정 손가락 결합, 손목 기울기, 미끄러짐 | 손목 회전 영점은 미측정. 나머지는 보정 범위 밖 | 해당 없음 |
| H02·H05 | 정지 상태 깊이 변동, 저장 관측 | 센서 노출·관측 흐름 문제 | 해당 없음 |

11/16 팀 공유 문제 5개(TF·캘리브레이션, 집기 후보, 정차, 오른팔 heartbeat, 타임라인) 기준:
- 1번은 적용과 실물 검증만 남았다.
- 4번은 원인 단서가 추가됐다.
- 2·3·5번은 범위 밖이다. 다만 팔 정확도가 오르면 정차에 필요한 정밀도가 완화된다.

## 부록 A. 데이터 (저장소 밖, 인물 포함)

`~/cal_20261116/`(노트북·Jetson). 실측값은 `measurements.json`에 있다(마커 66.2mm, 표적 중심 간 152.3mm, 체커보드 25.0mm, **리프트 최하단 raw 3900**, 휴식 자세, 펌웨어 기록). 분석 결과는 `~/cal_work/calib_enc_20261117.json`.

| 폴더 | 내용 |
|---|---|
| `T05b`/`T05d`/`T05f` | 투어, 리프트 raw 1663(45.9mm) |
| `T30` | 투어, raw 1584(299.5mm), 5cm 훑기 재사용 |
| `F-*`, `TB-*`, `TF-*` | near 평면(하드웨어 수리 후) |
| `W2-*` | 광각 고정 노출 평면, 미분석 |
| `IN-*`, `W-IN-*` | 렌즈 |
| `SW05-*` | 손 훑기(투어 경로 지도) |
| `TCP-L-*` | 집게 끝 접촉(미판정) |

## 부록 B. 재현

```bash
PY=~/.venvs/calib/bin/python; A=tools/diagnostics/extrinsic_capture/analysis; C=~/cal_20261116; T=$C/T05b,$C/T05d,$C/T05f,$C/T30
$PY $A/calib_encoder_lift.py . $C $T ~/cal_work/calib_enc_20261117.json --cv 5
$PY $A/export_measured_calibration.py . $C $T ~/cal_work/calib_enc_20261117.json "AL zeros + lift" \
    config/home_assistant/historical/pre-extrinsic-20261117/s2_coordinates.development.json NEW_measured_calibration.json
$PY tools/diagnostics/extrinsic_capture/apply_measured_calibration.py --check-only   # 배포 파일 검사
```

환경: `python3 -m venv ~/.venvs/calib && ~/.venvs/calib/bin/pip install numpy scipy opencv-contrib-python pillow`.
도구 변경 내역은 [결과 기록 7절](CALIBRATION_RESULTS_20261117.md)과 `tools/diagnostics/extrinsic_capture/README.md`에 있다.
