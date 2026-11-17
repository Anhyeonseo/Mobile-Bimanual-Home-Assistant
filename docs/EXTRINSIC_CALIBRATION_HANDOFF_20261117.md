# 카메라·팔·리프트 보정 인계 (2026-11-16~17 측정)

받는 사람: 집기 코드 담당(현서) · 작성: 여은 · 브랜치 `yeoeun/extrinsic-calibration`

**이 문서만 보고 새 보정값을 적용하고 시험할 수 있게 쓴 문서다.**
측정 과정은 [결과 기록](CALIBRATION_RESULTS_20261117.md), 현장 절차는 [현장 가이드](CALIBRATION_FIELD_GUIDE_20261116.md)에 있다.

---

## 1. 요약

- 집기가 **목표보다 아래를 잡던 가장 큰 원인은 리프트 높이 계산**이었다.
  로봇은 "리프트 1회전 = 84mm, 최하단 = 0"으로 계산하는데, 실제 리프트는 **약 4% 더 움직이고 영점도 몇 mm 낮다.**
  그래서 로봇은 팔이 실제보다 높이 있다고 착각하고, 그만큼 더 내려간다.
- 카메라는 **기울기, 높이, 앞뒤, 좌우, 수평 회전까지 모두 실측으로 정했다.** 이전에는 높이만 실측이었다.
- 팔 관절 영점과 오른팔 장착 위치도 같이 정했다.
- 이 값들을 **지금 설정 파일 형식 그대로** 담은 후보 파일 3개를 만들었다. 코드는 바꿀 필요 없고, **파일만 교체**하면 된다.
- 적용 전 팔 위치 오차(카메라로 본 팔 마커 기준)는 리프트 보정 없이 약 16mm, 적용 후 약 10mm다(교차 검증 중앙값). 좌우·상하 방향은 1–3mm다.
- **아직 실제 집기로 확인하지 않았다.** 적용 후 같은 조건에서 집기 전후 비교가 꼭 필요하다(4절).

## 2. 왜 아래를 잡았나 (쉬운 설명)

리프트를 엘리베이터, 팔을 엘리베이터에 탄 사람이라고 생각하면 된다. 카메라는 벽에 고정돼 있다.

1. 로봇은 **층 표시기(리프트 엔코더)**를 읽어 "엘리베이터가 몇 cm 올라갔는지" 계산한다.
2. 이 계산 공식이 실제와 조금 다르다.
   - 실제 리프트는 공식보다 **약 4% 더** 올라간다. 30cm 올리면 1cm 이상 차이 난다.
   - 공식의 출발점(최하단 = 0)도 실제보다 **몇 mm 위**에 잡혀 있다.
3. 결과적으로 로봇은 **팔이 실제보다 높이 있다고 생각한다.**
   "물체 높이까지 팔을 내려라"라고 하면, 실제로는 더 많이 내려가서 **물체의 아래쪽**을 잡는다.
4. S3 집기에서 본 "목표보다 약 8mm 아래" 현상과 방향이 같다.

추가로, 중력 때문에 서보가 목표 각도까지 다 올라가지 못하는 양(어깨 약 1°, 팔 끝 약 5mm 아래)도 측정했다. **이 부분은 이번 파일 교체로는 고쳐지지 않는다**(6절).

## 3. 무엇이 바뀌나

후보 파일 위치: `config/home_assistant/candidates/`

| 후보 파일 | 교체 대상 |
|---|---|
| `s2_coordinates.development.candidate-20261117.json` | `s2_coordinates.development.json` |
| `s2_coordinates.wide.development.candidate-20261117.json` | `s2_coordinates.wide.development.json` |
| `camera_height.calibrated.candidate-20261117.json` | `camera_height.calibrated.json` |

바뀌는 값:

| 항목 | 지금 | 새 값 | 의미 |
|---|---|---|---|
| 카메라 기울기(pitch / roll / yaw) | 45.00° / 0.00° / −0.68° (CAD) | **45.78° / −1.31° / +0.30°** | 실측 |
| 카메라 위치(x, y, z) | 100.7, 33.2, 1007.1mm | **121.9, 34.8, 1008.0mm** | 앞뒤가 약 2cm 차이 |
| 왼팔 관절 영점(어깨 / 팔꿈치 / 손목 굽힘) | 0 / 0 / 0 | **+5.37° / +0.91° / −0.03°** | `home_model_rad`에 더함 |
| 오른팔 관절 영점 | 0 / 0 / 0 | **+7.69° / +3.18° / −4.64°** | 〃 |
| 오른팔 장착 위치 | CAD | **약 (10.2, 1.2, −1.2)mm, 회전 약 3°** | 오른팔 장착 행렬에 곱함 |
| 리프트 `meters_per_count` | −2.0508e-05 (84mm/회전) | **−2.1310e-05** (+3.91%) | 리프트 배율 |
| 리프트 영점 | 0 | **양팔 장착 위치를 리프트 축으로 −5.7mm** | 팔이 그만큼 낮음 |
| `camera_height_adjustment` (−9.1mm) | 있음 | **삭제** | 새 카메라 값에 이미 반영 |
| `calibration_id` | `s2-dev-5bfc28c62078dfb0377bfdb7` | `s2-dev-74cf69505343b9cef8a76199` | 내용이 바뀌어 새로 계산 |

- 회전축(base) 영점은 기준이라 바꾸지 않는다.
- 후보 파일은 프로젝트 검증 함수(`s2_development.validate`)를 통과했다.
- 새 설정으로 런타임 방식대로 계산한 팔 위치가 보정 계산과 **0.24mm 안에서 일치**하는 것을 확인했다.
- `coordinate_use_allowed`, `motion_authorized` 같은 승인 플래그는 그대로 `false`다.

## 4. 적용하고 시험하는 순서

### 4-1. 적용 전

1. 지금 파일 3개를 백업한다(PC 저장소와 Jetson release 양쪽).
   ```bash
   cd config/home_assistant
   for f in s2_coordinates.development.json s2_coordinates.wide.development.json camera_height.calibrated.json; do cp "$f" "$f.before-20261117"; done
   ```
2. **기준 집기 기록을 먼저 남긴다.** 지금 설정 그대로 수납장 흰 리모컨을 집고, 잡은 위치가 목표보다 위·아래·좌·우로 몇 mm 어긋났는지 기록한다. 사진이나 자로 재면 된다.

### 4-2. 적용

```bash
cd config/home_assistant
cp candidates/s2_coordinates.development.candidate-20261117.json s2_coordinates.development.json
cp candidates/s2_coordinates.wide.development.candidate-20261117.json s2_coordinates.wide.development.json
cp candidates/camera_height.calibrated.candidate-20261117.json camera_height.calibrated.json
```

Jetson release에도 같은 세 파일을 반영한다. 평소 배포 방식대로 한다.

**`calibration_id`가 바뀌므로, 이전 id에 묶여 저장된 것들은 다시 만들어야 한다.**
코드상 `calibration_id`나 프로필 해시를 확인하는 곳은 아래와 같다. 예전 파일을 쓰면 거부된다.
- 저장된 집기 묶음(`unified_pick.py --bundle`), 장착 집게 기록(`installed_jaw`)
- 미션 대상(`mission_catalog`), 이동 조작 프로그램(`mobile_manipulation`)
- 높이 모델(`camera_height` → 새 파일에 이미 반영)

새 관측과 계획으로 다시 만들면 된다.

**리프트 원점은 지금처럼 최하단에서 등록한다.** 새 리프트 값은 "최하단 = 원점"을 전제로 한다.

### 4-3. 시험 (같은 조건)

| 시험 | 확인할 것 |
|---|---|
| 수납장 흰 리모컨 정지 집기(리프트 최하단 근처) | 4-1의 기준 기록과 비교. **아래로 어긋나던 게 줄었는지** |
| 같은 자리 2–3회 반복 | 반복할 때 흔들리는 정도 |
| (가능하면) 리프트를 높여서 집기(80cm 책상) | 리프트 배율 보정이 높은 곳에서도 맞는지 |
| 왼팔 위주 | 오른팔은 아직 결과가 덜 확실함(6절) |

### 4-4. 되돌리기

```bash
cd config/home_assistant
for f in s2_coordinates.development.json s2_coordinates.wide.development.json camera_height.calibrated.json; do cp "$f.before-20261117" "$f"; done
```

이전 id에 묶인 기록들도 원래대로 다시 쓸 수 있다.

## 5. 결과 근거 (숫자)

측정: 리프트 두 높이(약 5cm, 30cm)에서 토크를 켠 자동 촬영 120장. 사진 맞춤 오차 1.1–1.6px(이전 4–5px).
검증: 사진을 5묶음으로 나눠, 맞출 때 쓰지 않은 사진으로 오차를 잰다(5겹 교차 검증, 팔 마커 위치 오차 중앙값 mm).

| 모델 | 전체 | 왼팔 5cm | 왼팔 30cm | 오른팔 5cm | 오른팔 30cm | 적용 |
|---|---|---|---|---|---|---|
| 리프트 보정 없음 | 15.8 | 12.2 | 2.0 | 34.5 | 12.5 | – |
| **리프트 보정 (이번 후보 파일)** | **10.5** | 9.8 | 2.0 | 19.5 | 7.9 | **파일 교체만으로 가능** |
| 리프트 + 중력 처짐 | 5.1 | 5.7 | 2.2 | 9.3 | 4.4 | 코드에 처짐 항이 있어야 함 |

- 좌우·상하(카메라 화면 방향) 오차는 모든 경우 **0.7–3mm**다. 남은 오차는 주로 카메라 깊이 방향이다.
- 카메라 값은 모델을 바꿔도 x·y 약 5mm, 각도 0.5° 안에서 같다. 이전(11/14)에는 x가 10–29mm 흔들렸다.
- 바닥·탁자 높이 오차는 4mm 이내다.

## 6. 주의와 한계

1. **중력 처짐과 서보 추종 오차는 이번 파일로 고쳐지지 않는다.** 런타임에 넣을 자리가 없다.
   - 처짐까지 넣으면 오차가 약 5mm까지 줄어든다(위 표).
   - 서보 추종 오차는 어깨 약 1°로 팔 끝 약 5mm, 주로 아래 방향이다. 투어 사진마다 `commanded_raw`와 `settle_error_raw`로 기록돼 있다. 명령 보정식을 만들면 줄일 수 있다.
2. **리프트 배율과 영점은 모델에 따라 범위가 있다.** 배율 +3.4 ~ +5.1%, 영점 −6 ~ −16mm다. 방향은 항상 같다(더 움직이고, 팔이 낮음). 후보 파일은 처짐 없는 모델 값(+3.9%, −5.7mm)이다.
3. **오른팔은 덜 확실하다.** 두 리프트 높이의 결과가 하나의 장착 보정으로 잘 맞지 않는다. 오른팔 장착부 강성, 케이블, 촬영 중 정지 사고를 의심한다.
4. **손가락 끝까지의 검증은 아직이다.** 집게 끝을 표적에 대는 검증은 리프트 높이 기록 문제로 판정하지 못했다(9절).
5. **값을 섞어 쓰지 않는다.** 카메라만, 또는 영점만 바꾸면 오히려 나빠질 수 있다. 세 파일을 같이 바꾼다.
6. 하드웨어(카메라 받침, 팔 장착부, 리프트)를 건드리면 이 값은 다시 측정해야 한다.

## 7. 측정 중 발견한 하드웨어·소프트웨어 문제

| 문제 | 내용 | 제안 |
|---|---|---|
| **자동 이동 중 안전 정지 2회** | 오류 메시지는 `SAMPLE_DISCONTINUITY`(27)지만, 이는 펌웨어가 다른 이유로 거부할 때 붙이는 대체 코드다(`binary_control.c`). 실제 원인은 dispatch `failure_count=1`, 즉 **모터에 명령을 보내는 중 UART 오류** → 안전 정지다. 읽기 실패는 0. 두 번 모두 오른팔이 움직이는 중이었다. 명령 경로는 연속이었다(한 틱 최대 1,745µrad, 한도 9,000) | 오른팔 케이블, 커넥터, 전원 점검. 진단에 어느 버스인지 남기면 원인 구분이 쉬워짐 |
| 오른팔 6개 모터 동시 무응답(여러 번) | 케이블을 다시 꽂아 복구(기존 C01) | 위와 같음 |
| 리프트가 토크 없이 천천히 내려옴 | S3 펌웨어는 리프트를 잡지 않음. 투어 후 약 1cm 하강 | 운용 중 리프트 유지 상태 확인 |
| 베이스 수리 후 바퀴 3개 무응답 | 배선 분리 | 수리 후 점검 항목에 추가 |
| 노트북 ↔ Jetson 접속 불가 | 공유기 2.4GHz / 5GHz 대역 간 격리(Jetson은 2.4GHz) | 같은 대역으로 접속 |

## 8. 펌웨어와 로봇 상태

- 측정 중 S3(0x24911)로 바꿨다가 **S4(0x2492D)로 복구했다**(`install.sh` 종료 코드 0).
- 백업은 Jetson에 있다.
  - S4 원본: `diagnostics/s3/stationary/s3-stationary-9d2fd662e718/flash/preflash-full.bin`, SHA256 `8dae4b9e…`
  - S3 상태: `diagnostics/s4-unified/s4-unified-e004bef4f622/install/`, SHA256 `380e78a7…`
  - 이전 설치 기록은 `*-before-20261116` 폴더로 보존
- 최종 펌웨어 확인과 S4 준비 절차(리프트 원점 등록 등)는 평소 방식으로 진행.

## 9. 남은 일

1. 후보 파일 적용 → 집기 전후 비교(4절)
2. 런타임 FK·IK에 중력 처짐 항 추가 → 처짐 모델 적용(약 5mm)
3. 서보 추종 오차 명령 보정
4. 리프트 배율 확정: 높은 높이 한 번 더 측정. 55cm는 팔이 카메라에 6cm까지 가까워져서 그 높이에서 손 훑기가 필요
5. 오른팔 원인 조사
6. 집게 끝 검증: 집게 끝 대기 때 실제 리프트 높이(약 55cm로 추정)를 확인해 재계산하거나, 실제 집기로 대체

## 부록 A. 데이터 위치 (저장소에 없음, 사람이 찍힌 사진 포함)

| 위치 | 내용 |
|---|---|
| 노트북·Jetson `~/cal_20261116/` | 원본 전체 |
| `~/cal_20261116/measurements.json` | 실측값(마커 66.2mm, 표적지, 체커보드 25.0mm, **리프트 최하단 raw 3900**, 휴식 자세, 펌웨어 교체 기록) |
| 노트북 `~/cal_work/calib_enc_20261117.json` | 최종 분석 결과(모델별 값, 교차 검증) |
| `T05b`, `T05d`, `T05f` / `T30` | 리프트 약 5cm(raw 1663 → 45.9mm) / 약 30cm(raw 1584 → 299.5mm) 자동 촬영 |
| `F-*`, `TB-*`, `TF-*` | 바닥·80cm 탁자(near), 하드웨어 수리 후 |
| `W2-*` | 광각 고정 노출 바닥·탁자(31.8cm 협탁 포함), 아직 분석에 안 씀 |
| `IN-*`, `W-IN-*` | 체커보드(렌즈). near fx 618.1 vs 공장 617.6 |
| `SW05-*` | 손 훑기(로봇 이동 안전 범위 기록) |
| `TCP-L-*` | 집게 끝 대기(검증 미완) |

## 부록 B. 다시 계산하기

```bash
PY=~/.venvs/calib/bin/python; A=tools/diagnostics/extrinsic_capture/analysis; C=~/cal_20261116
# 모델 4개 교차 검증 + 집게 끝
$PY $A/calib_encoder_lift.py . $C $C/T05b,$C/T05d,$C/T05f,$C/T30 ~/cal_work/calib_enc_20261117.json --cv 5
# 후보 설정 파일 (새 파일만 씀, 기존 파일은 건드리지 않음)
$PY $A/make_candidate_profile.py . $C $C/T05b,$C/T05d,$C/T05f,$C/T30 ~/cal_work/calib_enc_20261117.json "AL zeros + lift" config/home_assistant/candidates
```

분석 환경: `python3 -m venv ~/.venvs/calib && ~/.venvs/calib/bin/pip install numpy scipy opencv-contrib-python pillow`

## 부록 C. 이번에 만들거나 고친 도구 (`tools/diagnostics/extrinsic_capture/`)

| 도구 | 내용 |
|---|---|
| `analysis/calib_encoder_lift.py`, `encoder_lift_model.py` | 엔코더 리프트 높이, 리프트 배율·영점, 모델 비교, 집게 끝 |
| `analysis/make_candidate_profile.py` | 결과 → 후보 설정 파일 3개(검증 포함) |
| `analysis/calib_model.py` | 팔별 마커 id, 실측 마커 크기, 사진 비교 버그 수정 |
| `analysis/plan_tour.py` | `--rest-sweeps`, `--rest-own`, `--plan-lift` |
| `calib_tour.py` | `--only-arm`, 빈 폴더 재사용 |
| `sweep_log.py` | 범위 끝 즉시 경고음, 빈 폴더 재사용 |
| `live_cam.py`, `raw_shot.py` | 오늘 폴더, 포트 지정, 정지 잠금 중 읽기 허용(토크 꺼짐 검사 유지) |
| `platform_check.py`, `rest_check.py` | 시작 검사 항목 표시, 휴식 자세 실시간 안내 |
