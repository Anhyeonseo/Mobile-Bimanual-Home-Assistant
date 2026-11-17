# 카메라·팔·리프트 보정 인계 (2026-11-16~17)

대상: 집기·좌표 코드 담당 · 작성: 여은 · 브랜치 `yeoeun/extrinsic-calibration`
측정 경과: [결과 기록](CALIBRATION_RESULTS_20261117.md) · 절차: [현장 가이드](CALIBRATION_FIELD_GUIDE_20261116.md) · 이전: [11/13 인계](EXTRINSIC_CALIBRATION_HANDOFF.md)

## 1. 결론

- 토크 ON 자동 투어로 리프트 2개 높이에서 120장을 촬영했다(재투영 RMS 1.1–1.6px, 11/13은 4–5px). 카메라 6자유도, 양팔 관절 영점, 오른팔 장착, **리프트 배율·영점**을 동시에 추정했다.
- 계통 오차의 주원인은 리프트 환산이다. 런타임은 `h = (raw − origin) × meters_per_count`(84mm/rev)로 계산하는데, 실제 리프트는 **+3.4~5.1% 더 이동**하고 **영점이 −6~−16mm**다. 팔이 프로필보다 낮게 위치하므로, S3에서 관찰한 "목표보다 약 8mm 아래"와 방향이 일치한다.
- 카메라 x·y·yaw는 이번에 식별됐다. 모델 간 편차가 x·y 약 5mm, yaw 0.5°다(11/14는 x가 10–29mm 범위). 기준은 왼팔 CAD 장착과 왼팔 base 영점이며, 바퀴 기준 절대값은 측정 불가다.
- 런타임 필드에 그대로 들어가는 모델(영점 + 장착 + 리프트, 처짐 없음)로 **후보 프로필 3개**를 생성했다(`config/home_assistant/candidates/`). `s2_development.validate` 통과, 런타임 식으로 재계산한 TCP가 보정 모델과 **≤0.24mm** 일치한다. **기존 프로필은 변경하지 않았다.**
- 마커 기준 5겹 교차 검증 중앙값은 리프트 미보정 15.8mm → **후보 10.5mm** → 처짐 포함 5.1mm(런타임 미지원)다. 화면 평면(lateral) 오차는 0.7–3mm다.
- 실물 집기 검증은 아직 안 했다.

## 2. 후보 프로필 변경 내용

| 필드 | 현재 | 후보 |
|---|---|---|
| `base_from_optical` pitch / roll / yaw | 45.00 / 0.00 / −0.68° | 45.78 / −1.31 / +0.30° |
| `base_from_optical` t (mm) | 100.7, 33.2, 1007.1 | 121.9, 34.8, 1008.0 |
| `camera_height_adjustment` (−9.1mm) | 있음 | 삭제(validator가 nominal+adjustment = 카메라 일치를 요구) |
| left `home_model_rad` Δ (shoulder/elbow/wrist_flex) | 0 | +5.37 / +0.91 / −0.03° |
| right `home_model_rad` Δ | 0 | +7.69 / +3.18 / −4.64° |
| `reference_model_rad` | – | 위와 같은 Δ(validator: `model_angles(reference_raw) == reference_model_rad`) |
| right `base_from_arm_at_lift_zero` | CAD | `CAD @ rm`, rm t ≈ (10.2, 1.2, −1.2)mm, 회전 ≈ 3° |
| 양팔 `base_from_arm_at_lift_zero` t | – | `+= axis_base × (−5.74mm)` (리프트 영점) |
| `lift.meters_per_count` | −2.0508e-05 | −2.1310e-05 (×1.0391) |
| `calibration_id` | `s2-dev-5bfc28c62078dfb0377bfdb7` | `s2-dev-74cf69505343b9cef8a76199` |
| wide `base_profile_id` / `calibration_id` | – | 위 id 기준으로 재계산 |
| `camera_height` `base_z_from_optical`, `model_id` | 11/14 높이 행 | 후보 카메라 z행, `height-` id 재계산 |

- base 영점은 게이지라 변경하지 않는다(오른팔 base 영점은 `rm`에 포함).
- `source_files_sha256`은 그대로다. `released_profile()`은 파일을 그대로 읽고 소스 해시만 확인하므로 로드된다. **단, `installation.development.json` 등 소스에서 프로필을 다시 빌드하면 보정값이 사라진다.** 영구 반영하려면 소스 쪽에도 같은 값을 넣는 별도 작업이 필요하다(6절).
- 승인 플래그(`coordinate_use_allowed`, `motion_authorized` 등)는 `false` 그대로다.

## 3. 적용 범위

**바꾸는 것은 설정 파일 3개뿐이다.** 코드와 문서에 값을 옮겨 적을 필요는 없다. 아래 모듈이 모두 같은 프로필을 읽으므로, 교체하면 자동으로 반영된다.

| 영역 | 프로필 사용처(예) | 영향 |
|---|---|---|
| 집기 좌표·계획 | `unified_pick.py`, `carry_live_inputs.py`, `carry_scene*.py`, `stationary_pick.py` | 물체 → base 변환, 팔 FK·IK, 리프트 높이 |
| 높이 판정 | `stationary_rgbd.load_surface_height_model` (`camera_height.calibrated.json`) | 탁자·물체 높이 |
| 센서 TF | `s4_sensor_frames.py`(base_link → color optical) | 주행·지도의 RGB-D 투영 |
| 정렬 후보 | `assistant/object_alignment.py` | 리프트 높이별 도달성 |

**교체 시 함께 처리할 것**
- `calibration_id`가 바뀌므로, 이전 id에 바인딩된 산출물은 재생성해야 한다. 그대로 쓰면 거부된다.
  - 저장된 집기 bundle(`--bundle`), `installed_jaw` 기록, `mission_catalog` 대상, `mobile_manipulation` 프로그램
- 기존 지도(RGB-D 융합층)는 이전 카메라 TF로 만들어졌다. 카메라 앞뒤가 약 2cm, roll이 1.3° 바뀌므로 RGB-D 장애물·시멘틱 층에 cm 단위 차이가 생길 수 있다. 라이다 2D 지도는 무관하다.
- 리프트 원점은 지금처럼 **최하단 등록**을 전제로 한다.
- **기존 문서는 수정하지 않는다.** 예전 값을 인용한 문서(`CALIBRATION_GUIDE.md` 12.3, 이력 문서)는 그 시점의 기록이다. 적용한 뒤에는 `CURRENT_STATUS.md`의 좌표·높이 항목만 갱신하면 된다.
- 카메라 값만 따로 가져가면 안 된다. 카메라, 팔 영점, 장착, 리프트는 함께 추정된 묶음이다. 11/14에는 카메라만 바꿔서 왼팔 오차가 11.1mm로 악화된 적이 있다.

## 4. 적용과 검증 절차

```bash
cd config/home_assistant
F="s2_coordinates.development.json s2_coordinates.wide.development.json camera_height.calibrated.json"
for f in $F; do cp "$f" "$f.before-20261117"; done                    # 백업
cp candidates/s2_coordinates.development.candidate-20261117.json      s2_coordinates.development.json
cp candidates/s2_coordinates.wide.development.candidate-20261117.json s2_coordinates.wide.development.json
cp candidates/camera_height.calibrated.candidate-20261117.json        camera_height.calibrated.json
# 롤백: for f in $F; do cp "$f.before-20261117" "$f"; done
```

Jetson release에도 같은 세 파일을 배포한 뒤, 이전 id에 묶인 산출물을 재생성한다.

| 검증 | 기준 |
|---|---|
| 적용 전 기준 기록 | 현재 프로필로 수납장 흰 리모컨 정지 집기. 접촉 위치 오프셋(상하·좌우, mm)을 사진이나 자로 기록 |
| 적용 후 같은 조건 | 상하 오프셋 감소 여부(목표: 아래쪽 치우침 제거) |
| 반복 2–3회 | 산포 |
| 리프트 상승 상태(80cm 책상) | 리프트 배율 보정의 효과 |
| 왼팔 우선 | 오른팔은 신뢰도가 낮음(5절) |

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

1. 후보 적용 → 집기 전후 비교
2. 런타임 FK(`MeasuredFK`, 계획 모델)에 처짐 항 추가 → 처짐 모델 적용
3. 명령 보정: `settle_error_raw`를 관절별 중력 토크에 회귀시켜 목표각에 오프셋 적용
4. 소스(`installation.development.json`, `arm_model_alignment.development.json`, `arm_mounts`, 리프트) 반영 → 재빌드 시에도 유지되게
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

기준 문서: `history/PROJECT_HISTORY_AND_DECISIONS_2026-11-15.md`(TalkFile)의 항목 번호. "적용 시"는 후보 프로필을 교체했을 때를 뜻한다.

| 항목 | 문제 | 이번 결과 | 상태 |
|---|---|---|---|
| H01 | 80.91 vs 80cm 한 점으로 Z −9.1mm 보정 | −9.1mm는 약 1° pitch 오차를 한 거리에서 상쇄한 값으로 확인. 카메라 6자유도 실측으로 대체, 후보에서 `camera_height_adjustment` 삭제 | 해결(적용 시) |
| H03 | 높이 성분만 사용, 카메라 XY·yaw·팔 보정 미적용 | 카메라·영점·장착·리프트 단일 묶음 + 같은 카메라에서 만든 높이 모델 | 해결(적용 시) |
| §1 S2 | 외부 보정 실측 정확도 미보증 | x·y·yaw 식별(모델 간 5mm·0.5°), 교차 검증 수치화(lateral 0.7–3mm) | 대부분 해결 |
| G06 | S3 약 8mm 아래 접촉, 원인 미확정 | 리프트 환산 오차(+3.9%, −5.7mm, 팔이 프로필보다 낮음)와 서보 추종 오차(어깨 약 1°)를 실측. 둘 다 아래 방향 | 원인 확인, 실물 검증 대기 |
| G01 | Home 영점 정확도 미검증 | 양팔 영점 보정값 실측. 처짐 포함 여부에 따라 수 도 차이 | 부분 |
| G05 | 손가락이 탁자를 끎, 모델·실물 높이 차이 | 높이 차이의 주요인(리프트) 확인 | 관련 원인 해소 |
| H04 | 다른 바닥 영역 −10.7mm 잔차 | 전체 카메라 자세 보정 후 near 평면 4mm 이내 | 개선(광각 재확인 필요) |
| H06 | 광각 실측 미완 | 광각 후보 프로필 생성(외부 보정 동일). 광각 렌즈 왜곡은 촬영만 하고 미분석 | 부분 |
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
$PY $A/make_candidate_profile.py . $C $T ~/cal_work/calib_enc_20261117.json "AL zeros + lift" config/home_assistant/candidates
```

환경: `python3 -m venv ~/.venvs/calib && ~/.venvs/calib/bin/pip install numpy scipy opencv-contrib-python pillow`.
도구 변경 내역은 [결과 기록 7절](CALIBRATION_RESULTS_20261117.md)과 `tools/diagnostics/extrinsic_capture/README.md`에 있다.
