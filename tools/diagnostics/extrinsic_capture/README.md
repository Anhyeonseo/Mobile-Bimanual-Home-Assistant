# D415 외부 보정 촬영·분석 도구

고정 D415(앞쪽 더듬이 장착)의 `base_link ← color optical` 변환을 실측 데이터로 추정하는 도구.
배경과 결과는 [인계 문서](../../../docs/EXTRINSIC_CALIBRATION_HANDOFF.md), 촬영 절차는
[촬영 가이드](../../../docs/EXTRINSIC_CALIBRATION_GUIDE.md)를 본다.

## 촬영 도구 (Jetson에서 실행)

모두 읽기 전용이다. 모터 토크를 켜거나 움직이지 않는다.

| 파일 | 역할 |
|---|---|
| `helpers.sh` | `source` 하면 실행 폴더로 이동하고 아래 명령을 등록한다. 릴리스 경로가 바뀌면 이 파일의 `cd` 경로를 고친다 |
| `live_cam.py` (`livecam`) | 브라우저 실시간 화면(ArUco 검출 표시) + 터미널에서 이름 입력으로 저장. `NAME` = 카메라만, `NAME LIFT_M` = 관절값 포함 |
| `raw_shot.py` (`rawshot`) | 사진 + 양팔 12축·리프트(ID10) 원시 엔코더. 기록 범위 검사를 건너뛴다 |
| `cam_shot.py` (`camshot`) | 카메라만 (바닥, 탁자, 체커보드) |
| `pose_check.py` (`pose`) | 각 관절이 `arm_ranges.development.json` 범위 안인지 출력 |

`raw_shot.py`와 `live_cam.py`는 `stationary_pick.py capture`의 범위 검사 대신
촬영 전후 엔코더를 읽어 3 count 넘게 움직였으면 거부한다. idle reader는 시작하지 않는다.
양팔 토크가 켜져 있으면 거부한다.

Jetson 사용 예:

```bash
scp tools/diagnostics/extrinsic_capture/{*.py,helpers.sh} hyper@<jetson>:~/cal_YYYYMMDD/
# Jetson에서
source ~/cal_YYYYMMDD/helpers.sh   # helpers.sh 안의 CAL 경로도 맞춘다
livecam
# PC에서 실시간 화면
ssh -N -L 18765:127.0.0.1:8765 hyper@<jetson>   # 그다음 http://127.0.0.1:18765
```

## 저장 형식

```
<CAL>/<NAME>/color.png, depth_aligned.png, metadata.json, record.json, joints.json   # 관절 포함
<CAL>/<NAME>/cam/color.png, depth_aligned.png, metadata.json, record.json             # 카메라만
```

`joints.json`: `raw_before`/`raw_after`(양팔 `left/<joint>`, `right/<joint>`, `lift/id10`),
`lift_m_operator`(사람이 입력한 리프트 높이), 제어기 HELLO, 전체 inventory 행.

## 분석 (PC에서 실행, 저장소 루트에서)

필요 패키지: `numpy scipy opencv-contrib-python pillow`

| 파일 | 입력 | 출력 |
|---|---|---|
| `analysis/cb_check.py DATA OUT` | `CB-*` | 체커보드 검출, 깊이 유효율, 모아보기 이미지 |
| `analysis/cb_depth.py DATA` | `CB-*` | 깊이 vs 체커보드 PnP 거리 차이, 깊이로 잰 칸 크기 |
| `analysis/planes.py DATA` | `F-*`, `TB-*`, `TF-*` | 바닥·탁자 평면으로 카메라 높이·pitch·roll |
| `analysis/marker_check.py DATA OUT [PREFIX]` | `L1-*` 등 | 마커 검출, 크기, 기울기, 화면 분포 |
| `analysis/fit.py . DATA OUT.json` | 왼팔 `L*` | 왼팔만으로 카메라(A), 카메라+리프트(B) 추정 |
| `analysis/fit_all.py . DATA OUT.json` | 양팔 전체 | 카메라 + 부착 묶음별 마커 위치 + 리프트 높이 + 오른팔 받침 보정, 묶음별 오차 |
| `analysis/fit_tilt.py . DATA OUT.json` | 양팔 (RB, RC 제외) | 위 + 마커 70mm 고정 + 리프트 축 기울기 |

`.`은 저장소 루트(= `config/home_assistant/s2_coordinates.development.json`과
`official_geometry.json`이 있는 곳)다. 분석은 이 프로필의 FK, 팔 장착, 리프트 축을 사용한다.

주의:

- 묶음 이름 규칙(`L`, `R3`, `RB2`, `RC`/`C-01`, `RD`)과 리프트 높이 대응이 `fit_all.py`의
  `group_of()`와 `OP`에 하드코딩되어 있다. 새 촬영에는 맞게 고친다.
- `planes.py`의 `T`는 2026-11-13 프로필의 `base_from_optical` 값이다.
- 오차 지표는 마커 중심의 카메라 PnP 위치와 FK 예측 위치의 거리다. 매 5번째 사용 가능 촬영을
  검증용으로 빼고(held-out) 나머지로 맞춘다.

## 자동 촬영 투어 (카메라 + 팔 동시 보정용)

로봇 몸체 충돌 모델이 없으므로, 팔은 **사람이 토크를 끈 채 손으로 지나가 본 공간 안에서만** 움직인다.

| 단계 | 어디서 | 도구 | 내용 |
|---|---|---|---|
| 1 | Jetson | `sweep_log.py` (`sweep`) | 토크 끈 채 한 팔씩, 리프트 높이마다 손으로 천천히 훑기. 관절·리프트를 ~25 Hz로 기록. `--camera`면 0.7초 멈출 때마다 사진 |
| 2 | PC | `analysis/plan_tour.py` | 훑은 기록으로 경로 지도(roadmap)를 만들고, 보정에 가장 도움되는 자세를 골라 순서·궤적을 만든 뒤 펌웨어 규칙(범위·속도·엔코더 경계)으로 검증 |
| 3 | Jetson | `calib_tour.py --simulate` | 같은 투어 파일을 가짜 컨트롤러로 먼저 돌려보기 |
| 4 | Jetson | `calib_tour.py` (`tour`) | 리프트 높이 하나씩 실행. 이동마다 Enter, `a` 자동, `q` 검증된 경로로 복귀 후 종료 |
| 5 | PC | `analysis/joint_calib.py` | 새 폴더와 11/13 폴더를 쉼표로 함께 넣어 보정 |

훑기 요령: 휴식 자세에서 시작해 휴식 자세로 끝낸다. 카메라 앞 작업 공간을 넓게(아래 관절 좌우, 위아래, 손목 굽힘과 회전) 천천히. 다른 팔은 휴식 자세에 둔다. 휴식 자세는 관절 범위 안쪽이어야 한다(11/13 왼팔 휴식 자세는 어깨·팔꿈치가 범위 밖이었다). 투어는 훑을 때와 같은 리프트 높이, 같은 주변 상태에서만 실행한다.

안전: 펌웨어 정지는 **토크를 끈다**(팔이 떨어진다). Ctrl+C는 즉시 정지다. 투어 중에는 사람이 팔 옆에 있는다. 대기 중에도 하트비트는 계속 보낸다. 실제 첫 실행 전에 이동 코드 담당자 검토를 받는다.

```bash
# Jetson (release 폴더에서, source helpers.sh 후)
sweep SW20-L left 0.20 --camera --attachment TL1      # 왼팔, 리프트 0.20
sweep SW20-R right 0.20 --camera --attachment TR1
# PC
A=tools/diagnostics/extrinsic_capture/analysis
python3 $A/joint_calib.py . ~/cal_20261113 calib.json                        # 현재 정보량(모델 C)
python3 $A/plan_tour.py . ~/cal_20261113 calib.json tour.json --sweeps ~/cal_X/SW20-L ~/cal_X/SW20-R
# Jetson
python calib_tour.py tour.json /tmp/rehearsal --lift-m 0.20 --left-attachment TL1 --right-attachment TR1 --simulate
tour tour.json T20 0.20 TL1 TR1
# PC
python3 $A/joint_calib.py . ~/cal_20261113,~/cal_X calib2.json --cv 5
```

| 파일 | 역할 |
|---|---|
| `analysis/calib_model.py` | 카메라 + 팔(관절 0점, 링크 길이, 중력 처짐, 오른팔 받침) 공통 모델, 여러 촬영 폴더 로드 |
| `analysis/joint_calib.py` | 보정, 식별 가능성(SVD), 불확실성, 교차 검증, 부트스트랩 |
| `analysis/design_poses.py` | A-optimal 촬영 자세 설계 |
| `analysis/tour_safety.py` | 훑은 공간 기반 경로 지도와 안전 검사 |
| `analysis/plan_tour.py` | 투어 계획·궤적 생성·펌웨어 규칙 검증 |
| `sweep_log.py`, `calib_tour.py` | Jetson 기록 / 자동 촬영 |
