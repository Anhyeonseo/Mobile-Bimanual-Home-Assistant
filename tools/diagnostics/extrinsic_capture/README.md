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
