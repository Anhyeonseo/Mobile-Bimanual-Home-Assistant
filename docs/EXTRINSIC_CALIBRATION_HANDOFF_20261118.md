# 외부 보정 현장 인계 — 2026-11-18

> 리프트 최상단에서의 점 이동 정확도 진단, 실효 리프트 높이 도출, 수납장 위 물체 집기 성공까지.
> 이전: [EXTRINSIC_CALIBRATION_HANDOFF_20261117.md](EXTRINSIC_CALIBRATION_HANDOFF_20261117.md) · 오늘 계획: [for_extrinsic_calibration.md](for_extrinsic_calibration.md)

## 목차

0. 결론 요약
1. 용어와 좌표
2. 실험 결과 (점 이동)
3. 원인 분석 — 왜 처음엔 안 맞았나
4. 상단 집기 — 실패 4회와 성공 1회
5. 도구와 데이터 위치
6. 실행 절차 (그대로 따라 하면 됨)
7. 남은 문제와 다음 작업 (우선순위)
8. 종료 시점 상태
9. 주의 — 오늘 숫자를 쓸 때

---

## 0. 결론 요약

| # | 결론 | 근거 | 확실도 |
|---|---|---|---|
| 1 | **카메라 외부 파라미터(11/17)는 수평 방향으로 정상** | 서로 다른 베이스 위치 6곳, 서로 다른 목표 자리에서 고정 집게 끝의 앞뒤·좌우가 육안상 거의 0 | 높음 |
| 2 | **상단에서 손끝이 낮던 주원인은 리프트 상단 등록값** | CAD `travel_m` 643.8mm로 등록 → 손끝 약 18.5mm 낮음. 모델 리프트만 바꾸면 오차가 비례해 줄어듦 | 높음 |
| 3 | **상단·팔을 편 자세에서는 "모델 리프트 630mm"로 위아래 ±4mm** | 3mm 위 목표 2회(-32, -39) | 중간(육안 측정) |
| 4 | **630은 리프트 실제 길이가 아니라 실효값** (리프트 + 처짐 + 그 자세의 팔 모델 오차) | 줄자 행정 635 → 639mm(재측정). 차이 9mm는 팔 쪽 몫 | 높음 |
| 5 | **팔꿈치를 접은 자세에서는 손끝이 모델보다 1.5–2cm 높다** | 집기 -37: 엔코더상 명령대로 갔는데 손끝이 지우개 윗면 근처에서 닫힘 | 중간 |
| 6 | **편 자세 + 집게 안쪽 기준 + 물체 높이 중앙 목표로 상단 집기 성공** | -40: 세운 지우개를 집어 6cm 들어 올림 | 1회 성공 |
| 7 | **프로필·펌웨어·정식 집기 프로그램에는 아무것도 반영하지 않음** | 630 등은 시험용 계획 스크립트 안에만 있음 | — |

**한 문장 요약:** 보정 자체(카메라)는 맞고, 리프트 상단 등록값과 팔 모델(처짐·관절 영점)이 자세에 따라 수 mm~2cm 오차를 만든다. 편 자세에서는 실효값 630mm로 집기까지 된다.

---

## 1. 용어와 좌표

| 용어 | 뜻 |
|---|---|
| base 좌표 | `base_link`. x = 로봇 앞, y = 로봇 왼쪽, z = 위. 수납장 윗면 ≈ z 733.8–736.9mm |
| 고정 집게 끝 | 고정 집게 메시에서 손가락 방향으로 가장 끝 꼭짓점 (모델: `fixed_vertices` 중 stock z 최대) |
| 움직이는 집게 끝 | 같은 정의, 집게 각도만큼 회전한 움직이는 집게 |
| 상단 등록 | 리프트가 기계적 최상단에 있을 때 현재 위치를 `travel_m`(643.8mm)으로 간주해 원점 등록 (`reference_supported_top`) |
| 모델 리프트 | MoveIt·IK 계산에 넣는 리프트 높이. **제어기 등록값과 별개로 계획에서만** 바꿨다 |
| 실효값 630mm | "이 높이로 계산하면 손끝이 맞는" 모델 리프트 값 |
| 편 자세 | 어깨 ≈ 0.8–1.1, 팔꿈치 ≈ −0.3–0, 손목 굽힘 ≈ 0.1–0.5 rad |
| 접은 자세 | 어깨 ≈ 1.4, 팔꿈치 ≈ −1.3, 손목 굽힘 ≈ 1.1 rad |
| 리프트 엔코더 | 리프트 서보 한 바퀴 안 raw(0–4095). 최상단 기준값 ≈ 1727–1882 (아래 3절) |

---

## 2. 실험 결과 (점 이동)

### 2-1. 조건

- 릴리스 `s4-unified-6624e5730336`, 펌웨어 0x2492e(149806), 프로필 `s2-dev-d705d5…`(wide 촬영 프로필), 기준 `tape-top-reference-20261117-preinstall-baseline.json`
- 리프트: 기계적 최상단 + 받침, 상단 등록(643.847mm)
- 목표: 수납장 위 검은 테이프. **목표점 = 테이프 픽셀 시선과, 테이프 주변 상판에 맞춘 평면의 교점** (테이프 자체 깊이는 쓰지 않음 → 어제 H1 회피)
- 기준점: 고정 집게 끝, 집게 닫힘
- 경로: 목표 위 7cm → 수직 하강 → 지정 높이에서 정지 → STOP

### 2-2. 결과표

| 실행 | 모델 리프트 | 목표(테이프 위) | 실측 | 위아래 오차 | 리프트 엔코더 | 비고 |
|---|---|---|---|---|---|---|
| -20 | 643.8 | 0 (접촉) | 파고들며 밀림 | 큼 | **2650** | 리프트가 최상단보다 약 19.5mm 아래였음 |
| -22 | 643.8 | 20 | 파고듦 | 큼 | 2650 | 같은 이유 |
| -26 | 643.8 | 20 | 1–2 | **≈18.5 낮음** | 1882 | 진짜 최상단 |
| -31 | 635 | 20 | ≈15 | **≈5 낮음** | 1750 | |
| -32 | **630** | 3 | ≈3 | **≈0** | 1766 | 육안 |
| -39 | **630** | 3 | 차이 ≈4 | **≈±4** | 1743 | 베이스 이동 후 재확인. 방향 미기록 |

- 모든 실행에서 **앞뒤·좌우는 육안상 거의 정확**.
- 실행 -28, -30 등 중간에 끊긴 실행은 7절 하드웨어 문제 참고.

### 2-3. 리프트 행정 줄자 측정

| 측정 | 결과 | 비고 |
|---|---|---|
| 1차 | 89.5 − 26.0 = **635mm** | 리프트 판 한 점 기준 |
| 2차 | **639mm** | 측정 위치·방법 차이 가능 |
| 공식값(84mm/rev × 30,717count) | 630mm | |
| CAD `travel_m` | 643.8mm | 상단 등록에 사용 |
| 11/17 보정 배율(−2.131e-5 m/count) | 654.6mm | **상단에서 틀림** (맞다면 손끝이 위로 떴어야 함) |

→ 줄자 ±3mm 수준이라 **정확한 행정은 체커보드로 다시 재야 한다**(7절 #1).

---

## 3. 원인 분석 — 왜 처음엔 안 맞았나

### 3-1. 손끝 높이를 정하는 것

```
손끝 = 리프트 높이(등록값) + 팔 모델(관절 각도 × 링크, 관절 영점) + 집게 형상 − 처짐
```

리프트 값은 **어깨가 붙은 높이**만 정한다. 같은 리프트에서도 팔 자세가 바뀌면 관절 영점·처짐 오차가 다른 방향으로 투영된다.

### 3-2. 오차 분해 (상단, 편 자세)

| 요소 | 행정 635 기준 | 행정 639 기준 | 방향 |
|---|---|---|---|
| CAD 643.8 − 실제 행정 = 리프트 몫 | ≈9mm | ≈5mm | 아래 |
| 나머지 = 처짐 + 편 자세의 팔 모델 오차 | ≈5mm | ≈9mm | 아래 |
| 합계 | ≈14–18.5mm | 같음 | |

630 = 실제 행정 − (처짐 + 팔 모델 오차). 그래서 **630은 편 자세에서만 정확**하다.

### 3-3. 11/17·오늘 오전 "1cm 아래 + 앞뒤 어긋남"의 원인

| 원인 | 크기 | 방향 |
|---|---|---|
| 상단 등록값 > 실제 행정 | 5–9mm | 아래 |
| 처짐·팔 모델 | 5–9mm | 아래 |
| 리프트가 최상단이 아닌데 상단으로 등록 (받침 위치) | 최대 ≈19.5mm | 아래 |
| 베이스가 밀린 뒤 예전 사진의 목표점 사용 (-23) | 수 cm | 엉뚱한 곳 |
| 어제: 테이프 픽셀 깊이로 목표 설정 (H1) | ≈7.5mm | 아래 + 앞뒤 |
| **앞뒤 어긋남** | — | **먼저 닿은 손끝이 남은 하강 명령으로 눌리며 미끄러진 결과** |

**검증된 진단:** 리프트 엔코더 값이 같은 위치(최상단)인지 매번 확인해야 한다. 1734(11/17), 1882, 1750–1768, 1727–1743은 서로 0–3mm. **2650은 19.5mm 아래**(받침 위치 문제)였다.

---

## 4. 상단 집기 — 실패 4회와 성공 1회

### 4-1. 물체와 목표 정의

- 흰 지우개 17.7 × 45.7 × 12.6mm (캘리퍼). 흰색이라 **D415 깊이가 깨짐**(빈 값·1m 이상 엉뚱한 값) → 컬러 윤곽 + 평면으로 위치를 구함.
  - 눕힌 경우: 윤곽을 (상판 + 두께) 평면에 투영 → 측정 17.8 × 44.3mm (실제 17.7 × 45.7)
  - 세운 경우: 윗면의 먼 쪽 모서리를 (상판 + 높이) 평면에 투영하고 깊이 절반만큼 앞으로 → 중심
- 집게: 왼쪽 집게 실측 패드 보정(raw 1926.5 → 5mm, 2144 → 30mm, 2366 → 60mm, 선형 보간). **열기 = 폭 + 22mm(최대 58), 닫기 = 폭 − 8mm(최소 5)**. 펌웨어 집게 추적 한계 0.15rad(≈13mm) 안.
- 경로: 집게 열기 → MoveIt으로 물체 위 6cm → 수직 하강 → 닫기(2초 유지) → 수직 6cm 들기(1초 유지) → STOP(들고 있음).

### 4-2. 시도별 결과

| 실행 | 물체 | 기준점 / 높이 | 팔 자세 (어깨/팔꿈치/손목굽힘) | 결과 | 원인 |
|---|---|---|---|---|---|
| -33, -34 | 눕힘 | 집게 끝 중간점 / 윗면−7.6mm | — | 시작 0.05초에 STOP | 경로 ≈1,860점 → 업로드 동안 호스트 신호 끊김(보드 정지 사유 1). **100ms 샘플(≈900점 이하)로 해결** |
| -35 | 눕힘 | 같음 | 1.42 / −1.29 / 1.10 | 못 잡음 | **예전 사진으로 계획**(사용자가 지우개를 돌림). 열림 39.7 < 실제 폭 44mm |
| -36 | 눕힘 | 같음 | 1.42 / −1.29 / 1.13 | 못 잡음 | 집게가 18° 기울어 **고정 끝이 움직이는 끝보다 7mm 높음** → 고정 집게가 윗면 높이에 머묾. 5축이라 수평 불가 |
| -37 | **세움** | 집게 끝 중간점 / 윗면−15mm | 1.42 / **−1.29** / 1.13 | 못 잡음, 지우개 1cm 밀림 | **접은 자세에서 손끝이 모델보다 1.5–2cm 높음.** 엔코더상 관절은 명령대로(어깨 1.2° 처짐 → 계산상 오히려 8mm 아래) |
| -38 | 세움 | 같음, 편 자세 선호 | 1.10 / −0.83 / 0.99 | 실행 안 함 | 실행 신호 직전 중단 |
| **-40** | **세움** | **집게 끝 중간점에서 손가락 따라 10mm 안쪽 / 물체 높이 중앙** | 0.76 / **−0.02** / 0.45 | **성공**. 집게 14.8mm에서 멈춤, 6cm 들어 올림 | 편 자세 + 깊은 삽입 |

### 4-3. 성공한 -40의 수치

| 항목 | 값 |
|---|---|
| 지우개 중심 (base) | (375.0, 148.2)mm, 상판 z 735.2mm, 윗면 780.9mm |
| 집게 방향 | 좌우로 닫힘(짧은 축 12.6mm) |
| 열기 / 닫기 목표 | 34.6 / 5.0mm → 실제 14.8mm에서 멈춤 |
| 기준점 높이 | 상판 위 22.8mm (= 45.7/2) |
| 가장 낮은 집게 | 상판 위 12.2mm |
| 기울기 | 21.6° |
| 시간 | 96.4초, 닫기 80.6초 |
| 리프트 | 엔코더 1727, 모델 630mm |

### 4-4. 집기에서 배운 것

1. **편 자세를 고르게 하라.** 계획에 테이프 시험 자세 선호 항(`REF_Q=[0.413,1.032,−0.239,0.127,1.505]`, `W_REF=3`)을 넣었다. 물체 자리(-40은 y=148mm)에 따라 편 자세가 쉽게 나오기도 한다.
2. **기준점을 집게 끝이 아니라 안쪽(10mm)에 두고 물체 높이 중앙을 목표로** 하면, 수 mm~1cm 오차에도 옆면을 깊게 감싼다(사용자 제안).
3. **납작한 물체는 현재 집게 형상으로 어렵다.** 고정·움직이는 집게 끝 높이 차 ≈7mm.
4. **사진을 찍은 뒤로는 물체·베이스를 절대 건드리지 않는다.** (-35 실패)
5. 깊이가 깨지는 물체(흰색·반짝임)는 컬러 기반 위치가 필요하다.

---

## 5. 도구와 데이터 위치

### 5-1. 도구 (저장소 밖, 커밋 안 됨)

위치: `/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation/workspaces/home-assistant/output/tape-tools-backup-20261118/`
계획 스크립트는 그 작업 공간(`codex/s4-local-integration`, HEAD `baf7062`)의 모듈(`tools.diagnostics.run_s3_stationary_lab.model`, `carry_moveit_session`, `carry_model` 등)을 import 한다. **이 저장소로 옮기려면 import 경로와 하드코딩된 경로 정리가 필요하다.**

| 파일 | 실행 위치 | 역할 | 비고 |
|---|---|---|---|
| `capture_tape_rgbd.py` | Jetson | 카메라만 RGB-D 3장, 깊이 노출 16ms → `npz`, `metadata.json`, `profile.json`, `report.json`, `photo.jpg` | 어제 Codex 로그에만 있던 것을 복원 |
| `freeze_tape27.py`(→`39`) | 노트북 | 검은 테이프 → 평면 교점 → `target.json` | 탐색 창 `gray[230:400,520:800]` 하드코딩. 출력 폴더 이름도 하드코딩(sed로 바꿔 씀) |
| `freeze_object_rgb.py` | 노트북 | 눕힌 흰 물체: 윤곽 → (상판 + 두께) 평면 → 중심·크기·짧은 축 | `CAPTURE U0 V0 U1 V1 THICK_MM` |
| `freeze_upright.py` | 노트북 | 세운 상자: 윗면 먼 모서리 → 중심·짧은 축 | `CAPTURE U0 V0 U1 V1 H_MM A_MM B_MM` |
| `run_tape_frozen_20261118.py` | Jetson `~/robot/diagnostics/` | **원본** 점 이동 실행기 | 왼팔 5축만, 그리퍼 고정 |
| `run_top_grasp.py` | Jetson `~/robot/diagnostics/` | 위 실행기 + **왼쪽 그리퍼 이동 허용**, 실패 시 `guard_last_observation` 기록 | `route.json`에 `kind: top_grasp_development` 필요 |
| `run_tape_frozen_top635.py` | Jetson | 상단을 635mm로 등록하는 시험본 | 등록 실패했으나 643.8에서도 같은 실패 → 원인 미확정 |
| `plan_tape_touch3mm_model630.py` | 노트북 | 테이프 위 3mm, 모델 리프트 630 | `python3 … OUT_DIR` |
| `plan_tape_hover2cm_*.py` | 노트북 | 20mm 위 정지 변형들 (643.8/635) | 출력 폴더 하드코딩 |
| `plan_top_grasp.py` | 노트북 | 집기(집게 끝 중간점 기준) | `OUT_DIR WIDTH_MM` |
| `plan_top_grasp_refpose.py` | 노트북 | + 편 자세 선호(`W_REF`) | |
| **`plan_top_grasp_inner.py`** | 노트북 | **+ 안쪽 기준점(`INNER_D_MM`), 높이 = 물체 중앙** | **-40 성공 버전.** `W_REF=3 INNER_D_MM=10 python3 … OUT_DIR 12.6` |

공통 설정(계획 스크립트): 모델 리프트 0.630, 속도 0.10rad/s, 가속 0.25, 저크 1.0, 100ms 샘플, 집게가 상판에서 4mm 이상, 쉬는 자세 그리퍼↔어깨 겹침 4쌍을 ACM에서 허용(아래 7절 #13).

### 5-2. 데이터

노트북: `…/home-assistant/output/`, Jetson 원본: `~/robot/diagnostics/` (같은 이름)

| 폴더 | 내용 |
|---|---|
| `tape-frozen-target-20261118-{14,24,27,29,32,39}` | 테이프 사진·목표 |
| `tape-frozen-execution-20261118-{26,31,32,39}` | **핵심 점 이동 기록**(643.8 / 635 / 630 / 630 재확인). `capture/anchor.json`, `route.json`, `contact-plan.json`, `state.json` |
| `tape-hover-arrival-rgbd-*`, `tape-arrival-rgbd-*` | 도착 사진 |
| `grasp-object-20261118-{33,36,37,38,40}` | 지우개 사진·`object.json`·`target.json` |
| `grasp-execution-20261118-{33..38,40}` | 집기 기록. **`-40` = 성공** |
| `grasp-arrival-rgbd-20261118-{35,36,37,40}` | 집기 후 사진 (`-40/photo.jpg`에 들린 지우개) |
| Jetson `torque-release-20261118-*.json` | 토크 해제 기록 |

사람이 찍힌 사진이 있으므로 저장소에 올리지 않는다.

---

## 6. 실행 절차 (그대로 따라 하면 됨)

경로 변수: `W=/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation/workspaces/home-assistant`, `B=$W/output/tape-tools-backup-20261118`, `PY=/home/hyper/robot/shared/venvs/s4-jazzy/bin/python`, `REL=/home/hyper/robot/releases/s4-unified-6624e5730336`, `BASE=/home/hyper/robot/diagnostics/tape-top-reference-20261117-preinstall-baseline.json`

### 6-1. 사람이 할 일 (매 시도 전)

1. 리프트 **기계적 최상단** + 받침. 왼팔·오른팔 받침 위.
2. (이전 시도가 STOP으로 끝났으면) 팔을 손으로 받치고 **모터 전원 OFF → 팔 원위치 → ON**.
3. **SWD 리셋**(sudo 비밀번호 필요, 사람이 입력):
   ```bash
   ssh -t hyper@192.168.35.2 'sudo sh /home/hyper/robot/releases/s1-inventory-reader-r2/reset.sh; sudo rm -f /run/user/1000/so101-backend.lock'
   ```
4. 이후 **물체·베이스를 건드리지 않는다.**

### 6-2. 자동 단계

```bash
# 1) 촬영 (카메라만)
ssh hyper@192.168.35.2 "$PY - ~/robot/diagnostics/<CAP>" < $B/capture_tape_rgbd.py
ssh hyper@192.168.35.2 "tar -czf - -C ~/robot/diagnostics/<CAP> ." | tar -xzf - -C $W/output/<CAP>

# 2) 목표 고정 (노트북, 작업 공간 루트에서)
python3 $B/freeze_upright.py output/<CAP> U0 V0 U1 V1 45.7 17.7 12.6     # 세운 상자
ssh hyper@192.168.35.2 "cat > ~/robot/diagnostics/<CAP>/target.json" < output/<CAP>/target.json

# 3) 읽기 전용 상태 확인: stop_latched, flags/reason=0, 토크, 관절 범위, 리프트 엔코더(≈1727–1882)
# 4) 토크가 남았으면 prepare_unified_controller.release_boot_supported (토크 끄기만, 사용자 승인)

# 5) 유지 실행 (백그라운드)
ssh hyper@192.168.35.2 "$PY ~/robot/diagnostics/run_top_grasp.py --release $REL --baseline $BASE \
  --frozen-capture ~/robot/diagnostics/<CAP> --output ~/robot/diagnostics/<RUN> --supported-empty-top --execute"
#    → state.json status == WAITING_CHECKED_MOVEIT_ROUTE (여기서부터 240초)

# 6) 계획 (노트북)
ssh hyper@192.168.35.2 "tar -czf - -C ~/robot/diagnostics/<RUN> capture" | tar -xzf - -C output/<RUN>
cp output/<CAP>/object.json output/<RUN>/capture/
source /opt/ros/jazzy/setup.bash
W_REF=3 INNER_D_MM=10 python3 $B/plan_top_grasp_inner.py output/<RUN> <짧은폭_mm>
ssh hyper@192.168.35.2 "cat > ~/robot/diagnostics/<RUN>/route.json" < output/<RUN>/route.json

# 7) 사용자 승인 후 실행 신호
python3 -c "import json,hashlib;print(json.dumps({'sha256':hashlib.sha256(open('output/<RUN>/route.json','rb').read()).hexdigest()}))" > output/<RUN>/execute.json
ssh hyper@192.168.35.2 "cat > ~/robot/diagnostics/<RUN>/execute.tmp && mv ~/robot/diagnostics/<RUN>/execute.tmp ~/robot/diagnostics/<RUN>/execute.json" < output/<RUN>/execute.json
```

점 이동(테이프)은 1)에서 `freeze_tape*.py`, 5)에서 `run_tape_frozen_20261118.py`, 6)에서 `plan_tape_touch3mm_model630.py output/<RUN>`을 쓴다.

### 6-3. 자주 나는 거부와 대처

| 메시지 | 원인 | 대처 |
|---|---|---|
| `exact unified controller identity and clear stop required` | STOP 걸림 (전원 재부팅 직후 첫 접속 시 UART 오류 포함) | SWD 리셋 |
| `fresh non-faulted boot required`, `unconfigured clean boot required` | 이번 부팅이 이미 "사용됨" | SWD 리셋 |
| `complete inventory required` | 모터 전원 꺼짐 | 모터 전원 ON |
| `expected torque already OFF` | 서보 토크 남음(리셋해도 유지) | `release_boot_supported`. 거부(reason 7)되면 모터 전원 OFF/ON |
| `expected locked, stopped, torque 0: (left, N)` | 쉬는 서보가 순간 speed 50 보고 | 손 떼고 몇 초 뒤 재시도(부팅이 사용되면 리셋) |
| `12-axis anchor failed … operational-limit` | 관절이 허용 범위 밖 | 해당 관절을 손으로 범위 안으로. 자주: 왼어깨 하한 833, 왼손목굽힘 상한 3180, 오른팔꿈치 상한 3001 |
| `Current full robot pose is in modeled collision` | 쉬는 자세 그리퍼↔어깨 모델 겹침 | ACM 쌍 허용(현재 스크립트에 포함) 또는 자세 조정 |
| `PermissionError …so101-backend.lock` | sudo 리셋이 잠금 파일을 root로 만듦 | `sudo rm -f /run/user/1000/so101-backend.lock` |
| `lift rejected or MCU restarted` | 상단 등록 간헐 실패 | 리셋 후 재시도 |
| 실행 중 `LIFT_BAD_FEEDBACK` / `heartbeat failed` | 리프트 1count 흔들림 / 긴 경로 | 재시도 / 경로 점 수 줄이기 |

---

## 7. 남은 문제와 다음 작업 (우선순위)

### 보정

| # | 작업 | 이유 | 방법 |
|---|---|---|---|
| 1 | **리프트 배율·행정 확정** | 줄자 635/639 흔들림. 11/17 배율은 상단에서 틀림. 중간 높이 정확도 미확인 | 계획서 D. 체커보드(`docs/calibration_checkerboard_A4.pdf`, 25mm 칸)를 휘지 않는 판에 붙여 리프트 판에 고정 → 토크 OFF로 손으로 35–40mm씩 올리며 raw + `camshot` → `lift_steps.py`. 반 바퀴(42mm) 미만 단계 |
| 2 | **팔 모델 자세 의존 오차** | 편 자세 ±4mm, 접은 자세 1.5–2cm | 계획서 C(손끝 접촉 피벗)를 **접은 자세 포함**해 수행. 손목 회전 영점·팔꿈치 영점·집게 형상 재추정 |
| 3 | 처짐 런타임 반영 | 처짐이 자세마다 다름 | `MeasuredFK`·계획 모델에 처짐 항 |
| 4 | 프로필 반영 | 지금은 시험 스크립트에만 630 | 1–3 결과로 `travel_m`, `meters_per_count` 갱신 → 새 `calibration_id` → Jetson `-cal1118`. **펌웨어 상단 등록값도 함께** |
| 5 | 최하단 + 낮은 탁자(E1) 재확인 | 낮은 높이는 11/17 값으로 맞는지 오늘 미확인 | 같은 3mm 위 시험 |

### 집기

| # | 작업 |
|---|---|
| 6 | 상단 집기 반복(같은 조건 3회 이상, 자리 바꿔서). 오늘 성공 1회 |
| 7 | 접은 자세가 필요한 자리용 보정(#2 결과) 또는 계획에서 편 자세 강제 |
| 8 | 납작한 물체 집기(집게 끝 높이 차 7mm 대책: 손목 회전 자유도, 다른 접근) |
| 9 | 정식 집기 프로그램(`unified_pick`, `pick_service`)은 리프트 0–10mm만 허용 → 상단 집기·모델 리프트 보정을 정식 경로로 |
| 10 | 흰색·반사 물체의 깊이 결손 대응(컬러 기반 위치 추정 일반화) |

### 하드웨어·운영 (현서에게)

| # | 문제 | 관찰 |
|---|---|---|
| 11 | 리프트 유지·등록 간헐 실패 | raw 1count 흔들림 + speed 50 → `LIFT_BAD_FEEDBACK`(-28 이동 37초에 정지), 상단 등록 거부(-27, -30). 11/17 리프트 콘솔 실패와 같은 패턴 |
| 12 | 전원 재부팅 직후 첫 접속에서 STOP | `BinaryControl_HandleHostUartError`, rejected_frame 2–3. **SWD 리셋으로는 안 생김** |
| 13 | 쉬는 자세 그리퍼↔어깨 모델 겹침 | 양팔. 오늘은 ACM 허용(왼팔은 이동 중에도 검사 꺼짐 — 정식 도구에서는 형상·쉬는 자세 정리 필요) |
| 14 | OpenOCD 리셋에 sudo 필요 | ST-LINK udev 권한 없음. 규칙 추가 시 잠금 파일 문제도 해결 |
| 15 | MCU 리셋 후 서보 토크 유지 | 매번 해제 필요. 팔이 눌린 상태면 해제 거부(reason 7) → 그 부팅은 사용됨 |
| 16 | 시작 검사의 speed 50 잡음 | 쉬는 서보가 가끔 보고 → 시작 거부 |
| 17 | 받침 위 쉬는 자세가 관절 범위 경계 | 왼어깨 833, 왼손목굽힘 3180, 오른팔꿈치 3001 → 받침 높이 조정 |

---

## 8. 종료 시점 상태

- 로봇: 실행 `grasp-execution-20261118-40` 종료 후 **STOP 상태로 지우개를 든 채 유지**. 내려놓으려면 지우개를 받친 뒤 모터 전원 OFF.
- 리프트: 최상단(엔코더 1727), 받침 있음.
- 베이스: 오늘 여러 번 움직였음. **다음 시험은 반드시 새 사진부터.**
- 이 저장소 변경: 이 문서만. 노트북 `/tmp`의 어제 스크립트는 5-1 폴더에 백업함.

---

## 9. 주의 — 오늘 숫자를 쓸 때

- **630mm는 "상단 + 편 자세 + 수납장 높이(≈73cm)"에서만 검증한 실효값**이다. 다른 리프트 높이·자세에 그대로 쓰지 않는다.
- 위아래 오차는 **육안·자 측정**이다(±2–3mm 불확실).
- 줄자 행정 635/639는 측정 방법 차이가 있다. 프로필 값으로 쓰지 말고 체커보드로 확정한다.
- 집기 성공은 **1회**다. 계획서 기준(같은 조건 3회 이상)으로 판정하지 않았다.
- 시험용 실행기(`run_top_grasp.py`)는 왼쪽 그리퍼 이동을 허용하고, 계획은 일부 충돌 쌍을 허용한다. **정식 도구로 승격하기 전에 안전 검사 검토가 필요하다.**
