# GraspNet 없는 SO101 파지 실험

`home_perception/grasp/so101.py`는 GraspNet 출력·가중치 없이 목표 점군으로 접촉 후보를 만든다. 기존 GraspNet 경로와 기본값은 비교 기준으로 보존한다. 하드웨어 명령을 보내지 않는다.

## 현재 구현

- 물체의 관측 윤곽에서 짧은 축과 대안 방향을 계산한다.
- 방향 4개 × 위치 3개 × 접근 기울기 3개 × 고정 손가락 방향 2개로 최대 72개 후보를 평가하고 기본 24개를 반환한다.
- 각 접촉선 주변에서 관측 폭을 다시 측정한다. 실제 그리퍼 폭 범위와 4 mm 진입 여유를 만족해야 한다.
- 접촉선의 low 쪽을 고정 손가락, high 쪽을 움직이는 손가락으로 명시한다. 양방향 후보는 서로 다른 배치이다.
- 기존 `observed_contact_line` 및 `jaw_closure` 규약을 사용한다. GraspNet과 같은 출력 좌표 규약을 쓰는 것은 인터페이스 호환 목적이며, 모델을 실행한다는 뜻이 아니다.
- 후보 생성 시간, 탈락 원인, 접촉 위치를 기록한다.

## 이미지 실행

입력 폴더에는 `color.png`, `depth_aligned.png`(16-bit), `metadata.json`이 필요하다. 메타데이터는 기존 D415 snapshot 형식이며 깊이 스케일과 정렬된 카메라 내부 파라미터를 포함해야 한다. RGB 사진만으로는 실제 미터 단위 파지 위치를 검증할 수 없다. 마스크는 원본 해상도의 0/255 단일 채널 PNG이다.

```bash
python3 tools/diagnostics/preview_so101.py \
  --snapshot /path/to/snapshot \
  --mask /path/to/mask.png \
  --output-dir /tmp/so101-preview
```

새 출력 폴더에 `candidates.json`, `contacts.png`를 생성한다. 빨강은 고정 손가락 쪽, 청록은 가동 손가락 쪽의 관측 접촉점이다. 그림은 최상위 후보만 표시한다. 전역 평면을 사용한 관측 미리보기이며 테이블과 장애물 충돌 검증이 아니다.

이미 해시로 연결된 탐지·분할 결과가 있으면 기존 재생 도구에서도 선택할 수 있다.

```bash
python3 tools/diagnostics/replay_s3_grasp.py \
  --snapshot /path/to/snapshot \
  --detections /path/to/detections.json \
  --label remote --candidate-source so101 \
  --output /tmp/so101-replay.json
```

## 아직 검증하지 않은 부분

관측된 상부 윤곽은 실제 손가락 접촉면이나 힘 평형의 증거가 아니다. 현재 생성기는 움직이는 손가락의 회전 궤적 전체, 마찰, 물체 밀림, 팔 IK, 접근·인양 경로 충돌을 검증하지 않는다. 도달 가능성 지도와 시뮬레이션 학습도 아직 구현하지 않았다. 실제 장면 없이 성공률이나 GraspNet 대비 속도 향상을 주장할 수 없다.

다음 단계는 기존 CAD 및 설치 패드 모델로 고정 접촉면에 TCP를 배치하고, 개방각부터 접촉각까지 가동 손가락의 swept volume을 검사하는 것이다. 그 다음 사전 계산 FK 지도를 IK 초기값 조회에 사용한다. 지도 조회만으로 경로의 안전성이나 파지 가능성을 확정하지 않는다.

## 대안과 평가

- GG-CNN: 깊이 영상 기반의 경량 파지 예측 대안. 비대칭 손가락 접촉과 SO101 운동학은 별도로 처리해야 한다. https://github.com/dougsm/ggcnn
- Contact-GraspNet: 접촉 기반 6D 파지 예측 대안. SO101 전용 손가락 모델 및 도달 가능성 검증을 대체하지 않는다. https://github.com/NVlabs/contact_graspnet
- 현재 기하 방식 + 소형 후보 점수 모델: 전용 후보와 실제·시뮬레이션 성공 데이터를 확보한 뒤 검토한다.

동일 RGB-D에서 전처리, 후보 생성, IK, 경로 검사 시간을 각각 측정하고 후보 수·실행 가능 비율을 기록한다. 첫 실행과 반복 실행을 구분한다. 실제 성공률은 같은 물체·배치에서 들어 올린 뒤 유지한 횟수로 따로 측정한다.

검증: `python3 -m pytest -c config/pytest.ini tests/test_so101_candidates.py tests/test_grasp_proposals.py -q`. 합성 직육면체, 비어 있는 점군, 폭 초과, 진입 여유 부족, 깊이 누락 및 잘못된 입력을 검사한다.

## 동일한 로봇 검사로 비교하기

일반 Ubuntu 터미널에서 실행한다. 기존 PyTorch 환경과 ROS2 Jazzy/MoveIt을 사용하는 오프라인 계획이며 하드웨어 제어 도구를 호출하지 않는다.

```bash
cd /home/an-hyeonseo/Mobile-Bimanual-Home-Assistant-edit
bash tools/diagnostics/run_grasp_feasibility.sh --maximum-candidates 8 --repeats 3
```

`output/so101_photo_experiment/fair-comparison/summary.json`에 요약이 저장된다. 각 세트·방식 아래 `prepared.json`, `validation.json`, `validation/attempts.jsonl`에는 입력 후보와 후보별 접촉·IK·경로 검사 결과가 있다. 기존 출력 폴더가 있으면 덮어쓰지 않는다.

기본 모드는 각 방식의 상위 8개 후보에 대해 원래 방향과 반대 고정 손가락 방향을 순서대로 검사하고, 처음 완전한 접근·닫힘·인양·운반 경로를 찾으면 종료한다. 최대 16개 방향별 후보이다. 중간에 종료하면 전체 후보 통과율은 `null`이다. 전체 통과율이 필요하면 다음처럼 별도 폴더에서 실행한다. 경로 계획 때문에 시간이 더 걸린다.

```bash
bash tools/diagnostics/run_grasp_feasibility.sh \
  --maximum-candidates 8 --repeats 3 --exhaustive \
  --output-dir output/so101_photo_experiment/fair-comparison-all
```

비교 조건:

- GraspNet은 GPU에 모델을 한 번 로드하고 재사용한다. `audit_enabled=False`로 일반 그리퍼 비교용 검사만 제외한다. 물체 연결과 후보 정제는 유지한다.
- 두 방식은 동일한 깊이·마스크·평면과 동일한 정제 점군을 사용한다. 기본 32개를 생성한 뒤 같은 상한으로 검사한다.
- 과거 실험의 3 cm 수동 접촉 이동은 새 비교에는 적용하지 않는다.
- 같은 `common_sections`, `bind_grasp`, `plan_pick_carry`, `MoveItCarryPlanner`를 사용한다. 점수의 수치는 서로 비교하지 않는다.
- 촬영 당시의 profile을 `capture-model/model.json`에서 읽고 bundle 해시를 검사한다. 당시 관절 범위는 Git 이력에서 복구한 `capture-model/ranges.json`으로 모델 기록의 해시와 일치한다. 현재 카메라 높이·관절 기준을 오래된 촬영에 혼용하지 않는다.
- 양쪽 모두 **공식 CAD의 nominal stock jaw**를 사용한다. 현재 실측 패드 기록을 과거 설치 상태에 억지로 적용하지 않는다. 물체 형상은 촬영에서 관측한 영역만 반영한다.

시간의 의미:

- `generation_median_ms`: 워밍업을 제외한 반복 생성 중앙값. 기본 3회이며 실행 순서를 교대한다.
- `common_check_ms`: 공통 관측 접촉선·개방 폭 검사 시간.
- `first_feasible_validation_ms`: 순차 손가락 바인딩·IK·전체 경로 검사에서 처음 완전한 경로를 찾을 때까지의 시간. 못 찾으면 `null`.
- `generation_to_first_feasible_ms`: 마지막 실제 후보 생성 시간 + 공통 검사 + 첫 경로 탐색 시간. 생성 중앙값과 단일 경로 시간을 임의로 합하지 않는다.
- `validation_setup_ms`: 로봇 모델·MoveIt·관측 장면 준비 시간으로, 위 탐색 시간에서 제외하고 따로 기록한다.

경로는 각 방식·장면당 한 번 검사하며 OMPL 난수는 고정하지 않는다. 엄밀한 경로 시간 비교는 새 출력 폴더로 전체 실험을 반복해야 한다. 실패율은 방향별 후보 기준이며, 기하적으로 중복되는 후보가 포함될 수 있다. `VALIDATION_ERROR`와 `INVALID_INITIAL_STATE`는 파지 실패율로 집계하지 않는다. 모델 로드, 인식/분할, 전처리와 실제 이동 시간은 총 탐색 시간에 포함하지 않는다.

이 검증은 관측 장면에서의 이산 상태 충돌 검사이며 연속 충돌 증명, 마찰·물체 변형·물체 밀림의 물리 시뮬레이션 또는 실제 파지 성공 검증이 아니다.

변경 파일: `graspnet.py`(비교용 검사 옵션), `carry_planning.py`(첫 완전 경로 종료 옵션), `stationary_pick_sim.py`(새 source의 좌표 규약 검사), `benchmark_grasp_feasibility.py`(공통 비교), `run_grasp_feasibility.sh`(실행 환경), `test_grasp_benchmark.py`와 `test_carry_planning.py`(회귀 검사). 기존 검사·계획 옵션의 기본값은 보존한다.

## RViz에서 MoveIt 경로 재생

```bash
bash tools/diagnostics/run_grasp_rviz.sh --set 1
# 두 번째 촬영본:
bash tools/diagnostics/run_grasp_rviz.sh --set 2
```

기본값은 `fair-comparison-all` 결과에서 각 방식의 첫 통과 경로를 골라 번갈아 반복 재생한다. 다른 결과 폴더는 `--results`로 지정한다. `--backend so101` 또는 `--backend graspnet`으로 한 방식만 볼 수도 있다. `--stage-seconds 3`으로 단계별 재생 시간을 늦출 수 있고, 창 닫기 또는 Ctrl+C로 종료한다.

화면에는 촬영 당시 로봇 모델, 테이블/장애물 형상, 청록색 물체와 현재 방식·단계가 표시된다. 정적 장애물 메시지는 한 번만 전송한다. 관절 상태와 시각화 마커만 발행하며 모터·컨트롤러·실행 액션을 호출하지 않는다. ROS 도메인 198의 로컬 재생이다.

기존 MoveIt 통과 경로를 보여주는 **기구학 재생**이다. 각 단계를 기본 2초로 재생하므로 실제 실행 속도나 계산 시간 비교에 사용하지 않는다. 집게 개폐 보간은 시각화용이며 인양부터 물체가 TCP를 따라 움직이는 것은 부착 가정이다. 마찰·미끄러짐·물체 밀림은 계산하지 않는다. 루프마다 다음 경로의 촬영 초기 자세로 되돌아간다. 물리 시뮬레이션은 별도의 접촉 동역학 환경이 필요하다.

`--dry-run`은 ROS 통신이나 창을 열지 않고 촬영·경로 연결, 초기 관절 상태, URDF와 RViz 설정 생성만 검증한다. 관련 코드는 `replay_grasp_rviz.py`, `run_grasp_rviz.sh`, 테스트는 `test_grasp_rviz.py`이다.
