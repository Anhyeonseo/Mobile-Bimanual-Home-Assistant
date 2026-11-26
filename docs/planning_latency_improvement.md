# SO-ARM101 계획 지연 개선 실험

## 실제 계획 기본 정책: 첫 완전 경로에서 종료

`plan_pick_carry`는 이제 `stop_after_first=True`가 기본이다.
접근/닫힘 기하/인양/운반까지 통과한 첫 경로를 반환하고 남은 후보의 경로는 만들지 않는다.
계획 도중 후보 또는 운반 목표가 실패하면 다음 대안을 시도한다.
`MobileRoutePlanner`와 `plan_capture_carry.py`에도 이 정책을 적용했다.
모든 후보의 저렴한 우선순위 계산/바인딩은 여전히 할 수 있으나, 완전 경로의 전수 탐색은 하지 않는다.
품질 비교가 필요할 때만 `plan_capture_carry.py --compare-all-routes` 또는
`plan_pick_carry(..., stop_after_first=False)`를 명시한다.
기존 벤치마크의 `--exhaustive`는 평가용 옵션으로 유지한다.

이는 계획 실패 시 다음 후보 탐색까지의 변경이다. 실제 동작/파지 실패를 자동 재실행으로
연결한 것은 아니다. 실물 재시도에는 정지 확인, 빈 집게/보유 상태 확인, 실패 이후의 새 관측과
현재 관절/리프트 상태로 새 경로 요청이 필요하다. 오래된 경로나 후보를 자동 재생하지 않는다.

## 현재 권장 비교: 기존 MoveIt + 요청 대기 겹치기

첫 실측(set-1, 각 3회)에서는 baseline 1.376초, batch 4.475초,
parallel 5.333초로 기존 방식이 가장 빨랐다. 모두 첫 후보에서 경로를 찾았다.
이에 native batch/parallel은 명시적 실험 옵션으로만 남겨두었다.
기본 비교 대상은 이제 **baseline / pipelined**이며 C++ 패키지를 빌드하지 않는다.

`pipelined`는 기존 MoveIt `check_state_validity` 서비스를 그대로 사용한다.
한 응답씩 기다리는 대신 최대 8개 비동기 요청을 유지해 왕복 대기를 겹친다.
새 PlanningScene 생성이나 전체 장면 전송은 없다. 검사할 경로 샘플, 집게 각도,
부착 물체, 계획기/시간 예산/후보 순서는 그대로다. 서버 내부 계산이 병렬이라는 뜻은 아니다.
정상 완료 시 모든 응답을 받은 후에만 다음 단계로 진행한다. 검사 중 장면은 바꾸지 않는다.
오류/시간 초과 시 세션 전체를 종료하며, 부분 응답으로 성공을 선언하지 않는다.
실패가 있어도 해당 묶음의 모든 상태를 검사하므로 조기 탈락 장면에서는 손해일 수 있다.

```bash
# 기존 서비스와 판정 일치 검증 (성능 측정용 아님)
bash tools/diagnostics/run_planning_latency.sh \
  --modes pipelined --verify-batch --repeats 1 \
  --output-dir output/planning-pipelined-equivalence

# 기존 방식과 새 방식 시간 비교
bash tools/diagnostics/run_planning_latency.sh \
  --repeats 3 --request-window 8 \
  --output-dir output/planning-pipelined-comparison
```

`--request-window 1`은 비동기 요청을 겹치지 않는 대조군으로 사용할 수 있다.
4/8/16 등의 효과는 별도 출력 폴더로 비교하며 성공률과 시간 모두 확인한다.
소켓 제한 때문에 개발 환경에서는 ROS 실측을 완료하지 못했으며, 실제 개선률은 미확인이다.
이번 변경은 `windowed_requests.py`, `test_windowed_requests.py` 추가와 실행/비교 도구 수정이다.
기존 런타임의 기본 `baseline`은 변경하지 않았다.

아래는 최초 native 실험 구현 기록이다. 세 방식의 과거 비교를 재현하려면
`--modes baseline batch parallel`을 명시한다.

이번 변경은 **오프라인 계획/검증 속도 비교**이다. 모터, 리프트, 카메라를 실행하지 않는다.
기존 작업 파일의 변경을 보존했고, 기본 계획 경로는 baseline 그대로이다.

## 변경 내용

| 방식 | 경로 계획 | 추가 상태 충돌 검사 |
|---|---|---|
| baseline | 기존 MoveIt 서비스, 기존 순차 fallback | 상태마다 ROS 서비스 호출 |
| batch | baseline과 동일 | 동일 샘플들을 C++ 서버에 묶어서 전달 |
| parallel | 별도 두 OMPL 파이프라인 병렬 탐색, 첫 성공 시 중단 요청 | batch와 동일 |

parallel은 MoveItCpp에서도 사용하는 MoveIt `planning_pipeline_interfaces` API를 사용한다.
일반 연결은 RRTConnect의 자동 range와 range=0.3 변형을 경쟁시키고,
방향 제약 운반 연결은 RRTGoal과 RRTConnect를 경쟁시킨다. 로봇에서 최적인 조합으로 입증된 것은 아니다.
파이프라인마다 한 번 탐색한다. 스레드별 시간이므로 순차 방식과 CPU 예산이 같다는 뜻은 아니다.
두 파이프라인 종료 처리가 끝난 후 반환하므로 조기 종료가 즉시 응답을 보장하지 않는다.

후보 하나의 연결 구간마다 병렬화한다. 여러 후보를 동시에 실행하지 않는다.
요청마다 **완전한 장면 스냅샷**을 전달하고 별도 PlanningScene을 만든다.
배치 검증도 같은 world/ACM/padding과 전체 로봇, 부착 물체를 사용하며 관절 범위를 확인한다.
관절 약 1도 간격 경로 샘플, 16개 집게 개폐 샘플, 물체 기울기 조건을 유지한다.
이산 검사이며 연속 충돌의 증명은 아니다. 장면 복사/직렬화 비용도 실제 측정에 포함한다.

## 수정/추가 파일

- 수정: `ros2_ws/src/home_robot_tasks/home_robot_tasks/carry_moveit.py`
  - 기본 5초/2회 설정 유지, 선택적 계획 예산 및 시도 횟수.
  - 서비스별 시간 계측, 선택적 배치 상태 검사.
- 수정: `tools/diagnostics/carry_moveit_session.py`
  - baseline/batch/parallel 실행, 네이티브 서버 수명 관리, 완전한 장면 전달.
- 수정: `tools/diagnostics/benchmark_grasp_feasibility.py`
  - 기존 비교의 기본 동작 유지, 계획 옵션과 단계/서비스 시간 기록 추가.
- 추가: `ros2_ws/src/so101_planning_accel/`
  - `package.xml`, `CMakeLists.txt`, `srv/PlanSnapshot.srv`, `srv/CheckStates.srv`, `src/planning_server.cpp`.
- 추가: `tools/diagnostics/benchmark_planning_latency.py`, `tools/diagnostics/run_planning_latency.sh`.
- 추가: `tests/test_planning_acceleration.py`, 이 문서.

## 실행

프로젝트 루트의 일반 Ubuntu 터미널에서 실행한다. ROS2 Jazzy, MoveIt, colcon,
시스템 Python의 numpy/scipy/PIL 등 기존 프로젝트 의존성이 필요하다.
스크립트가 C++ 패키지를 `output/planning-accel-workspace`에 빌드한다.
ROS domain 197 및 localhost로 격리하며 컨트롤러/실행 액션은 켜지 않는다.

먼저 배치 검사와 기존 서비스의 판정 일치를 검증한다. 이 모드는 비교를 위해
느린 서비스를 추가 호출하므로 성능 측정값으로 사용하면 안 된다.

```bash
bash tools/diagnostics/run_planning_latency.sh \
  --modes batch --verify-batch --repeats 1 --maximum-candidates 8 \
  --output-dir output/planning-batch-equivalence
```

그다음 동일 입력/계획 예산/후보 수로 세 방식을 비교한다.

```bash
bash tools/diagnostics/run_planning_latency.sh \
  --repeats 3 --maximum-candidates 8 \
  --output-dir output/planning-latency-comparison
```

기본 입력은 기존 set-1/so101/prepared.json과 촬영 당시 모델이다.
다른 입력은 `--prepared`, `--model-dir`, `--snapshot`, `--mobile-mesh-dir`로 지정한다.
`GRASP_SOURCE_REPO`로 원본 메시가 있는 저장소 루트를 바꿀 수 있다.
출력 폴더가 이미 있으면 덮어쓰지 않는다.

예산 축소의 효과는 별도 실험으로 분리한다. 예를 들어 다음은 세 방식 모두 1초/1회 조건이다.

```bash
bash tools/diagnostics/run_planning_latency.sh \
  --budget 1 --attempts 1 --repeats 3 \
  --output-dir output/planning-latency-short-budget
```

## 결과 해석

`summary.json`의 `summary`에서 각 방식의 다음 항목을 비교한다.

- `successes/runs`: 전체 접근·닫힘 기하·인양·운반 경로를 찾은 횟수.
- `median_first_complete_route_ms`: 성공한 실행에서 첫 완전 경로를 찾기까지 중앙값.
- `median_search_wall_ms`: 실패한 탐색까지 포함한 실제 탐색 시간 중앙값.
- `errors`: 환경/프로그램 오류. 오류는 파지 실패나 0ms 성공으로 취급하지 않는다.

각 `run-N/MODE/report.json`에는 다음도 기록한다.

- `setup_ms`: 모델/서버/장면 준비 시간. 첫 경로 시간에서 제외하므로 단발 작업의 총 대기에는 따로 더해야 한다.
- `planner_metrics[].services`: 서비스별 호출 수와 누적 대기 시간.
- `planner_metrics[].stages`: pregrasp, approach, close, lift, carry 등 단계별 시간/성공 여부.
- `attempts`: 후보별 바인딩 시간, 전체 경로 시간, 실패 이유와 경로.

성공 횟수가 같거나 유지되는 조건에서 시간이 줄어야 개선으로 해석한다.
성공한 실행만 빨라지고 실패가 늘면 단순히 개선됐다고 결론 내리지 않는다.
OMPL은 확률적이며 실행 순서는 회차별로 회전한다. 다른 장면(set-2)에서도 반복한다.

**이 실험은 실제 계획 시작→집게 닫힘 시간을 측정하지 않는다.** 센서 인식, 후보 생성,
물리 이동/닫힘은 제외되고, 저장 후보의 바인딩부터 완전 경로 탐색까지 측정한다.
따라서 실물 총 소요시간 향상은 별도 하드웨어 실험이 필요하다.

## 검증 상태

- ROS2 Jazzy C++ 패키지 Release 빌드 통과.
- 관련 Python 회귀/추가 테스트 56개 통과(기존 scipy 경계 clipping 경고 7개).
- 실제 ROS 비교를 시도했지만 실행 환경의 UDP 소켓 권한 제한으로 discovery/service 통신 실패.
  `output/planning-latency-smoke/`는 VALIDATION_ERROR 기록이며 속도 개선 수치가 아니다.
- native/reference 판정 일치와 병렬 계획의 실제 경로/시간 검증은 위 터미널 명령 실행 후 확인해야 한다.

이번에 바꾸지 않은 항목: 후보 순위 정책, 운반 계획 분리, 이동 속도, 리프트 동시 이동,
리프트 RGB 판정, 자동 재시도, Servo. 계획 성능 개선 효과를 먼저 분리해서 확인한다.

## 참고

- [MoveIt 병렬 계획](https://moveit.picknik.ai/main/doc/how_to_guides/parallel_planning/parallel_planning_tutorial.html)
- 구현은 설치된 Jazzy 헤더의 `planWithParallelPipelines` 시그니처에 맞춰 빌드했다.
