# S4b · S5 시험 실행 준비

현재 인계 기준은 [실험 직전 준비](S4_PREEXPERIMENT.md)와
[통합 제어기 인계](S4_UNIFIED_CONTROLLER_HANDOFF.md)의 2026-11-14 기록이다.
아래의 제어기 접근 금지는 최초 준비 당시의 조건이며, 이후 사용자 허용으로 해제됐다.
현재 준비는 모터 명령·재시작 없이 진행한다. 실물 시험 통과로 자동 승격하지 않는다.

2026-11-13. 작업 위치는 `workspaces/home-assistant`이다. 다른 개발자가 사용하는
실물 제어기는 이 준비 작업에서 조회·설치·리셋하지 않는다. 최초 검증은 PC에서만 했다.
이후 사용자 지시로 젯슨 SSH 접속과 제어기를 사용하지 않는 준비는 허용됐다.
제어기 USB 장치·모터 명령·관련 서비스/실행 프로세스는 그대로 유지한다.
아래 명령은 로컬 검사이며 실물 실행 명령이 아니다.

## 이번 준비에서 실행한 결과

이전 준비 단계의 단일 명령 결과: **`output/s4b-s5-ready-20261113-final/summary.md`**.
새 자동 연결 구현의 후속 결과는 `output/s4b-mobile-handoff-final-20261113/`에 기록한다.
같은 폴더의 `report.json`은 전체 검사 완료·PASS이며 실물 준비 판정은 false다.
아래는 이 최종 실행 결과다. 앞선 단계별 실행 기록도 원래 폴더에 보존했다.

| 검사 | 결과 | 기록 |
|---|---|---|
| S4b 기하 회귀 | 249 통과 | 최종 폴더 `s4b_geometry.xml` |
| S4b 실행/중단 회귀 | 347 통과 | 같은 폴더 `s4b_execution.xml` |
| S5 앱/언어/실제 집 자료 경계 회귀 | 179 통과; 실행 도구 14개 포함 | 같은 폴더 `s5_application.xml` |
| 형상 후보 | 36/36조건에서 제안 생성 | 같은 폴더 `alignment-shapes/summary.json` |
| 텍스트 → 전체 실행기 | 정상/취소/실패 8상황 예상 결과 | 같은 폴더 `rehearsal-*.json` |
| 실제 로컬 언어 모델 | 197문장 검사; 과거 133/133, S5 범위 판단 64/64, 명확한 S5 요청 30/30 슬롯 일치 | 같은 폴더 `language-*.json` |
| 로컬 모델 → 15단계 전체 모의 실행 | SUCCEEDED | 같은 폴더 `language-full-runtime.json` |
| 실제 Nav2/모의 바퀴 | 도착·취소, scan 손실, TF 손실 모두 통과 | 같은 폴더 `nav2-*/report.json` |
| 실제 MoveIt/공식 STL | 집기·운반·놓기와 적재물/환경 충돌 거부 통과 | 같은 폴더 `moveit-carry-place/report.json` |

회귀 검사는 서로 다른 검사 **775개**이며 실패·건너뜀이 없다.
언어 모델의 전체 슬롯 완전 일치는 183/197이며, 범위 밖/모호한 나머지는 질문 또는
거부로 처리했다. 이를 문장 이해 100%로 보고하지 않는다. 각 검사 시작/종료 시 비교한
제어기 소스 해시의 변경은 0개이며, 실물 제어기와의 통신도 0회다.

최초 Nav2 시도는 임시 의존성 부재로 실행되지 않았고, 저장소 전용 prefix 준비 뒤
위 ROS 결과를 얻었다. 이 과정에서 시스템 ROS 패키지는 설치/교체하지 않았다.

## 바로 실행하는 로컬 검사

기존 `home_assistant.py`에 시험 입구를 추가했다. 기존 결과가 있는 출력 폴더는 거부한다.
원본 지도, 촬영 자료, 실행 중인 앱의 DB를 바꾸지 않는다.

```bash
python3 tools/run/home_assistant.py verify \
  --stage all --with-ros \
  --local-language-url http://127.0.0.1:18767 \
  --output output/s4b-s5-check-next
```

언어 서버가 꺼져 있으면 별도 터미널에서 기존 명령으로 켠다. 시험 도구가 서버를
자동 재시작하거나 모델을 내려받지는 않는다.

```bash
python3 tools/run/home_assistant.py language-server
```

부분 재검사는 `--stage s4b`, `--stage s5`, `--stage language`, `--stage ros`로 선택한다.
`language`는 모델 주소가 필수이고, `ros`는 로컬 ROS 검사만 실행한다. 모델 옵션을 생략한
결과는 `language_model: NOT_RUN`이다. 이를 S5 모델 시험 통과로 합산하지 않는다.

전체 작업 하나를 자세히 확인할 때:

```bash
python3 tools/run/home_assistant.py rehearse \
  --local-language-url http://127.0.0.1:18767 \
  --output output/s5-next-full-runtime.json
```

`--scenario`는 `success`, `cancel_before_execute`, `cancel_carry`, `sensor_loss`,
`load_loss`, `reboot`, `pick_failure`, `place_failure`를 지원한다. 실제 성공 여부뿐 아니라
중단 이후 놓기/개방이 실행되지 않았는지, 정지 확인이 불가능하면 제어권을 보존하는지 검사한다.

`rehearse`는 단순 FakeRobot 시연을 대신해 **기존 OfflineStack → build_runtime →
MissionCatalog → capture/search → PickPlacePort → payload/stop 조정기**를 사용한다.
대화 해석·미리보기에서는 장치 단계가 시작되지 않아야 하며, 테스트용 확인 이후에만
실행한다. 센서/바퀴 운동/집기 결과는 합성이다. 실제 모델 호출 여부와 모의 확인의 출처도 기록한다.

## 결과 읽기

- `report.json`: 검사별 상태, 파일 해시, 모델 평가, 실물과의 경계.
- `summary.md`: 사람이 읽는 검사 결과 요약.
- `physical-cases.NOT_RUN.json`: 집기 24조건·배송 10회·S5 8상황의 미실행 기록 양식.
  원본은 `config/home_assistant/s4b_s5_test_cases.json`이며 실행 명령이나 승인 파일이 아니다.
- `s4b_geometry.xml`, `s4b_execution.xml`, `s5_application.xml`: 검사 개수·실패·건너뜀.
- `alignment-shapes/`: 4형상 × 3방향 × 3높이의 베이스/리프트 제안. 성공률이 아니라 후보 생성 검사다.
- `rehearsal-*.json`: 요청/확인/15단계 실행, 11개 집기·놓기 하위 단계, 새 촬영 ID, 실패 원인.
- `language-*.json`: 고정 문장 197개의 원문·모델 답·추출 결과·질문 필요 여부.
  모호하거나 범위 밖인 문장을 질문으로 돌린 판정과 슬롯 정확도는 별도다.
- `nav2-*/`, `moveit-carry-place/`: 로컬 Nav2와 MoveIt 결과 및 로그.

실패나 건너뜀은 통과로 처리하지 않는다. 도중 중단한 보고서는 `complete: false`로 남는다.
재실행은 새 폴더를 사용한다. 모든 결과의 `physical_task_completed`는 false다.

## 시험 환경

Python 3.12, 기존 numpy/scipy/pytest, ROS Jazzy, 로컬 Qwen3-4B Q4_K_M을 사용한다.
Nav2 1.3.13과 의존 ROS 패키지는 시스템 설치 없이 `artifacts/s4-test-ros/prefix/`에
압축 해제했다. 원본 패키지의 버전·SHA256은 `artifacts/s4-test-ros/manifest.json`에 있다.
실행 도구가 이 prefix를 시험 자식 프로세스에만 적용한다.

ROS 검사는 localhost, 시험용 DDS domain 189, offline namespace를 사용한다.
로봇 discovery 설정을 상속하지 않는다. MoveIt은 경로 서비스만 시작하며 trajectory
실행을 켜지 않는다. Nav2는 기존 `build/offline-core/libactuator_host_simulator.so`를
메모리 안의 바퀴 모형으로 사용한다. 펌웨어 재빌드·보드 연결은 하지 않는다.

실제 집 자료는 다음 조합을 고정해 검사한다.

- 평면 지도: `output/s4-room-mapping-20261113-lidar-final-v1/occupancy.json`
- 이름/가구 검토본: `output/s4-room-mapping-20261113-context-review-v1`
- 베이스 폭 + RGB-D 상단 장애물 반영 영역: `output/s4-room-mapping-20261113-movement-fused-v1`

이동 가능 영역 검토본은 주행 허가가 아니다. 접힌 팔 가정, 방향별 베이스 폭, 미관측
높이 영역과 현재 위치 추정의 한계를 유지한다. 지도와 semantic 이름의 원본 해시가
달라지면 별도 자료를 조용히 섞지 않는다. 9.1 mm 모델 보정을 다시 더하지 않으며,
S3 잔여 8 mm 관측을 새 보정값으로 사용하지 않는다.

## 제어기 인계 시 필요한 연결

현재 제어기 버전은 이 작업에서 조회하지 않았다. 과거 `s4_controller.inspected.json`을
현재 보드의 버전으로 간주하면 안 된다. 아래 자료를 인계받은 뒤 기존 runtime factory에
어댑터를 연결한다. 테스트용 합성 신호를 실물 신호로 바꾸는 작업이 남아 있다.

| 입력/경계 | 연결 확인 내용 | 기존 소비자 |
|---|---|---|
| 세션/소유권 | 한 실행기, boot ID·sequence·profile 해시, 다른 조종 세션 종료 | TaskLease, EvidenceProducer |
| 베이스 | 실제 속도/정지, 위치 추정 epoch, 재부팅 후 오래된 목표 재전송 없음 | navigation binding, RobotStateStore |
| 양팔/리프트 | 관절값, 원점·높이·정지·하중 유지, 모드 전환 결과 | motion ports, RobotStateStore |
| 새 관측 | capture ID, 원래 노출 시각, TF·calibration·scene·map 버전 | CapturePort, MissionCatalog |
| 집기/운반 | 측정 gap/effort와 독립 시각 확인, held-object 변환 | PayloadMonitor, carry scene |
| 놓기 | 현재 빈 지지면, 물체 전체 경계, 개방 후 독립 해제/놓임 확인 | SurfaceObserver, place planner |
| 취소/통신 단절 | 실제 바퀴 정지와 팔·리프트/하중 유지 확인, 자동 재시작 금지 | SystemStopBackend |

통신 timeout이나 위치 품질 기준을 이 준비 작업에서 변경하지 않았다.
다른 개발자의 제어기 구현과 위 계약이 다르면 기존 어댑터 경계에서 조정한다.

## 자동 정렬부터 놓기까지 추가 구현

이전 775검사 결과는 위에 보존한다. 이후 사용자 요청으로 남아 있던 호스트 연결을
구현했다. 현재 구현·입력 계약·시험 경계는 **[S4B_HOST_PIPELINE.md](S4B_HOST_PIPELINE.md)**를 따른다.

- 고정 높이 lookup 우선, stock STL 단면 중심을 사용하는 `FixedHeightAlignmentPlanner`.
- SE(2) 정렬 경로·카메라 가시성 검사와 Nav2 FollowPath 연결.
- `PrealignmentPolicy`로 정렬 → 실제 도착 → 새 노출 → GraspNet/전체 경로를 연결.
- 기존 단일 실행기에 typed pick/place 프로그램 연결, probe 이후 측정 부착으로 적재 경로 재계획.
- 관측된 빈 영역과 전체 물체 외곽으로 놓기 후보 선택, 독립 해제 증거 후 후퇴.
- 한 공유 비동기 계획 작업자, 모든 단계 취소, 문맥 변경·오래된 결과 폐기.

새 15단계 연결 시험과 실제 MoveIt/STL 검사는 각각 합성 센서/집기 후보를 사용한다.
둘을 실제 집의 인식·실행 성공으로 합치지 않는다. 실제 집 앱은 여전히 검토 모드이며
개발 중인 제어기의 인계 계약과 실제 센서 입력 확인 전에는 활성화하지 않는다.

## 인계 후 실물 시험 순서

| 순서 | 시험 | 기록/판정 |
|---|---|---|
| P0 | 새 제어기 계약·현재 자세·정지/유지·세션 전환 | 인계 버전, 실제 응답/정지 증거. 기존 제어기 작업과 동시 실행 금지 |
| P1 | 물체 관측 → 자동 베이스/높이 정렬 → 새 촬영 → 집기 경로 생성 | 사람이 베이스를 집기 좋게 미세 조정하지 않음. 실제 이동과 파지 실행은 구분 |
| P2 | 4물체 × 3방향 × 2위치 = 24조건 | 최초/재정렬 후 성공, 끌림·미끄러짐·개입 모두 기록. 개발 목표 20/24 |
| P3 | 전원선 범위 안의 집기 → 운반 → 놓기 10회 | 개발 목표 8/10, 떨어짐/사용자 개입은 실패. 집 전체 성능으로 확대하지 않음 |
| P4 | 경로 막힘, 물체 이동, 목적지 점유, 취소 | 하중 유지와 설명 가능한 정지/재관측, 의도하지 않은 개방 없음 |
| S5 | 실제 집의 구체적인 텍스트 요청 → 한 번 확인 → P1~P3 → 결과 | 이름/모호함 질문, 취소 즉시 반영, 실제 새 센서 증거로 완료 판정 |

자동 집 전체 탐색은 배터리 도입 후다. 음성, 손 전달, 복수 작업, 젯슨 온디바이스
최적화는 이번 시험 범위에 추가하지 않는다. S3 완료 판정도 다시 열지 않는다.

## 젯슨 SSH 허용 후의 별도 준비

사용자는 젯슨 접속을 허용하되 다른 개발자가 사용하는 제어기는 계속 제외했다.
기존 SSH 키/호스트 키 검증을 재사용했다. 확인한 대상은 ARM64, Python 3.12.3이며
기존 `shared/venvs/s4-jazzy`의 numpy/scipy/pytest 환경을 재사용한다.

기존 포장 도구에 `--stage s4 --host-only`를 추가해 펌웨어 소스까지 제외한 시험 묶음을
만들었다. 시스템 패키지 설치, 기존 릴리스 교체, 서비스 재시작은 하지 않는다.

- 시험 코드: `/home/hyper/robot/shared/preflight/s4b-s5-20261113/s4-local-e59756783c7b`
- 파일 검증: 757개 SHA256 일치, 펌웨어 바이너리/펌웨어 소스 제외.
- 시험 실행기: 같은 상위 폴더의 `run-host-checks.py`. 새 출력 폴더만 허용한다.
- 기존 활성 경로: `stationary-pick`와 `s4-candidate`를 전환하지 않는다.
- 검사 범위: 합성 관측·모의 장치의 S4b/S5 175개. 실제 센서/제어기·실물 동작은 제외.
- 제한: 낮은 CPU 우선순위, 연산 스레드 1개. Python 장치 열기 및 외부 프로세스 실행을
  차단하며, numpy의 ARM SVE 확인용 `lscpu` 조회만 허용한다.
- 모델 연결: 임시 SSH 역방향 포워딩의 젯슨 `127.0.0.1:18777`에서 PC의
  `127.0.0.1:18767` 로컬 모델로 연결한다. 외부 네트워크 공개나 클라우드 모델 호출은 없다.
  SSH 시험 명령이 끝나면 이 포워딩도 닫힌다.

전체 로컬 검사 명령 `verify --stage all`은 PC의 지도/모델/ROS 자료를 포함한다.
출력 자료를 모두 복사하지 않은 이 젯슨 시험 묶음에서 같은 명령을 실행하면 안 된다.
젯슨에서는 별도 실행기가 선택한 호스트 검사만 수행한다. 이 준비로 위의 미완료
S4b 자동 정렬/실물 어댑터 연결이 완료되거나 실물 실행 허가가 생기지는 않는다.

최종 결과는 `output/s4b-s5-jetson-stage-20261113/jetson-results/`에 보관했다.
젯슨 호스트 검사 **175/175 통과**, PC 로컬 언어 모델을 거친 **15단계 전체 모의 작업
SUCCEEDED**, 제어기 접근/실물 명령 0이다. 차단된 장치/외부 프로세스 시도는 없었고
numpy의 CPU 정보 조회만 허용됐다. 시험 전후 757개 소스 파일, 시험 실행기 해시와
`stationary-pick`/`s4-candidate` 활성 경로가 유지됨을 확인했다.

최초 검사는 175개 자체는 통과했지만 CPU 정보 조회를 외부 프로세스로 차단해 전체
보고서가 false였다. `audit-diagnostic.json`에서 numpy의 `lscpu` 요청임을 확인했고,
그 조회만 허용한 재검사 결과를 `report.json`에 저장했다. 최초 결과도 삭제하지 않았다.
