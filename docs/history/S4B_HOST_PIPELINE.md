# S4b 호스트 연결 구현 이력

이 문서는 당시 구현·실험의 근거다. 현재 상태와 개발 우선순위는 [현재 상태](../CURRENT_STATUS.md)와 [로드맵](../ROADMAP.md)을 따른다.

<details>
<summary>목차 펼치기</summary>

- [연결한 실행 순서](#연결한-실행-순서)
- [실행 경계](#실행-경계)
- [제어기 인계 후 연결할 입력](#제어기-인계-후-연결할-입력)
- [로컬 검증과 재실행](#로컬-검증과-재실행)
- [이번 작업의 확인 기록](#이번-작업의-확인-기록)
- [제어기 접근 허용 후 실제 인계 조회](#제어기-접근-허용-후-실제-인계-조회)

</details>

> 이력 문서: 아래의 현재 상태·다음 순서·승인·배포 조건은 각 기록 시점의 내용이다. 최신 상태는 [현재 상태](../CURRENT_STATUS.md), 개발 방향은 [로드맵](../ROADMAP.md)을 따른다. 과거 승인으로 로봇 작업을 재개하지 않는다.

후속 제어기 접근·설치 결과는 [통합 제어기 인계](S4_UNIFIED_CONTROLLER_HANDOFF.md)에 기록했다.
아래의 "제어기 변경 없음"은 최초 호스트 구현 당시의 범위다. 현재 MCU와 실행 제한은
후속 인계 문서를 먼저 확인한다.

2026-11-13. 제어기를 다른 개발자가 사용 중인 조건에서 구현했다.
펌웨어, 프로토콜, 보드 클라이언트, 기존 제어기 서비스와 활성 배포 경로는 변경하지 않는다.
이 문서의 구현·검사는 실물 수락 판정이 아니다.

## 연결한 실행 순서

기존 `build_runtime`와 15단계 작업 실행기를 그대로 사용한다.
`MobileManipulationBindings.runtime_options(alignment_port)`가 새 정책과 조작 포트를
기존 실행기에 제공한다. ROS 경로에서는 `build_ros_runtime(mobile_bindings=..., alignment_port=...)`로
같은 조합을 연결한다. 동일 state 소유자와 명시적인 정렬 포트가 필요하며 기존 조작
조합과 혼용하면 생성 전에 거부한다. 실행기·팔 writer·정지 조정기를 새로 만들지 않는다.

1. 관측한 대상 점군의 지역 폭·방향과 SO-101 5축 FK로 정렬 후보를 만든다.
   이름별 정차 좌표나 사전 GraspNet 후보를 사용하지 않는다.
2. 기존 `LiftHeightRule`의 높이를 먼저 검사한다. 그 높이에서 검사된 후보가 없을
   때만 +1/−1/+2/−2/+4/−4 cm를 차례로 탐색한다. 기준점은 공식 STL 접촉 단면 중점이다.
3. `AlignmentScreen`이 방향을 포함한 SE(2) 경로, 전체 몸체/리프트 전환,
   카메라 투영·가림을 확인한다. 경로는 1 cm·3° 이하로 샘플링한다.
   이것은 연속 충돌 검증이 아니며, 미관측 공간을 자동으로 통과 가능하다고 하지 않는다.
4. 검사한 경로를 `Nav2FollowPathPort`로 기존 Nav2 제어기에 전달한다.
   `build_guarded_navigation(..., follow_path_action=...)`의 반환 세션 하나를
   일반 이동과 정렬에 함께 전달한다. 둘 다 동일한 베이스 writer·정지 확인·새 장애물
   관측 절차를 통과한다. 적재 정렬에는 검사된 적재 운반 전환 경로가 필요하다.
5. 실제 베이스와 리프트의 정지를 확인하고 **그 이후 노출된 새 RGB-D**를 요구한다.
   새 ID만 붙인 옛 영상, 다른 map/boot/calibration/model/scene의 영상은 거부한다.
6. 새 촬영에서 기존 GraspNet → `bind_grasp` → `plan_pick_carry`를 실행한다.
   위쪽에서 수평 위치를 맞춘 후 내려오는 접근과 전체 STL 경로 검사를 유지한다.
   기존 9.1 mm 보정은 다시 적용하지 않는다. 닫힘 여유는 5 mm이며 측정된 힘 단위가 아니다.
7. 개방 → 위쪽 접근 → 하강 → 닫힘 → 짧은 들기 확인을 실행한다. 독립 시각·집게
   증거가 확인돼야 장면에 부착한다. 궤적 완료만으로 집기 성공을 선언하지 않는다.
8. 부착 후 **측정된 `tcp_from_object`와 현재 관절값**으로 남은 들기·운반 자세 경로를
   다시 계산한다. 처음 계획한 부착 위치가 8 mm 달라진 모의 사례도 새 경로를 검사한다.
   센서 결과를 기다리는 동안 기존 팔 유지 실행을 해제하거나 집게를 다시 열지 않는다.
9. 목적지 이동 후 현재의 빈 지지면을 다시 관측한다. 전체 물체 외곽과 측정 불확실성,
   셀 경계를 포함해 빈 공간을 줄여 놓기 후보를 만든다. 모르는 셀·새 물체가 있는 셀은 제외한다.
10. 측정한 부착 변환으로 `plan_place_carry`의 접근·하강·개방·후퇴·빈 운반 자세를 연결한다.
    독립 해제 증거와 장면 분리 확인 전에는 후퇴하지 않는다. 최종 도착 영상 검사는 기존
    `verify_delivery`가 수행한다.

## 실행 경계

- 정렬·GraspNet·MoveIt·적재 재계획은 하나의 공유 계산 작업자에서 실행한다.
  로봇 상태를 읽고 결과를 채택하는 부분은 실행기 스레드에서 처리한다.
- 계산 중 취소·재부팅·지도/모델 변경·본체 이동이 발생하면 결과를 폐기한다.
  취소된 계산이 아직 끝나지 않았으면 다른 단계가 같은 계획 장면을 덮어쓰지 않는다.
- 닫힘 명령과 물체에 걸려 멈춘 측정 집게 각도를 구분한다. `ContactEnvelope`와
  독립 접촉 검사 없이 기준을 완화하지 않는다. 적재 경로에서는 닫힘 명령을 일정하게 유지한다.
- 계획을 시작하기 전에 그 계획의 충돌 검사 결과를 요구하던 순환 조건을 제거했다.
  실제 팔 전달 시 전체 경로 검사와 현재 장면 검사는 그대로 필수다.
- 운반 준비 판정은 고정 Home과 비교하지 않는다. 검사한 동적 운반 목표를 등록하고
  측정 관절·물체에 걸린 실제 집게 각도·리프트 높이와 비교한다. 새 작업은 기존 빈 운반 목표로 복귀한다.
- 새 정책의 계산 포함 기한은 정렬 180초, 집기/놓기 300초, 전체 1,200초다.
  장치 통신·정지 확인·상태 신선도 제한을 늘린 것이 아니다.
- 현재 실제 집 앱의 `physical_execution_not_commissioned` 제한은 유지한다.
  합성 센서 테스트에 사용한 값을 실물 증거로 승격시키지 않는다.

## 제어기 인계 후 연결할 입력

모바일 정책의 생성자는 장치를 열지 않는다. 인계된 기존 서비스의 값을 다음 경계에 연결한다.

| 입력 | 제공 내용 | 검사/소비자 |
|---|---|---|
| `scene_for` | 원본 capture ID/노출 시각/해시, 동일 물체·가구, map 점군·지지면, 실제 베이스/높이 | `TaskScene`, `AlignmentSelection` |
| `configuration`, `epoch` | 신선한 현재 pose/height/정지, boot/map/calibration/model/scene revision | `PrealignmentPolicy` |
| `prepare` | 복사된 관측·관절 snapshot, 높이표, `AlignmentScreen`, 검사된 빈/적재 운반 경로 | 정렬 또는 `CaptureRouteInput` |
| 전체 몸체 검사 | 베이스 중심점만이 아니라 방향별 베이스 외곽·높이 장애물·리프트 전환·반대팔·적재물 | `AlignmentScreen.configuration_check/pose_free` |
| 정밀 관측 | 새 frame의 depth/intrinsics/mask/scene/plane와 기존 capture review/payload | `MobileRoutePlanner`, 기존 GraspNet·SO-101 binding |
| `loaded_snapshot`, `check_loaded` | probe 이후 측정 부착 변환·현재 관절·집게 유지값, 계획 중 하중·epoch 유지 | `LoadedSnapshot`, `LoadedCarryPort` |
| `context_for` | 실행 직전 상태·높이·모델·capture·map/boot context 및 측정 하중 변환 | `TaskRouteContext`, `ResidentRouteBuilder` |
| `scene_command`/장면 포트 | 검증된 측정 부착/분리와 장면 ACK | 기존 planning-scene 포트 |
| 운반 목표 | 적재 재계획 결과를 `RobotStateStore.bind_transport_program`에 등록, 실제 관절·집게 범위·높이와 비교 | 기존 prepare_transport/nav 단계 |
| `geometry_evidence` | 신선한 장면 부착·분리, probe 추종·여유, 개방 전 지지면 도달 증거 | `GeometryEvidence`, 접촉/개방 검사 |

이 연결은 기존 센서·장면·MoveIt 어댑터를 주입하는 경계다. 제어기를 조회하지 않았으므로
이번에 개발 중인 제어기의 실제 boot/profile/기능 계약을 확인했다는 뜻은 아니다.
실물에서 현재 촬영·측정 패킷을 받아 이 경계를 확인하는 P0 인계 검사가 먼저 필요하다.
환경별 장면 checker 또는 센서 입력이 없으면 시작을 거부하며, 검사 결과를 임의의 True로
채우거나 현재 실제 집 앱의 모드를 강제로 바꾸는 실행 명령은 제공하지 않는다.

## 로컬 검증과 재실행

```bash
python3 tools/run/home_assistant.py verify --stage all --with-ros \
  --output output/s4b-mobile-next
```

- `test_mobile_task_pipeline.py`: 실제 SO-101 FK/높이표/STL 단면 기준, 노출 순서,
  문맥 변경, 경로의 중간 장애물·회전, 영상 가림, 측정 부착·목적지 점유,
  비동기 취소·계산 소유권, 독립 집기·해제, 적재 재계획 진입, 집게 압력 유지.
- `test_mobile_runtime_connection.py`: 새 비동기 정책과 새 프로그램 포트를 기존 15단계
  실행기에 붙인 정상 완료 및 모든 15단계 취소. **센서·경로 결과는 합성**이다.
- `check_nav2_action_port.py`: 기존 NavigateToPose와 추가 FollowPath의 실제 ROS 전송·취소.
- `check_carry_moveit.py --include-place --compile-task-programs`: 공식 로봇 STL과 MoveIt으로
  집기·적재 재계획·놓기 전체 경로 생성, 실행용 시간 보간 후 다시 전 지점 충돌 검사.
  **집기 후보와 접촉 결과는 합성**이며 실물 성공 횟수로 세지 않는다.

실물에서 검증할 것은 기존 시험표 P0~P4와 S5다. 이번에 실제 베이스 정렬이나
집기·운반·놓기를 실행하지 않았고, 카메라 입력부터 실제 제어기까지의 새 연결을
실물 완료로 판정하지 않았다. 정밀 파지 실패 시 새 정렬을 자동 반복하는 복구와
집 전체 자동 탐색은 이 최초 실행 경로에서 켜지 않는다.

## 이번 작업의 확인 기록

- 최종 전체 회귀: `output/s4b-mobile-handoff-final-20261113/report.json`.
  기하 289개·실행 365개·앱 179개, 합계 **833개 통과**, 실패·건너뜀 없음.
- 새 연결 직접 검사: 위 회귀에 포함된 58개(기하/계약 40, 전체 15단계 연결/취소·ROS 구성 18).
  기존 reach-shell 리허설과 구분한다.
- Nav2 실제 action 전송/취소 및 모의 센서 손실 대응:
  `output/s4b-mobile-final-20261113/`의 `ros-nav2-*` 로그/보고서.
- 측정 부착 8 mm 차이를 포함한 최종 실제 MoveIt/STL 검사:
  `output/s4b-mobile-native-final-20261113/report.json`.
- 먼저 발견된 관절값 형변환·읽기 전용 배열 전달 오류의 실패 기록은 이전 출력 폴더에
  보존했다. 수정 후 재검사 결과로 판정한다.

현재 실물 실행 준비 판정은 여전히 false다. 제어기 인계·실측 센서 계약 확인이 필요한
상태를 숨기기 위해 true로 바꾸지 않았다. 펌웨어·프로토콜·보드 브리지 변경은 없다.

## 제어기 접근 허용 후 실제 인계 조회

2026-11-13 후속으로 사용자가 제어기 접근을 허용했다. 젯슨의 당일 보정 도구·기록을
확인한 뒤, 해당 도구가 사용하는 기존 S3 바이너리 프로토콜로 현재 보드를 조회했다.
앞서 조종용 텍스트 `INFO`가 응답하지 않은 결과는 보존하며 하드웨어 고장으로 판정하지 않는다.

- 현재 응답: protocol v2, S3 `0x00024911`, 12축, 정지 래치 false.
- 양팔 12개·바퀴 3개·리프트 1개 모두 inventory 완전 응답. 조회 시점 전부 토크 OFF,
  속도 0·moving 0. 이는 이후의 정지/지지 상태나 실물 실행 허가를 대신하지 않는다.
- 현재 모바일 endpoint 조회는 `mobile endpoint is not attached on this firmware`로 거부됐다.
  따라서 이 펌웨어에 호스트 코드만 연결해 자동 정렬/주행·리프트 동작을 시작할 수 없다.
- 기존 `stationary_pick.py check`의 제어기 신원·RGB-D 읽기 검사 통과: 2초 동안 60프레임,
  최대 노출 나이 약 43 ms. 새 집기 후보나 측정된 물체 부착 증거를 생성한 검사는 아니다.
- 설치된 팔 범위·좌표 모델·근거리 카메라 설정과 로컬 파일 해시가 일치한다.
  오늘 추가 촬영한 보정 데이터가 최종 모델에 반영됐다는 뜻은 아니다.
- 재시작·펌웨어 설치·모터 쓰기는 수행하지 않았다. 기존 활성 릴리스 경로도 유지한다.

기록: `output/s4b-controller-handoff-20261113/summary.json`, `live-query.json`,
`installed-and-capture-records.json`. 설치 파일의 해시는 실제 MCU 플래시 덤프 검증을
대신하지 않는다. 현재 높이는 마지막 보정 사진의 입력값으로 재사용하지 않는다.

다음 연결은 실측 근거가 있는 모바일/리프트 profile과 통합 endpoint, 현재 높이 기준,
실제 관측·하중 증거 어댑터의 인수다. 저장소의 `mobile_board.simulation.json`을 그대로
실물 profile로 승격시키거나 S3 이미지와 조종 이미지를 작업 중 교체하는 방식은 사용하지 않는다.
호스트 연결 검사 완료와 실제 MCU를 포함한 전체 실행 준비 완료를 구분한다.
