# ROS2 Pick Task Manager (Jazzy)

`yeoeun/perception-offline-lab`에서 시작한 하드웨어 없는 Pick 프로토타입입니다.
기존 `home_robot_tasks.execution.TaskExecutor`, `TaskLease`, `FakeRobot`을
그대로 재사용합니다. 기존 Assistant / Application / Gateway 파일은 수정하지 않습니다.
GPU, 카메라, AI 모델, 로봇 드라이버가 필요하지 않습니다.

구조와 코드 흐름은 [ROS2 Task Manager 구현 설명](../../../docs/ROS2_TASK_MANAGER_IMPLEMENTATION.md)을 참고하세요.

## 범위

- Step 1–3: Pick 계획, JSON 검증, ament_python 노드, 제출/취소/조회 서비스,
  상태 토픽 및 모의 실행 구현.
- Step 4–5: ROS2 Action Client 및 모의 Action 서버, 기존 Gateway의 인증·실행 허가·
  SQLite 기록을 재사용하는 Pick 연결 구현.
- Step 6: 실제 Perception/Navigation/Manipulation 제어기 및 하드웨어 연결은 미구현.
- 자연어 대신 AI가 생성했다고 가정한 JSON을 받습니다. 위치 및 파지 증거는
  `FakeRobot`이 만드는 합성 데이터이며 실제 물체 인식이나 3D pose가 아닙니다.

## 빌드와 실행

저장소 루트에서 Ubuntu 24.04 / ROS2 Jazzy의 시스템 Python을 사용합니다.
`colcon`, `rclpy`, `std_msgs`, `rosidl_default_generators`가 필요합니다.
의존성 빌드에 기존 `so101_interfaces`, `so101_arm_bridge`도 포함되지만
장치 연결이나 하드웨어 노드 실행은 하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-up-to home_task_manager --symlink-install
source install/setup.bash
ros2 run home_task_manager task_manager
```

다른 터미널에서도 `/opt/ros/jazzy/setup.bash`와 같은 workspace의
`install/setup.bash`를 source한 뒤 실행합니다.

```bash
ros2 topic echo /task/status std_msgs/msg/String --qos-durability transient_local
```

```bash
ros2 service call /task/submit home_task_interfaces/srv/SubmitTask \
  "{json_request: '{\"schema_version\":1,\"task_id\":\"pick-001\",\"operation\":\"pick\",\"target\":{\"object\":\"remote\",\"color\":\"black\",\"location\":\"desk\"}}'}"
ros2 service call /task/get_status home_task_interfaces/srv/GetTaskStatus \
  "{task_id: 'pick-001'}"
ros2 service call /task/cancel home_task_interfaces/srv/CancelTask \
  "{task_id: 'pick-001'}"
```

`/task/submit`은 최대 16 KiB의 JSON을 받으며 스키마 버전 1, `pick`만 허용합니다.
`task_id`는 영문/숫자/밑줄/하이픈 1–64자입니다. `target`은 `object`, `location`,
선택적인 `color`로 구성되며 값은 1–128자 문자열입니다. 알 수 없는 필드,
중복 JSON 키, 빈 문자열은 거부합니다. 모든 대상은 로봇이 이미 접근 가능한
작업 공간 안에 있다고 가정합니다. 방 사이 이동이나 Place 단계는 없습니다.

## 상태와 취소

정상 흐름은 `IDLE → VALIDATING → SEARCHING → LOCALIZING → ALIGNING →
GRASPING → VERIFYING → COMPLETED → IDLE`입니다. 검증 오류는 제출을 거부하고
기존 작업 상태를 유지합니다. `VALIDATING`은 검증 완료 후 첫 실행 tick을 기다리는
짧은 입장 상태입니다. 각 단계는 외부 어댑터의 성공 및 조건 증거를 확인합니다.

취소 또는 오류 발생 시 `STOPPING`으로 전환합니다. 정지 응답을 확인해야
`CANCELLED`/`FAILED`로 전환하고 lease를 해제합니다. 2초 안에 확인되지 않으면
`STOP_UNCONFIRMED`로 유지하면서 계속 정지 확인을 기다립니다. 이때 새 작업은
거부합니다. GRASPING 중이거나 이미 파지한 경우 기존 실행기의 하중 보존 규칙을
사용합니다. 서비스의 `accepted=true`는 **정지 완료를 뜻하지 않습니다**.

완료 상태는 한 tick 동안 현재 상태로 발행되고 이후 `IDLE`로 바뀝니다.
과거 결과는 `/task/get_status`에 ID를 전달해 확인할 수 있습니다. 동일 ID/동일
내용 재제출은 재실행하지 않고 결과를 반환하며, 다른 내용이면 충돌로 거부합니다.
프로세스당 128개까지 보관하며 자동 삭제하지 않습니다. 직접 ROS 서비스 모드는
재시작 시 기록이 사라집니다. Gateway 모드는 SQLite 기록을 유지합니다.
종료 시 최대 3초 동안 취소/정지 확인을 시도하고, 미확인 상태를 완료로 간주하지 않습니다.
실제 장치 정지를 보장하는 수명주기는 하드웨어 연결 전에 별도 구현해야 합니다.

ROS 서비스와 timer는 단일 executor에서 순차 실행됩니다. Gateway HTTP worker와는
기존 Gateway lock으로 같은 manager를 보호합니다. watchdog은 monotonic 시간을
사용하므로 ROS simulation clock이 멈춰도 동작합니다. 상태 토픽은 reliable /
transient-local, depth 10이며 늦게 연결한 구독자도 최근 상태를 받을 수 있습니다.
ROS 이름은 상대 경로여서 namespace/remap을 사용할 수 있습니다.

## 실패와 취소 실험

기존 노드를 종료하고 다음 중 하나를 실행합니다. 파라미터는 시작 시 읽습니다.

```bash
ros2 run home_task_manager task_manager --ros-args \
  -p mock_step_duration_s:=3.0 -p mock_stop_delay_s:=1.0
ros2 run home_task_manager task_manager --ros-args -p mock_fail_state:=GRASPING
ros2 run home_task_manager task_manager --ros-args -p mock_never_stop:=true
```

`mock_never_stop=true`는 취소 후 새 작업이 계속 거부되는 실험용입니다.
`step_timeout_s`는 기본 10초, 전체 작업 제한은 그 5배입니다.

## 테스트

ROS 설치 없이 코어 테스트 및 데모를 실행할 수 있습니다 (저장소 루트):

```bash
export PYTHONPATH="$PWD/ros2_ws/src/home_robot_tasks:$PWD/ros2_ws/src/home_task_manager${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s ros2_ws/src/home_task_manager/test -p test_core.py -v
python3 -m home_task_manager.demo
```

ROS 빌드 후에는 workspace에서:

```bash
colcon test --packages-select home_task_manager
colcon test-result --verbose
```

## ROS2 Action 실행

빌드 후 각 터미널에서 ROS와 workspace setup을 source합니다.
첫 터미널에 모의 Action 서버를 실행하고, 두 번째 터미널에 Task Manager를 실행합니다.

```bash
ros2 run home_task_manager mock_skills
```

```bash
ros2 run home_task_manager task_manager --ros-args -p backend:=ros_action
```

이제 위 `/task/submit` 호출은 별도 프로세스의 ROS2 Action 서버를 거칩니다.
`backend` 기본값은 기존 인프로세스 `mock`입니다.

| 단계 | Action 이름 | skill |
|---|---|---|
| SEARCHING | `/perception/find_object` | `search` |
| LOCALIZING | `/perception/localize_object` | `localize_object` |
| ALIGNING | `/navigation/align` | `align_for_pick` |
| GRASPING | `/manipulation/pick` | `pick` |
| VERIFYING | `/manipulation/verify_grasp` | `verify_grasp` |

모두 새 `home_task_interfaces/action/ExecuteSkill` 계약을 사용합니다.
기존 로봇 프로젝트에 이미 있던 Action 이름이나 Nav2/MoveIt의 표준 Action이라는
뜻은 아닙니다. 이름은 상대 경로이며 namespace/remap이 가능합니다.

Goal은 `run_id`, 단계별 `goal_id`, `skill`, `parameters_json`을 포함합니다.
`parameters_json`에는 `target`과 이전 단계 결과 `context`가 있습니다.
LOCALIZING 서버가 반환한 합성 `object_pose`를 이후 단계에 전달하며,
모의 정렬·파지 서버는 이 pose가 없으면 Goal을 거절합니다.
Feedback/Result에는 같은 `goal_id`, `simulated=true`, 조건 증거가 필요하고,
Result의 `output_json`은 다음 단계 context로 병합됩니다.
현재 freshness는 클라이언트 수신 시각 기준이며 실제 센서 관측 시각·TF·pose 정확도는
검증하지 않습니다. **실물에 사용할 인터페이스 계약으로 확정된 상태가 아닙니다.**

취소는 `cancel_goal_async()`로 보내되 Goal 승인이 늦게 도착하면 승인 직후 취소합니다.
취소 승인만으로 진행하지 않고 terminal Action 결과를 기다립니다. 이어서
`/task_skills/confirm_stop` (`home_task_interfaces/srv/ConfirmStop`)에서 해당 run의
베이스·리프트 정지, 팔 유지, 필요한 하중 보존 증거를 확인해야 lease를 해제합니다.
서버/피드백/정지 확인이 없거나 ID가 잘못되면 안전하게 중단하거나 소유권을 유지합니다.
Goal 전송 결과가 불확실한 경우에도 정지를 추정하지 않습니다.

모의 서버의 실험용 시작 파라미터:

```bash
ros2 run home_task_manager mock_skills --ros-args -p duration_s:=3.0 -p stop_delay_s:=1.0
ros2 run home_task_manager mock_skills --ros-args -p fail_skill:=pick
ros2 run home_task_manager mock_skills --ros-args -p reject_skill:=pick
ros2 run home_task_manager mock_skills --ros-args -p never_confirm_stop:=true
ros2 run home_task_manager mock_skills --ros-args -p silent_feedback:=true -p duration_s:=3.0
```

## 기존 Gateway 연결: HTTP → Task Manager → ROS2 Action

기존 `home_robot_tasks.gateway.Gateway`, `GatewayStore`, HTTP handler를 그대로 사용합니다.
`gateway_bridge.PickApplication`이 Gateway의 `request_id`를 Task Manager의 `task_id`로
변환합니다. 기존 fetch/navigation 실행기를 추가로 띄우지 않으며 하나의 manager와
lease를 공유합니다. 이 실행 모드는 Pick 전용입니다. 기존 웹 UI의 지도/이동 경로는
지원하지 않고 명확한 오류를 반환합니다.

저장소 루트에서 최초 한 번 토큰을 만듭니다. 이 디렉터리는 기존 `.gitignore`의
`artifacts/` 규칙에 포함됩니다. 토큰은 명령줄 인자나 로그에 출력하지 않습니다.

```bash
mkdir -p artifacts/task_manager
python3 -c 'import os,secrets; p="artifacts/task_manager/token"; fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600); os.write(fd,secrets.token_urlsafe(48).encode()); os.close(fd)'
```

모의 Action 서버를 실행한 상태에서 기존 Task Manager 노드를 종료한 뒤,
저장소 루트에서 Gateway 모드로 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run home_task_manager task_manager --ros-args \
  -p backend:=ros_action -p gateway_enabled:=true \
  -p gateway_token_file:="$PWD/artifacts/task_manager/token" \
  -p gateway_database:="$PWD/artifacts/task_manager/requests.sqlite3" \
  -p gateway_port:=8765
```

Gateway는 `127.0.0.1`에만 바인드합니다. Gateway 모드에서는 인증 없는
`/task/submit`, `/task/cancel`을 **생성하지 않습니다**. 상태 토픽과 조회 서비스는
개발용 ROS domain에 공개됩니다. 다른 Task Manager를 같은 domain에 함께 띄워
실행 권한을 우회하지 마세요. ROS 자체의 인증·암호화 설정은 이 프로토타입 범위 밖입니다.

다른 터미널에서 같은 setup을 source한 뒤 저장소 루트에서 실행합니다.
CLI는 토큰 파일을 읽고 `/v1/permit`에서 3초짜리 일회성 허가를 받아 제출합니다.

```bash
ros2 run home_task_manager gateway_client --token-file artifacts/task_manager/token \
  submit --request-file ros2_ws/src/home_task_manager/examples/pick_gateway.json
ros2 run home_task_manager gateway_client --token-file artifacts/task_manager/token status pick-001
ros2 run home_task_manager gateway_client --token-file artifacts/task_manager/token cancel pick-001
```

HTTP 경로는 기존 `/v1/permit`, `/v1/requests`, `/v1/requests/{id}`,
`/v1/requests/{id}/cancel`을 그대로 사용합니다. Bearer 인증과 `X-Request-Permit`이
필요하며 같은 ID 재제출은 재실행하지 않습니다. Gateway 요청 예시는 `examples/pick_gateway.json`입니다.

재시작 시 미완료 기록은 `INTERRUPTED`로 표시하고 새 작업 허가를 차단합니다.
원격 Action 서버가 정지했는지 재시작한 프로세스가 알 수 없기 때문입니다.
이 버전은 자동 복구 API가 없습니다. 모의 실험 복구 시 모든 모의 서버를 종료하고
기존 DB를 진단용으로 보관한 뒤 **새 DB 경로**로 Gateway와 모의 서버를 다시 실행합니다.
하드웨어 통합에서는 별도의 정지 검증·명시적 복구 절차가 필요합니다.

## 통합 경계와 후속 작업

`core.py`는 Pick 요청을 기존 실행기의 step 형식으로 변환합니다. 실행기 자체를
복제하지 않습니다. `ros_adapter.py`가 기존
`SkillAdapter.observe/start/poll/request_stop/poll_stop`를 ROS Action에 연결합니다.
현재 클라이언트/서버는 합성 증거만 사용하며 실제 장치를 구동하지 않습니다.

`RobotApplication`의 `fetch_object`는 이동과 배달을 포함하므로 이 Pick 요청을
그대로 전달하지 않습니다. 새 Pick Gateway 프로세스와 기존 fetch Gateway 프로세스를
동시에 운영하는 통합은 아직 지원하지 않습니다. 후속 작업은 기존
RobotApplication에 Pick 계획을 등록해 fetch/navigation까지 하나의 실행 권한으로
관리하는 것, 실제 Perception/Navigation/Manipulation 인터페이스 계약 확정,
관측 시각·TF 검증, 제어기의 실제 정지 증거와 재시작 복구 구현입니다.

인터페이스 정의는 별도 `ament_cmake` 패키지 `home_task_interfaces`에 둡니다.
참고: [ROS2 Jazzy custom interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).
