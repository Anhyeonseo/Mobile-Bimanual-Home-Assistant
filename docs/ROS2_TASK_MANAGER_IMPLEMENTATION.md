# ROS2 Task Manager 구현 설명

이 문서는 `feat/ros2-task-manager` 브랜치에서 구현한 Pick Task Manager의 구조와
동작을 설명합니다. 기준 브랜치는 `yeoeun/perception-offline-lab`이며,
구현·검증 기준일은 2026-11-05입니다.

## 1. 무엇을 구현했는가

AI가 해석했다고 가정한 JSON 요청을 받아 물체를 찾고, 위치를 확인하고,
정렬하고, 집은 뒤 파지 결과를 확인하는 작업을 순서대로 실행합니다.

사용자는 다음 두 경로로 작업을 요청할 수 있습니다.

- **ROS 서비스 경로:** ROS2 Service로 직접 제출·취소하고 Topic/Service로 상태를 확인합니다.
- **Gateway 경로:** HTTP 인증과 실행 허가를 거쳐 제출·취소하고 결과를 조회합니다.

실행 방식도 두 가지입니다.

- **`backend=mock`:** 프로세스 안의 기존 `FakeRobot`을 호출합니다.
- **`backend=ros_action`:** 실제 ROS2 Action Client로 별도 프로세스의 모의 Action 서버를 호출합니다.

ROS2 통신과 HTTP 연결은 실제로 동작하지만, 물체 탐색·pose·파지·정지 증거는
모의 서버가 생성합니다. AI 모델, 카메라, 모터, 실제 로봇 제어기는 연결하지 않았습니다.
지원하는 작업은 `pick` 하나이며 방 사이 이동, 물체 배달, 내려놓기는 포함하지 않습니다.

## 2. 기존 프로젝트와의 관계

기존 실행 로직을 복사하지 않고 아래 코드를 재사용했습니다.

| 기존 구성 요소 | 재사용하는 역할 |
|---|---|
| `home_robot_tasks.execution.TaskExecutor` | 단계 실행, 피드백 검증, 시간 제한, 취소와 실패 처리 |
| `home_robot_tasks.execution.TaskLease` | 실행 중인 작업의 소유권 관리 |
| `home_robot_tasks.fake_robot.FakeRobot` | 인프로세스 모의 실행과 오류 주입 |
| `home_robot_tasks.gateway.Gateway` | Bearer 인증, 일회성 실행 허가, HTTP 요청 처리 정책 |
| `home_robot_tasks.gateway.GatewayStore` | SQLite 요청 ID·결과 기록, 재시작 시 미완료 요청 표시 |
| `home_robot_tasks.gateway.handler_for` | HTTP 요청 파싱과 응답 처리 |

새 Task Manager는 Pick 요청을 기존 `TaskExecutor`의 단계 형식으로 변환하는
역할을 맡습니다. 자체적으로 모터 명령을 생성하지 않습니다.

기존 `RobotApplication`의 `fetch_object` 경로를 확장한 것은 아닙니다.
Gateway가 기대하는 application 인터페이스를 `PickApplication`이 구현해
새 Task Manager에 연결합니다. 따라서 **Gateway 코드는 재사용하지만 현재 실행 모드는
Pick 전용**입니다. 기존 Assistant 자연어 입력이나 fetch/navigation 웹 UI까지
한 프로세스에 통합한 상태는 아닙니다.

## 3. 추가한 패키지와 파일

```text
ros2_ws/src/
├── home_task_interfaces/              # ament_cmake: ROS 인터페이스 생성
│   ├── action/ExecuteSkill.action
│   └── srv/
│       ├── SubmitTask.srv
│       ├── CancelTask.srv
│       ├── GetTaskStatus.srv
│       └── ConfirmStop.srv
└── home_task_manager/                 # ament_python: 실행 코드
    ├── home_task_manager/
    │   ├── core.py
    │   ├── node.py
    │   ├── ros_adapter.py
    │   ├── mock_skills.py
    │   ├── gateway_bridge.py
    │   ├── gateway_client.py
    │   └── demo.py
    ├── examples/pick_gateway.json
    ├── test/
    │   ├── test_core.py
    │   ├── test_gateway_bridge.py
    │   ├── test_ros_adapter.py
    │   └── test_ros.py
    └── README.md
```

| 파일 | 핵심 클래스·함수 | 담당 범위 |
|---|---|---|
| `core.py` | `parse_request`, `plan_pick`, `TaskManager` | 요청 검증, 5단계 계획, 작업 보관, 실행기 호출 |
| `node.py` | `TaskManagerNode` | ROS 서비스·상태 Topic·timer, 실행 모드 선택, 종료 처리 |
| `ros_adapter.py` | `RosSkillTransport`, `RosSkillAdapter` | Action 전송, 피드백·결과 처리, 취소·정지 확인 |
| `mock_skills.py` | `MockSkills` | 하드웨어 없는 Action 서버 5개와 정지 확인 서비스 |
| `gateway_bridge.py` | `PickApplication`, `GatewayBridge` | 기존 Gateway와 manager 연결, HTTP 서버, 재시작 입장 차단 |
| `gateway_client.py` | `main` | 토큰 파일을 사용하는 제출·조회·취소 CLI |
| `demo.py` | `main` | ROS 없이 정상 상태 흐름 확인 |

## 4. 전체 실행 구조

Gateway와 Action을 모두 사용하는 경우의 구조입니다.

```text
gateway_client 또는 HTTP 클라이언트
    │ Bearer 토큰 + 일회성 실행 허가 + Pick JSON
    ▼
기존 Gateway + GatewayStore(SQLite)
    │
    ▼
PickApplication
    │ request_id → task_id 변환
    ▼
TaskManager
    │ Pick 단계를 기존 실행기 형식으로 생성
    ▼
기존 TaskExecutor + TaskLease
    │ observe / start / poll / request_stop / poll_stop
    ▼
RosSkillAdapter
    │ ROS2 Action 및 ConfirmStop Service
    ▼
별도 프로세스의 MockSkills
    ├─ 탐색 / 위치 확인
    ├─ 정렬
    └─ 파지 / 검증
```

Gateway와 Task Manager는 같은 프로세스에서 **하나의 manager와 lease**를 공유합니다.
HTTP 요청 스레드와 ROS timer의 상태 접근은 기존 Gateway의 lock으로 직렬화합니다.
Action 콜백과 어댑터 내부 상태는 별도의 lock으로 보호합니다.

다른 프로세스에서 실행한 기존 fetch Gateway와 lease를 공유하는 기능은 없습니다.
두 Gateway를 동시에 띄워 하나의 로봇을 관리하는 구성은 아직 지원하지 않습니다.

## 5. JSON 요청을 어떻게 처리하는가

ROS 제출 서비스의 `json_request`에 들어가는 JSON 예시입니다.

```json
{
  "schema_version": 1,
  "task_id": "pick-001",
  "operation": "pick",
  "target": {
    "object": "remote",
    "color": "black",
    "location": "desk"
  }
}
```

Gateway 요청은 같은 내용에서 `task_id` 대신 `request_id`를 사용합니다.
`PickApplication.submit()`이 필드 이름을 변환해 `TaskManager.submit()`에 전달합니다.

검증 규칙은 다음과 같습니다.

| 항목 | 규칙 |
|---|---|
| 요청 크기 | ROS JSON 문자열 최대 16 KiB; HTTP handler도 body 크기 제한 |
| 버전·작업 | 정수 버전 `1`, 작업 `pick`만 허용 |
| ID | 영문·숫자·밑줄·하이픈, 1–64자 |
| 대상 | `object`, `location` 필수, `color` 선택 |
| 대상 값 | 공백만으로 구성되지 않은 1–128자 문자열 |
| 잘못된 구조 | 알 수 없는 필드, 중복 JSON 키, 잘못된 자료형 거부 |

같은 ID와 같은 요청을 다시 보내면 새로 실행하지 않고 기존 상태를 반환합니다.
같은 ID에 다른 내용을 보내면 충돌로 거부합니다. 다른 작업이 lease를 가지고 있으면
`robot_busy`로 거부합니다. 대기열에 넣거나 기존 작업을 강제로 선점하지 않습니다.

manager의 메모리 기록은 프로세스당 최대 128개이며 자동으로 지우지 않습니다.
Gateway의 SQLite 기록은 이 메모리 기록과 별개로 유지됩니다.

## 6. 상태 머신과 단계 실행

정상 실행 흐름은 다음과 같습니다.

```text
IDLE → VALIDATING → SEARCHING → LOCALIZING → ALIGNING
     → GRASPING → VERIFYING → COMPLETED → IDLE
```

`VALIDATING`은 현재 구현에서 요청 검증이 끝난 뒤 첫 실행 tick을 기다리는 짧은
상태입니다. 검증에 실패한 요청은 수락하지 않으며 기존 작업 상태를 변경하지 않습니다.

| 외부 상태 | 기존 실행기에 전달하는 skill | 완료에 필요한 조건 |
|---|---|---|
| SEARCHING | `search` | `object_found` |
| LOCALIZING | `localize_object` | `object_pose_valid` |
| ALIGNING | `align_for_pick` | `aligned_for_pick` |
| GRASPING | `pick` | `object_grasped` |
| VERIFYING | `verify_grasp` | `load_retained` |

`TaskExecutor`는 각 단계에서 먼저 `observe()`로 시작 가능 여부를 확인하고,
`start()`로 작업을 시작한 뒤 `poll()`로 결과를 확인합니다. 성공 응답뿐 아니라
해당 단계의 완료 조건과 기존 실행기의 지속 조건도 충족해야 다음 단계로 넘어갑니다.

노드는 0.1초 간격으로 manager를 진행시킵니다. 기본 단계 제한 시간은 10초이며
전체 작업 제한은 단계 제한의 5배입니다. 시간 계산은 monotonic clock을 사용하므로
ROS `/clock`이 멈추거나 변경되어도 watchdog은 계속 동작합니다.

상태 응답에는 두 종류의 상태가 있습니다.

- `state`: 사용자가 보는 단계명. 예: `GRASPING`, `COMPLETED`.
- `status`: 기존 실행기의 상태. 예: `WAITING`, `RUNNING`, `SUCCEEDED`.

예를 들어 정상 완료 결과는 `state=COMPLETED`, `status=SUCCEEDED`입니다.
완료 상태는 다음 tick에서 현재 상태 `IDLE`로 바뀌며, 이전 결과는 ID로 조회할 수 있습니다.

## 7. ROS2 인터페이스

### 사용자 요청과 상태

| 경로 | ROS 종류 | 타입 | 설명 |
|---|---|---|---|
| `/task/submit` | Service | `home_task_interfaces/srv/SubmitTask` | JSON 제출 |
| `/task/cancel` | Service | `home_task_interfaces/srv/CancelTask` | ID로 취소 요청 |
| `/task/get_status` | Service | `home_task_interfaces/srv/GetTaskStatus` | 현재 상태 또는 ID별 결과 조회 |
| `/task/status` | Topic | `std_msgs/msg/String` | JSON 상태 발행 |

상태 Topic의 QoS는 reliable, transient-local, depth 10입니다.
Gateway 모드에서는 `/task/submit`과 `/task/cancel`을 생성하지 않습니다.
인증·실행 허가를 받지 않고 작업을 변경하는 별도 경로를 열지 않기 위해서입니다.
상태 Topic과 조회 서비스는 계속 사용할 수 있습니다.

### 하위 작업 Action

| 단계 | Action 이름 |
|---|---|
| SEARCHING | `/perception/find_object` |
| LOCALIZING | `/perception/localize_object` |
| ALIGNING | `/navigation/align` |
| GRASPING | `/manipulation/pick` |
| VERIFYING | `/manipulation/verify_grasp` |

모든 Action은 새로 정의한 `home_task_interfaces/action/ExecuteSkill`을 사용합니다.
이 이름과 타입은 이번 프로토타입의 계약이며 기존 로봇의 인터페이스나
Nav2·MoveIt의 표준 Action을 그대로 연결한 것이 아닙니다.
코드에서는 상대 이름을 사용하므로 namespace와 remap을 적용할 수 있습니다.

## 8. Action Client의 데이터 흐름

`RosSkillTransport`는 노드 시작 시 Action Client 5개와 정지 확인 Service Client를
생성합니다. 작업마다 별도의 `RosSkillAdapter`를 만들어 피드백·결과·취소 상태가
다른 작업과 섞이지 않게 합니다.

Action Goal에는 다음 정보가 들어갑니다.

| 필드 | 용도 |
|---|---|
| `run_id` | 작업 ID. 예: `pick-001` |
| `goal_id` | 작업 ID와 단계명. 예: `pick-001/LOCALIZING` |
| `skill` | 수행할 skill 이름 |
| `parameters_json` | 대상 `target`과 이전 단계 출력 `context` |

모의 LOCALIZING 서버는 `output_json`으로 합성 `object_pose`를 반환합니다.
어댑터는 성공한 단계의 출력을 context에 병합하고 다음 Goal에 전달합니다.
따라서 정렬·파지·검증 단계는 위치 확인 결과를 전달받습니다. 모의 서버는
이 단계들에 `object_pose`가 없으면 Goal을 거절합니다.

Feedback/Result의 `goal_id`가 현재 Goal과 맞는지 확인하고,
`simulated=true`인 응답만 허용합니다. 조건 증거는 bool 값으로 구성된 JSON 객체여야 합니다.
ROS Action이 성공 상태로 끝나고 결과의 `success`가 참이어야 실행기에 성공을 전달합니다.

피드백 freshness는 **클라이언트가 받은 시각**을 기준으로 판단합니다.
`poll()`을 반복한다고 마지막 피드백 시각을 갱신하지 않습니다. 첫 피드백도 기존
실행기의 0.5초 freshness 제한 안에 도착해야 합니다. 아직 센서 관측 시각이나
TF 변환, pose의 물리적 정확도를 검증하는 기능은 없습니다.

## 9. 취소와 실패를 어떻게 처리하는가

취소 요청 수락과 실제 정지 확인을 구분합니다.

```text
사용자 취소 또는 실행 오류
    ↓
STOPPING: lease 유지, 새 작업 거부
    ↓
Action 취소 요청
    ↓
진행 중인 Action의 terminal 결과 대기
    ↓
ConfirmStop 서비스에서 정지·팔 유지·필요한 하중 보존 확인
    ↓
CANCELLED 또는 FAILED: lease 해제
```

중요한 처리 규칙은 다음과 같습니다.

1. Goal 승인이 아직 도착하지 않았으면 기다립니다. 뒤늦게 승인되면 즉시 취소를 요청합니다.
2. `cancel_goal_async()` 응답은 정지 증거로 사용하지 않습니다.
3. Action이 끝난 뒤에도 `/task_skills/confirm_stop`에서 독립적인 정지 증거를 받습니다.
4. 응답의 run ID, simulation 표시, 정지 여부와 조건을 검사합니다.
5. GRASPING 중이거나 이미 파지했다면 하중 보존 조건도 요구합니다.
6. 2초 동안 정지를 확인하지 못하면 `STOP_UNCONFIRMED`가 되며 lease를 계속 유지합니다.

`STOP_UNCONFIRMED`는 자동으로 해제되는 오류 표시가 아닙니다.
늦게 유효한 정지 확인이 도착하면 최종 취소·실패 상태로 이동할 수 있지만,
확인 전에는 다른 작업을 허용하지 않습니다.

실행 중 실패 응답, 시간 초과, 오래된 피드백, 잘못된 Goal ID, 완료 조건 누락도
정지 처리 경로로 연결됩니다. 전송이나 결과의 상태가 불확실하면 정지를 추정하지 않습니다.
취소 서비스의 `accepted=true`는 취소 요청 수락을 의미하며 정지 완료를 뜻하지 않습니다.

## 10. Gateway의 요청 처리 흐름

`gateway_client` 또는 HTTP 클라이언트는 다음 순서로 요청합니다.

1. Bearer 토큰으로 `/v1/permit`을 호출해 3초 동안 유효한 일회성 실행 허가를 받습니다.
2. `X-Request-Permit` 헤더와 Pick JSON을 `/v1/requests`에 보냅니다.
3. 기존 Gateway가 인증, 기존 요청 ID, 실행 허가를 확인합니다.
4. `PickApplication`이 요청을 Task Manager에 전달합니다.
5. Gateway가 요청과 초기 결과를 SQLite에 기록합니다.
6. 이후 ROS timer에서 단계 실행을 진행하고 결과를 SQLite에 갱신합니다.
7. `/v1/requests/{id}`로 조회하거나 `/v1/requests/{id}/cancel`로 취소합니다.

Gateway 요청 처리와 timer가 같은 lock을 사용하므로 요청 수락·초기 저장 중에는
timer가 해당 작업의 Action을 시작하지 않습니다. 저장 오류가 나면 기존 Gateway의
취소 처리가 적용됩니다.

HTTP 서버는 loopback인 `127.0.0.1`에 바인드합니다. 토큰은 파일에서 읽으며
CLI 인자에는 토큰 값 대신 파일 경로를 전달합니다. 기존 지도·이동 API는
Pick 전용 application에서 지원하지 않습니다.

## 11. 종료와 재시작

종료 시 새 HTTP 작업 수락을 차단하고 진행 중인 작업을 취소한 뒤,
최대 3초 동안 ROS 콜백을 처리하면서 정지 확인을 기다립니다.
확인이 없으면 로그에 정지 미확인을 남기며 성공적으로 정지했다고 보고하지 않습니다.

Gateway 재시작 시 SQLite에 미완료 작업이 있으면 다음과 같이 처리합니다.

- 기존 저장소가 해당 요청을 `INTERRUPTED`로 표시합니다.
- bridge가 외부 `state`도 `INTERRUPTED`로 맞추고 `stop_confirmation_required=true`를 추가합니다.
- Gateway의 fault를 유지해 새 실행 허가와 새 작업 제출을 차단합니다.
- 이전 작업은 자동으로 재개하거나 재실행하지 않습니다.

이 구현에는 자동 복구 API가 없습니다. 모의 실험에서는 모의 서버들을 종료하고
이전 DB를 보관한 뒤 새 DB 경로로 재시작할 수 있습니다. 이는 실제 하드웨어의
정지 검증·복구 절차를 대신하는 방법이 아닙니다.

## 12. 검증한 내용

구현 검증 시 Ubuntu 24.04, Python 3.12, ROS2 Jazzy 환경에서 패키지 빌드와
아래 테스트를 통과했습니다. 테스트 개수는 위 기준일의 실행 결과입니다.

| 테스트 파일 | 개수 | 확인한 내용 |
|---|---:|---|
| `test_core.py` | 11 | 정상 흐름, 요청 검증, 중복·충돌, busy, 단계별 취소·실패, timeout, 피드백 오류 |
| `test_gateway_bridge.py` | 7 | 인증·허가, 같은 lease 공유, 저장, 취소, fault, 재시작 차단 |
| `test_ros_adapter.py` | 6 | 승인 전 취소, 잘못된 ID, 비모의 결과 거부, 하중 증거 누락, 피드백 시각 유지 |
| `test_ros.py` | 12 | 실제 ROS 프로세스·DDS 통신, Action 연결, 서버 부재, 피드백 끊김, HTTP 전체 흐름과 CLI |
| **신규 패키지 합계** | **36** | 오류·실패·skip 없이 통과 |
| 기존 `test_home_execution.py`, `test_home_gateway.py` | **42** | 재사용한 실행기와 Gateway 회귀 테스트 통과 |

통합 테스트는 설치된 `ros2 run` 실행 파일로 Task Manager와 모의 Action 서버를
별도 프로세스에 띄웁니다. HTTP 인증과 permit을 거쳐 제출한 요청이 Action 5단계를
완료하는지 확인하고, 인증 없는 ROS 제출·취소 서비스가 없는지도 확인합니다.

실행 명령과 테스트 재현 방법은
[패키지 README](../ros2_ws/src/home_task_manager/README.md)에 정리되어 있습니다.

## 13. 코드 검토 순서와 다음 통합 지점

다음 순서로 읽으면 요청부터 완료까지 따라갈 수 있습니다.

1. `core.py`: JSON 검증, Pick 단계 생성, 기존 실행기 재사용 방식.
2. `node.py`: backend 선택, Gateway 모드, timer와 상태 발행.
3. `ros_adapter.py`: Action Goal·피드백·결과와 취소 경쟁 상황 처리.
4. `mock_skills.py`: 합성 pose와 성공·실패·정지 증거 생성 방식.
5. `gateway_bridge.py`: 기존 Gateway와 같은 manager·lease를 공유하는 방식.
6. `test_ros.py`: 실제 프로세스를 통한 전체 동작 검증.

실제 로봇 통합 전에 남은 작업은 실제 제어기의 Action 계약 확정, 센서 관측 시각과
TF 검증, 실측 정지·하중 보존 증거 연결, 명시적 복구 절차 구현입니다.
기존 Assistant 및 fetch/navigation 기능까지 합치려면 `RobotApplication`에 Pick 계획을
등록하고 모든 작업이 하나의 실행 권한을 사용하도록 통합해야 합니다.
