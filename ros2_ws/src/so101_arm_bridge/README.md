# SO101 arm bridge

STM32가 제어하는 SO101 양팔의 protocol-v2 스트림을 ROS 2에 연결한다. 기존 resident 양팔 경로를 보존하고, 단일팔 v1 실행기와 고정 작업대용 명령 경로를 제거했다.

## 빌드

저장소 루트에서 ROS 2 Jazzy 환경을 불러온 뒤 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`rclpy`, `ament_index_python`, `launch_ros`, `sensor_msgs`, `std_srvs`, `trajectory_msgs`, ROS interface generators와 `python3-serial`이 필요하다. 팔 모델에는 `robot_state_publisher`와 `xacro`가 필요하다.

## 실행

```bash
ros2 launch so101_arm_bridge bimanual_stream.launch.py motion_authorized:=false
```

이 명령은 실제 장치와 통신하므로 여기서 자동 실행하지 않는다. 기본 설정에서는 새 위치 명령과 토크 활성화를 허용하지 않지만, 초기화는 연결된 펌웨어의 준비·토크 비활성화·상태 읽기 경로를 사용한다. 이미 다른 제어기가 동작 중인 장치에 연결하지 않는다.

기존 패키지 `single_arm_bridge`의 실행 명령은 사용할 수 없다. 배포할 때 오래된 overlay와 동시에 실행하지 않도록 한다.

## 인터페이스

- `~/command`: `BimanualStreamCommand`로 유한 경로, 연속 스트림, append·splice·stop 요청.
- `~/joint_states`, `/joint_states`: 실제 관절 위치.
- `~/feedback`: `BimanualJointFeedback`, 축별 측정 나이와 유효 축 표시.
- `~/status`: 상태 조회 서비스. 준비 상태·소유자·오류 정보는 구현의 응답을 따른다.

ROS 브릿지는 STM32로 가는 host USB UART를 단독 소유하고, 모터 UART는 STM32가 소유한다. 베이스 이동, 카메라 인식, 작업 성공 판정은 상위 기능의 책임이다. 관절 피드백만으로 물체의 집기·놓기를 확인할 수 없다.

현재 어댑터는 펌웨어 `0x00024908`와 보존된 관절 한계를 요구한다. 이는 이번 오프라인 소프트웨어 후보 계약이며 이전 `0x00024809`의 실기 승인과 구분한다. 이 버전도 새 모바일 플랫폼의 실물 동작 승인이 아니다. 하드웨어 변경 시 명령·충돌·정지·관측 조건을 다시 확인한다.


## 모바일 명령 연결 상태

`MobileClient(MobileV2Exchange(existing_transport), clock_ms)`는 팔 transport와
같은 포트·transaction lock을 공유한다. Python→v2 COBS→실제 C parser→모바일
endpoint/supervisor/router 왕복을 `test_mobile_framed_transport.py`에서 검사한다.
여기서 팔 GET_STATE는 합성 응답이며 실물이나 실제 팔 executor 시험은 아니다.

부분 송신은 오류로 반환하고 명령을 자동 재전송하지 않는다. USB 송신 완료의
무기한 flush를 제거했고 sequence는 wrap 전에 거절한다. 각 직렬 읽기/쓰기는
기존 finite timeout을 사용한다. 동기 요청 하나가 끝날 때까지 다른 요청은 기다리므로
실제 모바일 운용 전에 서비스/ARM 같은 긴 요청과 주기 출력을 모드별로 분리해야 한다.

실제 ROS node는 아직 모바일 client를 생성하거나 cmd_vel을 전달하지 않는다.
보드 endpoint 기본값도 미연결이다. 주기 출력·실제 feedback·전체 stop/hold를
연결한 뒤 같은 소유자 안에서 활성화해야 한다.


## SO-101 PID/모션 후보 호환성

현재 어깨는 P64/I0/D64, 팔꿈치는 P56/I0/D64, 나머지는 P16/I0/D32다.
firmware `0x00024908`, hash `0x2D90167E`이며 하중 오차 이력에 따라 이전 gain을 복원했다.
P16 일괄 후보는 철회했으며 실측 최적화 완료를 뜻하지 않는다.
`check_servo_profile.py --check`가 실제 C 설정과 host hash를 비교한다. PID만 바꾸고
이전 host를 쓰면 준비 단계에서 거부한다. 관절 한계·torque·추종 오류 제한은 그대로다.

기존 resident position stream 자체의 보간 계약은 선형 5ms다. task waypoint 평활화와
MoveIt Ruckig는 upstream에서 수행하며, 여기서 시간/위치를 임의로 필터링하거나 변경하지
않는다. 자세한 실측 비교/로그 형식은 [시스템 구조](../../../docs/ARCHITECTURE.md)를 따른다.
