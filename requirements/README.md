# Python 의존성

`host.txt`는 하드웨어 없는 Python 시험과 설정 검사에 사용한다. ROS 2 Jazzy, colcon, CMake와 ARM 컴파일러는 운영체제에서 별도로 설치한다. RGB-D 실험은 아래 별도 환경을 사용하며 LiDAR·통합 모델 런타임은 별도 선정한다.

현재 실물 제어 시험은 Pi 5/Ubuntu 24.04.3이다. D415 물체 탐지·분할은 Jetson Orin Nano Super에서 독립 실행을 검증했으며 제어 런타임의 Jetson 전환은 별도다.
VLM과 필요 시 열린 어휘 탐지/분할 런타임은 [U1 평가](../docs/ROADMAP.md#u1-vlm-요청-해석추가-학습-없는-물체-판단)에서
추론 시간·메모리·제어/센서 동시 부하를 측정한 뒤 추가한다. 기존 ONNX 의존성이 임의 VLM까지 지원한다는 뜻은 아니다.
모델/가중치/프롬프트·입력 크기·양자화·실행 위치와 라이선스를 고정해 재현한다. 새 모델을 실제 채택할 때
[제3자 고지](../docs/THIRD_PARTY_NOTICES.md)를 갱신한다. 현재 VLA 학습 환경이나 리더암 데이터 수집은 필수 의존성이 아니다.

## D415 RGB-D 실험 환경

- `realsense.txt`: PC 뷰어용 별도 `.venv-realsense` 환경.
- `grounding-rgbd.txt`: Grounding DINO + SAM 2.1 탐지·분할·카메라 표면점 좌표.
- `grounding-rgbd-jetson.txt`: JetPack 7.2.1 / CUDA 13.2 / Python 3.12용; 시스템 OpenCV 재사용.

설치와 원격 화면 실행은 [D415 안내](../docs/REALSENSE.md)를 따른다. 모델 가중치는 저장소에 포함하지 않는다.
