# D415 RGB-D 뷰어

로봇 연결 없이 D415의 RGB, RGB에 정렬한 깊이, 클릭한 픽셀의 카메라 기준
3D 좌표를 확인한다. 공장 보정값을 SDK에서 읽으며 장치 보정값을 변경하지 않는다.
기존 Top/wrist 카메라 보정 파일을 사용하지 않는다.

## 설치 및 실행

저장소 루트에서 Linux 데스크톱의 Python 3.12로 검증했다.
GUI용 OpenCV가 필요하므로 `requirements/host.txt`와 별도 환경을 사용한다.

```bash
python3 -m venv .venv-realsense
.venv-realsense/bin/python -m pip install -r requirements/realsense.txt
.venv-realsense/bin/python tools/diagnostics/realsense_viewer.py
```

- 왼쪽: RGB. 오른쪽: 같은 RGB 픽셀 좌표에 정렬한 깊이.
- 영상 클릭: 해당 픽셀의 Z 깊이, 카메라 원점부터의 직선거리(range), XYZ 표시.
- 좌표축: RGB 광학 좌표계 기준 X 오른쪽, Y 아래쪽, Z 전방. 단위는 m.
- 깊이 0은 `NO DEPTH`로 표시한다. 주변 픽셀이나 이전 프레임으로 대체하지 않는다.
- `S`: 현재 RGB, 원본 깊이, 정렬된 깊이, 미리보기, 메타데이터 저장.
- `Q` / `Esc` / 창 닫기: 카메라를 해제하고 종료.
- `F`: 원시 깊이와 공간 필터 미리보기 전환. 클릭 좌표와 Grounding DINO는 원시 깊이를 사용한다.
- 깊이 색상 범위 기본값은 0~3m. `--min-depth 0.3 --max-depth 1.2`로 작업대 거리를 강조할 수 있다.
  범위 밖 값은 색상만 포화되며 거리 표시·저장 데이터는 잘리지 않는다.
- RGB는 640×480 @ 30fps, 깊이 센서는 1280×720 @ 30fps로 수집한 뒤 RGB 좌표에 정렬한다.
  여러 D415가 있으면 `--serial`로 선택한다.

화면 없는 환경에서 유한 프레임 수신·저장 확인:

```bash
.venv-realsense/bin/python tools/diagnostics/realsense_viewer.py \
  --headless --frames 90 --save-on-exit
```

저장 경로는 기본 `tmp/realsense/<UTC 시각>/`이며 `--output`으로 변경한다.
`depth_native.png`, `depth_aligned.png`는 uint16 원시 깊이다.
미터 단위 깊이는 **픽셀 값 × metadata.json의 depth_scale_m**으로 계산한다.
메타데이터에는 센서별 intrinsics, depth→color extrinsics, 정렬된 intrinsics,
장치 시리얼·펌웨어, 센서 timestamp와 clock domain, 프레임 번호, 선택점 측정값이 있다.
PNG를 읽을 때는 OpenCV의 `IMREAD_UNCHANGED`를 사용해야 16비트가 유지된다.

## 해석 및 다음 단계

깊이 Z는 카메라 전방 축 방향 거리이며 일반 픽셀에서는 직선거리(range)와 다르다.
RGB와 깊이의 가려짐 차이 때문에 정렬 후에도 물체 경계와 가림 영역에는 빈 값이나
오차가 생길 수 있다. 좌표 계산용 깊이에는 hole filling이나 시간 필터를 적용하지 않는다.
프레임 번호 gap과 반복 수는 수신 상태 진단이며 거리 정확도 검증을 대신하지 않는다.

현재 범위는 독립 RGB-D 뷰어·스냅샷과 아래 Grounding DINO 물체 좌표 추정이다. ROS 카메라 매니저 및 perception runtime
연결은 아직 하지 않았다. 로봇 기준 좌표 변환도 아직 없으며 카메라 설치 후 별도로 보정한다.
실측 정확도는 실제 작업 거리의 평면 표적·알려진 높이를 준비한 뒤 비교한다.

장치가 없거나 권한/점유 문제로 실패하면 USB 연결과 다른 카메라 프로그램의 사용 여부를
확인한다. 창을 사용할 수 없는 세션에서는 위 headless 명령을 사용한다.

## Grounding DINO: 물체 탐지 → 카메라 XYZ

검증 환경은 RTX 5070 Ti, Python 3.12, CUDA 12.8 PyTorch다.

```bash
.venv-realsense/bin/python -m pip install torch==2.10.0 torchvision==0.25.0 \
  --index-url https://download.pytorch.org/whl/cu128
.venv-realsense/bin/python -m pip install -r requirements/grounding-rgbd.txt
.venv-realsense/bin/python tools/diagnostics/realsense_viewer.py \
  --prompt "black marker pen"
```

처음 실행할 때 공식 `IDEA-Research/grounding-dino-tiny` 모델을
`artifacts/models/huggingface/`에 다운로드한다. 모델 리비전은 코드에 고정한다.
영상과 추론은 로컬에 머문다. 모델 사용 방법은
[Hugging Face Grounding DINO 문서](https://huggingface.co/docs/transformers/model_doc/grounding-dino)를 따른다.

1. 물체가 보이도록 카메라를 둔다. 프롬프트는 영어로 지정한다.
2. **D**를 누르면 그 순간의 RGB와 정렬 깊이를 함께 저장해 분석한다.
3. 별도의 `Grounding DINO` 창에 **촬영 당시** 이미지, 탐지 박스, 점수와 XYZ가 표시된다.
   실시간 뷰어는 계속 재생되며 결과는 실시간 추적이 아니다. 다시 D를 눌러 갱신한다.
4. 원본 촬영 폴더 안 `grounding_<시각>/`에 `detections.jpg`, `detections.json`을 저장한다.
   JSON에는 모든 탐지 결과와 표면점의 픽셀·깊이·XYZ, 무효 사유, 모델 리비전,
   원본 파일 SHA256 및 RGB/depth 프레임 정보를 남긴다.

한 번 촬영·분석하고 종료:

```bash
.venv-realsense/bin/python tools/diagnostics/realsense_viewer.py \
  --prompt "black marker pen" --headless --frames 60
```

저장한 촬영본을 다른 프롬프트로 재분석:

```bash
.venv-realsense/bin/python tools/diagnostics/grounding_rgbd_snapshot.py \
  tmp/realsense/<촬영폴더> --prompt "black marker pen"
```

`--device cpu`로 CPU 실행이 가능하다. 기본값은 사용 가능한 GPU다.
`--box-threshold` 기본 0.30, `--text-threshold` 기본 0.25이며 탐지 점수는
거리 정확도나 해당 물체임을 보장하는 확률이 아니다.

### 좌표 의미와 깊이 거절 조건

출력은 **검출 박스 중앙 부근의 표면점 추정치**다. 물체의 3D 중심, 자세,
파지점 또는 로봇 좌표가 아니다. 카메라 기준 X 오른쪽·Y 아래·Z 전방, 단위 m다.
SDK의 정렬된 intrinsics와 `rs2_deproject_pixel_to_point`로 변환한다.

- 최대 9×9 중앙 패치를 박스 내부로 잘라 깊이를 조사한다.
- 유효 깊이 비율이 50% 미만이거나 유효 픽셀이 3개 미만이면 XYZ를 내지 않는다.
- 패치 깊이의 90백분위−10백분위 차이가 5cm를 넘으면 혼합 깊이로 거절한다.
- 통과하면 중앙값에 가장 가까운 **실제 깊이 픽셀**을 선택하고 그 픽셀 좌표로 역투영한다.
  중심 픽셀이 비어 있어도 주변 유효 픽셀을 선택할 수 있으며 선택 위치를 JSON에 기록한다.
- `NO_DETECTIONS`: 탐지 없음. `DETECTED_NO_DEPTH`: 박스는 있으나 유효 XYZ 없음.
  `OK`: 최소 한 후보의 표면점 계산 성공. 물체 인식·거리 정확도 검증 완료를 뜻하지 않는다.

마커펜처럼 얇은 물체는 검출 박스 중앙이 배경을 포함할 수 있다. 국소 깊이가
일관된 배경이면 이 검사만으로는 구별되지 않는다. 결과 화면에서 십자 표시가 물체
위인지 확인해야 한다. 픽셀 수준 물체 분리와 파지점 생성은 이 구현에 포함되지 않는다.

좌표 변환 시험:

```bash
.venv-realsense/bin/python -m pytest -c config/pytest.ini --rootdir=. -q tests/test_grounding_rgbd.py
```

## 깊이 품질과 표시 조정

기본 깊이 해상도를 D415 권장 1280×720으로 올렸다. RGB는 기존 크기로 유지한다.
높은 해상도에서는 최소 측정 거리가 늘어난다. D415 데이터시트의 Min-Z는
1280×720에서 약 45cm, 640×480에서 31cm, 640×360에서 24cm다.
작업 거리는 각각 50cm, 35cm, 30cm 이상에서 우선 확인한다. 이는 정확도 보장이
아니며 검은 재질, 광택, 각도와 가림에 따라 더 먼 거리에서도 누락될 수 있다.
가까운 작업에는 `--depth-width 640 --depth-height 360`을 사용할 수 있다.
[D400 데이터시트](https://www.realsenseai.com/wp-content/uploads/2022/04/Intel-RealSense-D400-Series-Datasheet-April-2022.pdf),
[공식 튜닝 가이드](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)

기본 `--depth-filter spatial`은 원시 해상도에서 깊이를 시차로 바꿔 경계를 보존하는
공간 필터를 적용하고, 다시 깊이로 변환한 후 RGB에 정렬한다. 얇은 물체의 정보를
줄이는 decimation과 hole filling은 적용하지 않는다. 실제 원시 정렬 픽셀이 무효면
미리보기도 무효로 유지한다.

- `--depth-filter off`: 원본 보기.
- `--depth-filter spatial`: 공간 평활화. 기본값.
- `--depth-filter temporal`: 공간+시간 평활화. 정지한 장면에 적합하며 움직일 때 잔상이 생길 수 있다.
  이전 깊이로 빈 픽셀을 메우는 persistence는 끈다.
- `--laser-power 240`: 지원 범위 내 IR 출력을 명시적으로 시험한다. 생략하면 현재 값을 유지한다.
  종료 시 기존 값으로 복원한다. 출력 증가는 모든 재질에서 개선을 보장하지 않는다.

필터는 **표시 전용**이다. 거리·XYZ 및 Grounding DINO는 `depth_aligned.png`의
원시 정렬 깊이로 계산한다. 필터 결과는 `depth_preview_filtered.png`에 별도로 저장한다.
`depth_native.png`에는 1280×720 원본이 남고, 메타데이터에는 해상도·센서 옵션·필터·색상 범위를 기록한다.
색상 범위를 좁혀 대비가 좋아져도 측정 정확도가 달라지는 것은 아니다.
[공식 후처리 설명](https://dev.realsenseai.com/docs/post-processing-filters/)

작업대용 실행 예:

```bash
.venv-realsense/bin/python tools/diagnostics/realsense_viewer.py \
  --prompt "black marker pen" --min-depth 0.3 --max-depth 1.2
```

검은색·광택·가는 물체의 누락은 필터만으로 해결되지 않는다. 물체 거리, 촬영 각도,
IR 영상의 노출을 먼저 확인하고 실측 오차는 별도로 검증한다. 프리셋과 장치 내부
캘리브레이션 값은 이 개선에서 변경하지 않는다.

## Jetson 배포 및 터미널 키워드 검색

배포 위치는 `hyper@192.168.35.236:/home/hyper/realsense-grounding`이다.
Jetson Orin Nano Super, JetPack 7.2.1 / L4T 39.2.1, Ubuntu 24.04,
Python 3.12, CUDA 13.2에서 설치했다. 다른 JetPack 버전에 이 PyTorch 조합을
그대로 적용하지 않는다.

현재 배포본 실행:

```bash
ssh hyper@192.168.35.236
cd ~/realsense-grounding
bash tools/run/realsense_jetson.sh near
```

`Find object>`에 `black marker pen`, `blue screwdriver`, `pliers` 같은 영어
물체 설명을 입력한다. 첫 검색에서 모델을 한 번 로드하고, 이후에는 같은 모델을
유지한다. 입력할 때마다 새 RGB-D를 촬영하고 분석한 뒤 다음 입력을 기다린다.
`q` 또는 Ctrl-D로 종료한다. SSH에서도 창 없이 동작한다.

- `near`: 깊이 640×360, 표시 범위 0.2–0.8m. 약 30cm 이상 근거리 시험용.
- `far`: 깊이 1280×720, 표시 범위 0.3–2m. 약 50cm 이상 원거리 시험용.
- 두 프로필 모두 RGB 640×480, 요청 30fps, 원시 정렬 깊이로 좌표를 계산한다.
- 매 검색은 노출 안정화 30프레임 후 기본 30프레임을 수신해 마지막 프레임을 분석한다.
  `--frames 60` 등으로 조정할 수 있다. 입력 후 촬영·저장 시간은 추론 시간과 별도다.
- 카메라는 검색 시 열고 분석 전에 해제한다. 자동 거리 전환이나 연속 추적은 하지 않는다.

한 번만 검색하고 종료하거나 자연어 처리 프로그램에서 호출하려면:

```bash
bash tools/run/realsense_jetson.sh near \
  --prompt "black marker" --headless --frames 30
```

PC에서도 `realsense_viewer.py --interactive-prompt`로 같은 입력 루프를 사용할 수 있다.
나중에 자연어 처리 모듈이 추출한 영어 `target_description`을 `--prompt`에 전달하거나,
Python에서 `GroundingRGBD.analyze_snapshot(snapshot_dir, target_description)`를 호출한다.
현재는 한국어 명령의 의도 분석이나 번역을 구현하지 않았다.

결과는 `tmp/realsense/<촬영시각>/grounding_<분석시각>/detections.json`과
`detections.jpg`에 저장한다. 터미널에는 후보별 점수와 카메라 XYZ(m)를 출력한다.
여러 후보 중 하나를 파지 대상으로 자동 확정하지 않는다. 실제 작업대 시험에서
`black marker`는 마커펜 외에 검정·파랑 드라이버도 검출했으므로 점수가 가장 높은
박스가 반드시 요청한 물체라는 가정은 사용할 수 없다.

새 환경을 준비하는 경우 시스템 OpenCV를 재사용한다:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install torch==2.12.1 torchvision==0.27.1 \
  --index-url https://download.pytorch.org/whl/cu132
.venv/bin/python -m pip install -r requirements/grounding-rgbd-jetson.txt
.venv/bin/python tools/diagnostics/realsense_environment.py --output tmp/environment.json
```

현재 모델 캐시는 `artifacts/models/huggingface/`에 함께 배포되어 있다.
런처는 기본 `HF_HUB_OFFLINE=1`로 실행한다. 새 설치에서 모델이 아직 없으면
첫 실행만 `HF_HUB_OFFLINE=0 bash tools/run/realsense_jetson.sh near`로 다운로드한다.
PC용 requirements의 PyTorch·OpenCV 핀은 이 환경에 설치하지 않는다.

CUDA tensor 연산, torchvision CPU NMS, D415 USB 3 스트리밍과 실제 Grounding DINO
CUDA 추론을 확인했다. 사용 중인 PyTorch wheel은 Orin의 CC 8.7 지원 경고를
출력한다. 현재 시험한 추론 경로는 성공했으나 Jetson 전용 최적화 빌드로 검증한
환경은 아니다. 다른 CUDA 연산을 추가할 때는 별도 확인이 필요하다.

### 2026-10-10 배포 시험 결과

- 기하·깊이 거절·원본 보존·검색 입력 전환 시험 13개가 PC와 Jetson 모두 통과했다.
- 카메라 단독 90프레임 시험: near 공간 필터 약 29.7fps, 프레임 번호 누락 0;
  far 공간 필터 약 16.4fps, 깊이/컬러 각각 73프레임 번호 누락. 이 시점에는 패키지
  설치도 병행 중이었으므로 성능의 상한이나 원인을 확정하는 벤치마크는 아니다.
- 실제 터미널 검색 30프레임 촬영: 약 27–28fps, 일부 프레임 번호 누락 2–3개.
  Grounding DINO는 촬영 후 실행하며 이 fps는 탐지 처리율이 아니다.
- 같은 프로세스에서 `black marker pen` → `blue screwdriver` 입력을 바꾸어
  새 촬영과 모델 재사용을 확인했다. 추론 시간은 각각 약 2.80초, 1.55초였다.
  초기 모델 로드·촬영·파일 저장 시간은 제외하며 한 장면에서의 관측값이다.
- 마커펜 촬영의 실제 마커 박스: 점수 0.49, 표면 XYZ 약
  `(0.093, -0.046, 0.415)`m. 드라이버 오탐도 점수 0.35로 함께 나왔다.
- `blue screwdriver`: 드라이버 한 개, 점수 0.73, 표면 XYZ 약
  `(0.060, -0.005, 0.286)`m. 좌표의 실측 오차와 파지 성공률은 아직 검증하지 않았다.

Jetson 증거 파일은 `tmp/jetson-terminal-test/` 아래, 배포 전후 환경 기록은
`tmp/jetson-environment.json`에 있다. 개발 PC 사본은
`tmp/jetson-deploy/results/terminal/` 및 `tmp/jetson-deploy/jetson-environment.json`이다.

## 이 컴퓨터에서 Jetson 영상 보기

개발 PC 브라우저에서 [RGB-D 화면](http://127.0.0.1:18765)을 연다.
RGB와 깊이 미리보기, 카메라 수신 fps·유효 깊이 비율, 물체 검색 입력란을 제공한다.
`찾고 마스킹`을 누르면 서버가 그 시점의 최신 RGB-D 프레임을 고정해 분석한다.
결과 박스·카메라 XYZ·촬영 시각은 실시간 영상 아래에 별도로 표시한다.
프레임 수신이 끊기면 연결 지연을 표시하고 새 검색을 막는다.

개발 PC의 저장소 루트에서 실행한다. Jetson 서버와 SSH 터널을 함께 유지하므로
이 명령이 실행 중이어야 브라우저가 연결된다:

```bash
bash tools/run/realsense_remote_viewer.sh near
```

현재 세션에서는 개발 PC의 임시 사용자 서비스 `realsense-jetson-viewer`로 실행했다.
브라우저를 닫아도 카메라는 켜져 있으며, 종료하려면 개발 PC에서 실행한다:

```bash
systemctl --user stop realsense-jetson-viewer.service
```

다음 사용 때 위 스크립트로 다시 연결한다. `far`를 주면 원거리 프로필로 실행한다.
기존 서버나 터미널 뷰어가 카메라를 사용 중이면 먼저 종료해야 한다.
기본 연결 대상은 `hyper@192.168.35.236`, 원격 배포 디렉터리는
`~/realsense-grounding`이며 필요하면 `JETSON_TARGET`으로 SSH 대상을 지정한다.

Jetson HTTP 서버는 `127.0.0.1:8765`, PC 터널은 `127.0.0.1:18765`에만 바인딩한다.
외부 호스팅 없이 기존 SSH 연결로 영상이 전달된다. 추가 웹 패키지는 필요 없다.
미리보기는 JPEG로 최대 8fps 갱신하며 화면의 카메라 수신 fps와는 다르다.
분석은 Jetson에서 원본 RGB와 16비트 정렬 깊이로 수행한다. 분석 중에도 카메라
수신은 계속되지만 연산 부하에 따라 속도는 낮아질 수 있다.
결과 원본은 Jetson의 `tmp/realsense-web/<촬영시각>/` 아래에 저장한다.

웹 배포 파일은 `tools/diagnostics/realsense_web.py`, `realsense_web.html`,
공통 `realsense_viewer.py` 및 기존 `tools/lib/grounding_rgbd.py`다.

## SAM 물체 마스킹

브라우저의 **찾고 마스킹**은 Grounding DINO 탐지 박스를
[SAM 2.1 Tiny](https://huggingface.co/facebook/sam2.1-hiera-tiny)에 전달한다.
동일 촬영본의 RGB에서 후보별 마스크를 구해 색 채움·윤곽으로 표시한다.
모델은 기존 Transformers 4.57.6의 `Sam2Model`/`Sam2Processor`를 사용하며,
리비전 `de431c4043854a71d8101e17995dfe596bf101a5`로 고정한다.
추가 pip 패키지 없이 SAM 모델 캐시와 `tools/lib/sam_rgbd.py`를 함께 배포한다.
[박스 프롬프트 사용법](https://huggingface.co/docs/transformers/v4.57.1/en/model_doc/sam2#bounding-box-input)

CLI에서는 기존 박스 방식에 `--sam`을 추가한다:

```bash
bash tools/run/realsense_jetson.sh near --sam
.venv/bin/python tools/diagnostics/grounding_rgbd_snapshot.py \
  tmp/realsense/<촬영폴더> --prompt "black marker pen" --device cuda --sam
```

- 결과 폴더에 후보별 `mask_000.png`, `mask_001.png` 등을 저장한다.
  원본 RGB와 동일한 해상도이며 255는 물체, 0은 배경이다.
- `detections.json` schema 2는 마스크 파일·SHA256·면적·SAM 예상 IoU,
  모델 리비전과 탐지/마스킹별 시간을 기록한다. `inference_seconds`는 두 시간의
  합이며 최초 모델 로드와 촬영·디스크 저장 시간은 제외한다.
- XYZ는 마스크를 1픽셀 줄인 내부에서 계산한다. 너무 얇아 내부 픽셀이
  3개 미만이면 원래 마스크를 사용한다. 마스크 중심에 가까운 실제 내부 픽셀
  주위의 최대 9×9 패치에서 **마스크 내부에 있는 깊이만** 검사한다.
- 패치 내부의 깊이 유효 비율 50%, 최소 3픽셀, 깊이 p90–p10 5cm 이하 조건을
  유지한다. 통과 시 깊이 중앙값에 가까운 실제 측정 픽셀로 XYZ를 계산한다.
  마스크 외부나 필터링된 깊이로 대체하지 않는다.
- 마스크가 비면 `empty_mask`, SAM 예상 IoU가 0.5 미만이면 `low_mask_quality`로
  좌표를 거절한다. 이 임계값은 초기 품질 필터이며 검증된 정확도 보장이 아니다.

SAM 품질 점수는 마스크 예측 품질이지 요청한 물체일 확률이 아니다. DINO가
드라이버를 마커펜으로 잘못 잡으면 SAM도 그 드라이버의 윤곽을 만들 수 있다.
마스킹이 깊이 센서의 최소 거리, 검은 재질의 깊이 누락, 잘못된 깊이값을
해결하지는 않는다. 출력은 보이는 표면점이며 물체의 3D 중심이나 파지 자세가 아니다.
현재는 촬영본별 마스킹이고 실시간 SAM 비디오 추적은 사용하지 않는다.

2026-10-10 검증: PC와 Jetson에서 관련 테스트 20개 통과. Jetson 브라우저의
`yellow scissor` 실물 검색에서 DINO 4.02초 + SAM 1.11초 = 5.12초를 관측했다.
가위 외에 마커펜 오탐도 마스킹되었으며, SAM 품질이 높아도 의미상 오탐을
제거하지 못한다는 점을 확인했다. 첫 요청의 모델 준비까지 포함한 응답은
약 31초였다. 모델은 이후 요청에 재사용한다. 기록 사본은 개발 PC의
`tmp/jetson-deploy/results/sam/`에 있다.
