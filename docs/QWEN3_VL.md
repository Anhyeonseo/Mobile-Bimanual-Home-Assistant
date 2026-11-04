# Qwen3-VL RGB-D grounding 실험

한국어 요청과 RGB 이미지 한 장을 Qwen3-VL에 함께 전달해 물체 이름과
bounding box를 JSON으로 받는다. 별도 번역기나 Grounding DINO를 호출하지 않는다.
현재는 독립 실험 스크립트이며 RealSense 웹/Jetson 인식 경로의 교체는 아직 아니다.

## 설치

```bash
python3 -m venv .venv-qwen3-vl
.venv-qwen3-vl/bin/pip install torch==2.11.0 torchvision==0.26.0 \
  --index-url https://download.pytorch.org/whl/cu128
.venv-qwen3-vl/bin/pip install -r requirements/qwen3-vl.txt
```

## 실행

### 가까운 책상 장면: GraspNet 공식 데모 샘플

RGB 1280×720에 바나나, 파란 컵, 드릴, 상자 등이 가까이서 촬영돼 있다.
공식 [GraspNet baseline 데모](https://github.com/graspnet/graspnet-baseline/tree/280c215129f759ed8649cb4e89fc5dfee55f4f80/doc/example_data)의
RGB·원본 깊이·workspace mask·카메라 파라미터를 사용한다.
2026-10-31에는 샘플 준비와 포인트클라우드 내보내기만 확인했고,
이 샘플에 대한 Qwen/SAM/GraspNet 추론은 실행하지 않았다.

저장소 루트 터미널에서 순서대로 실행한다.

1. 샘플 준비와 원본 이미지 확인:

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/prepare_graspnet_sample.py
xdg-open artifacts/datasets/graspnet-example/color.png
xdg-open artifacts/datasets/graspnet-example/depth_preview.png
```

2. 원하는 한국어 문장을 직접 입력해 Qwen 검출:

```bash
read -r -p '찾을 물체/요청: ' prompt
.venv-qwen3-vl/bin/python tools/diagnostics/qwen3_vl_rgbd.py \
  --image artifacts/datasets/graspnet-example/color.png \
  --model artifacts/models/qwen3-vl-4b --device cuda \
  --prompt "$prompt" --max-new-tokens 1024 \
  --output tmp/qwen-graspnet-test
xdg-open tmp/qwen-graspnet-test/detections.jpg
```

예: `바나나를 찾아줘.`, `파란 컵과 드릴을 각각 찾아줘.`
`response.txt`는 Qwen 원문, `detections.json`은 파싱한 박스다.
여기서 `--image`는 RGB만 읽으며, 깊이와 intrinsics는 원본 샘플 폴더에 유지한다.

3. 전체 장면 및 작업 영역 포인트클라우드 저장/보기:

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/prepare_graspnet_sample.py --pointcloud
.venv-qwen3-vl/bin/python tools/diagnostics/prepare_graspnet_sample.py --view
```

`--view`는 데스크톱 GUI가 필요하다. 화면 없는 환경은 `--pointcloud`만 사용한다.
Open3D 창에서 마우스로 회전·확대해 관찰하고 창을 닫으면 종료한다.

`artifacts/datasets/graspnet-example/`에 저장:

- `color.png`, `depth.png`, `workspace_mask.png`, `meta.mat`: 공식 원본.
- `source.json`: 고정 소스 리비전, 파일 SHA256, intrinsics와 깊이 단위.
- `depth_m.npy`: `raw_depth / factor_depth`로 변환한 미터 단위 깊이. 절단 없음.
- `depth_preview.png`: 0–1.5m 표시 범위, 무효 깊이는 검정색.
- `scene.ply`: 모든 유효 깊이의 컬러 포인트클라우드.
- `workspace.ply`: 공식 workspace mask 내부의 컬러 포인트클라우드.

실제 데이터의 `factor_depth=1000`, 유효 깊이 중앙값 약 0.451m다.
내보내기 확인: 전체 825,160점, workspace 513,688점.
좌표는 카메라 기준 X 오른쪽·Y 아래·Z 전방이며 단위는 m다.
workspace mask는 책상 작업 영역이고 개별 물체 마스크가 아니다.
전체 장면은 충돌 검사에 사용할 수 있도록 별도로 유지한다.
대상 물체 분리와 GraspNet 후보 생성은 아래 단계를 따른다.

### 기존 Open3D TUM 샘플

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/qwen3_vl_rgbd.py \
  --sample tum --device cuda \
  --prompt '지구본, 병, 노란 주사위, 책, 컵을 모두 각각 찾아서 물체마다 별도의 박스를 그려줘. 책상 자체는 제외해.'
```

### Qwen 박스 → SAM 마스크 → 물체 포인트클라우드

직접 만든 `tmp/qwen-graspnet-test/detections.json`을 입력한다.
RGB 내용이 GraspNet 샘플과 같은지 해시로 검사하고 기존 SAM 2.1 Tiny 모듈에
박스를 전달한다. 첫 실행에는 공식 SAM 모델 다운로드가 필요하다.

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/qwen_object_cloud.py \
  tmp/qwen-graspnet-test/detections.json --device cuda
xdg-open tmp/qwen-object-cloud/masks.jpg
```

3D 결과를 보려면 같은 명령에 `--view`를 추가한다.
전체 장면은 회색, 대상 물체는 초록색으로 표시한다. 저장한 PLY에는 원래 RGB를 유지한다.
여러 박스 중 하나만 처리하려면 `--index 0`처럼 0부터 시작하는 번호를 지정한다.

출력 `tmp/qwen-object-cloud/`:

- `mask_000.png`: 원본 RGB 크기 물체 마스크, 255가 물체.
- `masks.jpg`: RGB 위 초록색 마스크 미리보기.
- `object_000.ply`: 마스크 안의 유효 원시 깊이와 workspace ROI를 통과한 대상 점.
- `scene.ply`, `workspace.ply`: 전체 장면/작업 영역. 충돌 검사용으로 별도 보존.
- `objects.json`: 라벨, 마스크 면적, 유효점 수, SAM 예상 IoU, 단위, intrinsics,
  모델 리비전 및 입력 파일 SHA256.

SAM 예상 IoU가 0.5 미만이거나 유효점이 3개 미만이면 `REJECTED`로 기록하고
대상 PLY를 만들지 않는다. 이 값은 물체 인식 확률이나 검증된 정확도가 아니다.
결과에는 가려진 뒷면이 없으며 마스크/깊이 오차도 남을 수 있다.
이 명령은 물체 표면점 추출까지 수행한다. GraspNet 추론은 아래 별도 명령으로 실행한다.
SAM 박스 입력은 [공식 Transformers 문서](https://huggingface.co/docs/transformers/v4.57.1/en/model_doc/sam2#bounding-box-input)를 따른다.

기본 모델은 `Qwen/Qwen3-VL-4B-Instruct`(리비전 `ebb281ec70b05090aa6165b016eac8ec08e71b17`)이며
공식 모델 리비전을 고정한다. 2B와의 비교와 선택 이유는 아래 `2B·4B 비교`를 본다.
기본 TUM 샘플은 책상 위에 지구본, 여러 병, 컵, 주사위 등이 놓인 장면이다.
`--sample sun` 또는 `--sample redwood`로 다른 예제를, `--image /path/to/rgb.png`로 자체 RGB를
사용할 수 있다. 자체 RGB 입력에는 깊이 파일을 자동 연결하지 않는다.
`--prepare-only`는 모델 로드 없이 샘플만 준비한다. `--output`으로 결과 폴더를 바꾼다.

기본 출력 `tmp/qwen3-vl-rgbd/`:

- `rgb.png`: 추론 입력 RGB 원본 크기.
- `depth_original.png`: Open3D 예제의 깊이 원본 인코딩을 그대로 보관.
- `depth_m.npy`: Open3D의 해당 데이터셋 변환 함수를 사용한 미터 단위 깊이.
  기본 3m 절단이 적용되므로 원본 대신 쓰지 않는다.
- `depth_preview.png`: 0–3m 깊이 미리보기. 무효값은 검정색이다.
- `source.json`: 데이터 출처, 크기, RGB SHA256.
- `response.txt`: 모델 응답 원문.
- `detections.json`: 한국어 요청, 실제 모델 입력, 모델 리비전, 시간, 정규화/픽셀 박스.
- `detections.jpg`: 원본 크기 이미지 위에 박스와 영문 라벨 표시.

Qwen3-VL 공식 예제의 `[x1,y1,x2,y2]` 0–1000 좌표를 원본 이미지 크기에
비례 변환한다. JSON 구조, 좌표 범위와 순서를 검사하며 잘못된 응답은 실패로 기록한다.
4B는 닫는 `]`를 빠뜨리거나 두 물체를 한 `{...}`에 합쳐 내는 경우가 있어, JSON 파싱이
실패하면 완결된 `"label"`/`"bbox_2d"` 쌍만 추출한다. 추출한 박스도 같은 범위·순서 검사를
거치며, 완결된 쌍이 하나도 없으면 실패다.
빈 배열은 `NO_DETECTIONS`다. 모델 confidence 점수는 생성하지 않는다.

깊이는 보관하지만 **Qwen 입력은 RGB와 텍스트뿐**이다. 이 실험은 XYZ, 마스크,
로봇 좌표 또는 파지점을 산출하지 않는다. 박스의 정확도는 결과 이미지에서 확인한다.

## 2026-10-30 실제 GPU 실행

RTX 5070 Ti 16GB, PyTorch 2.11.0+cu128, Transformers 4.57.6,
Open3D 0.20.0에서 공식 2B 모델의 BF16 CUDA 추론을 실행했다.
현재 실험 환경은 `.venv-yolo`의 CUDA 패키지를 새 가상환경의 `local-cuda.pth`로
참조한다. 다른 컴퓨터에서는 위 설치 명령으로 독립 환경을 만든다.
이 실행 세션의 샌드박스에서는 GPU 장치가 보이지 않아, GPU 접근이 가능한
실행 환경에서 명령을 실행했다. 드라이버 변경은 필요하지 않았다.

- `책상 위의 노란 주사위를 찾아줘.`: 주사위 박스 1개, 약 0.65초.
- 위 물체 목록을 명시한 요청: 지구본 1개, 병 6개, 주사위 1개, 책 1개,
  컵 1개 등 박스 10개, 약 3.36초.
- `책상 위에 있는 물체들을 각각 찾아줘.`: 물체 목록 대신 책상 박스 1개를 반환했다.
  포괄적인 한국어 지시를 안정적으로 처리하는지는 추가 평가가 필요하다.

시간은 단일 장면에서 최초 모델 로드·다운로드·저장을 제외한 생성 시간이다.
정답 박스 대비 IoU나 recall을 측정한 결과는 아니다. 모델은 영문 라벨 지시에도
한국어 라벨을 반환했으며, 시각화에는 설치된 Nanum 한글 폰트를 사용한다.

결과는 `tmp/qwen3-vl-rgbd/{all,objects,dice}/`에 저장했다.
이번에 받은 공식 모델 파일은 `artifacts/models/qwen3-vl-2b/`에 있다.
재다운로드 없이 사용하려면 위 명령에 `--model artifacts/models/qwen3-vl-2b`를 추가한다.

출처: [Qwen 모델 카드](https://huggingface.co/Qwen/Qwen3-VL-2B-Instruct),
[공식 2D grounding 예제](https://github.com/QwenLM/Qwen3-VL/blob/main/cookbooks/2d_grounding.ipynb),
[Open3D 예제 데이터](https://www.open3d.org/docs/release/python_api/open3d.data.html).

## 2B·4B 비교 (2026-11-02)

RTX 5070 Ti 16GB, BF16, 같은 코드·데이터(`data/tabletop_rgbd_20261101`)로 두 모델을 측정했다.
요청 해석은 `run_request_eval.py`, 타깃 선택은 `run_target_selection_eval.py`,
존재 확인은 `run_presence_eval.py` 결과다.

| 항목 | 2B | 4B |
|---|---|---|
| 요청 해석, 개발셋 93문장 | 83 | 88 |
| 요청 해석, 홀드아웃 40문장 | 34 (85.0%) | 36 (90.0%) |
| 타깃 선택 15건 | 14 | 15 |
| 있는 물체 찾기, 검증 후 25건 | 24 | 24 |
| 없는 물체 오검출, 검증 후 25건 | 0 | 0 |
| 없는 물체 오검출, 검출기 단독 25건 | 25 | 4 |
| GPU 메모리 피크(단일 장면) | 4.7GB | 9.2GB |
| 요청 해석 시간(문장당) | 0.21초 | 0.39초 |

4B의 타깃 선택 15건은 위 JSON 복구가 있어야 나오는 값이다. 복구 없이 실패로 처리하면 11건이다.
홀드아웃과 타깃 선택의 차이는 각각 2문장, 1건으로 표본이 작다.
뚜렷한 차이는 검출기 단독 오검출(25→4)로, 검증 단계 부담이 줄어든다.
위 요청 해석 수치는 이전 프롬프트 기준이다. 이전 프롬프트는 수식어가 붙은 예시가
`왼쪽 모자 → 모자` 하나뿐이어서, 두 모델 모두 `검은 컵 → 컵`처럼 색·소유 꾸밈말까지 뗐다.
`task_parser.PROMPT`에 "색·`내`는 포함, 위치 단어는 제외" 규칙과 꾸밈말·찾기·`나한테`·부정문 예시를
넣었다(예시 단어는 평가셋에 없는 모자·책·선풍기·의자만 사용). 개발셋으로만 두 번 고친 뒤 홀드아웃을 한 번 쟀다.

| 요청 해석 | 2B | 4B |
|---|---|---|
| 개발셋 93문장 | 78 | 92 |
| 홀드아웃 40문장 | 34 | 40 |

새 프롬프트에서 2B는 `리모컨 집어줘`에도 target을 비우는 등 개발셋이 83→78로 떨어졌다.
2B로 돌아갈 경우(Jetson) 이 프롬프트를 그대로 쓰지 말고 다시 평가한다.

데스크톱(16GB)과 서버 GPU에서는 4B를 기본으로 쓴다. 온보드 Jetson은 9.2GB 피크를
실측하기 전까지 4B 탑재를 가정하지 않으며, 필요하면 `--model artifacts/models/qwen3-vl-2b`로 2B를 쓴다.
4B 받기:

```bash
.venv-qwen3-vl/bin/python - <<'PY'
from huggingface_hub import snapshot_download
snapshot_download('Qwen/Qwen3-VL-4B-Instruct', revision='ebb281ec70b05090aa6165b016eac8ec08e71b17',
                  local_dir='artifacts/models/qwen3-vl-4b',
                  allow_patterns=['*.json', '*.safetensors', '*.txt', '*.model', '*.jinja'])
PY
```

## GraspNet 파지 후보 생성

Qwen/SAM이 만든 `tmp/qwen-object-cloud/`를 입력한다. GraspNet은 작업 영역의
20,000점을 고정 seed로 샘플링해 추론하며, 대상 물체만 잘라 모델에 입력하지 않는다.
후보 중심이 SAM 마스크 안으로 투영되고 관측된 대상 표면에서 3cm 이내인 후보를
선택한 뒤, 전체 장면으로 충돌과 빈 파지를 검사한다. 이 연관 규칙은 초기 휴리스틱이다.

현재 컴퓨터에서 직접 실행:

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view
```

이미 저장한 결과를 추론 없이 다시 보기:

```bash
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view-only
```

기본 출력 `tmp/graspnet-candidates/grasps.npy`는 공식 GraspGroup 배열이며,
`grasps.json`에는 점수, 그리퍼 폭/깊이(m), 카메라 기준 위치(m), 회전 행렬,
모델 추론 시간과 단계별 후보 개수가 기록된다. `--index 0`은 SAM 출력의 물체 번호,
`--top-k 20`은 저장할 최대 후보 개수다. 후보가 없으면 빈 배열로 저장한다.

회색은 전체 장면, 초록색은 대상 표면, 그리퍼 모양은 파지 후보다.
이 결과는 baseline의 평행 그리퍼 기하에 대한 후보이며 실제 로봇 그리퍼 치수,
IK·도달성·로봇/환경 충돌·실행 경로는 검증하지 않는다. 센서에 보이지 않는 면의
충돌도 이 깊이 영상으로 판정할 수 없다.

설치 메모:

- 네트워크/체크포인트: [공식 GraspNet baseline](https://github.com/graspnet/graspnet-baseline),
  리비전 `280c215129f759ed8649cb4e89fc5dfee55f4f80`.
- 공식 README의 Google Drive RealSense 체크포인트를 `artifacts/models/checkpoint-rs.tar`에 저장.
- CUDA compiler가 없어 NVIDIA 공식 CUDA 12.8.1 redist의 nvcc/cudart/cccl을
  `artifacts/cuda-12.8/`에 설치했다. 드라이버는 변경하지 않았다.
- PointNet2 연산 소스는 [PyTorch 호환 fork](https://github.com/H-Freax/GraspNet-PointNet2-Pytorch-General-Upgrade)
  리비전 `a01e9c87cd0658d63685da53de29f70360a3c32d`를 사용한다.
  고정 SM80 옵션을 제거하고 SM120으로 빌드하며 CUDA stream 헤더/namespace를 c10으로 변경했다.
  모델 구조와 가중치는 공식 baseline을 사용한다.
- inference에서도 import되는 학습용 KNN 의존성은 `torch.cdist` 기반 1-based KNN으로 제공한다.
  평가 모드의 추론 경로에서는 학습용 label generation을 호출하지 않는다.
- `graspnetAPI`는 공식 저장소에서 `--no-deps --no-build-isolation`로 설치한다.
  불필요한 데이터셋 평가 의존성을 설치하지 않고 geometry submodule만 로드한다.
  `Cython`, `transforms3d`, `grasp_nms==1.0.2`를 별도로 설치했다.
- 빌드 예: `CUDA_HOME="$PWD/artifacts/cuda-12.8" TORCH_CUDA_ARCH_LIST=12.0 MAX_JOBS=4 .venv-qwen3-vl/bin/python -m pip install --no-build-isolation --no-deps ./artifacts/graspnet-modern/pointnet2`

다른 컴퓨터용 원클릭 설치 스크립트는 아직 제공하지 않는다.

2026-10-31 바나나 실제 실행: 전체 후보 317개 → 대상 연관 11개 → 충돌/빈 파지 필터 후 2개
→ NMS 후 1개. GPU 모델 추론 약 0.28초(설치·모델 로드·필터·저장 제외).
저장 배열의 유한값, 회전 행렬 직교성/행렬식과 폭/전방 깊이를 확인했다.
후보 폭 약 6.89cm, 점수 약 0.179이며 점수는 파지 성공 확률이 아니다.
`grasps.jpg`는 원본 RGB에 후보 그리퍼를 투영한 이미지다.

```bash
xdg-open tmp/graspnet-candidates/grasps.jpg
```

### 모든 후보와 필터 단계별 보기

한 번 다시 실행하면 `grasps_raw.npy`, `grasps_target.npy`, `grasps_filtered.npy`도 저장한다.
`--stage`는 표시할 단계를 선택하며 단계 내 모든 후보를 표시한다.

```bash
# 전체 장면의 모델 출력 후보 (바나나 외 물체 포함)
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view-only --stage raw
# 바나나 마스크/표면 거리로 연관된 후보 전체
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view-only --stage target
# 충돌/빈 파지 검사 후, 중복 제거 전
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view-only --stage filtered
# 중복 제거 및 상위 top-k 적용 후
.venv-qwen3-vl/bin/python tools/diagnostics/graspnet_candidates.py --view-only --stage final
```

`--top-k`를 늘려도 앞 단계에서 제거된 후보는 복원되지 않는다.
NMS 기본 임계값은 위치 3cm·회전 30도이며 가까운 후보를 중복으로 취급해 점수가 높은
대표 후보를 남긴다. 하나를 강제로 선택하는 규칙은 없다. 이번 장면에서는
필터를 통과한 2개가 NMS로 1개가 됐다. 점수는 네트워크가 예측한 파지 품질 순위용
값이며 실제 성공 확률이 아니다. 임계값과 대상 연관/표면 거리 규칙은 현재 실험 코드의 설정이다.
