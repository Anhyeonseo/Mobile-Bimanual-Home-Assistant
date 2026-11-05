# D415 손 촬영 — 방 스캔 자료 수집

카메라를 최종 고정하기 전, Jetson에 연결한 D415를 손으로 움직여 방을 기록한다.
목적은 이후 가구·물체 검색과 지도 생성 검토용 자료 수집이다. 이 촬영만으로 지도가
생성되거나 S2의 카메라↔로봇 좌표 보정이 완료되는 것은 아니다.

## 촬영 도구

`.bag`을 지원하는 RealSense 2.57.7 녹화 전용 Python 환경을 사용한다.
설치된 2.58.4는 `.db3`만 생성하므로 기존 인식 환경과 분리한다.
별도 ROS·인식 모델·모터 연결은 필요 없다.
다른 D415 뷰어는 종료한 뒤 실행한다. PC에서 SSH 포워딩을 열면 브라우저로 RGB와 깊이를 볼 수 있다.

최초 환경 준비(배포 시 한 번):

```bash
python3 -m venv ~/robot/shared/venvs/room-scan-bag
~/robot/shared/venvs/room-scan-bag/bin/python -m pip install -r requirements/room_scan.txt
```

```bash
# JETSON_TARGET은 본인의 SSH 호스트 별칭, ROOM_SCAN_RELEASE는 배포한 폴더로 설정한다.
ssh -t -L 127.0.0.1:18765:127.0.0.1:8765 "$JETSON_TARGET" \
  "bash '$ROOM_SCAN_RELEASE/tools/run/record_room_rgbd.sh'"
```

PC에서 `http://127.0.0.1:18765`를 연다. 카메라를 방의 주요 가구로 향하게 두고
5초간 밝기·색 자동 설정을 기다린다. 이후 RGB 노출·게인·화이트밸런스가 고정된다.
카메라를 이미 든 상태에서 화면을 확인한다. 깊이·밝기·선명도·화면 이동 상태가
**2초간 안정되면 SSH 터미널에서 Enter를 눌러 녹화**한다. 준비되지 않은 장면은
이유를 보여주며 시작을 기다린다. 단순 화질 지표가 잘못 판단한 장면은 `start` + Enter로
명시적으로 시작할 수 있고, 이 선택은 기록에 남는다.
밝기가 적절하지 않으면 `q` + Enter로 종료하고 다른 방향에서 다시 준비하거나,
`--exposure`와 `--white-balance`로 지정한다. 노출값은 SDK 단위(100μs), 색온도는 K다.

- RGB BGR8, 깊이 Z16: 각각 **1280×720, 30fps**, USB 3 필수.
- emitter ON, 깊이 자동 노출 등 나머지 깊이 설정은 유지한다.
- `.bag`에는 정렬·필터링 전 원본 깊이를 기록한다. 브라우저의 색상 깊이는 표시 전용이다.
- 기본 180초 후 종료. 일찍 끝내려면 Enter → 제자리 3초 정지 → 파일 저장.
- 시작·마지막 3초는 제자리에 머문다. 끝나기 전 시작 위치·방향으로 돌아온다.
  **종료 안내가 나올 때까지 카메라를 내려놓지 않는다.** 렌즈 위에 손가락이 올라오지 않도록 잡는다.
- 녹화 중 유효 깊이(0.5~3m) 비율과 화질 안내를 브라우저·터미널에 표시한다.
  깊이 부족, 낮은 선명도/무늬, 너무 어둡거나 밝은 화면, 빠른 화면 이동을 안내한다.
  안내는 휴리스틱이며 실제 카메라 속도·추적 성공·방 전체 촬영 여부를 뜻하지 않는다.
  촬영 중에는 경고만 남기고 원본 녹화를 계속한다.
- Ctrl+C 또는 SSH 연결 종료는 중단 기록으로 남기고 파일을 닫는다. 전원/USB를 뽑아 종료하지 않는다.
- 종료 시 기존 RGB 자동 설정과 emitter 설정을 복원하고, 파일 전체를 다시 읽어 RGB·깊이의
  길이·프레임 간격을 검사한다. 이 검사는 흔들림·장면 중첩·물체 식별 가능성을 보증하지 않는다.
- 녹화가 끝나면 미리보기 갱신은 멈추고 마지막 화면이 남는다. 최종 저장 경로와 재생 검사
  결과는 SSH 터미널에서 확인한다. 이전 도구의 `channel ... Connection refused` 반복은
  닫힌 미리보기에 브라우저가 계속 접속해서 발생했으며, 그 메시지만으로 파일 손상을 뜻하지 않는다.

녹화 길이는 `--seconds 120` 등으로 지정할 수 있다(6~600초).
3분 원시 데이터는 약 25GB 규모이므로 “수 GB”로 가정하지 않는다. 실제 `.bag` 크기는
SDK 압축 설정에 따라 달라진다. 도구는 원시 크기에 여유분을 더해 저장 공간을 확인한다.

## 방과 동선 준비

1. 소파·책상·협탁에 리모컨·컵·계산기 등을 평소처럼 놓고, 위치를 사진이나 메모로 남긴다.
   촬영 중 물체를 옮기지 않고 사람은 화면 밖에 있는다. 조명을 켜고 직사광선을 가린다.
2. 카메라와 가구를 약 0.5~3m 떨어뜨리고, 가까운 화면이 연속해서 겹치도록 천천히 이동한다.
   제자리에서 급하게 돌리지 않는다. 가구마다 정면·좌·우를 포함한다.
3. 눈높이 한 바퀴, 실제 로봇 카메라 높이 한 바퀴를 목표로 한다.
   방이 크면 3분을 맞추려 서두르지 말고 길이를 늘린다. 케이블 여유를 먼저 확보한다.
4. 시작 위치·방향에서 마지막 3초 정지 후 저장한다. 배치 사진 또는 메모를 같이 정리한다.

## 저장 및 전달

기본 위치는 Jetson의 `~/robot/shared/data/room-scans/<시각-구분자>/`다.

| 파일 | 내용 |
|---|---|
| `recording.bag` | RGB·깊이 원본, 카메라 정보 |
| `settings.json` | 해상도·노출·게인·화이트밸런스·내부 보정값·촬영 종료 상태 |
| `verification.json` | 전체 재생 결과, 프레임 간격과 길이. 검사 중 `checking`, 오류 시 `error` 기록 |
| `quality.json` | 약 2Hz 화질 기록·주의 구간. `image_hints_ok`도 지도 완성을 의미하지 않음 |
| `scene-start.jpg` | 첫 화질 양호 장면 참고 사진. 양호 장면이 없으면 생성되지 않음 |
| `ground-truth.txt` | 실제 물체 위치를 작성할 메모 양식 |

실제 물체 배치 사진을 추가하고 `ground-truth.txt`를 채워 폴더째 전달한다.
`verification.json`이 `pass`여도 RealSense Viewer 등으로 영상의 흔들림·밝기·중첩을 확인한다.
전체 재생 검사는 촬영 길이만큼 시간이 걸릴 수 있다. `checking`은 완료가 아니며
`pass` 또는 오류 결과가 나온 뒤 판단한다. 검사만 다시 실행하려면
`record_room_rgbd.sh --verify /path/to/recording.bag`를 사용한다.
촬영 자료·장치 설정은 Jetson 공유 데이터 폴더 또는 로컬의 무시되는 `output/`에 보관하고
GitHub에는 올리지 않는다. `.bag`은 저장소에서 제외된다.

SDK 기록/재생 방식: [RealSense 2.57.7 공식 문서](https://github.com/realsenseai/librealsense/blob/v2.57.7/doc/record-and-playback.md).

## 촬영 후 부분 3D 복원 검사

GPU 없이 PC에서 실행하는 진단 도구다. 원본은 수정하지 않는다. Jetson의 모터·실물
카메라에 접속하지 않고 저장한 `.bag`만 읽는다. PC의 별도 환경에 설치한다.

```bash
python3 -m venv .venv-room-reconstruction
.venv-room-reconstruction/bin/pip install -r requirements/room_reconstruction.txt

.venv-room-reconstruction/bin/python tools/diagnostics/reconstruct_room_rgbd.py extract \
  /path/to/recording.bag output/room-review/dataset
.venv-room-reconstruction/bin/python tools/diagnostics/reconstruct_room_rgbd.py reconstruct \
  output/room-review/dataset output/room-review/reconstruction
.venv-room-reconstruction/bin/python tools/diagnostics/review_room_reconstruction.py \
  output/room-review/dataset output/room-review/reconstruction
```

출력 폴더는 새 경로를 사용한다. 기본 10Hz로 샘플링하고, SDK로 깊이를 RGB에 맞춘 뒤
렌즈 왜곡을 제거하고 640×360으로 줄인다. 내부 파라미터도 이에 맞춰 조정한다.
이 변환본에는 밀리미터 단위 깊이 PNG, RGB PNG, 시간 및 보정 정보가 저장된다.
재생은 비실시간으로 진행하며 이미지 처리·저장 중 재생을 잠시 멈춰 누락을 줄인다.
파일의 끝 근처까지 읽었는지 검사하고, 프레임 사이 간격은 복원 단계에서도 확인한다.
SDK가 파일 끝에서 일시정지하면 멈추는 문제를 피하려고 마지막 약 0.2초 전에 종료한다.
미추출된 끝부분 길이는 `dataset.json`에 기록한다. 원본 전체의 무결성 검사는 별도다.

복원은 특징점으로 초기 이동을 추정한 뒤 Open3D RGB-D odometry와 TSDF를 사용한다.
깊이가 부족하거나 프레임 간격·겹침·이동 추정 검사가 실패하면 새 조각으로 분리한다.
**각 조각의 좌표 원점은 해당 조각의 첫 카메라다. 조각끼리 겹쳐서 한 지도로 취급하지 않는다.**
카메라 추적은 자체 검사를 통과해도 누적 오차가 있을 수 있다.

결과 폴더:

| 파일 | 내용 |
|---|---|
| `reconstruction.json` | 구간, 추정 경로, 실패 지점, 제한사항 |
| `fragment-XX.ply` | 관측 표면을 통합한 삼각형 메시 |
| `fragment-XX-points.ply` | 색상 3D 점군, 미터 단위 |
| `reconstruction-preview.png` | 가장 긴 구간들의 3D 미리보기 |
| `quality-timeline.png` | 유효 깊이 비율·경로 연결 구간 |
| `capture-contact-sheet.jpg` | 전체 촬영 장면 표본 |
| `review.html` | 회전·확대 가능한 로컬 3D 검토 화면 |

이 도구는 **부분 복원이 가능한지 확인하는 진단 단계**다. 전역 재위치 추정과 루프 폐쇄,
중력 기준 정렬, 방 사이 연결, 의미 지도, 로봇 높이를 반영한 장애물 지도를 제공하지 않는다.
한 구간이 길게 연결되더라도 `global_map_valid=false`, `robot_navigation_valid=false`를
유지한다. 집 전체 지도에는 별도 SLAM 및 실제 크기·경로 검증이 필요하다.

참고: [Open3D 공식 복원 시스템](https://www.open3d.org/docs/release/tutorial/reconstruction_system/index.html).

### Jetson에서 추출할 때

원본 `.bag`을 PC로 옮기기 부담스러우면 Jetson의 녹화용 환경에서 `extract`만 실행하고,
생성된 데이터 폴더를 PC로 옮겨 `reconstruct`와 `review_room_reconstruction.py`를 실행한다.
Jetson에서는 `extract`에 **`--realtime`**을 추가하는 편이 빠를 수 있다. 파일을 원래 촬영
속도로 재생하여, 프레임마다 멈췄다 이어갈 때 발생하는 반복 디코딩 비용을 피한다.
실물 카메라는 열지 않는다.

이 모드는 처리 부하에 따라 추출 프레임이 누락될 수 있으므로 `dataset.json`의
`effective_export_fps`, `export_max_gap_seconds`, `export_gaps_over_250ms`를 함께 확인한다.
복원 도구는 0.25초를 넘는 간격을 연결하지 않는다. 원본 녹화의 무결성 판정은 여전히
`verification.json`을 기준으로 한다. PNG 변환본은 원본을 대체하는 보존용 파일이 아니다.
