# 저장소·배포 파일 관리

2026-10-29. 소스의 기준은 `codex/home-assistant-v02` 브랜치다.
현재 PC에서는 바깥 수건 프로젝트 안의 `workspaces/home-assistant`가 이 브랜치의 Git 작업 폴더다.
GitHub에 올릴 때는 **이 작업 폴더 안에서** 변경을 확인한다. 바깥 저장소의 다른 브랜치와 섞지 않는다.
새로 clone한 사용자는 이 문서가 있는 저장소 루트에서 실행하며, 바깥 폴더나 `tmp` 링크가 필요 없다.

## 파일을 둘 위치

| 위치 | 내용 | GitHub 포함 |
|---|---|---|
| `firmware/` | STM32 소스·빌드 설정·호스트 시험 | 포함 |
| `ros2_ws/src/` | 제어 연결, 지도·인지·작업 실행, 앱 소스 | 포함 |
| `tools/` | 실행·설정·진단 도구 | 포함 |
| `config/` | 설정 예시·스키마·모의 프로필·장치 고유번호 없는 검토된 형상/범위 요약 | 포함 |
| `tests/` | 자동 검사와 작은 합성 입력 | 포함 |
| `requirements/`, `.github/` | 의존성·CI | 포함 |
| `docs/` | 설계·실행·검증 방법, 공유 가능한 결과 요약 | 포함 |
| `output/` | 실행 상태·접속 키·실측 결과·보정·로그·촬영 자료 | 제외 |
| `*.local.json`, `*.local.yaml`, `*.local.toml`, `.env*` | 장치별 설정·주소·인증 정보 | 제외 (`.env.example`은 공개 예시만) |
| `build/`, `install/`, `log/`, `.venv*/` | 재생성 가능한 빌드·실행 환경 | 제외 |
| `artifacts/`, `datasets/`, `archive/`, `tmp/` | 모델·데이터·과거 배포·임시 작업 | 제외 |

장치 고유 번호·집 내부 영상·집 지도·원본 펌웨어 덤프는 공개 문서에 복사하지 않는다.
필요한 실측 자료는 로컬에서 보관하고, 공유 문서에는 확인한 사실과 남은 검증만 기록한다.
일반 모델 가중치와 센서 기록은 Git 대신 별도 보관소를 사용한다. 재현에 필요한 출처·버전·해시는 문서에 남긴다.
로봇 형상을 표현하는 기존 STL 소스는 필요한 개발 자료이므로 유지한다.

## 젯슨 관리

현재 사용하는 영상 처리본·최신 왼팔 집기본·텔레옵은 기존 경로를 유지했다.
이전 시연 폴더, 배포 압축 파일, S0 임시 진단 폴더는 젯슨 홈의
`robot-archive/20261029-cleanup/`으로 이동했다. 같은 폴더의 `manifest.json`에
원래 경로·보관 경로·파일별 해시와 검증 결과가 있다. 삭제하지 않았으며, flash 원본도 젯슨에 남아 있다.
예전 실행 명령을 다시 쓸 때는 이 이동 기록을 먼저 확인한다.

앞으로 S1 배포부터는 다음 구조로 수렴한다. **아직 전환한 구조는 아니다.**

```text
robot/
  releases/<commit-or-release>/  # 선택한 실행 파일·설정 예시·배포 manifest
  current -> releases/...       # 검증된 실행본
  shared/config/                # 실측 설정·보정·장치 주소
  shared/data/                  # 지도·작업 결과
  shared/logs/                  # 실행 로그
  shared/models/                # 모델 캐시
  shared/venvs/                 # 버전을 기록한 실행 환경
robot-archive/                  # 이전 시연본·복구 자료
```

저장소 전체와 개발용 데이터를 매번 젯슨으로 복제하지 않는다. 배포할 파일을 명시하고,
기준 커밋·변경 유무·설정 버전·파일 해시를 남긴다. 새 환경 확인 전 기존 가상환경이나 모델을 지우지 않는다.
실행 경로·설정 참조·실행 중 프로세스를 확인한 뒤 이동하며, 이전 배포는 되돌릴 수 있게 보관한다.

## 업로드 전 확인

모바일 작업 폴더에서 실행한다.

```bash
git branch --show-current
git status --short
git diff --check
python3 tools/run/home_assistant.py demo
python3 tools/run/home_assistant.py demo --fail-skill pick
python3 -m pytest -c config/pytest.ini tests/test_home_assistant.py tests/test_assistant_integration_boundaries.py tests/test_scan_map_generation.py tests/test_home_robot_inventory.py -q
```

변경 파일을 검토해 관련 파일만 추가하고, staged diff에 키·비밀번호·실측 설정·대용량 결과물이
없는지 다시 확인한다. `.gitignore`는 이미 추적 중인 파일이나 Git 과거 이력을 정리하지 않는다.
이전에 비밀값을 올렸다면 값 폐기·교체와 이력 처리를 별도로 해야 한다.

기존 `Software contracts` CI는 Python 검사·C 코어 검사·펌웨어 빌드 등을 수행한다.
CI 통과는 실제 로봇 동작 검증과 구분한다. 변경량이 큰 현재 작업은 설계·인지/앱·제어/촬영·S0 도구를
검토 가능한 커밋 단위로 나누되, 서로 필요한 소스와 테스트가 누락되지 않게 한다.
기존 촬영 수정도 작업 폴더에 남아 있으므로 이번 파일 정리의 변경으로 모두 간주하지 않는다.

2026-11-01 사용자가 S2 준비까지 GitHub 업로드를 요청했다. 원본 측정 로그·장치 고유번호·
키·덤프는 제외하고 코드·검사·설계·검토된 형상/범위 요약을 공개 대상으로 정리한다.
커밋은 기존 사용자 Git 작성자 정보를 사용하며 별도 도구 공동 작성자를 추가하지 않는다.

공개 실행 도구의 기본 포트는 `/dev/ttyACM0`다. 장치가 여러 개라면 해당 환경에서 확인한
`/dev/serial/by-id/...` 경로를 `--port`로 전달한다. 이 공개 소스 정리가 이미 설치된 젯슨
릴리스나 그 안의 고정 포트 경로를 변경하지 않는다.
