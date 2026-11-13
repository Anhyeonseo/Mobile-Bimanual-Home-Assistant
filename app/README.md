# HOMETEKHU 앱

휴대폰과 패드를 중심으로 만든 홈 로봇 앱 시연입니다. 별도 런타임 의존성 없이 HTML, CSS, ES 모듈로 실행됩니다.

- 로봇 첫 연결, 물건 부탁, 루틴, 활동 기록, 예외 대응
- 같은 좌표를 사용하는 **우리 집 / SLAM** 지도 전환
- 리모컨 찾기 → 집기 → 이동 → 내려놓기의 연속 애니메이션
- 드래그 가능한 부탁 패널, 일시 정지·재개, 움직임 줄이기 지원
- Pretendard 로컬 폰트, HOMETEKHU 로고, 그래파이트와 반투명 표면

현재 동작과 지도는 시연용입니다. 로봇 API, 센서, 인증, 실제 주행·집기 제어는 연결하지 않았습니다.
기존 Python·ROS 앱과 독립적으로 실행하며, 체험 상태는 이 브라우저의 localStorage에 저장합니다.

## 로컬 실행

Node.js 22 이상에서 저장소 루트 기준으로 실행합니다. `npm install`은 필요하지 않습니다.

```bash
cd app
npm start
```

주소: <http://127.0.0.1:4173/>. 상단 **체험** 버튼에서 시연 흐름을 선택할 수 있습니다.
발표 대본은 `src/guide.html`에 있습니다.

```bash
npm run check
npm test
npm run build
npm run preview -- --base=/Mobile-Bimanual-Home-Assistant/
```

배포 경로 확인: <http://127.0.0.1:4173/Mobile-Bimanual-Home-Assistant/>.
`PORT` 환경 변수로 포트를 바꿀 수 있습니다.

## GitHub Pages 배포

[HOMETEKHU app 워크플로](../.github/workflows/hometekhu-pages.yml)는 다음 순서로 실행합니다.

1. JavaScript 문법 검사와 19개 동작 테스트
2. `src/`를 `dist/`로 빌드하고 프로젝트 하위 경로의 파일 참조 검사
3. GitHub Pages 아티팩트 업로드
4. Pages가 활성화되어 있으면 `github-pages` 환경으로 배포

`main` 또는 `feature/hometekhu-app`에 앱 변경을 푸시하면 실행됩니다. PR에서는 검사와 아티팩트 생성만 수행합니다.
배포 완료 주소는 해당 Actions 실행의 **Publish app** 작업에 표시됩니다.

### 처음 한 번 필요한 저장소 설정

저장소 소유자 또는 관리자·유지관리 권한이 있는 사용자가 설정합니다.

1. **Settings → Pages → Build and deployment → Source**에서 **GitHub Actions**를 선택합니다.
2. **Actions → HOMETEKHU app**의 최신 실행을 열어 **Re-run all jobs**를 누릅니다.
3. `github-pages` 환경에 브랜치 제한이 있으면 `feature/hometekhu-app`을 허용하거나 이 브랜치를 `main`에 병합한 뒤 배포합니다.

Pages가 꺼져 있으면 빌드와 테스트는 수행하고, 배포 작업은 건너뛰며 Actions 요약에 설정 안내를 남깁니다.
별도 토큰이나 저장소 secret을 추가할 필요는 없습니다.

기본 배포 주소는 `https://Anhyeonseo.github.io/Mobile-Bimanual-Home-Assistant/`입니다.
**실제 게시 여부는 Actions의 배포 성공 결과로 확인합니다.**

모든 앱 자산은 상대 경로를 사용하고, 화면 이동은 `#home`, `#map` 등 해시 경로를 사용합니다.
폰트·로고·지도는 외부 서비스에 의존하지 않습니다.

## 코드 구성

| 경로                             | 역할                                         |
| -------------------------------- | -------------------------------------------- |
| `src/app.js`                     | 화면 전환, 사용자 입력, 상태 저장            |
| `src/core.js`                    | 요청 해석과 시뮬레이션 작업 수명주기         |
| `src/navigation.js`              | 경로 탐색과 위치 보간                        |
| `src/transport.js`               | 이동·집기·운반·놓기의 공통 시간과 물건 위치  |
| `src/slam.js`, `src/home-map.js` | 같은 관측 좌표의 SLAM·가구 지도              |
| `src/ui/`                        | 화면, 다이얼로그, 지도 선택, 전달 애니메이션 |
| `src/styles/`                    | 공통 UI, 폰·패드 레이아웃, 움직임            |
| `src/assets/`                    | 로고, 캐릭터, Pretendard와 라이선스          |
| `tests/`                         | 요청·경로·저장·정지·전달 상태 검사           |
| `scripts/`                       | 문법 검사, 정적 빌드, 배포 자산 검증         |
| `dist/`                          | 재생성 가능한 배포 결과, Git에서 제외        |

코드 정리: `npm run format` (Prettier 3.6.2).
Pretendard는 [SIL OFL 1.1](src/assets/OFL.txt)을 따릅니다.
