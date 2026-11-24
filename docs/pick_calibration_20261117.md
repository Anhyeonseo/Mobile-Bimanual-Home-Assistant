# 11/17 캘리브레이션을 집기에 적용

원본: [yeoeun/extrinsic-calibration, 24c1ac0](https://github.com/Anhyeonseo/Mobile-Bimanual-Home-Assistant/tree/24c1ac099c95dfec9e2692b6b2625d5a76d77038).
로컬 `work/local-edits`에 필요한 파일만 반영했다. 브랜치 전체 병합·커밋·푸시는 하지 않았다.

## 적용한 값

| 항목 | 적용 |
| --- | --- |
| 카메라 → base | 원본 `base_from_optical` 4×4 행렬 전체, 반올림 없이 유지 |
| 왼팔 모델 영점 | 어깨 +5.3749°, 팔꿈치 +0.9124°, 손목 굽힘 −0.0339° |
| 오른팔 모델 영점 | 어깨 +7.6913°, 팔꿈치 +3.1780°, 손목 굽힘 −4.6420° |
| 오른팔 장착 | 원본 `right_mount_correction` 후곱 |
| 리프트 | 환산 배율 ×1.0391323769529873, 양팔 장착 위치에 영점 −0.005739680522841576 m 반영 |
| 높이·광각 | 같은 실측 카메라로 계산한 원본 프로필 |
| 그리퍼 기록 | 새 보정 ID에 연결. 측정 간격·닫힘 설정·매핑 동일 |
| 이전 −9.1 mm 높이 보정 | 새 행렬에 포함되어 중복 적용하지 않음 |

표의 각도는 설명용 반올림이다. 실제 설정 파일 5개는 GitHub blob과 바이트 단위로 동일하다.
관절 범위, 모터 EEPROM/Home 원시값, 속도·가속도, 펌웨어, 그리퍼 동작,
파지 후보 및 경로 선택 설정은 변경하지 않았다.

현재 통합 컨트롤러 LS의 `height_um`는 기존 84 mm/rev 단위다.
`unified_pick.py` 및 상주 집기의 촬영·유지 확인에서는 프로필의 `meters_per_count`로 환산한다.
원본 LS 기록과 리프트 명령은 그대로이며, 영점 오프셋은 이미 팔 장착에 들어 있어 높이에 다시 더하지 않는다.
예: LS 0.300 m → 계획용 리프트 이동량 0.311739713 m.
`--lift-target-um`의 기존 컨트롤러 단위와 0~10000 범위도 그대로다.
이 패키지는 기존 0x2492D 컨트롤러를 위한 호스트 보정이며 펌웨어를 재빌드·설치하지 않는다.

## 로컬 검사

```bash
cd /home/an-hyeonseo/Mobile-Bimanual-Home-Assistant-edit
python3 tools/diagnostics/extrinsic_capture/apply_measured_calibration.py --check-only
```

기본 보정 ID: `s2-dev-4683680db9f74ce58184eced`
광각 보정 ID: `s2-dev-d705d579ede27011cb5c0785`
높이 모델 ID: `height-accbcb85329e6564ef266208`

다른 수치 라이브러리로 재생성하면 행렬 곱이 약 1e-16 달라져 ID가 달라질 수 있다.
배포에는 검증된 원본 파일을 사용한다. 위 명령의 `--check-only`를 유지한다.

## Jetson 적용

노트북 터미널에서:

```bash
cd /home/an-hyeonseo/Mobile-Bimanual-Home-Assistant-edit
bash tools/run/deploy_pick_calibration.sh
```

`s4-unified-pick-20261116`을 새 `s4-unified-pick-20261116-cal1117` 폴더로 복사한 후
필요한 파일만 교체한다. 대상 폴더가 이미 있으면 보존하고 시각·식별자를 붙인 새 폴더를 만든다.
원본 경로가 없으면 `releases` 바로 아래에서 파일 해시가 정확히 맞는 집기 설치본을 찾는다.
일치하는 후보가 하나일 때만 자동 선택한다. 기존 원본의 해시가 바뀌었거나 후보가 여러 개면 중단한다.
실제로 선택한 경로는 `SOURCE`, `DESTINATION`에 출력되며 최종 `destination` 경로를 사용한다.
원본을 직접 지정하려면 `bash tools/run/deploy_pick_calibration.sh --source /실제/집기/설치경로`로 실행한다.
명시적으로 `--destination`을 지정했는데 그 경로가 이미 있으면 덮어쓰지 않고 중단한다.
기존 설치본과 이미 실행 중인 프로세스에는 적용하지 않으며, 모터 명령을 보내지 않는다.
설치가 끝나면 `CALIBRATION_COPY_READY`가 출력된다.
이전 프로필은 새 복사본의 `config/home_assistant/historical/pre-extrinsic-20261117/`에도 보존된다.

설치본이 상주 집기를 이미 포함하면 `pick_capture.py`와 `pick_hardware.py`도 함께 갱신한다.
기존 설치본에 상주 집기가 없으면 이 패키지가 새로 설치하지는 않는다.

## 다음 집기

기존 제어 프로세스를 정상 종료한 뒤 새 복사본 경로에서 실행한다.
현재 집기 실행기의 최하단 원점 등록 절차와 리프트 준비 범위는 유지한다.
리프트가 이미 올라가 있으면 최하단이라고 등록하면 안 된다.

새 촬영 → 대상 탐지 → 후보 생성 → 경로 계획/검증 → 실행 순서로 진행한다.
이전 보정의 촬영 보고서나 실행 bundle을 새 ID로 바꿔 재사용하지 않는다.
기존 `unified_pick.py`를 사용하면 새 복사본에서 생성한 새 bundle을 지정한다.
상주 `pick_service.py`가 이미 설치되어 있으면 새 복사본의 서비스에서 집기 요청을 보낸다.

## 확인 결과와 남은 확인

- 관련 회귀 테스트 469개 통과: 보정 적용, 좌표/카메라, 집기 파이프라인, 실행 바인딩, 그리퍼, 탐색.
- 기존 가상 장면 테스트는 그 장면을 만든 명목 보정에 고정했다. 새 실측 보정은 별도 테스트로 검사한다.
- 복사 설치 리허설: 기존 파일·무관한 파일 보존 및 다른 설치본 거부 확인.
- 설치 경로 수정 회귀 테스트 9개 통과: 대상 중복, 원본 탐색, 다중 후보, 해시 불일치, 경로 보호.
- Jetson SSH는 실행 환경의 소켓 제한으로 연결되지 않았다. 실제 설치·촬영·집기는 아직 수행하지 않았다.
- 원본 보정의 마커 교차 검증 중앙값은 약 10.5 mm이며 실제 집기 성공률이나 TCP 정확도는 아직 검증 전이다.
