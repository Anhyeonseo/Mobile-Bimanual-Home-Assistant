# Third-party notices

이 저장소의 자체 작성 코드와 문서는 루트 [Apache License 2.0](../LICENSE)을 따른다. 아래 디렉터리에는 STM32CubeIDE가 생성하거나 함께 배포한 제3자 구성 요소가 포함되어 있으며, 각 구성 요소의 원래 저작권 표시와 license가 우선 적용된다.

| 구성 요소 | 위치 | License |
|---|---|---|
| TheRobotStudio SO-101 URDF geometry and STL meshes (commit `fda892cba81032c46c40976a48c9ceadbf40a9ca`) | `ros2_ws/src/so101_description` | Apache-2.0; package README and root `LICENSE` 참조 |
| Anhyeonseo/Bimanual-Pick-And-Place (commit `b3d7a3714f134e760a4d146039686c08eeaa622d`) | `home_robot_tasks/grasp_yaw.py`, `classical.py`; 원본·수정 목록은 `config/pnp_source.json` | Apache-2.0; 이동식 팔 장착 FK와 경로 경계를 수정 |
| ARM CMSIS Core | `firmware/stm32_g474_single_arm/Drivers/CMSIS` | Apache-2.0, 해당 디렉터리의 `LICENSE.txt` 참조 |
| STM32G4 CMSIS Device | `firmware/stm32_g474_single_arm/Drivers/CMSIS/Device/ST/STM32G4xx` | Apache-2.0, 해당 디렉터리의 `LICENSE.txt` 참조 |
| STM32G4 HAL Driver | `firmware/stm32_g474_single_arm/Drivers/STM32G4xx_HAL_Driver` | BSD-3-Clause, 해당 디렉터리의 `LICENSE.txt` 참조 |
| STM32G4 NUCLEO BSP | `firmware/stm32_g474_single_arm/Drivers/BSP/STM32G4xx_Nucleo` | 각 source file의 STMicroelectronics 저작권 및 license 표시 참조 |

STMicroelectronics, STM32와 각 제품명은 해당 권리자의 상표다. 이 프로젝트는 STMicroelectronics의 공식 후원 프로젝트가 아니다.

## 선택적 RGB-D 인식 모델

다음 모델은 별도 환경에서 다운로드하며 가중치를 이 저장소에 재배포하지 않는다.

| 모델 | 고정 리비전 | License / 원본 |
|---|---|---|
| IDEA Research Grounding DINO Tiny | `a2bb814dd30d776dcf7e30523b00659f4f141c71` | [Apache-2.0, 공식 모델 카드](https://huggingface.co/IDEA-Research/grounding-dino-tiny) |
| Meta SAM 2.1 Hiera Tiny | `de431c4043854a71d8101e17995dfe596bf101a5` | [Apache-2.0, 공식 모델 카드](https://huggingface.co/facebook/sam2.1-hiera-tiny) |

실행 위치·입력 크기·추론 방식·제약은 [D415 실행 안내](REALSENSE.md)에 기록한다.
