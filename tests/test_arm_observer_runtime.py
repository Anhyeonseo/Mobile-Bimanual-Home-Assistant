"""Actual idle observer, canonical joint limits and snapshot; synthetic READ replies."""
from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module",params=[False,True])
def observer(tmp_path_factory,request):
    binary = tmp_path_factory.mktemp("idle-observer") / "observer"
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U", "-DHOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U",
        "-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U",
        "-DHOST_S3_STATIONARY_PICK="+str(int(request.param)),
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "arm_observer_harness.c"),
        *([str(board / "Src/s3_platform_snapshot.c")] if request.param else []),
        *(str(board / "Src" / f"{name}.c") for name in ("mobile_arm_observer", "bimanual_feedback_snapshot", "bimanual_operational_limits")),
        *(str(core / "src" / f"{name}.c") for name in ("bimanual_goal_map", "joint_unwrap")),
        "-o", str(binary)], check=True)
    return binary

@pytest.mark.parametrize("scenario", range(11))
def test_idle_observation_and_handoff(observer, scenario):
    subprocess.run([str(observer), str(scenario)], check=True)
