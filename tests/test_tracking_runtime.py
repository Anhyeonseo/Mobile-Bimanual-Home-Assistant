"""A failed arm response must not orphan the other bus's read lease."""
from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize("stationary",[False,True])
def test_paired_reads_drain_before_failure_and_keep_capture_time(tmp_path,stationary):
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    binary = tmp_path / "tracking"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U",
        "-DHOST_S3_STATIONARY_PICK="+str(int(stationary)),
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "tracking_harness.c"), str(board / "Src/bimanual_tracking_feedback.c"),
        *([str(board / "Src/s3_platform_snapshot.c")] if stationary else []),
        "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
