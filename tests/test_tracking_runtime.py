"""A failed arm response must not orphan the other bus's read lease."""
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]

def test_paired_reads_drain_before_failure_and_keep_capture_time(tmp_path):
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    binary = tmp_path / "tracking"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U",
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "tracking_harness.c"), str(board / "Src/bimanual_tracking_feedback.c"),
        "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
