"""Execute the production MCU re-entry function with real angle mapping/unwrap.

Only board readiness and observer ownership are stubbed. No motor I/O is used.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_observer_to_motion_rebind_accepts_valid_wrapped_angles(tmp_path):
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    source = (board / "Src/binary_control.c").read_text()
    begin = source.index("static bool Host_PrepareArmMotionReference(void)")
    end = source.index("\n}\n", begin) + 3
    harness = tmp_path / "motion_reference.c"
    harness.write_text('''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "bimanual_operational_limits.h"
#include "actuator_core/joint_unwrap.h"
static int32_t host_v2_shadow_executor_anchor_urad[12];
static actuator_joint_unwrapper_t host_v2_shadow_unwrappers[12];
static bool ready = true, observed = true;
static bool MobileBoard_PrepareArmMotion(int32_t *a) {(void)a; return ready;}
static bool MobileArmObserver_HasStarted(void) {return observed;}
''' + source[begin:end] + '''
int main(void) {
    /* Shoulder angles beyond the encoder's modulo boundary, inside limits. */
    host_v2_shadow_executor_anchor_urad[1] = 3200000;
    host_v2_shadow_executor_anchor_urad[7] = 3200000;
    assert(Host_PrepareArmMotionReference());
    assert(host_v2_shadow_unwrappers[1].unwrapped_raw > 4095);
    assert(host_v2_shadow_unwrappers[7].unwrapped_raw > 4095);
    for (unsigned j=0; j<12; ++j) assert(host_v2_shadow_unwrappers[j].bound);
    /* A second finite move must be able to acquire the observer's reference. */
    assert(Host_PrepareArmMotionReference());
    ready = false; assert(!Host_PrepareArmMotionReference());
    ready = true; observed = false; assert(Host_PrepareArmMotionReference());
    observed = true; host_v2_shadow_executor_anchor_urad[1] = 99000000;
    assert(!Host_PrepareArmMotionReference());
    return 0;
}
''')
    binary = tmp_path / "motion_reference"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U",
        "-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U",
        "-I", str(ROOT / "tests/fixtures/servo_hal"), "-I", str(board / "Inc"),
        "-I", str(core / "include"), str(harness),
        str(board / "Src/bimanual_operational_limits.c"),
        str(core / "src/bimanual_goal_map.c"), str(core / "src/joint_unwrap.c"),
        "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
