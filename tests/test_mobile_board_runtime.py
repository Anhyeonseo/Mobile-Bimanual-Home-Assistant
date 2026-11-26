"""Actual board initialization, stop writers and measured hold integration."""
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.system_stop import SystemStopStatus
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module")
def board(tmp_path_factory):
    binary = tmp_path_factory.mktemp("mobile-board") / "board"
    fake = ROOT / "tests/fixtures/servo_hal"
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U", "-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U",
        "-DHOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U", "-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U",
        "-I", str(fake), "-I", str(board / "Inc"), "-I", str(core / "include"),
        str(fake / "mobile_board_harness.c"),
        *(str(board / "Src" / f"{name}.c") for name in (
            "mobile_board", "mobile_hold_output", "mobile_servo_output", "servo_transport",
            "mobile_arm_observer", "bimanual_feedback_snapshot", "bimanual_operational_limits")),
        *(str(core / "src" / f"{name}.c") for name in (
            "mobile_output", "mobile_supervisor", "mobile_feedback", "sts3215_response",
            "mobile_endpoint", "mobile_wire", "device_startup", "lift_endpoint", "lift_controller",
            "system_stop", "robot_evidence", "arm_hold_monitor", "crc32c", "bus_schedule", "bus_router", "shared_bus",
            "motor_groups", "sts3215_packet", "joint_unwrap", "bimanual_goal_map")), "-o", str(binary)], check=True)
    return binary

@pytest.mark.parametrize("scenario", range(11))
def test_board_stop_and_startup(board, scenario):
    result = subprocess.run([str(board), str(scenario)], check=True, text=True, capture_output=True)
    if scenario == 0:
        status = SystemStopStatus.decode(bytes.fromhex(result.stdout.strip()))
        assert status.confirmed and status.boot_id == 42 and status.sequence == 7

@pytest.fixture(scope='module')
def commissioning_board(tmp_path_factory):
    root=tmp_path_factory.mktemp('unified-board')
    fake=ROOT/'tests/fixtures/servo_hal';board=ROOT/'firmware/stm32_g474_single_arm/Core'
    core=ROOT/'firmware/stm32_actuator';binary=root/'board'
    text=(fake/'mobile_board_harness.c').read_text()
    initializer=text.split('static const MobileBoardProfile profile=',1)[1].split('\n};',1)[0]+'\n};\n'
    header=root/'profile.h';header.write_text('static const MobileBoardProfile commissioned_mobile_profile='+initializer)
    # Exercise the real late attachment gate, not an always-true substitute.
    source=(board/'Src/binary_control.c').read_text()
    start=source.index('bool BinaryControl_AttachMobileEndpoint(')
    attach=source[start:source.index('\n}\n',start)+3]
    stub='bool BinaryControl_AttachMobileEndpoint(actuator_mobile_endpoint_t *e){return e!=NULL;}'
    globals='''static uint8_t host_binary_mode=1,host_stop_latched;
static actuator_mobile_endpoint_t *host_mobile_endpoint;
static bool Host_InventoryPermitted(void){(void)host_binary_mode;return MobileBoard_MaintenanceAllowed();}
'''
    harness=root/'harness.c';harness.write_text(text.replace(stub,globals+attach))
    command=['gcc','-std=c11','-Wall','-Wextra','-Werror',
        '-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U','-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U',
        '-DHOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U','-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U',
        '-DHOST_S4_UNIFIED_CANDIDATE=1U','-DACTUATOR_MOBILE_FIRST_ID=7',
        f'-DMOBILE_BOARD_PROFILE_HEADER="{header}"','-I',str(fake),'-I',str(board/'Inc'),'-I',str(core/'include'),
        str(harness),
        *(str(board/'Src'/f'{n}.c') for n in ('mobile_board','mobile_hold_output','mobile_servo_output','servo_transport',
          'mobile_arm_observer','bimanual_feedback_snapshot','bimanual_operational_limits')),
        *(str(core/'src'/f'{n}.c') for n in ('mobile_output','mobile_supervisor','mobile_feedback','sts3215_response',
          'mobile_endpoint','mobile_wire','device_startup','lift_endpoint','lift_controller','system_stop','robot_evidence',
          'arm_hold_monitor','crc32c','bus_schedule','bus_router','shared_bus','motor_groups','sts3215_packet','joint_unwrap','bimanual_goal_map')),
        '-o',str(binary)]
    subprocess.run(command,check=True);return binary

@pytest.mark.parametrize('scenario',list(range(20,50))+list(range(50,70))+list(range(70,90)))
def test_explicit_commissioning_without_torque_or_motion(commissioning_board,scenario):
    subprocess.run([str(commissioning_board),str(scenario)],check=True,capture_output=True,text=True)


def test_registered_top_preserves_whole_stop_and_blocks_high_lift_driving(commissioning_board):
    subprocess.run([str(commissioning_board),'90'],check=True,capture_output=True,text=True)
