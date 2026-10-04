from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
CORE=ROOT/'firmware/stm32_actuator'

@pytest.mark.parametrize('first',[7,8])
def test_motor_ids_match_startup_feedback_motion_and_both_stop_paths(tmp_path,first):
    names=('motor_groups','device_startup','mobile_output','mobile_feedback','mobile_supervisor',
           'sts3215_packet','sts3215_response','bus_schedule','bus_router','shared_bus','system_stop',
           'lift_endpoint','lift_controller','crc32c')
    exe=tmp_path/'ids'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',f'-DACTUATOR_MOBILE_FIRST_ID={first}',
                    '-I',str(CORE/'include'),str(ROOT/'tests/fixtures/mobile_ids_harness.c'),
                    *(str(CORE/'src'/f'{n}.c') for n in names),'-o',str(exe)],check=True)
    subprocess.run([str(exe),str(first)],check=True)

@pytest.mark.parametrize('first',[0,6,251,254])
def test_overlapping_arm_or_reserved_ids_cannot_compile(tmp_path,first):
    src=tmp_path/'bad.c';src.write_text('#include "actuator_core/mobile_ids.h"\nint main(void){return 0;}\n')
    result=subprocess.run(['gcc',f'-DACTUATOR_MOBILE_FIRST_ID={first}','-I',str(CORE/'include'),str(src),'-o',str(tmp_path/'bad')],capture_output=True,text=True)
    assert result.returncode and 'Mobile IDs must exclude' in result.stderr
