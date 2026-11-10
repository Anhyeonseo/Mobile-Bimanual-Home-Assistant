"""Compile the real servo table for each profile; no board access."""
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[1]
CORE=ROOT/'firmware/stm32_g474_single_arm/Core'


def test_stationary_profile_changes_only_the_gripper_output_cap(tmp_path):
    if not shutil.which('gcc'):pytest.skip('host C compiler unavailable')
    src=tmp_path/'main.c'
    src.write_text('#include <stdio.h>\n#include "servo_joint_config.h"\n'
                   'int main(void){for(unsigned i=0;i<servo_joint_count;i++)'
                   'printf("%u %u %u %u\\n",servo_joints[i].id,servo_joints[i].p_gain,'
                   'servo_joints[i].d_gain,servo_joints[i].torque_limit);}\n')
    rows=[]
    for name,options in [('default',[]),('stationary',['-DHOST_S3_STATIONARY_PICK=1'])]:
        exe=tmp_path/name
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',*options,'-I',str(CORE/'Inc'),
                        str(src),str(CORE/'Src/servo_joint_config.c'),'-o',str(exe)],check=True,capture_output=True)
        rows.append(subprocess.check_output([str(exe)],text=True).splitlines())
    assert rows[0][:5]==rows[1][:5]
    assert rows[0][5]=='6 16 32 150'
    assert rows[1][5]=='6 16 32 80'


def test_stationary_profile_keeps_gripper_command_after_measured_terminal_settle():
    source=(CORE/'Src/binary_control.c').read_text()
    a=source.rfind('#if HOST_S3_STATIONARY_PICK',0,source.index('host_v2_shadow_executor_anchor_urad[5] = host_v2_output_urad[5]'))
    block=source[a:source.index('#endif',a)]
    assert 'host_v2_shadow_executor_anchor_urad[5] = host_v2_output_urad[5]' in block
    assert 'host_v2_shadow_executor_anchor_urad[11] = host_v2_output_urad[11]' in block
    # Only after the existing fresh feedback and convergence checks.
    assert source.rfind('BimanualFeedbackSnapshot_Copy',0,a)>source.rfind('host_v2_terminal_settle_active != 0U',0,a)
    cmake=(ROOT/'firmware/stm32_g474_single_arm/CMakeLists.txt').read_text()
    assert 'HOST_BINARY_FIRMWARE_VERSION=0x00024911UL HOST_S3_STATIONARY_PICK=1U' in cmake
