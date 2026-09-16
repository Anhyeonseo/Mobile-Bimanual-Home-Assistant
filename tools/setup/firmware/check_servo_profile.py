#!/usr/bin/env python3
"""Compile the actual arm table/hash function and verify the ROS expectation.

--write updates the host hash after a deliberate PID-table edit. No device I/O.
Joint limits, calibration zero/direction, torque limits and firmware are untouched.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[3]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group();group.add_argument('--check',action='store_true');group.add_argument('--write',action='store_true')
    args=p.parse_args()
    text=(BOARD/'Src/binary_control.c').read_text()
    start=text.index('static uint32_t Host_CalibrationHash(void)')
    end=text.index('\nstatic uint8_t Host_InitBufferedRoute(',start)
    code='#include <stdio.h>\n#include "servo_joint_config.h"\n#include "actuator_core/crc32c.h"\n'+text[start:end]+r'''
int main(void){
 printf("%08X\n",Host_CalibrationHash());
 for(unsigned n=0;n<servo_joint_count;n++)
 printf("%u %u %u\n",servo_joints[n].id,servo_joints[n].p_gain,servo_joints[n].d_gain);
}
'''
    with tempfile.TemporaryDirectory(prefix='servo-profile-') as tmp:
        src=Path(tmp)/'profile.c';exe=Path(tmp)/'profile';src.write_text(code)
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(src),
            str(BOARD/'Src/servo_joint_config.c'),str(CORE/'src/crc32c.c'),'-o',str(exe)],check=True)
        lines=subprocess.check_output([str(exe)],text=True).splitlines()
    value=int(lines[0],16)
    host=ROOT/'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/bimanual_stream_adapter.py'
    original=host.read_text();pattern=r'^CALIBRATION_HASH = 0x[0-9A-Fa-f]+$'
    matches=re.findall(pattern,original,flags=re.M)
    if len(matches)!=1:raise ValueError('exactly one host calibration hash required')
    expected=f'CALIBRATION_HASH = 0x{value:08X}'
    if args.write and matches[0]!=expected:
        host.write_text(re.sub(pattern,expected,original,flags=re.M))
    elif matches[0]!=expected:
        raise ValueError(f'firmware/ROS servo profile mismatch: expected {expected}')
    print(json.dumps(dict(calibration_hash=f'0x{value:08X}',gains=[dict(zip(('servo_id','p','d'),map(int,row.split()))) for row in lines[1:]],matched=True,hardware_commands=0)))
if __name__=='__main__':main()
