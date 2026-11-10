#!/usr/bin/env python3
"""Temporary centering service: backup first, six explicit center writes, no motion."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/so101_arm_bridge'))

# Compatibility exports for legacy commissioning and range-recording tools.
from so101_arm_bridge.calibration_maintenance import Link, distance, run, save, validate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='/dev/ttyACM0')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--center',action='store_true',help='Write centering offsets at the supported, operator-set Home pose')
    args=p.parse_args()
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,args.port,921600,.1) as port:
        port.reset_input_buffer()
        record=run(Link(port),args.output,center=args.center)
    print(json.dumps(record,indent=2))
if __name__=='__main__':main()
