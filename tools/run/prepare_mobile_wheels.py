#!/usr/bin/env python3
"""Dedicated wheel setup image only. --prepare persists mode=1 on wheels 7..9.

No torque enable, nonzero speed, or arm/lift writes. No automatic retry.
"""
import argparse
import json
from pathlib import Path
import sys
import time
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial


def receive(port, seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        raw=port.readline()
        if raw:
            value=json.loads(raw.decode("ascii"))
            if not isinstance(value,dict): raise ValueError("invalid response")
            return value
    raise RuntimeError("response timeout; do not automatically retry setup")


def prepare(port, perform=False):
    port.reset_input_buffer()
    if port.write(b"INFO\n")!=5: raise RuntimeError("partial INFO write")
    info=receive(port,2)
    if (info.get("firmware")!="mobile-wheel-setup-v1" or
        info.get("motion_enabled") is not False or info.get("ids")!=[7,8,9,10] or
        info.get("host_baud")!=921600 or info.get("servo_baud")!=1000000):
        raise RuntimeError("unexpected firmware; no setup command sent")
    report={"info":info,"setup_requested":False}
    if not perform: return report
    command=b"PREPARE_WHEELS\n"
    if port.write(command)!=len(command): raise RuntimeError("partial setup write; do not retry")
    result=receive(port,6)
    wheels=result.get("wheels")
    if (type(result.get("setup_ok")) is not bool or result.get("motion_enabled") is not False or
        not isinstance(wheels,list) or len(wheels)!=3 or
        any(not isinstance(w,dict) for w in wheels) or [w.get("id") for w in wheels]!=[7,8,9]):
        raise RuntimeError("invalid setup result; inspect before any further action")
    if result["setup_ok"] and not all(w.get("ok") is True and
        w.get("mode")==1 and w.get("torque")==0 and w.get("lock")==1 for w in wheels):
        raise RuntimeError("inconsistent setup readback")
    report.update(setup_requested=True,result=result)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",required=True)
    parser.add_argument("--prepare",action="store_true",help="Persist wheel modes; keep wheels raised and lift supported")
    args=parser.parse_args()
    try:
        import serial
        with acquire_backend_lease("stm32",0):
            with open_exclusive_serial(serial,args.port,921600,.2) as port:
                report=prepare(port,args.prepare)
        print(json.dumps(report,indent=2))
        return 0 if report.get("result",{}).get("setup_ok",True) else 2
    except (ImportError,OSError,RuntimeError,ValueError) as error:
        print(json.dumps({"error":str(error),"motion_commands_supported":False}),file=sys.stderr)
        return 2


if __name__=="__main__": raise SystemExit(main())
