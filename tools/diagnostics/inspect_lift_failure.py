#!/usr/bin/env python3
"""Collect a failed lift log and cached controller diagnostics. READ ONLY.

No heartbeat renewal, PREPARE_SHADOW, arm/torque command, movement, reference,
STOP request, reset, reflash or calibration write. Query a latched stop without
trying to recover it. A current independent STOP proof is recorded if available.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


def collect_controller(transport, *, clock=time.monotonic):
    from so101_arm_bridge.unified_commissioning import CommissionClient, FIRMWARE
    from so101_arm_bridge.system_stop import SystemStopClient
    from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
    result = dict(read_only=True, motion_commands=0, torque_commands=0,
                  stop_requests=0, reset_requests=0, reference_requests=0, errors={})
    def read(name, operation):
        try:
            value=operation()
            result[name]=value
            return value
        except Exception as error:
            result['errors'][name]=str(error)
            return None
    hello=read('controller',lambda:asdict(transport.enter_binary_mode()))
    if hello is None or hello.get('firmware_version')!=FIRMWARE:
        result['errors']['identity']='expected integrated firmware; further queries skipped'
        return result
    # These two methods read cached evidence, without accessing motor buses.
    read('runtime',transport.stationary_runtime)
    read('motor_read_failures',transport.stationary_feedback_failures)
    commission=read('commission',lambda:asdict(CommissionClient(transport).query()))
    boot=None if commission is None else commission.get('boot_id')
    if type(boot) is int and boot>0:
        stop=SystemStopClient(transport,boot,clock)
        def stop_proof():
            proof,observed=stop.status()  # AQ query only; never stop.request().
            return dict(status=asdict(proof),confirmed=proof.confirmed,
                        observed_host_monotonic_s=observed)
        read('stop_proof',stop_proof)
        packet=LiftCommand(4,1,1,0,boot).encode()  # LS status; no active lease.
        read('lift',lambda:asdict(LiftStatus.decode(transport.exchange_mobile(packet))))
    else:
        result['errors']['boot']='current boot identity unavailable; lift and stop queries skipped'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-root',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--port',default='/dev/ttyACM0')
    args=parser.parse_args()
    encoded=args.report.read_bytes()
    execution=json.loads(encoded)
    result=dict(kind='read_only_failed_lift_diagnostics',
                collected_at_utc=datetime.now(timezone.utc).isoformat(),
                execution_path=str(args.report),execution_sha256=hashlib.sha256(encoded).hexdigest(),
                execution=execution,motion_commands=0,torque_commands=0)
    try:
        sys.path.insert(0,str(args.runtime_root/'tools/run'))
        import unified_lift
        _,manifest=unified_lift.runtime_sources(unified_lift.DEFAULT_ROOT)
        if manifest is None:raise ValueError('verified runtime bundle required')
        import serial
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
        with acquire_backend_lease('stm32',189):
            with open_exclusive_serial(serial,args.port,921600,.02) as port:
                transport=StreamValidationTransportV2(port,response_timeout_s=1)
                result['current']=collect_controller(transport)
    except Exception as error:
        result['collection_error']=str(error)
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    main()
