"""Firmware/capability inspection only. No ARM, enable, velocity or homing."""
from dataclasses import asdict
from .mobile_wire import MobileStatus, mobile_query


def inspect_mobile(transport):
    hello = transport.enter_binary_mode()
    report = dict(hello=asdict(hello), firmware_version_hex=f'0x{hello.firmware_version:08X}',
                  motor_commands_sent=0, motor_ids_verified=False, motion_authorized=False)
    if hello.protocol_version != 2:
        report.update(mobile_available=False, error='protocol v2 required; do not retry with motion commands')
        return report
    # HELLO/STATUS query opcodes only. Do not synchronize/arm a motion session.
    try:
        for kind, label in ((4, 'mobile_hello'), (5, 'mobile_status')):
            status = MobileStatus.decode(transport.exchange_mobile(mobile_query(kind, kind, kind)))
            if status.kind != kind or status.request_sequence != kind:
                raise RuntimeError('mobile query response mismatch')
            if kind == 5 and status.boot_id != report['mobile_hello']['boot_id']:
                raise RuntimeError('MCU restarted during inspection')
            report[label] = asdict(status)
        report['mobile_available'] = True
    except (RuntimeError, ValueError) as error:
        report.update(mobile_available=False, error=str(error))
    return report
