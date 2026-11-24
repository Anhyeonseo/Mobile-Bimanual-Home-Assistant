"""Opt-in read-only arm tuning snapshot; no configuration-write command exists.

Inventory request schema 2 retains the 56-byte envelope and validity fields.
Blocks: identity (0..6), tuning (16..29), runtime (40..49). These are raw
register values, not SI gains. Maintenance admission is enforced by firmware.
"""
import struct

from .device_inventory import decode_inventory


class TuningQueryUnsupported(ValueError):
    pass


def tuning_request(bus, motor_id):
    if type(bus) is not int or bus not in (0, 1) or type(motor_id) is not int or not 1 <= motor_id <= 6:
        raise ValueError('arm bus 0/1 and motor ID 1..6 required')
    return bytes((2, bus, motor_id, 0))


def decode_tuning(data, bus, motor_id):
    tuning_request(bus, motor_id)
    if isinstance(data, bytes) and len(data) == 56 and data[0] == 1:
        legacy = decode_inventory(data, bus, motor_id)
        if legacy['status'] == 'invalid_request':
            raise TuningQueryUnsupported('firmware lacks tuning query schema 2; no settings were changed')
    if (not isinstance(data, bytes) or len(data) != 56 or data[0] != 2 or
            data[2:4] != bytes((bus, motor_id)) or any(data[43:]) or data[1] > 3 or data[4] & ~7):
        raise ValueError('invalid or uncorrelated tuning response')
    status, mask, failed, error, domain = data[1], data[4], data[5], data[6], data[7]
    if ((status == 0 and (mask != 7 or failed != 255 or error or domain)) or
        (status in (1, 2) and (mask or failed != 255 or error or domain)) or
        (status == 3 and (mask == 7 or failed > 2 or mask & (1 << failed) or not error or domain != bus+1))):
        raise ValueError('inconsistent tuning validity fields')
    for block, (offset, size) in enumerate(((12, 7), (19, 14), (33, 10))):
        if not mask & (1 << block) and any(data[offset:offset+size]):
            raise ValueError('invalid tuning block contains data')
    def word(offset): return struct.unpack_from('<H', data, offset)[0]
    result = dict(bus=('left', 'right')[bus], requested_id=motor_id,
                  status=('complete', 'invalid_request', 'maintenance_unavailable', 'partial')[status],
                  valid_block_mask=mask, first_failed_block=None if failed == 255 else failed,
                  read_error=error, error_domain={0: None, 1: 'left_hal', 2: 'right_read_status'}.get(domain),
                  sample_started_tick_ms=struct.unpack_from('<I', data, 8)[0],
                  identity=None, tuning=None, command=None, raw_response_hex=data.hex())
    if mask & 1:
        result['identity'] = dict(firmware_major=data[12], firmware_minor=data[13],
                                 model_raw=word(15), reported_id=data[17], baud_code=data[18])
    if mask & 2:
        result['tuning'] = dict(maximum_torque_limit_raw=word(19), phase_raw=data[21], p_gain_raw=data[24],
                               d_gain_raw=data[25], i_gain_raw=data[26],
                               minimum_startup_force_raw=data[27], integral_limit_raw=data[28], cw_dead_zone_raw=data[29],
                               ccw_dead_zone_raw=data[30], protection_current_raw=word(31))
    if mask & 4:
        result['command'] = dict(torque_enabled_raw=data[33], acceleration_raw=data[34],
                                goal_position_raw=word(35), goal_time_raw=word(37),
                                goal_speed_raw=word(39), torque_limit_raw=word(41))
    result['identity_matches'] = bool(result['identity'] and result['identity']['reported_id'] == motor_id)
    result['complete'] = status == 0 and result['identity_matches']
    return result
