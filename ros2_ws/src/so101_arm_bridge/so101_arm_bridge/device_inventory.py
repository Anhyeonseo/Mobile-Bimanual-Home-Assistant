"""L2 fixed-block, read-only commissioning snapshots. Values remain motor raw units."""
import struct

FIRMWARE_VERSION = 0x00024909
BLOCKS = ((0, 7, 12), (9, 4, 19), (31, 3, 23), (40, 10, 26), (55, 16, 36))
EXPECTED_DEVICES = tuple((0, i) for i in range(1, 11)) + tuple((1, i) for i in range(1, 7))


def inventory_request(bus, motor_id):
    if type(bus) is not int or type(motor_id) is not int or (bus, motor_id) not in EXPECTED_DEVICES:
        raise ValueError('expected left bus IDs 1..10 or right bus IDs 1..6')
    return bytes((1, bus, motor_id, 0))


def decode_inventory(data, bus, motor_id):
    inventory_request(bus, motor_id)
    if (not isinstance(data, bytes) or len(data) != 56 or data[0] != 1 or
            data[2:4] != bytes((bus, motor_id)) or any(data[52:]) or data[1] > 3 or data[4] & ~31):
        raise ValueError('invalid or uncorrelated inventory response')
    status, mask, failed, error, domain = data[1], data[4], data[5], data[6], data[7]
    if ((status == 0 and (mask != 31 or failed != 255 or error or domain)) or
        (status in (1, 2) and (mask or failed != 255 or error or domain)) or
        (status == 3 and (mask == 31 or failed > 4 or mask & (1 << failed) or not error or domain != bus+1))):
        raise ValueError('inconsistent inventory validity fields')
    result = dict(bus='left' if bus == 0 else 'right', requested_id=motor_id,
                  status=('complete', 'invalid_request', 'maintenance_unavailable', 'partial')[status],
                  valid_block_mask=mask, first_failed_block=None if failed == 255 else failed,
                  read_error=error, error_domain={0:None, 1:'left_hal', 2:'right_read_status'}.get(domain),
                  sample_started_tick_ms=struct.unpack_from('<I', data, 8)[0],
                  identity=None, limits=None, offset_and_mode=None, command=None, feedback=None)
    def word(offset): return struct.unpack_from('<H', data, offset)[0]
    if mask & 1:
        result['identity'] = dict(firmware_major=data[12], firmware_minor=data[13],
                                 model_raw=word(15), reported_id=data[17], baud_code=data[18])
    if mask & 2:
        result['limits'] = dict(minimum_raw=word(19), maximum_raw=word(21))
    if mask & 4:
        result['offset_and_mode'] = dict(offset_raw=word(23), operating_mode=data[25])
    if mask & 8:
        result['command'] = dict(torque_enabled_raw=data[26], goal_position_raw=word(28),
                                goal_speed_raw=word(32), torque_limit_raw=word(34))
    if mask & 16:
        result['feedback'] = dict(lock_raw=data[36], position_raw=word(37), speed_raw=word(39),
                                 load_raw=word(41), voltage_raw=data[43], temperature_raw=data[44],
                                 moving_raw=data[47], current_raw=word(50))
    result['identity_matches'] = bool(result['identity'] and result['identity']['reported_id'] == motor_id)
    result['complete'] = status == 0 and result['identity_matches']
    return result


def collect_inventory(transport):
    """Caller already checked firmware and owns maintenance serial lease."""
    snapshots = []
    interrupted = None
    for bus, motor_id in EXPECTED_DEVICES:
        try:
            snapshots.append(transport.read_device_inventory(bus, motor_id))
        except (OSError, RuntimeError, ValueError) as error:
            interrupted = dict(bus=bus, motor_id=motor_id, error=str(error))
            break  # Preserve earlier observations; do not retry a broken connection.
    return dict(devices=snapshots, expected_motor_count=16,
                complete_motor_count=sum(s['complete'] for s in snapshots),
                interrupted=interrupted,
                all_snapshots_complete=len(snapshots) == 16 and all(s['complete'] for s in snapshots),
                physical_calibration_verified=False, motion_authorized=False,
                motor_write_commands=0)
