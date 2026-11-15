import struct

from tools.diagnostics.lds01_capture import collect, inspect_capture


class Port:
    def __init__(self, data=b'abc', read_error=False, stop_error=False):
        self.data, self.commands, self.now = data, [], 0.
        self.read_error, self.stop_error = read_error, stop_error

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, size):
        self.now += .1
        if self.read_error:
            raise OSError('device disappeared')
        result, self.data = self.data[:size], self.data[size:]
        return result

    def write(self, value):
        self.commands.append(value)
        if value == b'e' and self.stop_error:
            raise OSError('stop write failed')
        return len(value)

    def flush(self):
        raise AssertionError('tcdrain must not gate acquisition or cleanup')


def test_receive_and_stop_do_not_wait_for_hardware_drain():
    port = Port()
    data, chunks, result = collect(port, .3, start_sensor=True, clock=lambda: port.now)
    assert data == b'abc' and chunks
    assert port.commands == [b'b', b'e']
    assert result['error'] is None and result['stop_command_written']
    assert not result['physical_stop_verified']


def test_read_error_still_attempts_sensor_pause():
    port = Port(read_error=True)
    _, _, result = collect(port, .3, start_sensor=True, clock=lambda: port.now)
    assert result['error'] == 'device disappeared'
    assert port.commands == [b'b', b'e'] and result['stop_command_written']


def test_failed_pause_is_reported_without_claiming_stop():
    port = Port(stop_error=True)
    _, _, result = collect(port, .3, start_sensor=True, clock=lambda: port.now)
    assert result['stop_error'] == 'stop write failed'
    assert not result['stop_command_written']


def test_passive_capture_never_writes_sensor_commands():
    port = Port()
    collect(port, .3, clock=lambda: port.now)
    assert port.commands == []


def sector(index):
    data = bytearray(42)
    data[:2] = bytes((250, 160 + index))
    struct.pack_into('<H', data, 2, 3000)
    for offset in range(4, 40, 6):
        struct.pack_into('<HHH', data, offset, 100, 1000, 0)
    data[40] = data[41] = (255 - sum(data[:40])) & 255
    return bytes(data)


def test_zero_bytes_is_not_a_checksum_success_or_a_scan():
    report, last = inspect_capture(b'', [])
    assert report['bytes'] == report['complete_scans'] == 0 and last is None
    assert not report['motion_authorized']


def test_startup_partial_scan_is_distinct_from_a_dropped_sector():
    frame = b''.join(sector(i) for i in range(60))
    data = sector(40) + frame
    report, last = inspect_capture(data, [[.2, len(data)]])
    assert report['complete_scans'] == 1 and last == [1.] * 360
    assert len(report['sequence_errors']) == 1 and report['steady_sequence_error_count'] == 0
    data = frame[:-42] + frame
    report, _ = inspect_capture(data, [[.2, len(data)]])
    assert report['complete_scans'] == 1 and report['steady_sequence_error_count'] == 1


def test_corrupt_sector_cannot_form_a_complete_scan():
    frame = b''.join(sector(i) for i in range(60))
    corrupt = bytearray(frame)
    corrupt[10] ^= 1
    data = bytes(corrupt) + frame
    report, _ = inspect_capture(data, [[.2, len(data)]])
    assert report['checksum_failed_sectors'] == 1 and report['complete_scans'] == 1
