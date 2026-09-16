import struct
import pytest
from so101_arm_bridge.system_stop import SystemStopClient, SystemStopStatus, stop_query
from so101_arm_bridge.mobile_wire import crc32c
from home_robot_tasks.system_stop_backend import SystemStopBackend


def response(seq=1, boot=42, tick=120, observed=119, actions=110, state=2, flags=255):
    data = bytearray(64)
    data[:4] = b"AT\x01\x01"
    struct.pack_into("<5I", data, 4, seq, tick, boot, 100, actions)
    data[24:26] = bytes((state, flags))
    struct.pack_into("<3I", data, 28, observed, (tick-observed) & 0xFFFFFFFF, 100)
    struct.pack_into("<I", data, 60, crc32c(data[:60]))
    return bytes(data)


@pytest.mark.parametrize("kwargs", [{"state": 1}, {"flags": 127}, {"observed": 110}, {"observed": 121}, {"tick": 219}])
def test_incomplete_or_stale_proof_is_not_confirmed(kwargs):
    assert not SystemStopStatus.decode(response(**kwargs)).confirmed


class Transport:
    def __init__(self):
        self.now = 1.0
        self.delay = 0.001
        self.stops = 0
        self.options = {}
    def safe_stop(self):
        self.stops += 1
        return object()  # Admission is deliberately not proof.
    def exchange_mobile(self, packet):
        assert packet[:4] == b"AQ\x01\x01" and crc32c(packet[:32]) == struct.unpack_from("<I", packet, 32)[0]
        self.now += self.delay
        return response(seq=struct.unpack_from("<I", packet, 4)[0], **self.options)


def test_task_stop_keeps_ownership_until_all_evidence_is_fresh():
    t = Transport()
    backend = SystemStopBackend(SystemStopClient(t, 42, lambda: t.now))
    backend.request("run", True, t.now)
    backend.request("run", True, t.now)
    assert t.stops == 1
    t.options = {"state": 1}
    assert backend.poll("run", t.now).status == "RUNNING"
    t.options = {}
    result = backend.poll("run", t.now)
    assert result.status == "STOPPED" and all(result.conditions.values())
    assert result.observed_s < t.now
    with pytest.raises(RuntimeError):
        backend.request("other", False, t.now)
    with pytest.raises(RuntimeError):
        backend.poll("other", t.now)


@pytest.mark.parametrize("options,delay", [({"boot": 43}, 0.001), ({}, 0.2), ({}, -0.1), ({"observed": 25, "actions": 20}, 0.01)])
def test_restart_delay_or_clock_jump_keeps_stop_unconfirmed(options, delay):
    t = Transport(); t.options = options; t.delay = delay
    client = SystemStopClient(t, 42, lambda: t.now)
    with pytest.raises(RuntimeError):
        client.status()


def test_query_and_reply_are_strict():
    for value in (0, -1, 2**32, True):
        with pytest.raises(ValueError): stop_query(value)
    data = bytearray(response()); data[40] = 1
    struct.pack_into("<I", data, 60, crc32c(data[:60]))
    with pytest.raises(ValueError): SystemStopStatus.decode(data)
    data = bytearray(response()); data[32] ^= 1
    struct.pack_into("<I", data, 60, crc32c(data[:60]))
    with pytest.raises(ValueError): SystemStopStatus.decode(data)


def test_real_shared_transport_wraps_stop_query_in_v2():
    from so101_arm_bridge.stream_protocol_v2 import FrameV2, StreamMessageTypeV2 as Msg, decode_frame_v2, encode_frame_v2
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2, StreamTransportV2Error
    class Serial:
        reply = b""
        writes = 0
        def write(self, packet):
            request = decode_frame_v2(packet)
            assert request.message_type == Msg.MOBILE_REQUEST
            self.writes += 1
            seq = struct.unpack_from("<I", request.payload, 4)[0]
            self.reply = encode_frame_v2(FrameV2(message_type=Msg.MOBILE_RESPONSE,
                sequence=request.sequence, payload=b"\0" + response(seq=seq)))
            return len(packet)
        def read_until(self, delimiter):
            value, self.reply = self.reply, b""
            return value
    serial = Serial()
    transport = StreamValidationTransportV2(serial)
    assert SystemStopStatus.decode(transport.exchange_mobile(stop_query(17))).sequence == 17
    invalid = bytearray(stop_query(18)); invalid[8] = 1
    struct.pack_into("<I", invalid, 32, crc32c(invalid[:32]))
    with pytest.raises(StreamTransportV2Error): transport.exchange_mobile(bytes(invalid))
    assert serial.writes == 1


def test_failed_stop_request_can_be_retried_without_releasing_owner():
    t = Transport()
    original = t.safe_stop
    t.safe_stop = lambda: (_ for _ in ()).throw(RuntimeError("disconnected"))
    backend = SystemStopBackend(SystemStopClient(t, 42, lambda: t.now))
    with pytest.raises(RuntimeError): backend.request("run", True, t.now)
    with pytest.raises(RuntimeError): backend.request("other", True, t.now)
    t.safe_stop = original
    backend.request("run", True, t.now)
    assert t.stops == 1


def test_nested_children_share_root_stop_but_unrelated_or_late_calls_are_rejected():
    from home_robot_tasks.execution import TaskLease
    lease = TaskLease(); lease.acquire('run')
    t = Transport()
    backend = SystemStopBackend(SystemStopClient(t, 42, lambda: t.now), lease)
    for goal in ('run/03_search:2/0', 'run/03_search:3', 'run'):
        backend.request(goal, True, t.now)
        assert backend.poll(goal, t.now).goal_id == goal
    assert t.stops == 1 and backend.run_id == 'run'
    for goal in ('runner/03_search', 'other', 'run:1'):
        with pytest.raises(RuntimeError): backend.request(goal, True, t.now)
        with pytest.raises(RuntimeError): backend.poll(goal, t.now)
    lease.release('run')
    with pytest.raises(RuntimeError): backend.poll('run/03_search:3', t.now)
    lease.acquire('next')
    with pytest.raises(RuntimeError): backend.request('next', False, t.now)
