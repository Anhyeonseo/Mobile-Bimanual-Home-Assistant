"""L2 host reader for the isolated STM32 observer. No ROS or geometry dependency."""
from dataclasses import dataclass
import json
import secrets
import time

EXPECTED = [(0, i) for i in range(1, 7)] + [(0, 10)] + [(1, i) for i in range(1, 7)]
ARM_AXES = [axis for axis in EXPECTED if axis[1] != 10]
FIRMWARE = 's2-observer-v1'
DRIFT_PPM = 5000
RATE = DRIFT_PPM / 1_000_000
TICK_S = .001
MAX_RTT = .020
MAX_GAP = .200
MAX_SPAN = .050
# EEPROM identity is read once before motion observations. It is not an RGB-D
# exposure and has more response bytes; retain the separate 50 ms pose bound.
MAX_CALIBRATION_SPAN = .100


def uint(row, key, maximum=0xffffffff):
    value = row.get(key)
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(f'invalid {key}: expected integer in 0..{maximum}')
    return value


def info_tick(row, *, epoch=None, sequence=None):
    if row.get('firmware') != FIRMWARE or uint(row, 'motor_writes') != 0:
        raise ValueError('isolated S2 observer firmware required')
    if epoch is not None and (uint(row, 'epoch') != epoch or uint(row, 'sequence') != sequence):
        raise ValueError('epoch or sequence mismatch')
    return uint(row, 'tick_ms') / 1000


@dataclass(frozen=True)
class ClockWindow:
    tick: float
    host: float
    lo: float
    hi: float

    @classmethod
    def bind(cls, tick, sent, received):
        if not 0 <= received - sent <= MAX_RTT:
            raise ValueError('excessive bind RTT')
        return cls(tick, received, sent - tick - TICK_S, received - tick)

    def advance(self, *, first, tick, sent, received, sync_tick, sync_sent, sync_received):
        if not self.tick <= first <= tick or not 0 < received - self.host <= MAX_GAP:
            raise ValueError('MCU clock reset or observation gap')
        if not 0 <= sync_received - sync_sent <= MAX_RTT or not 0 <= sync_tick - tick <= .03:
            raise ValueError('clock reset or delayed sync')
        # Intersect the prior drift envelope with the scan and short INFO exchange.
        drift = RATE * (tick - self.tick)
        sync_drift = RATE * (sync_tick - tick)
        lo = max(self.lo - drift, sent-first-TICK_S-RATE*(tick-first), sync_sent-sync_tick-TICK_S-sync_drift)
        hi = min(self.hi + drift, received-tick, sync_received-sync_tick+sync_drift)
        if lo > hi:
            raise ValueError('MCU clock outside bounded drift envelope')
        return ClockWindow(tick, received, lo, hi)

    def decode(self, row, first, sent, *, calibration=False):
        start, end = uint(row, 'start_ms')/1000, uint(row, 'end_ms')/1000
        if not first <= start <= end <= self.tick or end-start > .02:
            raise ValueError('motor transaction too old or MCU tick wrapped')
        if row.get('ok') is not True:
            raise ValueError('invalid motor feedback')
        fields = (('offset_raw',4095),('mode',255),('model_high',255),('model_low',255),('stored_id',253),('baud',255)) if calibration else (('position',4095), ('speed',65535), ('torque',1))
        for key, maximum in fields:
            uint(row, key, maximum)
        drift = RATE * (self.tick-start)
        lo, hi = start+self.lo-drift, end+self.hi+drift
        if hi < sent-.005 or lo > self.host+.005:
            raise ValueError('MCU clock reset or drift')
        return {**row, 'acquired_s':(lo+hi)/2, 'time_uncertainty_s':(hi-lo)/2}


class Reader:
    def __init__(self, port, *, clock=time.monotonic, token=None):
        self.port, self.clock = port, clock
        self.token = secrets.randbelow(2**32-1)+1 if token is None else token
        if type(self.token) is not int or not 0 < self.token < 2**32:
            raise ValueError('nonzero uint32 token required')
        self.last_sequence = 0
        self.window = None
        self.valid = True
        self.left_grip_feedback = False
        self.arm_calibration = False
        self.last_exchange = None
        self.stage = 'idle'

    def exchange(self, command, *, sample=False):
        if not self.valid:
            raise ValueError('reader invalidated; reconnect and re-reference')
        if command not in ('INFO', 'SAMPLE', 'GRIP', 'CALIB', f'BIND {self.token}') or sample != (command in ('SAMPLE', 'GRIP', 'CALIB')):
            raise ValueError('unsupported observer request')
        if command == 'GRIP' and not self.left_grip_feedback:
            raise ValueError('active firmware does not provide left gripper load/current')
        if command == 'CALIB' and not self.arm_calibration:
            raise ValueError('active firmware does not provide arm calibration reads')
        started = self.clock()
        rows = []
        trace = {'command': command, 'request_s': started, 'rows': rows}
        if self.stage == 'sample.clock' and self.last_exchange is not None:
            self.last_exchange['clock_sync'] = trace
        else:
            self.last_exchange = trace
        try:
            packet = (command+'\n').encode('ascii')
            if self.port.write(packet) != len(packet):
                raise ValueError('partial request write')
            while self.clock()-started < 1.:
                line = self.port.readline(1024)
                if not line:
                    continue
                if len(line)>=1024 or not line.endswith(b'\n'):
                    raise ValueError('truncated observation reply')
                row = json.loads(line)
                rows.append(row)
                if not isinstance(row, dict) or 'error' in row:
                    raise ValueError('observer rejected command: '+str(row))
                if sample and 'firmware' in row:
                    raise ValueError('controller rebooted during capture')
                if len(rows)>len(EXPECTED)+1:
                    raise ValueError('unexpected sample length')
                if not sample or row.get('sample_end') is True:
                    end = self.clock()
                    trace['received_s'] = end
                    if not sample:
                        trace['row'] = row
                    return started, end, rows
            raise ValueError('observer reply timeout')
        except (ValueError, OSError, TypeError):
            self.valid = False
            raise

    def bind(self):
        try:
            if self.window is not None:
                raise ValueError('reader already bound; new reference required')
            self.stage = 'bind.identity'
            _, _, rows = self.exchange('INFO')
            info_tick(rows[0])
            self.stage = 'bind.clock'
            a, b, rows = self.exchange(f'BIND {self.token}')
            tick = info_tick(rows[0], epoch=self.token, sequence=0)
            self.left_grip_feedback = rows[0].get('left_grip_feedback') is True
            self.arm_calibration = rows[0].get('arm_calibration') is True
            self.window = ClockWindow.bind(tick, a, b)
            self.last_sequence = 0
            return {'epoch':self.token, 'host_clock_offset_interval_s':[self.window.lo,self.window.hi],
                    'round_trip_s':b-a, 'motor_writes':0}
        except (ValueError, KeyError, TypeError, OSError):
            self.valid = False
            raise

    def sample(self, *, left_gripper_only=False, arm_calibration_only=False):
        if self.window is None:
            raise ValueError('bind required')
        if left_gripper_only and arm_calibration_only:
            raise ValueError('choose one observer scan')
        try:
            self.stage = 'sample.axes'
            a,b,rows = self.exchange('CALIB' if arm_calibration_only else 'GRIP' if left_gripper_only else 'SAMPLE', sample=True)
            axes, trailer = rows[:-1], rows[-1]
            if [(uint(r,'bus'),uint(r,'id')) for r in axes] != (ARM_AXES if arm_calibration_only else [(0,6)] if left_gripper_only else EXPECTED):
                raise ValueError('missing, duplicate or wrong motor samples')
            seq = self.last_sequence+1
            if any(uint(r,'epoch')!=self.token or uint(r,'sequence')!=seq for r in rows):
                raise ValueError('reset or sequence mismatch')
            tick,first = uint(trailer,'tick_ms')/1000, uint(axes[0],'start_ms')/1000
            self.stage = 'sample.clock'
            sa,sb,reply = self.exchange('INFO')
            st = info_tick(reply[0], epoch=self.token, sequence=seq)
            window = self.window.advance(first=first,tick=tick,sent=a,received=b,sync_tick=st,
                                         sync_sent=sa,sync_received=sb)
            samples = []
            for r in axes:
                self.stage = f"sample.feedback.bus{r['bus']}.id{r['id']}"
                samples.append(window.decode(r,first,a,calibration=arm_calibration_only))
                if left_gripper_only:
                    for key, maximum in (('load_raw', 65535), ('current_raw', 65535), ('motor_status_raw', 255)):
                        uint(r, key, maximum)
            self.stage = 'sample.span'
            span = (max(r['acquired_s']+r['time_uncertainty_s'] for r in samples)
                    - min(r['acquired_s']-r['time_uncertainty_s'] for r in samples))
            limit = MAX_CALIBRATION_SPAN if arm_calibration_only else MAX_SPAN
            if span > limit:
                raise ValueError(f'{"calibration" if arm_calibration_only else "motor"} acquisition span exceeds {limit*1000:g} ms: {span*1000:.3f} ms')
            self.window, self.last_sequence = window, seq
            return {'epoch':self.token,'sequence':seq,'request_s':a,'received_s':b,'samples':samples,
                    'acquisition_span_bound_s':span,'clock_offset_interval_s':[window.lo,window.hi],
                    'clock_drift_bound_ppm':DRIFT_PPM,'motor_writes':0}
        except (ValueError, KeyError, TypeError, OSError):
            self.valid = False
            raise
