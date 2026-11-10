"""Stationary test I/O ownership and measured STOP verification.

The resident safety stop disables arm torque. It is not a payload-hold command.
Never translate its ACK alone into a physical stop confirmation.
"""
from types import SimpleNamespace
import time
from .development_ranges import unwrap_in_range
from .limit_manifest import JOINTS


def verify_inventory(transport,ranges):
    rows=[]
    for bus in (0,1):
        side=('left','right')[bus]
        for index in range(6):
            row=transport.read_device_inventory(bus,index+1);rows.append(row)
            if (not row['complete'] or row['offset_and_mode']['offset_raw']!=ranges['calibration'][side]['offsets_raw'][index]
                    or row['offset_and_mode']['operating_mode']!=0 or row['command']['torque_enabled_raw']!=0):
                raise ValueError('arm calibration/mode/torque-off mismatch: '+side+str(index+1))
            # A supported, torque-off arm can have been moved beyond the
            # recorded envelope. Reject before starting the idle observer:
            # that observer otherwise latches the whole controller on conversion.
            name=JOINTS[index]
            try:unwrap_in_range(ranges['arms'][side][name],row['feedback']['position_raw'])
            except ValueError as exc:
                raise ValueError('current arm pose outside recorded range; idle reader not started: '+side+'/'+name) from exc
    for motor in range(7,11):
        row=transport.read_device_inventory(0,motor);rows.append(row)
        if not row['complete'] or row['command']['goal_speed_raw']!=0:
            raise ValueError('stationary platform must have no velocity goal')
    return rows


class ResidentStop:
    def __init__(self,transport,clock=time.monotonic):
        self.transport,self.clock=transport,clock;self.requested=False;self.previous=None
        self.status='STOPPING';self.reason='';self.last_poll=-1.
        self.evidence={};self.first_error=None
    def request(self,goal,hold,now):
        if self.requested:return
        self.requested=True
        try:self.transport.safe_stop()
        except Exception as exc:self.reason=str(exc);self.first_error=self.reason
    def poll(self,goal,now):
        if self.status=='STOPPED':return SimpleNamespace(status=self.status)
        if now-self.last_poll<.25:return SimpleNamespace(status='STOPPING')
        self.last_poll=now
        try:
            self.evidence=dict(stage='latch')
            state=self.transport.get_state()
            if not state.stop_latched:raise ValueError('stop latch absent')
            positions=[]
            for bus,motor in [(b,i) for b in (0,1) for i in range(1,7)]+[(0,i) for i in range(7,11)]:
                self.evidence=dict(stage='inventory',bus=bus,motor_id=motor)
                row=self.transport.read_device_inventory(bus,motor)
                self.evidence['row']=row
                if not row['complete']:raise ValueError('stop feedback incomplete')
                if motor<=6 and row['command']['torque_enabled_raw']!=0:raise ValueError('arm torque disable unconfirmed')
                if motor>=7 and row['command']['goal_speed_raw']!=0:raise ValueError('platform goal not zero')
                positions.append(row['feedback']['position_raw'])
            observed=self.clock()
            self.evidence=dict(stage='stability',observed_s=observed,positions_raw=positions)
            self.reason='waiting for a second stationary observation'
            if self.previous:
                stamp,old=self.previous
                delta=[abs((a-b+2048)%4096-2048) for a,b in zip(positions,old)]
                self.evidence.update(interval_s=observed-stamp,maximum_delta_raw=max(delta))
                self.reason='position stability not yet confirmed'
                if observed-stamp>=.2 and max(delta)<=2:self.status='STOPPED';self.reason=''
            self.previous=(observed,positions)
        except Exception as exc:
            self.reason=str(exc);self.previous=None
            if self.first_error is None:self.first_error=self.reason
        return SimpleNamespace(status=self.status,reason=self.reason)

    def diagnostics(self):
        return dict(status=self.status,reason=self.reason,first_error=self.first_error,evidence=self.evidence)
