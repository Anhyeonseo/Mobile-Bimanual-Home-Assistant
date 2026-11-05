"""Read-only s2-observer-v1 transport and bounded continuous-count acquisition."""
import json
import math
import secrets
import time
from . import s2_commissioning as s2
EXPECTED = [(0,i) for i in range(1,7)]+[(0,10)]+[(1,i) for i in range(1,7)]


class Reader:
    def __init__(self,port,*,clock=time.monotonic,token=None):
        self.port,self.clock=port,clock
        self.token=token or secrets.randbelow(2**32-1)+1
        self.last_sequence=0
        self.offset_interval=None
        self.clock_tick=None
        self.clock_host=None
        self.last_exchange=None
        self.valid=True

    def exchange(self,command,*,sample=False):
        if not self.valid: raise ValueError('reader invalidated; reconnect and re-reference')
        started=self.clock()
        if self.port.write((command+'\n').encode())!=len(command)+1:
            self.valid=False;raise ValueError('partial request write')
        rows=[]
        try:
            while self.clock()-started<1.:
                line=self.port.readline(1024)
                if not line: continue
                if len(line)>=1024 or not line.endswith(b'\n'):raise ValueError('truncated observation reply')
                row=json.loads(line)
                if not isinstance(row,dict) or 'error' in row:raise ValueError('observer rejected command: '+str(row))
                if sample and 'firmware' in row:raise ValueError('controller rebooted during capture')
                rows.append(row)
                if len(rows)>14:raise ValueError('unexpected sample length')
                if not sample or row.get('sample_end') is True:return started,self.clock(),rows
            raise ValueError('observer reply timeout')
        except (ValueError, OSError, TypeError):
            self.valid=False
            raise

    def bind(self):
        _,_,rows=self.exchange('INFO')
        if rows[0].get('firmware')!='s2-observer-v1' or rows[0].get('motor_writes')!=0:
            self.valid=False;raise ValueError('isolated S2 observer firmware required')
        a,b,rows=self.exchange('BIND '+str(self.token))
        r=rows[0]
        if r.get('epoch')!=self.token or r.get('sequence')!=0 or b-a>.02:
            self.valid=False;raise ValueError('epoch/clock bind failed or excessive RTT')
        tick=r['tick_ms']/1000
        # 1 ms MCU tick quantization is included in the lower offset bound.
        self.offset_interval=(a-tick-.001,b-tick)
        self.clock_tick=tick
        self.clock_host=b
        return {'epoch':self.token,'host_clock_offset_interval_s':list(self.offset_interval),
                'round_trip_s':b-a,'motor_writes':0}

    def sample(self):
        if self.offset_interval is None:raise ValueError('bind required')
        a,b,rows=self.exchange('SAMPLE',sample=True)
        self.last_exchange={'request_s':a,'received_s':b,'rows':rows}
        try:
            if len(rows)!=14 or [(r.get('bus'),r.get('id')) for r in rows[:-1]]!=EXPECTED:
                raise ValueError('missing, duplicate or wrong motor samples')
            seq=self.last_sequence+1
            if any(r.get('epoch')!=self.token or r.get('sequence')!=seq for r in rows):
                raise ValueError('reset or sequence mismatch')
            # STM32 and host clocks do not run at exactly the same rate. Bound
            # the offset at EACH exchange using request/response causality,
            # rather than keeping the startup offset forever. 5000 ppm is an
            # explicit oscillator envelope, not a relaxed association limit.
            if type(rows[-1].get('tick_ms')) is not int or type(rows[0].get('start_ms')) is not int:
                raise ValueError('integer MCU timing required')
            tick=rows[-1]['tick_ms']/1000
            first=rows[0]['start_ms']/1000
            if not self.clock_tick<=first<=tick or not 0<b-self.clock_host<=.2:
                raise ValueError('MCU clock reset or observation gap')
            rate=.005
            elapsed=tick-self.clock_tick
            drift=rate*elapsed
            offset_lo=max(self.offset_interval[0]-drift,a-first-.001-rate*(tick-first))
            offset_hi=min(self.offset_interval[1]+drift,b-tick)
            # A short INFO exchange bounds the clock without the long motor
            # scan/USB buffering delay. It never resets the epoch or sequence.
            sa,sb,sync_rows=self.exchange('INFO')
            sync=sync_rows[0]
            self.last_exchange['clock_sync']={'request_s':sa,'received_s':sb,'row':sync}
            if (sync.get('firmware')!='s2-observer-v1' or sync.get('epoch')!=self.token
                    or sync.get('sequence')!=seq or sync.get('motor_writes')!=0
                    or type(sync.get('tick_ms')) is not int or sb-sa>.02):
                raise ValueError('clock sync identity/latency failed')
            st=sync['tick_ms']/1000
            if not tick<=st or st-tick>.03:raise ValueError('clock reset or delayed sync')
            sync_drift=rate*(st-tick)
            offset_lo=max(offset_lo,sa-st-.001-sync_drift)
            offset_hi=min(offset_hi,sb-st+sync_drift)
            if offset_lo>offset_hi:
                raise ValueError('MCU clock outside bounded drift envelope')
            samples=[]
            for r in rows[:-1]:
                for key in ('position','speed','torque','start_ms','end_ms'):
                    if type(r[key]) is not int:raise ValueError('integer motor telemetry required')
                if r['ok'] is not True or not 0<=r['position']<=4095 or not 0<=r['speed']<=65535 or r['torque'] not in (0,1):
                    raise ValueError('invalid motor feedback')
                start,end=r['start_ms']/1000,r['end_ms']/1000
                if not first<=start<=end<=tick or not 0<=end-start<=.02:raise ValueError('motor transaction too old or MCU tick wrapped')
                event_drift=rate*(tick-start)
                lo=start+offset_lo-event_drift;hi=end+offset_hi+event_drift
                if hi<a-.005 or lo>b+.005:raise ValueError('MCU clock reset or drift')
                samples.append({**r,'acquired_s':(lo+hi)/2,'time_uncertainty_s':(hi-lo)/2})
            span=max(r['acquired_s']+r['time_uncertainty_s'] for r in samples)-min(r['acquired_s']-r['time_uncertainty_s'] for r in samples)
            if span>.05:raise ValueError(f'motor acquisition span exceeds 50 ms: {span*1000:.3f} ms')
            self.last_sequence=seq
            self.offset_interval=(offset_lo,offset_hi)
            self.clock_tick=tick
            self.clock_host=b
            return {'epoch':self.token,'sequence':seq,'request_s':a,'received_s':b,
                    'acquisition_span_bound_s':span,'samples':samples,'motor_writes':0,
                    'clock_offset_interval_s':[offset_lo,offset_hi],'clock_drift_bound_ppm':5000}
        except (ValueError,KeyError,TypeError):
            self.valid=False
            raise


class EncoderTracker:
    """Unwrap only continuous short steps from an explicitly confirmed Home/bottom.

    Modulo arithmetic here tracks measured motion, never chooses a motion path.
    A gap or any discontinuity invalidates ALL axes and requires re-anchoring.
    """
    def __init__(self,profile,first,*,home_confirmed,bottom_confirmed):
        if home_confirmed is not True or bottom_confirmed is not True:
            raise ValueError('operator reference and physical bottom required')
        self.profile=profile;self.epoch=first['epoch'];self.last={};self.counts={};self.valid=True
        for row in first['samples']:
            key=(row['bus'],row['id'])
            raw=row['position']
            if key==(0,10):count=raw
            else:
                side='left' if key[0]==0 else 'right'
                ref=profile['arms'][side]['reference_raw'][s2.JOINTS[key[1]-1]]
                delta=(raw-ref+2048)%4096-2048
                if abs(delta)>128:raise ValueError('arm no longer at captured reference pose')
                count=ref+delta
            self.last[key]=row;self.counts[key]=count
        if set(self.last)!=set(EXPECTED):raise ValueError('all 13 axes required')
        self.origin=self.counts[(0,10)]
        self.sequence=first['sequence']

    def update(self,record):
        try:
            if not self.valid or record['epoch']!=self.epoch or record['sequence']!=self.sequence+1:
                raise ValueError('encoder epoch/sequence changed')
            if [(r['bus'],r['id']) for r in record['samples']]!=EXPECTED:raise ValueError('axis inventory changed')
            for row in record['samples']:
                key=(row['bus'],row['id']);old=self.last[key]
                if not 0<row['acquired_s']-old['acquired_s']<=.2:raise ValueError('encoder sample gap or reset')
                if type(row['position']) is not int or not 0<=row['position']<=4095:raise ValueError('invalid encoder')
                delta=(row['position']-old['position']+2048)%4096-2048
                if abs(delta)>256:raise ValueError('encoder discontinuity')
                self.counts[key]+=delta
                self.last[key]=row
            self.sequence=record['sequence']
            return self.state()
        except (ValueError,KeyError,TypeError):
            self.valid=False
            raise

    def state(self):
        if not self.valid:raise ValueError('turn reference invalid')
        return {'raw':{side:{j:self.counts[(bus,i+1)] for i,j in enumerate(s2.JOINTS)} for bus,side in enumerate(s2.ARMS)},
                'lift_raw_unwrapped':self.counts[(0,10)],
                'lift_height_m':(self.counts[(0,10)]-self.origin)*self.profile['lift']['meters_per_count'],
                'joint_times_s':{side:{j:self.last[(bus,i+1)]['acquired_s'] for i,j in enumerate(s2.JOINTS)} for bus,side in enumerate(s2.ARMS)},
                'lift_time_s':self.last[(0,10)]['acquired_s']}
