from copy import deepcopy
from pathlib import Path
import json
import pytest
from home_robot_tasks import s2_acquisition as acq, s2_development as dev
ROOT=Path(__file__).resolve().parents[1]

class Port:
    def __init__(self):self.now=100.;self.rows=[];self.epoch=0;self.sequence=0;self.failure=None;self.commands=[]
    def write(self,data):
        text=data.decode().strip();self.commands.append(text)
        self.now+=.001
        if text.startswith('BIND '):self.epoch=int(text.split()[1]);self.sequence=0
        if text=='SAMPLE':
            self.sequence+=1
            self.rows=[{'epoch':self.epoch,'sequence':self.sequence,'bus':bus,'id':i,'start_ms':int((self.now-99)*1000)+k,
                        'end_ms':int((self.now-99)*1000)+k+1,'position':2048,'speed':0,'torque':0,'ok':True} for k,(bus,i) in enumerate(acq.EXPECTED)]
            if self.failure=='missing':self.rows.pop()
            if self.failure=='reboot':self.rows[0]['epoch']=0
            if self.failure=='bad':self.rows[0]['ok']=False
            if self.failure=='wide':self.rows[-1]['end_ms']+=100
            self.rows.append({'sample_end':True,'epoch':self.epoch,'sequence':self.sequence,'tick_ms':int((self.now-99)*1000)+13})
        else:self.rows=[{'firmware':'s2-observer-v1','epoch':self.epoch,'sequence':self.sequence,'tick_ms':int((self.now-99)*1000),'motor_writes':0}]
        return len(data)
    def readline(self,n):
        self.now+=.001
        return (json.dumps(self.rows.pop(0))+'\n').encode() if self.rows else b''


def test_reader_only_sends_info_bind_sample_and_validates_times():
    port=Port();r=acq.Reader(port,clock=lambda:port.now,token=123)
    binding=r.bind();record=r.sample()
    assert binding['epoch']==123 and len(record['samples'])==13
    assert 0<record['acquisition_span_bound_s']<.05
    assert port.commands==['INFO','BIND 123','SAMPLE','INFO']

@pytest.mark.parametrize('failure',['missing','reboot','bad','wide'])
def test_bad_packet_invalidates_reader(failure):
    port=Port();r=acq.Reader(port,clock=lambda:port.now,token=123);r.bind();port.failure=failure
    with pytest.raises(ValueError):r.sample()
    assert r.valid is False
    with pytest.raises(ValueError):r.sample()


class DriftingPort(Port):
    def __init__(self,rate):
        super().__init__();self.rate=rate;self.step=0.
    def write(self,data):
        result=super().write(data)
        # Convert the simulated MCU clock independently of the host clock.
        for row in self.rows:
            for key in ('tick_ms','start_ms','end_ms'):
                if key in row:row[key]=round(row[key]*self.rate+self.step*1000)
        return result


@pytest.mark.parametrize('rate',[.997,1.003])
def test_reader_tracks_bounded_clock_rate_difference_for_five_seconds(rate):
    port=DriftingPort(rate);r=acq.Reader(port,clock=lambda:port.now,token=123);r.bind()
    for _ in range(100):
        port.now+=.035
        record=r.sample()
        assert record['acquisition_span_bound_s']<.05
        for sample in record['samples']:
            actual_start=sample['start_ms']/1000/rate+99
            assert abs(sample['acquired_s']-actual_start)<=sample['time_uncertainty_s']+.002


@pytest.mark.parametrize('failure',['clock_step','gap','tick_reversal'])
def test_clock_correction_does_not_hide_discontinuities(failure):
    port=DriftingPort(1.);r=acq.Reader(port,clock=lambda:port.now,token=123);r.bind();r.sample()
    if failure=='clock_step':port.step=.02
    if failure=='tick_reversal':port.step=-.1
    if failure=='gap':port.now+=.3
    with pytest.raises(ValueError):r.sample()
    assert r.valid is False


def fixture():
    p=dev.build(ROOT)
    samples=[]
    for bus,i in acq.EXPECTED:
        raw=2048 if i==10 else p['arms']['left' if bus==0 else 'right']['reference_raw'][acq.s2.JOINTS[i-1]]
        samples.append({'bus':bus,'id':i,'position':raw,'acquired_s':100.})
    return p,{'epoch':1,'sequence':1,'samples':samples}


def test_continuous_unwrap_crosses_zero_without_a_full_turn_jump():
    p,a=fixture();a['samples'][6]['position']=5
    t=acq.EncoderTracker(p,a,home_confirmed=True,bottom_confirmed=True)
    b=deepcopy(a);b['sequence']=2
    for r in b['samples']:r['acquired_s']+=.05
    b['samples'][6]['position']=4090
    out=t.update(b)
    assert out['lift_raw_unwrapped']==-6
    assert out['lift_height_m']==pytest.approx(11*.084/4096)

@pytest.mark.parametrize('failure',['gap','jump','epoch','sequence','missing','home','unconfirmed'])
def test_tracker_loses_reference_on_bad_stream(failure):
    p,a=fixture()
    if failure in ['home','unconfirmed']:
        if failure=='home':a['samples'][0]['position']+=600
        with pytest.raises(ValueError):acq.EncoderTracker(p,a,home_confirmed=failure!='unconfirmed',bottom_confirmed=True)
        return
    t=acq.EncoderTracker(p,a,home_confirmed=True,bottom_confirmed=True)
    b=deepcopy(a);b['sequence']=2
    for r in b['samples']:r['acquired_s']+=.05
    if failure=='gap':b['samples'][0]['acquired_s']+=1
    if failure=='jump':b['samples'][0]['position']+=900
    if failure=='epoch':b['epoch']=2
    if failure=='sequence':b['sequence']=1
    if failure=='missing':b['samples'].pop()
    with pytest.raises(ValueError):t.update(b)
    with pytest.raises(ValueError):t.state()


def test_rgbd_clock_rejects_hardware_domain_and_wall_steps():
    from home_robot_tasks.s2_rgbd_acquisition import GlobalClock
    times={'wall':1000.,'mono':10.}
    clock=GlobalClock(wall=lambda:times['wall'],mono=lambda:times['mono'])
    assert clock.convert(999950.,'timestamp_domain.global_time')==pytest.approx(9.95)
    with pytest.raises(ValueError):clock.convert(999950.,'timestamp_domain.hardware_clock')
    with pytest.raises(ValueError):clock.convert(999000.,'timestamp_domain.global_time')
    times['wall']+=1
    with pytest.raises(ValueError):clock.convert(1000950.,'timestamp_domain.global_time')


def test_association_accounts_for_motor_read_interval():
    from home_robot_tasks.s2_rgbd_acquisition import association
    motors={'samples':[{'acquired_s':10.,'time_uncertainty_s':.01},{'acquired_s':10.02,'time_uncertainty_s':.01}]}
    good=association(motors,{'color_time_s':10.02,'depth_time_s':10.015})
    assert good['acquisition_span_s']==pytest.approx(.04)
    assert good['physical_hardware_sync_accuracy_verified'] is False
    with pytest.raises(ValueError):association(motors,{'color_time_s':10.041,'depth_time_s':10.04})


def test_camera_matches_delayed_exposure_instead_of_first_delivered_frame():
    from home_robot_tasks.s2_rgbd_acquisition import Capture
    camera=Capture.__new__(Capture);camera.last_matched=None
    camera.buffer=[{'capture_id':'old','color_time_s':9.9,'depth_time_s':9.9}]
    class Pipeline:
        calls=0
        def wait_for_frames(self,timeout):
            self.calls+=1
            return {'capture_id':'good','color_time_s':10.02,'depth_time_s':10.019}
    camera.pipeline=Pipeline()
    camera._record=lambda row:camera.buffer.append(row)
    motors={'samples':[{'acquired_s':10.,'time_uncertainty_s':.001},
                       {'acquired_s':10.045,'time_uncertainty_s':.001}]}
    result=camera.match(motors)
    assert result['capture_id']=='good' and camera.pipeline.calls==1
    assert result['acquisition_span_s']<.05
