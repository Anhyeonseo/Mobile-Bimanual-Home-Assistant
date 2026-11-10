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


def test_partial_scan_reports_the_current_failure_not_a_previous_good_sample():
    from contextlib import nullcontext
    from types import SimpleNamespace
    from tools.diagnostics.capture_s2_observer import capture

    class FailingPort(Port):
        def reset_input_buffer(self): pass

        def write(self,data):
            n=super().write(data)
            if data==b'SAMPLE\n':
                self.rows=[{**self.rows[0],'ok':False}, {'error':'motor_read_failed'}]
            return n

    p=FailingPort()
    report=capture(SimpleNamespace(port='fake'), {'records':[]},None,None,None,None,
                   lambda *args:nullcontext(), lambda *args:nullcontext(p))
    assert report['status']=='failed' and report['failure_stage']=='L2.sample.axes'
    assert report['last_valid_sequence']==0
    assert report['last_raw_exchange']['rows'][0]['id']==1
    assert report['last_raw_exchange']['rows'][-1]=={'error':'motor_read_failed'}
    assert p.commands[0]=='INFO' and p.commands[-1]=='SAMPLE'


def test_failed_write_invalidates_reader_and_cannot_resume():
    port=Port();reader=acq.Reader(port,clock=lambda:port.now,token=7);reader.bind()
    def disconnected(_):raise OSError('link dropped')
    port.write=disconnected
    with pytest.raises(OSError):reader.sample()
    assert not reader.valid
    with pytest.raises(ValueError, match='invalidated'):reader.sample()


@pytest.mark.parametrize('change', ['wrong firmware','missing tick','bool sequence'])
def test_bad_bind_reply_cannot_establish_a_clock(change):
    class BadBind(Port):
        def write(self,data):
            n=super().write(data)
            if data.startswith(b'BIND'):
                if change=='wrong firmware':self.rows[0]['firmware']='other'
                elif change=='missing tick':self.rows[0].pop('tick_ms')
                else:self.rows[0]['sequence']=False
            return n
    port=BadBind();reader=acq.Reader(port,clock=lambda:port.now,token=7)
    with pytest.raises(ValueError):reader.bind()
    assert not reader.valid and 'SAMPLE' not in port.commands


@pytest.mark.parametrize('bad',['duplicate','nan','invalid_raw'])
def test_first_encoder_anchor_rejects_bad_inventory_and_values(bad):
    p,first=fixture()
    if bad=='duplicate':first['samples'].append(first['samples'][0])
    if bad=='nan':first['samples'][0]['acquired_s']=float('nan')
    if bad=='invalid_raw':first['samples'][6]['position']=4096
    with pytest.raises(ValueError):
        acq.EncoderTracker(p,first,home_confirmed=True,bottom_confirmed=True)


def test_mutation_of_returned_record_does_not_rewrite_an_encoder_anchor():
    profile, first=fixture()
    tracker=acq.EncoderTracker(profile,first,home_confirmed=True,bottom_confirmed=True)
    later=deepcopy(first);later['sequence']+=1
    for r in later['samples']:r['acquired_s']+=.05
    first['samples'][6]['position']+=100
    assert tracker.update(later)['lift_height_m']==0


RANGES=ROOT/'config/home_assistant/arm_ranges.development.json'


def current_pose_fixture():
    p,first=fixture()
    ranges=json.loads(RANGES.read_text())
    first['sequence']=6
    for row in first['samples']:
        row.update(position=2048,speed=0,torque=0)
    # These readbacks differ from Home within the rebased operational ranges.
    first['samples'][0]['position']=2200
    first['samples'][1]['position']=1000
    first['samples'][4]['position']=3900
    calibration=dict(epoch=1,sequence=1,motor_writes=0,samples=[
        dict(bus=bus,id=i,offset_raw=ranges['calibration'][acq.s2.ARMS[bus]]['offsets_raw'][i-1],
             stored_id=i,mode=0,model_high=9,model_low=3,baud=0,acquired_s=99.75)
        for bus,i in acq.EXPECTED if i!=10])
    quiet=[]
    for i in range(4):
        row=deepcopy(first);row['sequence']=i+2
        for sample in row['samples']:sample['acquired_s']-=.20-.05*i
        quiet.append(row)
    return p,first,calibration,[*quiet,first]


def test_current_pose_anchors_unique_turns_without_claiming_home_or_motion_permission():
    p,first,calibration,quiet=current_pose_fixture()
    tracker=acq.EncoderTracker(p,first,home_confirmed=False,bottom_confirmed=True,
                             range_reference=RANGES,calibration=calibration,bottom_samples=quiet)
    state=tracker.state()
    assert state['raw']['left']['shoulder']==1000 and state['raw']['left']['wrist_roll']==3900
    assert state['raw']['left']['base']==2200 and state['lift_height_m']==0
    assert tracker.anchor_evidence['motion_authorized'] is False
    later=deepcopy(first);later['sequence']+=1
    for row in later['samples']:row['acquired_s']+=.05
    later['samples'][1]['position']=1001
    assert tracker.update(later)['raw']['left']['shoulder']==1001


@pytest.mark.parametrize('bad',['offset','mode','model','id','range_hash','epoch','sequence','stale',
                              'missing','torque','moving','outside','bottom','both','leaves_range'])
def test_current_pose_reference_rejects_incompatible_or_ambiguous_evidence(bad):
    p,first,calibration,quiet=current_pose_fixture()
    row=calibration['samples'][0]
    if bad=='offset':row['offset_raw']+=1
    if bad=='mode':row['mode']=1
    if bad=='model':row['model_low']=4
    if bad=='id':row['stored_id']=2
    if bad=='range_hash':p['source_files_sha256']['config/home_assistant/arm_ranges.development.json']='0'*64
    if bad=='epoch':calibration['epoch']=2
    if bad=='sequence':calibration['sequence']=3
    if bad=='stale':row['acquired_s']=100.01
    if bad=='missing':calibration['samples'].pop()
    if bad=='torque':first['samples'][0]['torque']=1
    if bad=='moving':first['samples'][0]['speed']=0x8006
    if bad=='outside':first['samples'][0]['position']=100
    with pytest.raises(ValueError):
        tracker=acq.EncoderTracker(p,first,home_confirmed=bad=='both',bottom_confirmed=bad!='bottom',
                                 range_reference=RANGES,calibration=calibration,bottom_samples=quiet)
        if bad=='leaves_range':
            later=deepcopy(first);later['sequence']+=1
            for row in later['samples']:row['acquired_s']+=.05
            later['samples'][1]['position']=140
            tracker.update(later)
    if bad=='leaves_range':
        with pytest.raises(ValueError):tracker.state()


def test_bottom_origin_uses_position_stability_despite_velocity_quantization():
    p,first,calibration,quiet=current_pose_fixture()
    for i,r in enumerate(quiet):
        r['samples'][6].update(position=[4095,0,4095,0,4095][i],speed=[50,32818,0,50,0][i])
    tracker=acq.EncoderTracker(p,first,home_confirmed=False,bottom_confirmed=True,
                             range_reference=RANGES,calibration=calibration,bottom_samples=quiet)
    assert tracker.origin==4096 and tracker.state()['lift_raw_unwrapped']==4095
    assert tracker.state()['lift_height_m']==pytest.approx(.084/4096)
    assert tracker.anchor_evidence['bottom']['span_counts']==1
    quiet[2]['samples'][6]['position']=5
    with pytest.raises(ValueError,match='lift moved'):
        acq.stationary_bottom(quiet)


@pytest.mark.parametrize('bad',['missing','epoch','sequence','gap','short','torque'])
def test_bottom_stability_proof_requires_continuous_stationary_evidence(bad):
    _,_,_,quiet=current_pose_fixture()
    if bad=='missing':quiet.pop()
    if bad=='epoch':quiet[2]['epoch']+=1
    if bad=='sequence':quiet[2]['sequence']+=1
    if bad=='gap':quiet[2]['samples'][6]['acquired_s']+=.3
    if bad=='short':
        for i,r in enumerate(quiet):r['samples'][6]['acquired_s']=100+i*.01
    if bad=='torque':quiet[1]['samples'][6]['torque']=1
    with pytest.raises(ValueError):acq.stationary_bottom(quiet)


def test_calibration_reader_requires_capability_and_keeps_scan_sequence():
    class CalibrationPort(Port):
        def write(self,data):
            n=super().write(data)
            if data.startswith((b'INFO',b'BIND')):self.rows[0]['arm_calibration']=True
            if data==b'CALIB\n':
                self.sequence+=1
                self.rows=[dict(epoch=self.epoch,sequence=self.sequence,bus=bus,id=i,
                    start_ms=int((self.now-99)*1000)+k,end_ms=int((self.now-99)*1000)+k+1,
                    offset_raw=595,mode=0,model_high=9,model_low=3,stored_id=i,baud=0,ok=True)
                    for k,(bus,i) in enumerate(acq.EXPECTED) if i!=10]
                self.rows.append(dict(sample_end=True,epoch=self.epoch,sequence=self.sequence,
                                      tick_ms=int((self.now-99)*1000)+13))
            return n
    port=CalibrationPort();reader=acq.Reader(port,clock=lambda:port.now,token=123);reader.bind()
    row=reader.sample(arm_calibration_only=True)
    assert row['sequence']==1 and len(row['samples'])==12 and row['motor_writes']==0
    assert reader.sample()['sequence']==2
    old=Port();reader=acq.Reader(old,clock=lambda:old.now,token=123);reader.bind()
    with pytest.raises(ValueError,match='calibration reads'):reader.sample(arm_calibration_only=True)
    assert 'CALIB' not in old.commands


@pytest.mark.parametrize('calibration,step_ms,accepted',[(True,5,True),(True,10,False),(False,5,False)])
def test_eeprom_read_budget_does_not_relax_pose_exposure_budget(calibration,step_ms,accepted):
    class SlowScan(Port):
        scanning=False
        def write(self,data):
            super().write(b'SAMPLE\n' if data==b'CALIB\n' else data)
            if data.startswith((b'INFO',b'BIND')):self.rows[0]['arm_calibration']=True
            self.scanning=data in (b'CALIB\n',b'SAMPLE\n')
            if self.scanning:
                rows=[r for r in self.rows[:-1] if data!=b'CALIB\n' or r['id']!=10]
                start=rows[0]['start_ms']
                for i,row in enumerate(rows):
                    row.update(start_ms=start+i*step_ms,end_ms=start+i*step_ms+1)
                    if data==b'CALIB\n':row.update(offset_raw=595,mode=0,model_high=9,model_low=3,stored_id=row['id'],baud=0)
                self.rows=[*rows,dict(self.rows[-1],tick_ms=start+len(rows)*step_ms)]
            return len(data)
        def readline(self,n):
            if self.scanning:self.now+=(step_ms-1)/1000
            return super().readline(n)
    port=SlowScan();reader=acq.Reader(port,clock=lambda:port.now,token=7);reader.bind()
    if accepted:
        report=reader.sample(arm_calibration_only=calibration)
        assert .050<report['acquisition_span_bound_s']<.100
    else:
        with pytest.raises(ValueError,match='acquisition span exceeds'):
            reader.sample(arm_calibration_only=calibration)
        assert not reader.valid


class GripPort(Port):
    def __init__(self, change=None):
        super().__init__()
        self.change = change or {}

    def write(self, data):
        result = super().write(data)
        if data == b'GRIP\n':
            self.sequence += 1
            tick = int((self.now - 99) * 1000)
            axis = dict(epoch=self.epoch, sequence=self.sequence, bus=0, id=6, start_ms=tick,
                        end_ms=tick+1, position=2083, speed=0, torque=0, ok=True,
                        load_raw=1026, current_raw=1, motor_status_raw=0)
            self.rows = [{**axis, **self.change}, dict(sample_end=True, epoch=self.epoch,
                                                       sequence=self.sequence, tick_ms=tick+1)]
        elif data != b'SAMPLE\n':
            self.rows[0]['left_grip_feedback'] = True
        return result


def test_grip_and_whole_robot_can_share_a_bound_session():
    port = GripPort()
    reader = acq.Reader(port, clock=lambda: port.now, token=3)
    reader.bind()
    grip = reader.sample(left_gripper_only=True)
    assert [s['load_raw'] for s in grip['samples']] == [1026]
    whole = reader.sample()
    assert whole['sequence'] == 2 and len(whole['samples']) == 13
    assert port.commands == ['INFO', 'BIND 3', 'GRIP', 'INFO', 'SAMPLE', 'INFO']


def test_old_firmware_never_receives_the_optional_request():
    port = Port()
    reader = acq.Reader(port, clock=lambda: port.now, token=3)
    reader.bind()
    with pytest.raises(ValueError, match='does not provide'):
        reader.sample(left_gripper_only=True)
    assert 'GRIP' not in port.commands


@pytest.mark.parametrize('change', [{'id': 5}, {'bus': 1}, {'load_raw': True},
                                    {'current_raw': -1}, {'motor_status_raw': 256}])
def test_bad_gripper_sample_invalidates_the_binding(change):
    port = GripPort(change)
    reader = acq.Reader(port, clock=lambda: port.now, token=3)
    reader.bind()
    with pytest.raises(ValueError):
        reader.sample(left_gripper_only=True)
    assert not reader.valid
