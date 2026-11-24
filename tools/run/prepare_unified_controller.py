#!/usr/bin/env python3
"""Inspect S4 integration; explicit --prepare changes modes with torque OFF.

Default inspection and --prepare never enable torque or move.
Explicit --check-zero-power enables only mobile torque at zero speed, then
checks supported release, without ARM or nonzero motion.
Explicit --check-hold-stop holds the measured arm pose, references the physical
lift bottom and requests integrated STOP with torque retained. No automatic reset.
Explicit --release-boot-supported releases retained torque only on a fresh,
unconfigured boot with the empty mechanism physically supported. Never resets.
"""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/so101_arm_bridge'),
                str(ROOT/'ros2_ws/src/home_robot_tasks')]
from home_robot_tasks.platform_preparation import (
    measured_hold_source, supported_lift_is_holding, move_observed_lift,
    reference_supported_bottom,
)
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from so101_arm_bridge.device_inventory import collect_inventory
from so101_arm_bridge.unified_commissioning import CommissionClient,FIRMWARE,pick_board_scale
from so101_arm_bridge.mobile_inspection import inspect_mobile
from so101_arm_bridge.lift_wire import LiftCommand,LiftStatus


def supported_base_pulse(mobile,observe,result):
    """First forward pulse only: raw +/-50, <=200 ms, then zero and STOP.

    Not a distance calibration or autonomous navigation. The caller owns the
    same-pose arms/lift and must request whole-system STOP on every exit.
    """
    result['base_pulse']={'direction':'forward','wheel_raw':[-50,50,0],
        'maximum_seconds':.2,'distance_calibrated':False,'samples':[]}
    rows=result['base_pulse']['samples'];mobile.arm(1)
    started=last=time.monotonic()
    try:
        while time.monotonic()-started<.2:
            now=time.monotonic()
            if now-last>.1:raise RuntimeError('base pulse host observation gap')
            status=mobile.velocity((-50,50,0,0),lifetime_ms=150)
            result['nonzero_mobile_velocity_commands']+=1
            rows.append(asdict(status))
            if not status.ready_for_motion(1):raise RuntimeError('base pulse feedback unhealthy')
            if not supported_lift_is_holding(observe()):raise RuntimeError('lift hold lost during base pulse')
            last=now;time.sleep(.01)
    finally:
        # Whole-system STOP remains the caller's finally path if this fails.
        mobile.velocity((0,0,0,0),lifetime_ms=150)
        result['base_pulse']['zero_requested']=True
    deadline=time.monotonic()+2;quiet_since=None;quiet_count=0
    while time.monotonic()<deadline:
        status=mobile.velocity((0,0,0,0),lifetime_ms=150);rows.append(asdict(status))
        if not status.ready_for_motion(1):raise RuntimeError('base settling feedback unhealthy')
        if not supported_lift_is_holding(observe()):raise RuntimeError('lift hold lost during settling')
        now=time.monotonic()
        if all(abs(v)<=10 for v in status.velocity_raw):
            if quiet_since is None:quiet_since=now
            quiet_count+=1
            if quiet_count>=3 and now-quiet_since>=.4:
                result['base_pulse']['zero_feedback_verified']=True;return
        else:quiet_since=None;quiet_count=0
        time.sleep(.01)
    raise TimeoutError('base pulse stop feedback unconfirmed')


def check_hold_stop(transport,*,baseline,supported_empty,bottom_confirmed,report=None,
                    base_pulse=False,base_clearance=False,lift_target_mm=None):
    """Explicit supervised constant-pose hold, bottom reference, then SAFE_STOP.

    STOP retains torque. This is not a torque-release or moving-stop trial.
    Empty grippers/support/bottom are operator-confirmed physical invariants;
    measured joints must remain at the initial pose throughout the short trial.
    """
    from dataclasses import replace
    from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
    from so101_arm_bridge.mobile_client import MobileClient
    if lift_target_mm is not None and (type(lift_target_mm) is not int or not 1<=lift_target_mm<=10 or base_pulse):
        raise ValueError('explicit 1..10mm rise, separate from base motion, required')
    from so101_arm_bridge.lift_wire import LiftClient
    from so101_arm_bridge.robot_evidence import RobotEvidenceClient
    from so101_arm_bridge.system_stop import SystemStopClient
    from so101_arm_bridge.stream_transport_v2 import MobileV2Exchange
    if supported_empty is not True or bottom_confirmed is not True or baseline is None:
        raise ValueError('supported empty robot, actual bottom confirmation and baseline required')
    if base_pulse and base_clearance is not True:
        raise ValueError('base motion requires current clearance/cable/support confirmation')
    result=inspect(transport,baseline=baseline,report=report)
    if result['commission_final']['flags']!=17:raise ValueError('prepared torque-off state required')
    ranges=ROOT/'config/home_assistant/arm_ranges.development.json'
    preview=UnifiedArmAdapter(transport,ranges)
    adapter=UnifiedArmAdapter(transport,ranges,authorized_limits=replace(preview.motion_limits,hardware_authorized=True),motion_authorized=True)
    prepared=adapter.prepare()
    result.update(status='HOLD_STOP_IN_PROGRESS',initial_arm_pose=asdict(prepared),
        nonzero_mobile_velocity_commands=0,lift_move_commands=0,torque_release_requested=False,
        arm_motion_scope='constant measured pose',observations=[],operator_bottom_confirmed=True,
        operator_empty_supported=True)
    commission=CommissionClient(transport)
    mobile=MobileClient(MobileV2Exchange(transport),lambda:int(time.monotonic()*1000));mobile.synchronize()
    boot=result['commission_final']['boot_id']
    if mobile.boot_id!=boot:raise ValueError('boot changed before hold test')
    lift=LiftClient(mobile);evidence=RobotEvidenceClient(transport,mobile,time.monotonic,maximum_age_ms=250)
    stop=SystemStopClient(transport,boot,time.monotonic)
    armed_attempted=False;power_attempted=False;stop_attempted=False
    def observe(*,stopped=False):
        heartbeat=transport.heartbeat()
        if not stopped and (heartbeat.stop_latched or heartbeat.status_code):
            raise RuntimeError('runtime stopped before requested test STOP')
        sent=time.monotonic();snapshot=transport.get_feedback_snapshot();received=time.monotonic()
        measured_hold_source(snapshot,prepared.positions_urad,adapter.motion_limits,sent,received)
        # Empty payload invariant + measured unchanged grippers, for this
        # test only. Firmware independently proves lift holding from drive
        # readback; never substitute an asserted lift_hold=True.
        evidence.publish_mcu_snapshot(snapshot,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
        ls=lift.status()
        result['observations'].append(dict(host_s=received,arm=asdict(snapshot),lift=asdict(ls)))
        if ls.fault:raise RuntimeError('lift feedback fault')
        return ls
    try:
        power_attempted=True
        result['power_activation']=asdict(commission.activate_zero(supported=True,empty_grippers=True))
        armed_attempted=True
        adapter.start_current_pose_hold('supervised-current-pose')
        prepared=adapter.prepared_state
        result['commanded_hold_anchor']=asdict(prepared)
        deadline=time.monotonic()+3
        while adapter.state.value!='ready':
            if time.monotonic()>=deadline:raise TimeoutError('finite current-pose hold did not finish')
            adapter.poll('supervised-current-pose');time.sleep(.02)
        result['arm_hold_completed']=True
        if lift_target_mm is None:
            deadline=time.monotonic()+1
            while True:
                ls=observe()
                if ls.flags&4:break
                if time.monotonic()>=deadline:raise TimeoutError('lift reference interlocks unavailable')
                time.sleep(.02)
            result['lift_reference']=asdict(lift.reference_bottom(1,operator_confirmed=True))
            deadline=time.monotonic()+1
            while True:
                ls=observe()
                if ls.holding():break
                if time.monotonic()>=deadline:raise TimeoutError('independent lift hold not established')
                time.sleep(.02)
        else:
            reference_supported_bottom(observe,mobile,lift,clock=time.monotonic,sleep=time.sleep)
            result['lift_reference']=asdict(lift.status())
        result['lift_hold_verified']=True
        until=time.monotonic()+1
        while time.monotonic()<until:
            if not supported_lift_is_holding(observe()):raise RuntimeError('lift hold evidence lost')
            time.sleep(.02)
        if lift_target_mm is not None:
            result['lift_move_commands']=1
            result['lift_rise']=move_observed_lift(lift,observe,stop,lift_target_mm*1000)
        if base_pulse:supported_base_pulse(mobile,observe,result)
        stop_attempted=True;result['stop_ack']=asdict(stop.request())
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            if not supported_lift_is_holding(observe(stopped=True)):
                raise RuntimeError('lift hold unconfirmed during STOP')
            proof,_=stop.status();result['stop_proof']=asdict(proof)
            if proof.confirmed:
                result['integrated_stop_confirmed']=True;break
            time.sleep(.02)
        if not result.get('integrated_stop_confirmed'):raise RuntimeError('integrated STOP unconfirmed')
        result['status']=('SUPPORTED_LIFT_RISE_STOP_CONFIRMED_TORQUE_RETAINED' if lift_target_mm is not None else
                          'SUPPORTED_BASE_PULSE_STOP_CONFIRMED_TORQUE_RETAINED' if base_pulse else
                          'SUPPORTED_HOLD_REFERENCE_STOP_CONFIRMED_TORQUE_RETAINED')
    except Exception as error:
        result.update(status='FAILED',error=str(error))
    finally:
        if armed_attempted:
            if not stop_attempted:
                try:result['cleanup_stop_ack']=asdict(stop.request())
                except Exception as error:result['cleanup_stop_error']=str(error)
            try:
                proof,_=stop.status();result['final_stop_proof']=asdict(proof)
                if not proof.confirmed and result['status'].startswith('SUPPORTED_'):
                    result.update(status='FAILED',error='final integrated STOP evidence no longer confirmed')
            except Exception as error:result['final_stop_error']=str(error)
            if 'final_stop_error' in result and result['status'].startswith('SUPPORTED_'):
                result.update(status='FAILED',error='final integrated STOP query failed')
            try:result['runtime_diagnostics']=transport.stationary_runtime()
            except Exception as error:result['runtime_diagnostics_error']=str(error)
            result['final_power_state']='torque retained or unconfirmed; keep supports; no generic disable'
        elif power_attempted:
            try:result['power_release']=asdict(commission.release_supported(supported=True,empty_grippers=True))
            except Exception as error:result['release_error']=str(error)
    return result

def release_boot_supported(transport,*,baseline,supported_empty,report=None,pick_board=None):
    if supported_empty is not True or baseline is None:
        raise ValueError('current support, empty grippers and calibration baseline required')
    result={} if report is None else report
    hello=transport.enter_binary_mode()
    result.update(status='BOOT_RELEASE_PREFLIGHT',controller=asdict(hello),
        reset_commands=0,nonzero_mobile_velocity_commands=0,automatic_retry=False)
    expected=FIRMWARE if pick_board is None else pick_board[0]
    if hello.firmware_version!=expected or hello.protocol_version!=2 or hello.stop_latched or hello.joint_count!=12:
        raise ValueError('exact fresh unified firmware required')
    for key in ('left_calibration_hash','right_calibration_hash'):
        if getattr(hello,key)!=baseline['hello'][key]:raise ValueError('arm calibration hash changed')
    client=CommissionClient(transport);initial=client.query()
    if initial.flags or initial.reason:raise ValueError('unconfigured clean boot required')
    if pick_board is not None:
        pick_board_scale(*pick_board)
        if initial.profile_crc32!=pick_board[1]:raise ValueError('pick controller profile changed before release')
    before=collect_inventory(transport);result['inventory_before']=before
    compare_calibration(baseline['inventory'],before,allow_mobile_mode_change=True,configuration_only=True)
    result['release_attempted']=True
    result['release']=asdict(client.release_boot_supported(supported=True,empty_grippers=True))
    after=collect_inventory(transport);result['inventory_after']=after
    compare_calibration(before,after,allow_pending_wheel_reports=True,allow_pending_lift_reports=True)
    window=ZeroPowerObservation(after,powered=False);window.observe(after)
    result['unpowered_observations']=[after];started=time.monotonic()
    while not window.proven:
        if time.monotonic()-started>2:raise TimeoutError('released torque-off observation unavailable')
        time.sleep(.05);sample=collect_inventory(transport)
        result['unpowered_observations'].append(sample);window.observe(sample)
    result['unpowered_stationarity']=window.report()
    result.update(status='SUPPORTED_BOOT_RELEASED',calibration_preserved=True,
                  final_all_torque_off=True,physical_test_ready=False)
    return result

def compare_calibration(before,after,*,allow_mobile_mode_change=False,powered_mobile=False,
                        configuration_only=False,
                        allow_pending_wheel_reports=False,allow_pending_lift_reports=False):
    def indexed(inventory):
        rows=inventory['devices']
        result={(d['bus'],d['requested_id']):d for d in rows}
        expected={('left',i) for i in range(1,11)}|{('right',i) for i in range(1,7)}
        if not inventory['all_snapshots_complete'] or len(rows)!=16 or set(result)!=expected:
            raise ValueError('complete unique sixteen-device inventory required')
        return result
    b,a=indexed(before),indexed(after)
    for key,d in a.items():
        old=b[key]
        if (old['identity']!=d['identity'] or old['limits']!=d['limits'] or
            old['offset_and_mode']['offset_raw']!=d['offset_and_mode']['offset_raw']):
            raise ValueError(f'calibration/identity changed: {key}')
        mode=1 if allow_mobile_mode_change and key[0]=='left' and key[1]>=7 else old['offset_and_mode']['operating_mode']
        if d['offset_and_mode']['operating_mode']!=mode:
            raise ValueError(f'unexpected mode change: {key}')
        if configuration_only:
            # Explicit boot-release preflight only; MCU independently checks
            # stable, zero-speed state before any supported torque removal.
            continue
        expected_torque=1 if powered_mobile and key[0]=='left' and key[1]>=7 else 0
        wheel_pending=key[0]=='left' and ((allow_pending_wheel_reports and 7<=key[1]<=9) or (allow_pending_lift_reports and key[1]==10))
        if wheel_pending and (d['offset_and_mode']['operating_mode']!=1 or d['command']['goal_speed_raw']!=0):
            raise ValueError('pending wheel reports require speed mode and zero goal')
        speed=d['feedback']['speed_raw'];moving=d['feedback']['moving_raw']
        stationary_report=(0<=speed<=65535 and (speed&0x7fff)<=50 and moving in (0,1)) if wheel_pending else speed==0 and moving==0
        if (d['command']['torque_enabled_raw']!=expected_torque or not stationary_report or
            d['feedback']['lock_raw']!=1):
            actual=d['command']['torque_enabled_raw'];lock=d['feedback']['lock_raw']
            raise ValueError(f'expected locked, stopped, torque {expected_torque}: {key}; '
                             f'actual torque={actual}, lock={lock}, speed={speed}, moving={moving}')
        if powered_mobile and expected_torque and d['command']['goal_speed_raw']!=0:
            raise ValueError(f'nonzero mobile goal: {key}')

class ZeroPowerObservation:
    """Supported maintenance test only, not a runtime STOP or load-hold proof.

    Reuse the existing teleop 400 ms observation window and <=50 wheel speed
    report bound, but keep this test's tighter two-count position span. A small
    speed/moving report removes proof until three fresh, strictly zero reports
    span 400 ms. It is retained verbatim, never relabeled as sensor noise.
    Arms stay strict. Torque-OFF lift may use its two-count position window;
    powered tests still require quiet lift reports. Power readback is mandatory.
    """
    def __init__(self,before,*,powered=True):
        self.powered=powered
        self.before=before;self.axes={};self.events=[];self.samples=0;self.failed=False
        for d in before['devices']:
            if d['bus']=='left' and 7<=d['requested_id']<=10:
                pos=d['feedback']['position_raw']
                if type(pos) is not int or not 0<=pos<4096:raise ValueError('invalid initial mobile position')
                self.axes[d['requested_id']]=dict(anchor=pos,low=0,high=0,stamp=None,quiet_since=None,quiet_samples=0)
        if set(self.axes)!={7,8,9,10}:raise ValueError('four unique mobile axes required')

    def observe(self,sample):
        if self.failed:raise ValueError('failed observation cannot be reused')
        try:return self._observe(sample)
        except Exception:
            self.failed=True
            raise

    def _observe(self,sample):
        compare_calibration(self.before,sample,powered_mobile=self.powered,allow_pending_wheel_reports=True,allow_pending_lift_reports=not self.powered)
        for d in sample['devices']:
            if d['bus']!='left' or d['requested_id'] not in self.axes:continue
            mid=d['requested_id'];a=self.axes[mid];f=d['feedback'];stamp=d['sample_started_tick_ms']
            if type(stamp) is not int or not 0<=stamp<2**32:raise ValueError('invalid motor sample timestamp')
            if a['stamp'] is not None and not 0<(stamp-a['stamp'])%2**32<=250:
                raise ValueError('duplicate, reordered or stale powered observation')
            pos=f['position_raw']
            if type(pos) is not int or not 0<=pos<4096:raise ValueError('invalid mobile position')
            delta=(pos-a['anchor']+2048)%4096-2048
            a['low']=min(a['low'],delta);a['high']=max(a['high'],delta)
            tolerance=8 if not self.powered and mid<=9 else 2
            if a['high']-a['low']>tolerance:raise ValueError(f'position span exceeds {tolerance} counts: {mid}')
            a['stamp']=stamp
            nonzero=f['speed_raw'] or f['moving_raw']
            if nonzero:
                self.events.append(dict(motor_id=mid,sample_started_tick_ms=stamp,
                    position_raw=pos,speed_raw=f['speed_raw'],moving_raw=f['moving_raw']))
            if nonzero and self.powered:
                a['quiet_since']=None;a['quiet_samples']=0
            else:
                if a['quiet_since'] is None:a['quiet_since']=stamp
                a['quiet_samples']+=1
        self.samples+=1
        return self.proven

    @property
    def proven(self):
        return not self.failed and all(a['quiet_samples']>=3 and a['quiet_since'] is not None and
                   (a['stamp']-a['quiet_since'])%2**32>=400 for a in self.axes.values())

    def report(self):
        return dict(proven=self.proven,samples=self.samples,events=self.events,powered=self.powered,
                    axes=self.axes,scope='supported maintenance zero-speed observation only')

def check_zero_power(transport,*,baseline,supported_empty,seconds=3,report=None):
    """Supervised mobile torque test. Arms stay OFF; no nonzero speed target.

    Explicit confirmed supports remain in place for the complete test, including
    release. A server-side fresh stopped-state check guards the release itself.
    This maintenance test never configures the periodic motion runtime.
    """
    if baseline is None or supported_empty is not True or type(seconds) not in (int,float) or not 1<=seconds<=5:
        raise ValueError('supported empty robot and bounded 1..5 second test required')
    result=inspect(transport,baseline=baseline,report=report)
    if result['commission_final']['flags']!=17:
        raise ValueError('prepared, unused power test state required')
    client=CommissionClient(transport);before=result['inventory_before'];attempted=False
    observation=ZeroPowerObservation(before)
    result.update(status='ZERO_POWER_CHECK_IN_PROGRESS',samples=[],torque_commands=1,
        physical_test_ready=False,nonzero_velocity_commands=0,arms_enabled=False)
    started=time.monotonic()
    try:
        attempted=True
        result['power_activation']=asdict(client.activate_zero(supported=True,empty_grippers=True))
        started=time.monotonic()
        while time.monotonic()-started<seconds or not observation.proven:
            if time.monotonic()-started>=seconds+1:
                raise TimeoutError('powered quiet window not established within bounded observation')
            sample=collect_inventory(transport)
            result['samples'].append(dict(elapsed_s=time.monotonic()-started,inventory=sample))
            observation.observe(sample)
            if time.monotonic()-started>seconds+1:
                raise TimeoutError('zero-power inspection exceeded bounded window')
            time.sleep(.05)
        result['power_check_passed']=True
    except Exception as error:
        result.update(power_check_passed=False,error=str(error))
    finally:
        result['powered_observation']=observation.report()
        if attempted:
            try:
                result['power_release']=asdict(client.release_supported(supported=True,empty_grippers=True))
                after=collect_inventory(transport);result['inventory_after']=after
                compare_calibration(before,after)
                result['released_confirmed']=True
            except Exception as error:
                result.update(released_confirmed=False,release_error=str(error))
    result['status']='ZERO_POWER_CHECK_PASSED_RELEASED' if (
        result.get('power_check_passed') and result.get('released_confirmed')) else 'FAILED'
    return result

def inspect(transport,*,prepare=False,supported=False,empty=False,baseline=None,report=None,
            pick_board=None,release_retained=False):
    if type(release_retained) is not bool or (release_retained and not (prepare and supported is True and empty is True)):
        raise ValueError('supported empty preparation required for retained torque release')
    if prepare and (supported is not True or empty is not True):
        raise ValueError('current support and empty-gripper confirmation required')
    if prepare and baseline is None:raise ValueError('original calibration baseline required')
    result={} if report is None else report
    hello=transport.enter_binary_mode()
    result.update(controller=asdict(hello),checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  motion_commands=0,torque_commands=0,reset_requested=False,physical_test_ready=False)
    expected = FIRMWARE if pick_board is None else pick_board[0]
    if hello.firmware_version!=expected or hello.protocol_version!=2 or hello.joint_count!=12 or hello.stop_latched:
        raise ValueError('exact unified controller identity and clear stop required')
    client=CommissionClient(transport)
    result['commission_before']=asdict(client.query())
    if pick_board is not None:
        pick_board_scale(*pick_board)
        if result['commission_before']['profile_crc32'] != pick_board[1]:
            raise ValueError('pick controller profile changed before preparation')
    result['inventory_before']=collect_inventory(transport)
    if not result['inventory_before']['all_snapshots_complete']:raise ValueError('incomplete device inventory')
    if baseline is not None:
        for key in ('left_calibration_hash','right_calibration_hash'):
            if result['controller'][key]!=baseline['hello'][key]:raise ValueError('arm model calibration hash changed')
        retained=any(d['command']['torque_enabled_raw']==1 for d in result['inventory_before']['devices'])
        if release_retained and retained:
            # A request-specific operator confirmation authorizes this one
            # release. Firmware verifies rest before writes; no reset/retry.
            result['startup_release']={}
            release_boot_supported(transport,baseline=baseline,supported_empty=True,
                                   report=result['startup_release'],pick_board=pick_board)
            result['torque_commands']=1
            result['unpowered_observations']=result['startup_release']['unpowered_observations']
            result['unpowered_stationarity']=result['startup_release']['unpowered_stationarity']
        else:
            compare_calibration(baseline['inventory'],result['inventory_before'],allow_pending_wheel_reports=True,allow_pending_lift_reports=True)
            observation=ZeroPowerObservation(result['inventory_before'],powered=False)
            result['unpowered_observations']=[result['inventory_before']]
            observation.observe(result['inventory_before']);started=time.monotonic()
            while not observation.proven:
                if time.monotonic()-started>2:raise TimeoutError('torque-OFF stationarity window unavailable')
                time.sleep(.05);sample=collect_inventory(transport)
                result['unpowered_observations'].append(sample);observation.observe(sample)
            result['unpowered_stationarity']=observation.report()
        result['baseline_calibration_preserved']=True
    if prepare:
        result['commission_after']=asdict(client.prepare(supported=supported,empty_grippers=empty))
        result['inventory_after']=collect_inventory(transport)
        compare_calibration(result['inventory_before'],result['inventory_after'],allow_mobile_mode_change=True,
                            allow_pending_wheel_reports=True,allow_pending_lift_reports=True)
        result['calibration_preserved']=True
    status=client.query()
    result['commission_final']=asdict(status)
    if pick_board is not None and status.profile_crc32 != pick_board[1]:
        raise ValueError('pick controller profile changed during preparation')
    if status.flags&1:
        result['mobile']=inspect_mobile(transport)
        result['lift']=asdict(LiftStatus.decode(transport.exchange_mobile(LiftCommand(4,1,1,0,status.boot_id).encode())))
        if not result['mobile']['mobile_available'] or result['lift']['flags']&3:
            raise ValueError('expected connected, unreferenced idle endpoints')
    result['status']='CONNECTED_NOT_ARMED' if status.flags&1 else (
        'PREPARATION_REJECTED' if status.flags&16 else 'INSTALLED_NOT_PREPARED')
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prepare',action='store_true');p.add_argument('--supported-empty',action='store_true')
    p.add_argument('--baseline',type=Path,help='original read-only S3 handoff report')
    p.add_argument('--check-zero-power',action='store_true',help='physical supervised 3-second zero-speed mobile torque test; releases after fresh stopped readback')
    p.add_argument('--check-hold-stop',action='store_true',help='supervised same-pose arm hold, lift bottom reference and integrated STOP; retains torque')
    p.add_argument('--move-lift-mm',type=int,choices=range(1,11),help='with hold-stop: explicitly authorized bottom-origin 1..10mm rise, then whole STOP')
    p.add_argument('--bottom-confirmed',action='store_true',help='operator confirmed actual lift bottom; no downward homing')
    p.add_argument('--release-boot-supported',action='store_true',help='explicit supported empty all-torque release on a fresh unconfigured boot; no reset or automatic prepare')
    p.add_argument('--check-base-pulse',action='store_true',help='with hold-stop: first <=200ms forward raw50 pulse, settle, whole STOP')
    p.add_argument('--base-clearance-confirmed',action='store_true',help='operator confirmed moving supports, nearby clearance and cable slack')
    a=p.parse_args()
    if a.output.exists():p.error('new output required')
    if a.move_lift_mm is not None and (not a.check_hold_stop or a.check_base_pulse):p.error('lift rise requires hold-stop without base pulse')
    if a.prepare and not a.supported_empty:p.error('--prepare requires current --supported-empty confirmation')
    if a.prepare and not a.baseline:p.error('--prepare requires --baseline')
    if a.check_zero_power and (a.prepare or not a.baseline or not a.supported_empty):
        p.error('--check-zero-power requires --baseline --supported-empty and cannot accompany --prepare')
    if a.check_hold_stop and (a.prepare or a.check_zero_power or not a.baseline or not a.supported_empty or not a.bottom_confirmed):
        p.error('--check-hold-stop requires --baseline --supported-empty --bottom-confirmed and no other action')
    if a.bottom_confirmed and not a.check_hold_stop:p.error('--bottom-confirmed is only for --check-hold-stop')
    if a.release_boot_supported and (a.prepare or a.check_zero_power or a.check_hold_stop or not a.baseline or not a.supported_empty):
        p.error('--release-boot-supported requires --baseline --supported-empty and no other action')
    if a.check_base_pulse and (not a.check_hold_stop or not a.base_clearance_confirmed):
        p.error('--check-base-pulse requires --check-hold-stop --base-clearance-confirmed')
    if a.base_clearance_confirmed and not a.check_base_pulse:p.error('clearance flag requires base-pulse test')
    import serial
    result={}
    try:
        with acquire_backend_lease('stm32',87),open_exclusive_serial(serial,a.port,921600,.02) as port:
            transport=StreamValidationTransportV2(port,response_timeout_s=3)
            baseline=json.loads(a.baseline.read_text()) if a.baseline else None
            if a.release_boot_supported:
                release_boot_supported(transport,baseline=baseline,supported_empty=a.supported_empty,report=result)
            elif a.check_hold_stop:
                check_hold_stop(transport,baseline=baseline,supported_empty=a.supported_empty,bottom_confirmed=a.bottom_confirmed,report=result,
                                base_pulse=a.check_base_pulse,base_clearance=a.base_clearance_confirmed,lift_target_mm=a.move_lift_mm)
            elif a.check_zero_power:
                check_zero_power(transport,baseline=baseline,supported_empty=a.supported_empty,report=result)
            else:
                inspect(transport,prepare=a.prepare,supported=a.supported_empty,empty=a.supported_empty,
                        baseline=baseline,report=result)
    except Exception as e:result.update(status='FAILED',error=str(e),automatic_retry=False)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['status'] in ('CONNECTED_NOT_ARMED','INSTALLED_NOT_PREPARED','ZERO_POWER_CHECK_PASSED_RELEASED','SUPPORTED_BOOT_RELEASED','SUPPORTED_HOLD_REFERENCE_STOP_CONFIRMED_TORQUE_RETAINED','SUPPORTED_BASE_PULSE_STOP_CONFIRMED_TORQUE_RETAINED','SUPPORTED_LIFT_RISE_STOP_CONFIRMED_TORQUE_RETAINED') else 2
if __name__=='__main__':raise SystemExit(main())
