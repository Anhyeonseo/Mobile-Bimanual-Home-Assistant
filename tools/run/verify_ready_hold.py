#!/usr/bin/env python3
"""Bounded empty-arm hold; optional native-checked two-degree elbow return.

Requires explicit execution and an observed resting or supported pose. A plain
hold disables torque normally. A finite round trip ends with coordinated STOP,
because its completed stream otherwise prevents inventory readback. Never
clears a stop, changes EEPROM, or moves to a saved/Home pose.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time


def check_positions(rows, ranges, names, unwrap):
    for side in ('left', 'right'):
        selected=[r for r in rows if r['bus']==side and r['requested_id']<=6]
        if [r['requested_id'] for r in selected]!=list(range(1,7)):
            raise ValueError('six complete arm positions required')
        for name,row in zip(names,selected,strict=True):
            unwrap(ranges['arms'][side][name],row['feedback']['position_raw'])


def check_resting_samples(samples):
    """Prove only observed torque-off stability, not an external arm support."""
    if len(samples)!=3:raise ValueError('three resting inventories required')
    previous=None
    for rows in samples:
        selected=[r for r in rows if r['requested_id']<=6]
        keys=[(r['bus'],r['requested_id']) for r in selected]
        if keys!=[(side,i) for side in ('left','right') for i in range(1,7)]:
            raise ValueError('complete ordered resting arm inventory required')
        if any(not r['complete'] or r['command']['torque_enabled_raw']!=0 or
               r['feedback']['speed_raw']!=0 for r in selected):
            raise ValueError('arms must be stationary with torque OFF before hold')
        raw=[r['feedback']['position_raw'] for r in selected]
        if previous is not None and max(abs(a-b) for a,b in zip(raw,previous))>2:
            raise ValueError('torque-off resting pose moved before hold')
        if previous is None:previous=raw


def load_round_trip(path,checksum,range_hash):
    """Accept only the reviewed native-checked empty left-elbow 2 degree return."""
    data=path.read_bytes()
    if len(checksum)!=64 or hashlib.sha256(data).hexdigest()!=checksum:
        raise ValueError('round-trip checksum mismatch')
    r=json.loads(data)
    if (r['kind']!='empty_left_arm_round_trip' or r['schema_version']!=1 or
            r['status']!='NATIVE_GEOMETRY_PASSED' or r['hypothetical_lift'] is not False or
            r['range_sha256']!=range_hash or r['moving_joint_index']!=2):
        raise ValueError('native round-trip identity mismatch')
    points=r['points'];anchor=r['start_rad']
    reference=r['platform_reference_raw']
    if len(reference)!=4 or any(type(v) is not int or not 0<=v<4096 for v in reference):
        raise ValueError('captured platform reference required')
    if (len(anchor)!=12 or any(not math.isfinite(v) for v in anchor) or
            not math.isfinite(r['fixed_lift_m']) or not 0<=r['fixed_lift_m']<=.6 or
            not 3<=len(points)<=500 or r['checked_states']!=len(points)):
        raise ValueError('all round-trip samples must be checked')
    previous=0;peak=0.
    for p in points:
        q=p['positions_rad'];tick=p['offset_ms']
        if (len(q)!=12 or any(not math.isfinite(v) for v in q) or type(tick) is not int or
                not 5<=tick-previous<=100 or tick%5):raise ValueError('invalid round-trip sample')
        if any(abs(q[i]-anchor[i])>1e-9 for i in range(12) if i!=2):
            raise ValueError('only left elbow may move; jaw and right arm stay fixed')
        delta=q[2]-anchor[2]
        if not -1e-9<=delta<=math.radians(2)+1e-9:raise ValueError('round-trip exceeds 2 degree envelope')
        peak=max(peak,delta);previous=tick
    if (points[0]['positions_rad']!=anchor or points[-1]['positions_rad']!=anchor or
            not 1000<=previous<=10000 or peak<math.radians(1.9)):
        raise ValueError('bounded round trip must return exactly to start')
    return r


def check_round_trip_motion(samples,anchor):
    if not samples:raise ValueError('round-trip feedback absent')
    peak=max(s['positions_urad'][2]/1e6-anchor[2] for s in samples)
    returned=max(abs(v/1e6-a) for v,a in zip(samples[-1]['positions_urad'],anchor))
    if peak<math.radians(1):raise ValueError('measured elbow motion below 1 degree; following not verified')
    if returned>.004603:raise ValueError('round-trip return differs by more than three encoder counts')
    return dict(measured_peak_rad=peak,return_error_rad=returned)


def run_round_trip(s,t,adapter,route,report,save):
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    from home_robot_tasks.carry_live_inputs import ObservationNotFresh
    anchor=route['start_rad']
    if max(abs(v/1e6-a) for v,a in zip(adapter._prepared.positions_urad,anchor))>.003069:
        raise ValueError('round-trip anchor changed since native check')
    profile=s.camera_profile()
    guard=s.FixedPlatformGuard(route['fixed_lift_m'],profile['lift']['meters_per_count'])
    platform_before=[r['feedback']['position_raw'] for r in report['before'] if r['requested_id']>=7]
    if any(abs((a-b+2048)%4096-2048)>tol for a,b,tol in
           zip(platform_before,route['platform_reference_raw'],(4,4,4,8))):
        raise ValueError('platform differs from native checked capture')
    def observe_platform():
        sent=time.monotonic();packet=t.stationary_platform();received=time.monotonic()
        try:guard.update(packet,sent,received)
        except ObservationNotFresh:pass
        guard.require_fresh(time.monotonic())
        if guard.origin is None or any(abs((a-b+2048)%4096-2048)>tol for a,b,tol in
                                      zip(guard.origin,platform_before,(4,4,4,8))):
            raise ValueError('platform moved before round-trip admission')
    observe_platform()
    deadline=time.monotonic()+.5
    while time.monotonic()-guard.stamp>.120:
        if time.monotonic()>deadline:raise ValueError('no fresh platform handoff window')
        adapter.keepalive();time.sleep(.005);observe_platform()
    points=tuple(TimedJointPoint(p['offset_ms'],tuple(p['positions_rad'])) for p in route['points'])
    report['round_trip_samples']=[];report['status']='ROUND_TRIP_STARTING';save()
    print(json.dumps({'status':'LEFT_ELBOW_ROUND_TRIP','degrees':2,'seconds':points[-1].offset_ms/1000}),flush=True)
    report['trajectory_commands']=1
    adapter.start('ready-round-trip',points,finite=True)
    deadline=time.monotonic()+points[-1].offset_ms/1000+5
    while True:
        if time.monotonic()>deadline:raise ValueError('round-trip completion timeout')
        adapter.poll('ready-round-trip')
        observe_platform()
        packet=t.get_feedback_snapshot()
        if packet.status_code or packet.present_mask!=4095 or max(packet.sample_age_ms)>150:
            raise ValueError('round-trip feedback invalid or stale')
        row=asdict(packet);report['round_trip_samples'].append(row)
        for i,(v,a) in enumerate(zip(packet.positions_urad,anchor)):
            delta=v/1e6-a
            if (i==2 and not -.02<=delta<=math.radians(2)+.02) or (i!=2 and abs(delta)>.02):
                raise ValueError('measured motion left short-test envelope')
        if adapter.state.value=='ready':break
        time.sleep(.01)
    report['round_trip_result']=check_round_trip_motion(report['round_trip_samples'],anchor)
    report['round_trip_platform']=guard.last_query


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=8.)
    basis=parser.add_mutually_exclusive_group(required=True)
    basis.add_argument('--supported-empty-arms',action='store_true')
    basis.add_argument('--resting-empty-arms',action='store_true',
                       help='same-pose hold only; verify three torque-off stationary inventories first')
    parser.add_argument('--execute-hold',action='store_true')
    parser.add_argument('--round-trip',type=Path)
    parser.add_argument('--round-trip-sha256')
    parser.add_argument('--execute-round-trip',action='store_true')
    args=parser.parse_args()
    if not args.execute_hold or not 2<=args.seconds<=10:
        parser.error('empty-arm basis, explicit hold and 2..10 seconds required')
    if args.output.exists():parser.error('use a new output file')
    sys.path.insert(0,str(args.release/'tools/run'))
    import stationary_pick as s,serial
    from so101_arm_bridge.development_ranges import unwrap_in_range
    from so101_arm_bridge.limit_manifest import JOINTS
    from so101_arm_bridge.stream_protocol_v2 import StreamMessageTypeV2,parse_state_v2
    ranges,_,digest=s.read_development_ranges(args.release/'config/home_assistant/arm_ranges.development.json')
    if bool(args.round_trip)!=bool(args.round_trip_sha256):parser.error('round-trip file and reviewed checksum required together')
    route=load_round_trip(args.round_trip,args.round_trip_sha256,digest) if args.round_trip else None
    if bool(route)!=args.execute_round_trip:parser.error('round-trip requires explicit --execute-round-trip')
    if route and not args.resting_empty_arms:parser.error('round-trip requires verified original resting pose')
    report={'status':'PREFLIGHT','range_sha256':digest,'trajectory_commands':0,
            'home_or_saved_pose_commanded':False,'eeprom_writes':0,'samples':[],
            'hold_seconds':args.seconds,'supported_empty_arms_confirmed':args.supported_empty_arms,
            'hold_basis':'observed_torque_off_resting_pose' if args.resting_empty_arms else 'operator_supported_empty_arms'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    def save():args.output.write_text(json.dumps(report,indent=2)+'\n')
    with s.acquire_backend_lease('stm32',0),s.open_exclusive_serial(serial,s.DEVICE,921600,.02) as port:
        t=s.StreamValidationTransportV2(port,response_timeout_s=.4)
        enabling=False
        try:
            report['controller']=s.identity(t)
            report['before']=s.verify_inventory(t,ranges)
            check_positions(report['before'],ranges,JOINTS,unwrap_in_range)
            if args.resting_empty_arms:
                report['resting_samples']=[report['before']]
                for _ in range(2):
                    time.sleep(.5)
                    report['resting_samples'].append(s.verify_inventory(t,ranges))
                check_resting_samples(report['resting_samples'])
            if route:
                report['round_trip_sha256']=args.round_trip_sha256
                report['round_trip_capture_id']=route['capture_id']
            check=s.StationaryPickAdapter(t,args.release/'config/home_assistant/arm_ranges.development.json')
            adapter=s.StationaryPickAdapter(t,args.release/'config/home_assistant/arm_ranges.development.json',
                authorized_limits=replace(check._limits,hardware_authorized=True),motion_authorized=True)
            adapter.prepare()
            report['status']='CONFIGURING_CURRENT_POSITION_HOLD';save()
            enabling=True
            adapter.activate_stationary_hold()
            target=tuple(adapter._prepared.positions_urad)
            report['hold_anchor']=asdict(adapter._prepared)
            report['status']='HOLDING';save()
            print(json.dumps({'status':'HOLDING_CURRENT_POSE','seconds':args.seconds}),flush=True)
            start=time.monotonic()
            while time.monotonic()-start<args.seconds:
                adapter.keepalive(force=True)
                packet=t.get_feedback_snapshot()
                row=asdict(packet);row['elapsed_s']=time.monotonic()-start
                report['samples'].append(row)
                if packet.status_code or packet.present_mask!=4095 or max(packet.sample_age_ms)>150:
                    if time.monotonic()-start>.3:raise ValueError('hold feedback incomplete or stale')
                else:
                    error=max(abs(a-b) for a,b in zip(packet.positions_urad,target))
                    row['maximum_anchor_error_urad']=error
                    # Existing firmware terminal arm tolerance, including jaw
                    # conservatively for this empty-arm stationary check.
                    if error>46020:raise ValueError('stationary hold exceeds existing 46020 urad tolerance')
                time.sleep(.05)
            if route:run_round_trip(s,t,adapter,route,report,save)
            if route:
                # DISABLE removes torque but leaves a SUCCEEDED stream. The
                # firmware permits inventory only when CLOSED or stop-latched.
                # Use the existing coordinated termination, never weaken that gate.
                stop=s.ResidentStop(t);stop.request('round-trip-complete',True,time.monotonic())
                deadline=time.monotonic()+4
                while time.monotonic()<deadline and stop.poll('round-trip-complete',time.monotonic()).status!='STOPPED':
                    time.sleep(.05)
                report['coordinated_stop']=stop.diagnostics()
                if stop.status!='STOPPED':raise ValueError('round-trip stop not verified')
            else:
                adapter.keepalive(force=True)
                t.stationary_platform(2)
                adapter.keepalive(force=True)
                seq,_=t._send(StreamMessageTypeV2.DISABLE)
                state=parse_state_v2(t._receive(seq,StreamMessageTypeV2.STATE_FEEDBACK,timeout_s=3.).payload,expected_joint_count=12)
                report['disable_response']=asdict(state)
                if state.status_code or state.stop_latched:raise ValueError('supported torque disable failed')
            report['after']=s.verify_inventory(t,ranges)
            enabling=False
            report['runtime_after']=t.stationary_runtime()
            valid=[v['maximum_anchor_error_urad'] for v in report['samples'] if 'maximum_anchor_error_urad' in v]
            if not valid:raise ValueError('no valid hold observations')
            report.update(status=('RESTING_POSE_HOLD_VERIFIED_TORQUE_OFF' if args.resting_empty_arms else
                                  'SUPPORTED_HOLD_VERIFIED_TORQUE_OFF'),maximum_anchor_error_urad=max(valid),
                          physical_free_standing_hold_verified=False)
            if route:report['status']='EMPTY_ELBOW_ROUND_TRIP_OBSERVED_STOPPED_TORQUE_OFF'
        except BaseException as exc:
            report.update(status='HOLD_NOT_COMPLETED',error=str(exc))
            if enabling:
                try:report['stop_response']=asdict(t.safe_stop())
                except Exception as stop_error:report['stop_error']=str(stop_error)
            try:report['runtime_after']=t.stationary_runtime()
            except Exception as query_error:report['runtime_error']=str(query_error)
            raise
        finally:save()
    print(json.dumps({'path':str(args.output),'status':report['status'],
        'maximum_anchor_error_urad':report.get('maximum_anchor_error_urad'),
        'torque_after':'OFF all 12','trajectory_commands':report['trajectory_commands'],
        'round_trip_result':report.get('round_trip_result')}),flush=True)


if __name__=='__main__':main()
