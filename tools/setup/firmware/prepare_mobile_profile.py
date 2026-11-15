#!/usr/bin/env python3
"""Validate/hash explicit board settings. Never writes to a device."""
import argparse
import json
from pathlib import Path
import sys
import hashlib
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.commissioning import export_profile

def from_handoff(path):
    """Use actual inventory and existing calibration; keep policy unqualified."""
    sys.path.insert(0,str(ROOT/'ros2_ws/src/so101_arm_bridge'))
    from so101_arm_bridge.encoder_units import LIFT_METERS_PER_TURN
    from so101_arm_bridge.development_ranges import read_development_ranges,calibration_hash
    path=Path(path).resolve();report=json.loads(path.read_text())
    hello=report['hello'];inventory=report['inventory']
    if hello['firmware_version']!=0x24911 or hello['stop_latched'] or not inventory['all_snapshots_complete']:
        raise ValueError('complete original S3 torque-off handoff required')
    _,_,digest=read_development_ranges(ROOT/'config/home_assistant/arm_ranges.development.json')
    if any(hello[k]!=calibration_hash(digest) for k in ('left_calibration_hash','right_calibration_hash')):
        raise ValueError('original controller and retained arm range model mismatch')
    devices={(d['bus'],d['requested_id']):d for d in inventory['devices']}
    if len(devices)!=16 or any(not d['complete'] or d['command']['torque_enabled_raw']!=0 for d in devices.values()):
        raise ValueError('all sixteen devices must be identified with torque OFF')
    policy=json.loads((ROOT/'config/home_assistant/s4_unified.policy.json').read_text())
    geometry=ROOT/'config/home_assistant/s2_coordinates.development.json'
    lift=json.loads(geometry.read_text())['lift']
    scale=-lift['meters_per_count']*4096
    if abs(scale-LIFT_METERS_PER_TURN)>1e-10:raise ValueError('lift model/runtime scale mismatch')
    mobiles=[devices['left',i] for i in (7,8,9,10)]
    baud={d['identity']['baud_code'] for d in mobiles}
    if len(baud)!=1:raise ValueError('mixed mobile motor baud')
    raw=policy['maximum_current_raw'];age=policy['feedback_max_age_ms']
    direction=[1,1,1,-1]
    board=dict(
        devices=dict(model=[d['identity']['model_raw'] for d in mobiles],baud_code=baud.pop(),timeout_ms=3000),
        mobile=dict(max_velocity_raw=[policy['maximum_wheel_speed_raw']]*3+[policy['maximum_lift_speed_raw']],
                    stopped_velocity_raw=policy['stopped_speed_raw'],command_timeout_ms=400,feedback_timeout_ms=age,
                    lift_min_um=0,lift_max_um=policy['maximum_lift_height_um']),
        feedback=dict(axes=[dict(velocity_direction=d,minimum_voltage_raw=90,maximum_voltage_raw=150,
                      maximum_temperature=70,maximum_current_raw=raw) for d in direction],
                      sample_timeout_ms=age,mode_timeout_ms=1000,response_timeout_ms=4,maximum_sample_skew_ms=100),
        output=dict(schedule=dict(arm_period_us=5000,arm_reserved_us=1000,guard_us=200,
                    wheels_period_us=20000,lift_period_us=20000,feedback_period_us=10000,maximum_poll_gap_us=4000),
                    wheels_budget_us=600,lift_budget_us=500,stop_budget_us=700,job_lifetime_us=10000,
                    velocity_direction=direction),
        lift=dict(um_per_turn=round(scale*1e6),maximum_height_um=policy['maximum_lift_height_um'],
                  tolerance_um=policy['lift_tolerance_um'],maximum_speed_raw=policy['maximum_lift_speed_raw'],
                  homing_speed_raw=20,stopped_speed_raw=policy['stopped_speed_raw'],slowdown_distance_um=10000,
                  home_current_ma=1040,maximum_current_ma=raw*policy['current_microamps_per_raw']//1000,
                  feedback_timeout_ms=age,homing_timeout_ms=1000,contact_dwell_ms=100,
                  maximum_encoder_step_raw=256,encoder_up_direction=-1),
        stop=dict(job_lifetime_us=10000,zero_budget_us=700,hold_budget_us=700,
                  timeout_ms=policy['stop_timeout_ms'],feedback_max_age_ms=age),
        read_period_us=10000,read_budget_us=3000,quiet_us=50,
        current_microamps_per_raw=policy['current_microamps_per_raw'],
        arm_hold_tolerance_raw=policy['arm_hold_tolerance_raw'],arm_hold_dwell_ms=100,arm_read_period_ms=20,
        initialize_erased_boot_storage=False)
    record=lambda p:dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    return dict(schema_version=1,mode='commissioning',board=board,measurement_records=dict(
        motor_identity=record(path),joint_limits=record(ROOT/'config/home_assistant/arm_ranges.development.json'),
        lift_geometry=record(geometry),camera_extrinsics=record(ROOT/'config/home_assistant/installation.development.json')))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--profile',type=Path)
    group.add_argument('--from-handoff',type=Path,help='actual S3 inventory; produces commissioning-only profile')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();document=from_handoff(a.from_handoff) if a.from_handoff else json.loads(a.profile.read_text())
    print(json.dumps(export_profile(document,a.output),indent=2))
if __name__=='__main__':main()
