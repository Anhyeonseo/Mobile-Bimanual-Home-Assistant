import copy
import math
from pathlib import Path
import pytest
from home_robot_tasks.s4_base_preparation import prepare, review_capture

ROOT=Path(__file__).resolve().parents[1]


def test_official_geometry_matches_existing_teleop_directions_without_enable():
    result=prepare(ROOT)
    assert result['wheel_order']==[7,8,9]
    x=result['unit_twist_wheel_rates']['forward'];y=result['unit_twist_wheel_rates']['left']
    assert x[0]<0<x[1] and abs(x[2])<.4
    assert y[0]<0 and y[1]<0<y[2]
    assert all(v>0 for v in result['unit_twist_wheel_rates']['counterclockwise'])
    assert result['raw_per_rad_s'] is None and not result['hardware_enabled']
    assert result['nominal_vs_referenced_floor_difference_m']==pytest.approx(.020704478)


def sample_capture(moving=True):
    records=[{'inventory':dict(all_reads_ok=True,motor_write_commands=0,
        info=dict(motion_enabled=False,wheel_samples_supported=True))}]
    position=4000
    for n in range(40):
        delta=(20 if n<20 else -20) if moving else 0
        position=(position+delta)%4096
        speed=(100 if delta>0 else 0x8064) if moving else 0
        records.append(dict(sequence=n,host_started_s=n*.04,host_received_s=n*.04+.002,
            wheels=[dict(id=i,ok=True,hal_status=0,started_tick_ms=n*40,ended_tick_ms=n*40+1,
                         position_raw=position,speed_raw=speed) for i in (7,8,9)]))
    return records


def test_scale_fit_uses_feedback_and_handles_encoder_wrap_without_claiming_drive_gain():
    result=review_capture(sample_capture())
    assert result['all_candidates_available']
    for wheel in result['wheels']:
        assert wheel['raw_per_encoder_rad_s']==pytest.approx(100*.04*4096/(20*2*math.pi))
        assert wheel['reported_speed_sign_vs_encoder']==1
    assert not result['command_speed_gain_qualified'] and not result['floor_travel_scale_qualified']
    assert not result['motion_authorized']


def test_stationary_capture_cannot_calibrate_scale():
    result=review_capture(sample_capture(False))
    assert not result['all_candidates_available']
    assert all(w['raw_per_encoder_rad_s'] is None for w in result['wheels'])


@pytest.mark.parametrize('fault',['reset','gap','alias','invalid','host_reset'])
def test_discontinuous_or_ambiguous_measurements_are_rejected(fault):
    records=copy.deepcopy(sample_capture())
    if fault=='reset':records[5]['sequence']=0
    if fault=='gap':records[5]['wheels'][0]['started_tick_ms']=20000
    if fault=='alias':records[5]['wheels'][0]['position_raw']=(records[4]['wheels'][0]['position_raw']+2048)%4096
    if fault=='invalid':records[5]['wheels'][0]['ok']=False
    if fault=='host_reset':records[5].update(host_started_s=0,host_received_s=.002)
    with pytest.raises(ValueError):review_capture(records)
