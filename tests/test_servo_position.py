"""Encoder-seam regressions, including planner, wire and calibration admission."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_bundle import validate_installed_binding
from home_robot_tasks.carry_program import CarryStage, validate_program_winding
from home_robot_tasks.stationary_pick_sim import StationaryArm
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
from so101_arm_bridge.home_rebase import centering_preflight
from so101_arm_bridge.servo_position import raw_from_wire, single_turn_window, validate_wire_route
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from tools.diagnostics.carry_stream_sim import SimClock, SimTransport

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/home_assistant'
read=lambda name:json.loads((CONFIG/name).read_text())
wire=lambda raw,sign=1:round(sign*(raw-2048)*6283185/4096)


@pytest.mark.parametrize('sign',[-1,1])
def test_all_encoder_codes_and_adjacent_turns_roundtrip(sign):
    for raw in range(-4096,8192):
        assert raw_from_wire(wire(raw,sign),sign)==raw


@pytest.mark.parametrize('raws',[(10,-46),(4090,4106),(10,4050),(4050,10),(0,2048)])
@pytest.mark.parametrize('sign',[-1,1])
def test_small_crossing_and_sparse_modulo_jump_never_turn_into_long_way(raws,sign):
    with pytest.raises(ValueError,match='seam|ambiguous'):
        validate_wire_route([(wire(raw,sign),) for raw in raws],(sign,))


@pytest.mark.parametrize('raws',[(10,9,0),(4050,4020,4000),(2048,2040,2032),(4090,4095)])
def test_continuous_in_branch_direction_is_preserved(raws):
    route=[(wire(r),) for r in raws];original=deepcopy(route)
    validate_wire_route(route,(1,))
    assert route==original


def test_stage_boundary_is_part_of_the_whole_program():
    profile=read('s2_coordinates.development.json')
    def point(raw):
        q=[0.]*12;q[4]=wire(raw)/1e6
        return TimedJointPoint(80,tuple(q))
    stages=[CarryStage('approach',(point(1),),{},False),CarryStage('close',(point(-1),),{},False)]
    with pytest.raises(ValueError,match='seam'):validate_program_winding(stages,profile)


def test_ik_and_moveit_input_bounds_exclude_seam_without_rewriting_evidence():
    profile=read('s2_coordinates.development.json');ranges=read('arm_ranges.development.json')
    original=deepcopy(ranges)
    arm=StationaryArm(profile,geo.load_geometry(CONFIG/'official_geometry.json'),ranges,0.,None)
    for name,lo,hi in zip(geo.JOINTS[:5],arm.lo,arm.hi):
        alignment=profile['arms']['left']['alignment'][name]
        for model_q in np.linspace(lo,hi,101):
            q=geo.control_from_model(float(model_q),alignment)
            assert 0<=raw_from_wire(round(q*1e6),alignment['raw_to_control_sign'])<=4095
    assert ranges==original
    assert arm.raw_windows['shoulder'][1]==min(4095,ranges['arms']['left']['shoulder']['maximum_unwrapped_raw'])
    assert arm.raw_windows['wrist_roll'][0]==max(0,ranges['arms']['left']['wrist_roll']['minimum_unwrapped_raw'])
    assert single_turn_window(dict(minimum_unwrapped_raw=-100,maximum_unwrapped_raw=2100),-1)==(-100,-1)


def adapter_at(raw):
    q=[0.]*12;q[4]=wire(raw)/1e6
    clock=SimClock();transport=SimTransport(clock,tuple(q))
    # Exercise the historical seam envelope; new calibrated ranges exclude it.
    path=CONFIG/'historical/pre-home-20261110/arm_ranges.development.json'
    check=StationaryPickAdapter(transport,path)
    adapter=StationaryPickAdapter(transport,path,authorized_limits=replace(check._limits,hardware_authorized=True),motion_authorized=True)
    hello=transport.enter_binary_mode
    def identity():
        result=hello();result.firmware_version=adapter.firmware_version
        result.left_calibration_hash=result.right_calibration_hash=adapter.calibration_hash
        return result
    transport.enter_binary_mode=identity
    transport.stationary_platform=lambda op:None
    adapter.prepare()
    return adapter,transport,q


@pytest.mark.parametrize('finite',[True,False])
def test_seam_is_rejected_before_arm_or_enable_even_when_steps_are_tiny(finite):
    adapter,transport,q=adapter_at(2)
    points=[]
    for t,raw in ((80,1),(130,-1)):
        row=q.copy();row[4]=wire(raw)/1e6
        points.append(TimedJointPoint(t,tuple(row)))
    with pytest.raises(Exception,match='seam'):adapter.start('test',points,finite=finite)
    assert 'arm' not in transport.calls and 'enable' not in transport.calls
    assert 'safe_stop' not in transport.calls and adapter.state.value=='ready'


def test_rolling_batch_uses_previous_tail_as_winding_reference():
    adapter,_,q=adapter_at(1)
    reference=tuple(round(v*1e6) for v in q)
    q[4]=wire(-1)/1e6
    with pytest.raises(Exception,match='seam'):
        adapter._validate_points([TimedJointPoint(80,tuple(q))],first_reference_urad=reference,first_span_ms=80)


def test_preflight_preserves_measured_travel_and_copied_provenance():
    d=read('arm_ranges.development.json');original=deepcopy(d);rows=[]
    # Midpoints fit each full range around 2048, including old wrapped ranges.
    for id,(name,offset) in enumerate(zip(geo.JOINTS,d['calibration']['right']['offsets_raw']),1):
        e=d['arms']['right'][name];raw=bytearray(40);raw[31:33]=offset.to_bytes(2,'little')
        position=(e['minimum_unwrapped_raw']+e['maximum_unwrapped_raw'])//2
        rows.append(dict(id=id,position=position%4096,eeprom_hex=raw.hex()))
    result=centering_preflight(d,'right',rows)
    assert result['safe_to_center'] and d==original
    for r in result['joints']:
        # Match by name, not JSON key iteration order.
        e=d['arms']['right'][r['joint']]
        assert r['proposed_maximum_raw']-r['proposed_minimum_raw']==e['maximum_unwrapped_raw']-e['minimum_unwrapped_raw']
        assert r['range_source']=='copied_from_left_by_operator_request'


@pytest.mark.parametrize('defect',['model_angle','id','ranges','stale_profile'])
def test_old_bundle_cannot_survive_rezero_or_same_id_coordinate_edits(defect):
    profile=read('s2_coordinates.development.json')
    key='config/home_assistant/arm_ranges.development.json'
    checksum=profile['source_files_sha256'][key]
    model={'profile':deepcopy(profile),'range_sha256':checksum}
    validate_installed_binding(model,profile,checksum)
    if defect=='model_angle':profile['arms']['left']['alignment']['shoulder']['home_model_rad']+=.1
    elif defect=='id':profile['calibration_id']='new-home'
    elif defect=='ranges':model['range_sha256']='old'
    else:profile['source_files_sha256'][key]='old'
    with pytest.raises(ValueError):validate_installed_binding(model,profile,checksum)


@pytest.mark.parametrize('name',['run_s3_stationary_lab','run_s3_base_stops','run_s3_lift_and_arms','replay_s3_grasp'])
def test_standalone_planning_tools_load_shared_position_contract(name):
    subprocess.run([sys.executable,str(ROOT/'tools/diagnostics'/f'{name}.py'),'--help'],
                   check=True,capture_output=True,timeout=20)


def test_outside_numerical_seed_recovers_without_changing_requested_grasp():
    arm=StationaryArm(read('s2_coordinates.development.json'),geo.load_geometry(CONFIG/'official_geometry.json'),
                      read('arm_ranges.development.json'),0.,None)
    target=arm.fk((arm.lo+arm.hi)/2)[0];original=target.copy()
    q,position_error,angle_error=arm.solve(target,arm.lo-.01)
    assert np.all(q>=arm.lo) and np.all(q<=arm.hi)
    assert position_error<=.005 and angle_error<=np.deg2rad(10)
    np.testing.assert_array_equal(target,original)
