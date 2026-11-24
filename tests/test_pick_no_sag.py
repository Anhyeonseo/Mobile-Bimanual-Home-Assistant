from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import numpy as np
import pytest
from so101_arm_bridge.lift_wire import LiftStatus
from home_robot_tasks.pick_calibration import make_resumed_height_reference,top_model_height,RISE_BOARD,planning_lift_limit
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw
from home_robot_tasks import s2_geometry as geo
from test_pick_lift_transition import arrival_fixture

ROOT=Path(__file__).resolve().parents[1]


def test_station_uses_calibrated_height_but_explicit_historical_mode_remains_available():
    from home_robot_tasks.pick_web_backend import WebPickBackend
    profile,data=arrival_fixture()
    source=deepcopy(data['start_revalidation']['arrival_evidence']['platform'])
    fresh=replace(LiftStatus(**source['lift']),height_um=654588,position_raw=1747)
    options=WebPickBackend({'mode':'station'}).options
    old_options=WebPickBackend({'mode':'station','lift_height_model':'field_effective'}).options
    ref=dict(source['top_reference'],height_um=fresh.height_um,position_raw=fresh.position_raw,
             height_model=options['lift_height_model'])
    actual=top_model_height(fresh,profile,RISE_BOARD,ref)
    historical=top_model_height(fresh,profile,RISE_BOARD,dict(ref,height_model=old_options['lift_height_model']))
    assert .02<actual-historical<.03
    assert historical==pytest.approx(.630)

def test_no_sag_resume_uses_bottom_distance_not_registered_top_or_630():
    profile,data=arrival_fixture()
    source=deepcopy(data['start_revalidation']['arrival_evidence']['platform'])
    source['lift'].update(height_um=652841,position_raw=1907)
    source['top_reference'].update(height_um=652841,position_raw=1907)
    fresh=replace(LiftStatus(**source['lift']),boot_id=10,session=3,height_um=654588,position_raw=1908)
    ref=make_resumed_height_reference(fresh,profile,RISE_BOARD,source,operator_height_unchanged=True,height_model='encoder_no_sag')
    ratio=abs(profile['lift']['meters_per_count'])*4096/.087287
    expected=.652841*ratio+profile['lift']['meters_per_count']
    assert top_model_height(fresh,profile,RISE_BOARD,ref)==pytest.approx(expected)
    assert expected>.65
    # Re-registering the same position must not add the registered top offset.
    moved=replace(fresh,height_um=654567,position_raw=1909)
    assert top_model_height(moved,profile,RISE_BOARD,ref)==pytest.approx(expected-.000021*ratio)
    assert source['lift']['height_um']==652841


def arm_fixture():
    c=ROOT/'config/home_assistant';profile=json.loads((c/'s2_coordinates.development.json').read_text())
    jaw=load_stock_jaw(c/'official_sources/so101_new_calib.urdf',ROOT/'ros2_ws/src/so101_description/meshes')
    return StationaryArm(profile,geo.load_geometry(c/'official_geometry.json'),json.loads((c/'arm_ranges.development.json').read_text()),.65282,jaw)


def test_encoder_extent_uses_existing_recorded_counts_without_changing_profile():
    arm=arm_fixture();p=arm.profile
    assert planning_lift_limit(p)==pytest.approx(30717*abs(p['lift']['meters_per_count']))
    assert p['lift']['travel_m']==pytest.approx(.6438468174282681)
    with pytest.raises(ValueError,match='outside model'):
        StationaryArm(p,geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json'),json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text()),.7,arm.jaw)
