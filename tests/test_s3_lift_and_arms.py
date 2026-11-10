import pytest

from home_robot_tasks.stationary_lift import SupportedContact, choose_height, height_candidates
from tools.diagnostics import run_s3_lift_and_arms as study


def test_support_height_is_a_constraint_and_grasp_height_is_the_seed():
    arm = study.lab.model(.45)
    contact = SupportedContact(.60, .68)
    seed, values = height_candidates([arm], [contact])
    assert seed == pytest.approx(.68 - study.lab.model(0).fk(arm.anchor)[0][2,3], abs=.00001)
    assert values[0] == .45 and len(values) == len(set(values))
    assert height_candidates([arm], [SupportedContact(.54, .68)])[0] == seed
    assert height_candidates([arm], [SupportedContact(.60, .70)])[0] == pytest.approx(seed + .02)
    with pytest.raises(ValueError, match='above its support'):
        height_candidates([arm], [SupportedContact(.70, .68)])


def test_floor_clamps_then_rejects_without_a_verified_path():
    arm = study.lab.model(.4)
    floor = arm.profile['ground_z_m']
    contact = SupportedContact(floor, floor+.012)
    out = choose_height([arm], [contact], lambda _: 'NO_POSE_SOLUTION')
    assert out['face_seed_m'] == 0. and [r['lift_m'] for r in out['checks'][:2]] == [.4,0.]
    assert out['lift_m'] is None and not out['motion_authorized']
    assert all(0 <= r['lift_m'] <= arm.profile['lift']['travel_m'] for r in out['checks'])


def test_common_height_and_reobservation():
    arms = [study.lab.model(.4, side) for side in ('left','right')]
    contacts = [SupportedContact(.60,.63),SupportedContact(.60,.66)]
    out = choose_height(arms,contacts,lambda h: 'VIRTUAL_PATH_FOUND' if h == .41 else 'NO_PATH')
    assert out['lift_m'] == .41 and out['requires_reobservation']
    assert out['checks'][0]['lift_m'] == .40 and not out['motion_authorized']
    with pytest.raises(ValueError, match='share the lift'):
        height_candidates([arms[0],study.lab.model(.5,'right')], contacts)


def test_shared_lift_barrier_and_drift_are_separate_from_ik():
    case = study.pair_trial('shared',[[.186,.109,.852],[.186,-.204,.852]],[-5,-5],.61,True)
    assert case['synchronous']['status'] == 'PAIR_SCREEN_PASSED'
    assert case['late_right']['status'] == 'SHARED_BODY_DRIFT'
    assert case['missing_right_contact']['status'] == 'STOP_BEFORE_LIFT'
    assert not case['end_to_end_pick_verified'] and not case['both_patches_fully_in_frame']


def test_independent_payloads_can_be_staged_at_one_height():
    case = study.pair_trial('separate',[[.186,.109,.852],[.182,-.204,.852]],[-5,-5],.61,False)
    assert case['left_then_right']['status'] == case['synchronous']['status'] == 'PAIR_SCREEN_PASSED'
    assert case['late_right']['status'] == 'PAIR_SCREEN_PASSED'
    assert not case["height"]["motion_authorized"]


@pytest.fixture
def lift_rule():
    from home_robot_tasks.stationary_lift import LiftHeightRule
    return LiftHeightRule.from_profile(study.lab.read(study.lab.CONFIG/'s2_coordinates.development.json'))


@pytest.mark.parametrize('surface,travel_cm', [
    (0.,0), (.3089,0), (.309,1), (.50,20), (.72,42), (.75,45), (.78,48),
    (.9389,63), (.939,64), (1.10,64)])
def test_lift_setting_selects_only_a_preset(lift_rule, surface, travel_cm):
    out = lift_rule.setting(surface)
    assert out['preset'] == f'{travel_cm}cm' and out['target_travel_from_bottom_m'] == travel_cm/100
    assert out['target_plate_height_from_floor_m'] == pytest.approx(.25559 + travel_cm/100, abs=1e-5)
    assert not out['motion_authorized']


def test_all_64_boundaries_and_same_support_jitter(lift_rule):
    for higher in range(1,65):
        edge_mm = 304 + higher*10 - 5
        lower_id, higher_id = f'{higher-1}cm', f'{higher}cm'
        assert lift_rule.setting((edge_mm-1)/1000)['preset'] == lower_id
        # Also exercise a CLI value specified in cm, not just the stored edge in metres.
        assert lift_rule.setting(edge_mm/10/100)['preset'] == higher_id
        for jitter_mm in (-2, 2):
            assert lift_rule.setting((edge_mm+jitter_mm)/1000, previous_preset=lower_id)['preset'] == lower_id
            assert lift_rule.setting((edge_mm+jitter_mm)/1000, previous_preset=higher_id)['preset'] == higher_id
        assert lift_rule.setting((edge_mm+3)/1000, previous_preset=lower_id)['preset'] == higher_id
        assert lift_rule.setting((edge_mm-4)/1000, previous_preset=higher_id)['preset'] == lower_id


@pytest.mark.parametrize('old,cm', [('bottom',0),('low',20),('middle',45),('high',62)])
def test_legacy_preset_ids_keep_the_previous_travel(lift_rule,old,cm):
    height = (304+10*cm+6)/1000
    assert lift_rule.setting(height)['preset'] == f'{cm+1}cm'
    assert lift_rule.setting(height, previous_preset=old)['preset'] == f'{cm}cm'


def test_offset_jump_and_travel_cap(lift_rule):
    from dataclasses import replace
    assert lift_rule.setting(.72,.03)['preset'] == '45cm'
    assert lift_rule.setting(.78,previous_preset='bottom')['preset'] == '48cm'
    assert replace(lift_rule,travel_m=.632).setting(1.10)['preset'] == '63cm'
    for previous in ('unknown','65cm'):
        with pytest.raises(ValueError,match='previous'):
            lift_rule.setting(.75,previous_preset=previous)


@pytest.mark.parametrize('surface,offset', [(-.01,0),(.75,-.01),(float('nan'),0),(.75,float('inf'))])
def test_unsupported_lift_inputs(lift_rule, surface, offset):
    with pytest.raises(ValueError):
        lift_rule.setting(surface, offset)


def test_optional_tabletop_table_is_monotone_and_limited_to_its_calibration_band(lift_rule):
    for support_mm in range(250, 1101):
        h = support_mm/1000
        base = lift_rule.setting(h)['target_travel_from_bottom_m']
        adjusted = lift_rule.setting(h, tabletop_calibration=True)['target_travel_from_bottom_m']
        delta = .01 if 900 <= support_mm < 929 else 0.
        assert adjusted == pytest.approx(base + delta)
    assert lift_rule.setting(.900, previous_preset='60cm', tabletop_calibration=True)['preset'] == '60cm'
    assert lift_rule.setting(.903, previous_preset='60cm', tabletop_calibration=True)['preset'] == '61cm'
    with pytest.raises(ValueError, match='unshifted'):
        lift_rule.setting(.90, .02, tabletop_calibration=True)
