#!/usr/bin/env python3
"""Offline paired comparison of one-shot lift rules. Each rule chooses one Z per support."""
import argparse
from collections import Counter
from functools import lru_cache
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks')]
from home_robot_tasks.stationary_lift import LiftHeightRule, MAX_PRESET_CM, preset_step
from tools.diagnostics import run_s3_stationary_lab as lab


# The prior discrepancy was on surfaces 90–92 cm high. Do not extrapolate past this band.
SUPPORT_BAND_M = (.90, .93)
FORWARD_SPLIT_M = .19
OFFSETS_CM = (0, 1, -1, 2)  # also the tie-break order when fitting
SPLITS = {'fit': 73, 'validation_a': 271, 'validation_b': 997, 'confirmation': 20261106}
SCENES_PER_BIN = 18


@lru_cache(maxsize=16)
def arm_at(step_cm):
    return lab.model(step_cm / 100)


def layouts(seed):
    rng = np.random.default_rng(seed)
    for lo, hi in ((.176, FORWARD_SPLIT_M), (FORWARD_SPLIT_M, .204)):
        for _ in range(SCENES_PER_BIN):
            yield dict(x=rng.uniform(lo, hi), y=rng.uniform(.096, .122), yaw=rng.uniform(-20, 20),
                       length=rng.uniform(.026, .032), support=rng.uniform(*SUPPORT_BAND_M))


def evaluate_scene(layout, rule, ground_z):
    nominal = round(100 * rule.setting(layout['support'])['target_travel_from_bottom_m'])
    surface = ground_z + layout['support'] + .012
    case = (layout['x'], layout['y'], layout['yaw'])
    responses = {}
    forward = None
    for delta in OFFSETS_CM:
        step = max(0, min(MAX_PRESET_CM, nominal+delta))
        try:
            trial = lab.run_case(arm_at(step), case, 1, candidate_source='yeoeun-surface',
                                 marker_length_m=layout['length'], surface_z=surface)
        except ValueError as exc:
            if str(exc) != 'synthetic target outside camera view':
                raise
            responses[delta] = 'OUTSIDE_CAMERA_VIEW'
            continue
        # The stationary D415 is independent of lift travel. Read the perceived location
        # once, from the nominal scene, rather than assigning a bin from object truth.
        if delta == 0:
            center = trial['review'].get('base_points_m', {}).get('center_xyz_m')
            if center is not None:
                forward = center[0]
        responses[delta] = trial['status']
    region = None if forward is None else 'near' if forward < FORWARD_SPLIT_M else 'far'
    return dict(layout=layout, nominal_cm=nominal, observed_forward_m=forward, region=region, responses=responses)


def success(scene, delta):
    return scene['responses'][delta] == 'SIMULATED_PICK_HELD'


def fitted_offset(scenes):
    if not scenes:
        return 0
    return max(OFFSETS_CM, key=lambda delta: sum(success(s, delta) for s in scenes))


def score(scenes, offsets=None):
    counts = Counter()
    for s in scenes:
        if offsets is None:
            # Score the actual monotone table, including its cap near the upper band edge.
            delta = preset_step(s['layout']['support'], MAX_PRESET_CM/100, tabletop_calibration=True) - s['nominal_cm']
        else:
            delta = offsets.get(s['region'], 0)
        ok, baseline = success(s, delta), success(s, 0)
        counts['held'] += ok
        counts['gained'] += ok and not baseline
        counts['lost'] += baseline and not ok
    return dict(total=len(scenes), **counts)


def compare():
    arm = arm_at(61)
    rule = LiftHeightRule.from_profile(arm.profile)
    ground_z = arm.profile['ground_z_m']
    train = [evaluate_scene(layout, rule, ground_z) for layout in layouts(SPLITS['fit'])]
    valid_train = [s for s in train if s['region'] is not None]
    pooled = fitted_offset(valid_train)
    policies = dict(baseline=dict(near=0, far=0), plus_one=dict(near=1, far=1),
                    pooled=dict(near=pooled, far=pooled),
                    forward_bins={k:fitted_offset([s for s in valid_train if s['region'] == k])
                                  for k in ('near','far')})
    splits = dict(fit=train)
    for name,seed in SPLITS.items():
        if name != 'fit':
            splits[name] = [evaluate_scene(layout, rule, ground_z) for layout in layouts(seed)]
    return dict(mode='offline_nominal_simulation', hardware_commands=0, motion_authorized=False,
                support_band_m=SUPPORT_BAND_M, forward_split_base_x_m=FORWARD_SPLIT_M, seeds=SPLITS,
                offsets_cm=policies, runtime_lift_targets_per_policy=1,
                results={name:{**{p:score(scenes,v) for p,v in policies.items()},
                               'tabletop_calibration':score(scenes)} for name,scenes in splits.items()},
                scenes=splits)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    path = parser.parse_args().output
    if path.exists():
        parser.error('choose a new output file')
    report = compare()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k:report[k] for k in ('offsets_cm','results')}, indent=2))


if __name__ == '__main__':
    main()
