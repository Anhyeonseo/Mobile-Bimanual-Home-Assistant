"""Offline proposals for the shared lift of a stationary robot. All heights are travel from bottom."""
from bisect import bisect_right
from dataclasses import asdict, dataclass
import math

import numpy as np



# Home hand height with the lift at its lower stop, rounded from the S2 model (30.38 cm).
HOME_HEIGHT_MM = 304
MAX_PRESET_CM = 64
SWITCH_MARGIN_MM = 3
# Entry to step n is halfway between n-1 and n. Use integer mm to define exact ties.
EDGES_MM = tuple(HOME_HEIGHT_MM + cm * 10 - 5 for cm in range(1, MAX_PRESET_CM + 1))
EDGES_M = tuple(edge / 1000 for edge in EDGES_MM)
TRAVEL_M = tuple(cm / 100 for cm in range(MAX_PRESET_CM + 1))
PRESET_STEPS = {f'{cm}cm': cm for cm in range(MAX_PRESET_CM + 1)}
# Accept previous IDs from saved four-band reports at their original physical positions.
PRESET_STEPS.update(bottom=0, low=20, middle=45, high=62)


# Optional nominal left-arm tabletop calibration. Advance entry into steps 61–63;
# stop at step 63 so the lookup stays monotone when returning to the standard table.
TABLETOP_EDGES_MM = tuple({61: 900, 62: 909, 63: 919}.get(step, edge)
                         for step, edge in enumerate(EDGES_MM, start=1))
TABLETOP_EDGES_M = tuple(edge / 1000 for edge in TABLETOP_EDGES_MM)
ARRIVAL_TOLERANCE_M = .003


def preset_step(support_height_m, maximum_travel_m, *, above_support_m=0., previous_preset=None,
                tabletop_calibration=False):
    """Select a single integer-centimetre target, without trying any arm plans."""
    for value in (support_height_m, above_support_m, maximum_travel_m):
        if not math.isfinite(value) or value < 0:
            raise ValueError('finite nonnegative heights required')
    height = support_height_m + above_support_m
    if not math.isfinite(height):
        raise ValueError('finite effective support height required')
    if tabletop_calibration and above_support_m != 0:
        raise ValueError('tabletop calibration requires the unshifted support height')
    edges_m, edges_mm = ((TABLETOP_EDGES_M, TABLETOP_EDGES_MM) if tabletop_calibration else (EDGES_M, EDGES_MM))
    height = round(height, 12)
    last = bisect_right(TRAVEL_M, maximum_travel_m) - 1
    step = bisect_right(edges_m, height, hi=last)
    if previous_preset is not None:
        previous = PRESET_STEPS.get(previous_preset)
        if previous is None or previous > last:
            raise ValueError('unknown or unavailable previous lift preset')
        lower = (edges_mm[previous-1] - SWITCH_MARGIN_MM) / 1000 if previous else -math.inf
        upper = (edges_mm[previous] + SWITCH_MARGIN_MM) / 1000 if previous < last else math.inf
        if lower <= height < upper:
            step = previous
    return step


@dataclass(frozen=True)
class LiftHeightRule:
    """Look up one shared lift setting from the support height; no object label or arm pose."""
    plate_low_m: float
    travel_m: float
    axis_z: float

    @classmethod
    def from_profile(cls, profile, geometry=None, side='left'):
        # Keep the earlier profile/geometry call signature. Only lift geometry is needed.
        lift = profile['lift']
        if side not in ('left', 'right'):
            raise ValueError('unknown plate reference side')
        return cls(lift['floor_height_low_m'][side], lift['travel_m'], lift['axis_base'][2])

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.plate_low_m, self.travel_m, self.axis_z)):
            raise ValueError('invalid lift geometry')
        if self.axis_z > 1:
            raise ValueError('invalid vertical lift axis')

    def setting(self, support_height_m, above_support_m=0., *, previous_preset=None, tabletop_calibration=False):
        step = preset_step(support_height_m, self.travel_m, above_support_m=above_support_m,
                           previous_preset=previous_preset, tabletop_calibration=tabletop_calibration)
        name = f'{step}cm'
        target = TRAVEL_M[step]
        return dict(status='HEIGHT_SETTING', preset=name, table='s3_left_tabletop' if tabletop_calibration else 'nominal',
                    support_height_from_floor_m=support_height_m,
                    above_support_m=above_support_m, target_travel_from_bottom_m=target,
                    target_plate_height_from_floor_m=self.plate_low_m + target*self.axis_z,
                    limit='LOWER' if target == 0 else 'UPPER' if target == self.travel_m else 'NONE',
                    motion_authorized=False)


@dataclass(frozen=True)
class SupportedContact:
    """Base-frame Z of a proposed jaw centre and the supporting surface directly underneath."""
    support_z_m: float
    contact_z_m: float


def height_candidates(arms, contacts):
    """Align reference hand heights with observed grasp faces, then probe nearby alternatives.

    One common lift must serve every arm. The median seed balances their vertical offsets.
    Current height is tried first. Within the sampled set prefer smaller lift travel.
    """
    if not arms or len(arms) != len(contacts):
        raise ValueError('one supported contact per arm required')
    for contact in contacts:
        if not np.isfinite([contact.contact_z_m, contact.support_z_m]).all() or contact.contact_z_m <= contact.support_z_m:
            raise ValueError('contact centre must lie above its support')
    current = arms[0].lift_m
    travel = arms[0].profile['lift']['travel_m']
    if any(a.lift_m != current or a.profile['lift']['travel_m'] != travel for a in arms):
        raise ValueError('arms must share the lift state and range')
    offsets = [(c.contact_z_m - arm.fk(arm.anchor)[0][2, 3]) / arm.profile['lift']['axis_base'][2]
               for arm, c in zip(arms, contacts, strict=True)]
    seed = float(np.clip(current + np.median(offsets), 0., travel))
    # A local centimetre grid around the face plus a coarser sweep of the full travel.
    near = round(seed, 2) + np.arange(-4, 5) * .01
    probes = [current, seed, current-.01, current+.01, *near, *np.arange(0, travel, .05), travel]
    candidates = {min(travel, max(0., round(float(h), 6))) for h in probes}
    candidates.discard(round(current, 6))
    candidates.discard(current)
    order = sorted(candidates, key=lambda h: (round(abs(h-current), 8), abs(h-seed), h))
    # For a below-range grasp evaluate the lower stop immediately after the current pose.
    if seed == 0. and current != 0.:
        order.remove(0.)
        order.insert(0, 0.)
    return seed, [current, *order]


def choose_height(arms, contacts, evaluate):
    """Accept only a complete geometric path at a common height, never a hardware trajectory."""
    seed, heights = height_candidates(arms, contacts)
    checks = []
    for height in heights:
        status = evaluate(height)
        checks.append(dict(lift_m=height, status=status))
        if status == 'VIRTUAL_PATH_FOUND':
            return dict(status='HEIGHT_FOUND', lift_m=height, face_seed_m=seed, contacts=[asdict(c) for c in contacts], checks=checks,
                        requires_reobservation=height != arms[0].lift_m, motion_authorized=False)
    return dict(status='NO_TESTED_HEIGHT', lift_m=None, face_seed_m=seed, contacts=[asdict(c) for c in contacts], checks=checks,
                requires_reobservation=True, motion_authorized=False)
