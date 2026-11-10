"""Explicit measured-front or nominal-STL left-jaw conversions; no device I/O."""
import math
import numpy as np
from .jaw_calibration import JawCalibration
from .jaw_closure import next_gap, target_gap
from .s2_geometry import model_angle_from_raw, control_from_model

SECTION = 'installed_left_front_opposing_contact_faces'
INTERIOR_SECTION = 'about_6mm_inboard_of_fixed_tip'
MAX_RESIDUAL_MM = 2.0  # Development check for S3; not a bound across all apertures.


class StockLeftJaw:
    """Use the very same official TCP section as the collision model.

    This is nominal geometry, not a replacement hand measurement or a fitted
    calibration. Callers must select it explicitly for stock physical fingers.
    """
    def __init__(self, stock, coordinates, raw_limits):
        self.stock=stock
        self.joint=coordinates['arms']['left']['alignment']['gripper']
        if (self.joint['raw_to_control_sign']*self.joint['control_to_model_sign'] != 1
                or raw_limits['opening_raw_direction'] != 1):
            raise ValueError('stock jaw requires increasing raw to open')
        values=np.arange(max(0,raw_limits['minimum_unwrapped_raw']),
                         min(4095,raw_limits['maximum_unwrapped_raw'])+1)
        angles=np.array([model_angle_from_raw(int(v),self.joint,'gripper') for v in values])
        valid=(angles>=stock.angles[0])&(angles<=stock.angles[-1])
        if not valid.any():raise ValueError('no stock jaw section within raw limits')
        self.raw_min=int(values[valid][0]);self.raw_max=int(values[valid][-1])

    def gap_mm(self, raw):
        if not math.isfinite(raw) or not self.raw_min<=raw<=self.raw_max:
            raise ValueError('raw outside nominal STL section')
        angle=model_angle_from_raw(float(raw),self.joint,'gripper')
        return float(np.interp(angle,self.stock.angles,self.stock.gaps)*1000)

    def raw_at_least(self, gap_mm):
        if not math.isfinite(gap_mm) or not self.gap_mm(self.raw_min)<=gap_mm<=self.gap_mm(self.raw_max):
            raise ValueError('gap outside nominal STL section')
        # Binary search the discrete encoder domain: no extrapolation or
        # inverse-interpolation rounding that could command a smaller opening.
        lo,hi=self.raw_min,self.raw_max
        while lo<hi:
            mid=(lo+hi)//2
            if self.gap_mm(mid)>=gap_mm:hi=mid
            else:lo=mid+1
        return lo

    def preview(self, plan):
        if plan.get('status')!='BOUNDED_CLOSE':return dict(status='WIDTH_UNRESOLVED',hardware_authorized=False)
        try:
            entry=self.raw_at_least(plan['entry_gap_m']*1000)
            floor=self.raw_at_least(target_gap(plan)*1000)
        except ValueError as exc:return dict(status='OUTSIDE_NOMINAL_STL_RANGE',reason=str(exc),hardware_authorized=False)
        return dict(status='NOMINAL_STL_GAP_PREVIEW',section='official_tcp_closing_section',
            entry_raw=entry,lower_search_raw=floor,entry_gap_mm=self.gap_mm(entry),
            lower_search_gap_mm=self.gap_mm(floor),physically_measured=False,hardware_authorized=False)

    def contact_envelope(self, plan):
        if self.preview(plan)['status']!='NOMINAL_STL_GAP_PREVIEW':
            raise ValueError('nominal STL gap plan required')
        upper=plan['upper_contact_gap_m']*1000
        if not (math.isfinite(upper) and target_gap(plan)*1000<=upper<=plan['entry_gap_m']*1000):
            raise ValueError('invalid nominal contact envelope')
        raw=self.raw_at_least(upper)
        if self.gap_mm(raw)>upper:raw-=1
        return dict(contact_envelope_policy='visible_span_nominal_stl_section_v1',
            nominal_upper_contact_gap_mm=upper,gap_check_allowance_mm=0.,
            upper_contact_gap_mm=upper,upper_contact_raw=raw,physically_measured=False)


def fitted_curve(points, coordinates):
    joint = coordinates['arms']['left']['alignment']['gripper']
    def angle(raw):
        return control_from_model(model_angle_from_raw(raw,joint,'gripper'),joint)
    for p in points:
        if p.get('section') != SECTION:
            raise ValueError('front contact samples required; do not mix jaw sections')
        if not (0 <= p['raw_min'] <= p['raw'] <= p['raw_max'] <= 4095
                and math.isfinite(p['gap_mm']) and 0 <= p['gap_mm'] <= 100):
            raise ValueError('invalid measured pad sample')
    ordered = sorted(points,key=lambda p:p['raw'])
    if (len(ordered)<3 or any(a['raw_max'] >= b['raw_min'] or a['gap_mm'] >= b['gap_mm']
                               for a,b in zip(ordered,ordered[1:]))):
        raise ValueError('three distinct increasing hand-measured apertures required')
    samples = sorted((angle(p['raw']),p['gap_mm']) for p in ordered)
    return JawCalibration(samples,open_rad=angle(ordered[-1]['raw']),close_rad=angle(ordered[0]['raw']))


class InstalledLeftJaw:
    def __init__(self, record, coordinates):
        if (record.get('schema_version') != 1 or record.get('arm') != 'left'
                or record.get('motor_id') != 6 or record.get('section') != SECTION
                or record.get('status') != 'CHECK_PASSED'
                or record.get('calibration_id') != coordinates['calibration_id']
                or record.get('assembly_revision') != coordinates['assembly_revision']):
            raise ValueError('verified pad calibration on this installation required')
        self.curve = fitted_curve(record['samples'],coordinates)
        joint=coordinates['arms']['left']['alignment']['gripper']
        self.sign=joint['raw_to_control_sign']
        if self.sign not in (-1,1):
            raise ValueError('invalid encoder sign')
        check=record['check']['measurement']
        if check.get('section') != SECTION:
            raise ValueError('independent measurement at front contact faces required')
        if any(not (check['raw_max'] < p['raw_min'] or p['raw_max'] < check['raw_min'])
               for p in record['samples']):
            raise ValueError('pad check requires an independent aperture')
        predicted=self.gap_mm(check['raw'])
        if not (math.isfinite(check['gap_mm']) and abs(predicted-check['gap_mm']) <= MAX_RESIDUAL_MM):
            raise ValueError('pad check residual exceeds development tolerance')
        # Recompute from independent evidence, never a cached summary or a trial
        # endpoint. This is a development allowance, not a force measurement.
        self.check_residual_mm = abs(predicted-check['gap_mm'])
        self.raw_max=max(p['raw'] for p in record['samples'])

    def gap_mm(self, raw):
        return self.curve.gap(self.sign*(raw-2048)*math.tau/4096)

    def raw_at_least(self, gap_mm):
        angle=self.curve.angle_for_gap(gap_mm)
        # Increasing raw opens this installation. Round towards the larger gap for both bounds.
        raw=math.ceil(2048 + self.sign*angle*4096/math.tau - 1e-10)
        if raw>self.raw_max:
            raise ValueError('rounded command outside measured raw interval')
        return raw

    def preview(self, plan):
        if plan.get('status') != 'BOUNDED_CLOSE':
            return dict(status='WIDTH_UNRESOLVED',hardware_authorized=False)
        try:
            entry=self.raw_at_least(1000*plan['entry_gap_m'])
            floor=self.raw_at_least(1000*target_gap(plan))
        except ValueError as exc:
            return dict(status='OUTSIDE_MEASURED_PAD_RANGE',reason=str(exc),hardware_authorized=False)
        return dict(status='MEASURED_GAP_PREVIEW',section=SECTION,entry_raw=entry,lower_search_raw=floor,
                    entry_gap_mm=self.gap_mm(entry),lower_search_gap_mm=self.gap_mm(floor),
                    hardware_authorized=False)

    def contact_envelope(self, plan):
        """Include measured gap-conversion error without altering the close goal."""
        bounds = self.preview(plan)
        if bounds['status'] != 'MEASURED_GAP_PREVIEW':
            raise ValueError('contact envelope requires a measured gap plan')
        nominal = plan['upper_contact_gap_m'] * 1000
        entry = plan['entry_gap_m'] * 1000
        if not (math.isfinite(nominal) and 1000*target_gap(plan) <= nominal <= entry):
            raise ValueError('invalid nominal contact envelope')
        upper = min(nominal+self.check_residual_mm, entry, self.gap_mm(self.raw_max))
        raw = self.raw_at_least(upper)
        if self.gap_mm(raw) > upper + 1e-9:
            raw -= 1
        return dict(contact_envelope_policy='visible_span_plus_gap_check_v1',
                    nominal_upper_contact_gap_mm=nominal,
                    gap_check_allowance_mm=self.check_residual_mm,
                    upper_contact_gap_mm=upper, upper_contact_raw=raw)

    def step_preview(self, plan, measured_raw, effort, *, healthy=True):
        """Apply the S3 gap policy on representable encoder positions. Never dispatch."""
        bounds = self.preview(plan)
        if bounds['status'] != 'MEASURED_GAP_PREVIEW':
            return bounds['status'], None
        if type(measured_raw) is not int or not bounds['lower_search_raw'] <= measured_raw <= bounds['entry_raw']:
            return 'STOP_OUTSIDE_PLAN', None
        representable_target = bounds['lower_search_gap_mm'] / 1000
        quantized = dict(plan, entry_gap_m=bounds['entry_gap_mm']/1000,
                         target_gap_m=representable_target, min_search_gap_m=representable_target,
                         upper_contact_gap_m=self.contact_envelope(plan)['upper_contact_gap_mm']/1000)
        action, gap = next_gap(quantized, self.gap_mm(measured_raw)/1000, effort, healthy=healthy)
        if gap is None:
            return action, None
        target = self.raw_at_least(gap*1000)
        if not bounds['lower_search_raw'] <= target < measured_raw:
            return 'STOP_ENCODER_RESOLUTION', None
        return action,target
