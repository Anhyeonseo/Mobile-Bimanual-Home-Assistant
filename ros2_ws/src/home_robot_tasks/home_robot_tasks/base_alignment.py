"""Lateral base alignment for the top-pick demo: plan from one photo, timed strafe.

The plan is computed from the object's base-frame position in a fresh camera-only
capture. Motion follows the supervised S4 navigation trial order (transport mode,
parked admission, bounded velocity leases, measured zero interval) and always ends
in the caller's whole-system STOP. A move is never a claim that the object is now
reachable: the next capture decides that.
"""
import json
import math
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[4]
PROFILE = ROOT/'config/home_assistant/base_lateral_alignment.development.json'


def load_alignment_profile(path=PROFILE):
    profile = json.loads(Path(path).read_text())
    if profile.get('schema_version') != 1 or profile.get('record_kind') != 'base_lateral_alignment_demo':
        raise ValueError('base lateral alignment profile required')
    for key in ('left_target_y_m', 'tolerance_m', 'maximum_shift_m', 'maximum_duration_s'):
        value = profile.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'finite {key} required')
    if not 0 < profile['tolerance_m'] < profile['maximum_shift_m'] <= .5 or not 0 < profile['maximum_duration_s'] <= 120:
        raise ValueError('bounded alignment tolerance, shift and duration required')
    for side in ('left', 'right'):
        raw = profile['wheel_raw'][side]
        speed = profile['speed_m_per_s'][side]
        if (len(raw) != 4 or any(type(v) is not int or abs(v) > 100 for v in raw) or raw[3] != 0
                or not any(raw) or type(speed) not in (int, float) or not 0 < speed < .05):
            raise ValueError('bounded strafe wheel command and positive speed required')
    if [-v for v in profile['wheel_raw']['left']] != profile['wheel_raw']['right']:
        raise ValueError('left/right strafe commands must be opposite')
    return profile


def plan_lateral_alignment(object_base_xyz, profile, *, reviewed_shift_m=None):
    """Base +y (left) by shift moves the object by -shift in the base frame."""
    xyz = [float(v) for v in object_base_xyz]
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        raise ValueError('finite object base position required')
    target = float(profile['left_target_y_m'])
    if reviewed_shift_m is None:
        shift = xyz[1]-target
    else:
        shift = float(reviewed_shift_m)
        if not math.isfinite(shift) or abs(shift) > .15:
            raise ValueError('검토한 좌우 정차점 범위를 벗어났습니다.')
        target = xyz[1]-shift
    plan = dict(object_base_xyz_m=xyz, target_y_m=target, shift_m=shift,
                tolerance_m=profile['tolerance_m'], arm='left')
    if abs(shift) <= profile['tolerance_m']:
        return dict(plan, direction=None, within_tolerance=True, duration_s=0., wheel_raw=[0, 0, 0, 0])
    if abs(shift) > profile['maximum_shift_m']:
        raise ValueError(f'정렬 이동량 {abs(shift)*100:.1f} cm가 허용 최대 {profile["maximum_shift_m"]*100:.0f} cm를 넘습니다. '
                         '로봇을 물체 가까이 옮긴 뒤 다시 검사해 주세요.')
    direction = 'left' if shift > 0 else 'right'
    speed = float(profile['speed_m_per_s'][direction])
    duration = abs(shift)/speed
    if duration > profile['maximum_duration_s']:
        raise ValueError(f'예상 이동 시간 {duration:.0f}초가 제한 {profile["maximum_duration_s"]}초를 넘습니다.')
    return dict(plan, direction=direction, within_tolerance=False, duration_s=duration,
                expected_speed_m_per_s=speed, wheel_raw=list(profile['wheel_raw'][direction]))


def describe(plan):
    x, y, _ = plan['object_base_xyz_m']
    where = f'물체 base 좌표 x {x*100:.1f} cm, y {y*100:+.1f} cm · 왼팔 목표 y {plan["target_y_m"]*100:+.1f} cm'
    if plan['within_tolerance']:
        return where+' · 허용 범위 안 · 이동 없음'
    side = '왼쪽' if plan['direction'] == 'left' else '오른쪽'
    return where+f' · 베이스를 {side}으로 {abs(plan["shift_m"])*100:.1f} cm 이동 (예상 {plan["duration_s"]:.0f}초)'


def run_lateral_shift(plan, *, mobile, observe, enter_transport, admit, check=lambda: None,
                      clock=time.monotonic, sleep=time.sleep, log=lambda event, **data: None,
                      settle_ms=400, settle_timeout_s=3.):
    """Timed strafe inside an already prepared, single-owner controller session.

    ``enter_transport`` and ``admit`` are the existing platform_preparation
    steps (``enter_base_transport`` and ``arm_observed_stationary``). The caller
    owns the whole-system STOP that must follow on every exit.
    """
    if plan.get('within_tolerance') or plan.get('direction') not in ('left', 'right'):
        raise ValueError('nonzero lateral plan required')
    values = tuple(plan['wheel_raw'])
    duration = float(plan['duration_s'])
    if len(values) != 4 or not 0 < duration <= 120:
        raise ValueError('bounded lateral command required')
    if enter_transport is not None:
        enter_transport()
        log('base_transport_admitted')
    admit()
    log('mobile_session_armed', session=mobile.session)
    started = previous = clock()
    commands = 0
    while clock()-started < duration:
        check()
        now = clock()
        if now-previous > .2:
            raise RuntimeError('바퀴 명령 주기 지연')
        previous = now
        observe()
        check()
        if clock()-started >= duration:
            break
        reply = mobile.velocity(values, lifetime_ms=150)
        commands += 1
        log('base_command', raw=list(values), reply=reply)
        if not reply.ready_for_motion(mobile.session, 250):
            raise RuntimeError('바퀴 명령 상태 확인 실패')
        sleep(.02)
    commanded_s = clock()-started
    settle_wheels(mobile, observe, check=check, clock=clock, sleep=sleep, log=log,
                  settle_ms=settle_ms, settle_timeout_s=settle_timeout_s)
    return dict(commanded_s=commanded_s, nonzero_commands=commands, settled_s=clock()-started)


class WheelRestWindow:
    """Corroborate speed quantization with three independent post-zero encoders."""
    def __init__(self):
        self.first_command = None
        self.samples = []
        self.last_tick = None

    def observe(self, reply, snapshot):
        now = reply.mcu_tick_ms
        if self.first_command is None:
            self.first_command = now
        if not reply.feedback_is_fresh(250) or max(map(abs, reply.velocity_raw[:3])) > 50:
            self.samples.clear()
            self.last_tick = None
            return
        if snapshot is None:
            return
        tick = snapshot['firmware_tick_ms']
        ages = snapshot['sample_age_ms'][:3]
        if snapshot['present_mask'] & 7 != 7 or max(ages) >= 250:
            self.samples.clear()
            self.last_tick = None
            return
        stamps = tuple((tick-age) & 0xffffffff for age in ages)
        if any(not 0 < (t-self.first_command) & 0xffffffff < 0x80000000 for t in stamps):
            return
        if self.last_tick:
            steps = [(t-p) & 0xffffffff for t,p in zip(stamps,self.last_tick)]
            if max(steps) >= 250:
                self.samples.clear()
            elif not min(steps):
                return
        self.last_tick = stamps
        positions = tuple(snapshot['positions_raw'][:3])
        self.samples.append((stamps, positions))
        anchor = self.samples[0][1]
        deltas = [tuple((p-a+2048)%4096-2048 for p,a in zip(row,anchor)) for _,row in self.samples]
        if any(max(axis)-min(axis)>2 for axis in zip(*deltas)):
            self.samples = self.samples[-1:]

    def proven(self, tick, required_ms):
        if len(self.samples) < 6:
            return False
        first, last = self.samples[0][0], self.samples[-1][0]
        # Encoder data are acquired just after the command reply.
        return (max(min((tick-t)&0xffffffff, (t-tick)&0xffffffff) for t in last) < 250
                and min((n-o)&0xffffffff for o,n in zip(first,last)) >= required_ms)


def settle_wheels(mobile, observe, *, check, clock=time.monotonic, sleep=time.sleep,
                  log=lambda event, **data: None, settle_ms=900, settle_timeout_s=3., encoder_rest=None):
    """Keep sending zero until fresh encoder samples prove the base stationary."""
    zero_since = None
    first_zero_tick = None
    last_sample = None
    deadline = clock()+settle_timeout_s
    while True:
        check()
        observe()
        reply = mobile.velocity((0, 0, 0, 0), lifetime_ms=150)
        log('base_command', raw=[0, 0, 0, 0], reply=reply)
        if not reply.ready_for_motion(mobile.session, 250):
            raise RuntimeError('바퀴 정지 명령 상태 확인 실패')
        age = (reply.mcu_tick_ms-reply.feedback_tick_ms) & 0xffffffff
        if first_zero_tick is None:
            first_zero_tick = reply.mcu_tick_ms
        sample = reply.feedback_tick_ms
        # Bit 8 is the firmware's measured stop, already tolerant of encoder noise.
        # Only a new post-zero sample advances the window. A moving sample
        # restarts it while the controller continues commanding zero velocity.
        after_zero = (sample-first_zero_tick) & 0xffffffff < 0x80000000
        forward = last_sample is None or (sample-last_sample) & 0xffffffff < 0x80000000
        if age < 250 and reply.flags & 8 and after_zero and forward:
            if zero_since is None:
                zero_since = sample
            if ((sample-zero_since) & 0xffffffff >= settle_ms or
                    (encoder_rest is not None and encoder_rest.proven(reply.mcu_tick_ms, settle_ms))):
                break
        else:
            zero_since = None
        last_sample = sample
        if clock() > deadline:
            raise TimeoutError('바퀴 영속도 유지 확인 시간 초과')
        sleep(.02)
