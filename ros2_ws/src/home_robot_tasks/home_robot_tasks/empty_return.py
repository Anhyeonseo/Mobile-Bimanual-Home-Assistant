"""One checked empty-hand return, using existing retiming and resident points.

No hardware access or automatic replay. Only an explicitly empty/supporting
scene permits opening; loaded or unknown-payload recovery needs another plan.
"""
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from .carry_program import _control, _wire_interior, CarryStage, validate_program_winding
from .carry_trajectory import MotionBounds, retime
from .carry_bundle import sha
from .s2_geometry import JOINTS, digest
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


def run_empty_return(adapter, points, *, owner, service, clock, sleep, cancelled):
    """A single finite resident route: no artificial intermediate handoffs."""
    service()
    measured=np.asarray(adapter.feedback_snapshot().positions_urad)/1e6
    if np.max(abs(measured-np.asarray(points[0].positions_rad)))>.02:
        raise ValueError('empty return anchor changed; rebuild before moving')
    if cancelled():raise ValueError('empty return cancelled before dispatch')
    adapter.start(owner,points,finite=True)
    deadline=clock()+points[-1].offset_ms/1000+4.
    while True:
        if cancelled():raise ValueError('empty return cancelled')
        adapter.poll(owner);service()
        if adapter.state.value=='ready':return
        if adapter.state.value!='active':raise ValueError('empty return controller not active')
        if clock()>deadline:raise ValueError('empty return completion timeout')
        sleep(.005)


def compile_empty_return(anchor, up, back, open_control, profile, bounds):
    anchor=np.asarray(anchor,float)
    if anchor.shape!=(12,) or not np.isfinite(anchor).all():raise ValueError('measured 12-axis anchor required')
    previous=anchor.copy();opened=anchor.copy();opened[5]=open_control
    stages=[]
    paths=[('open_empty',np.vstack([previous,opened]))]
    previous=opened
    for name,path in [('rise_empty',up),('return_empty',back)]:
        path=np.asarray(path,float)
        if path.ndim!=2 or path.shape[1]!=5 or len(path)<2:raise ValueError('complete checked return path required')
        controls=[]
        for row in path:
            q=opened.copy()
            q[:5]=[_wire_interior(_control(profile,'left',n,float(v)),bounds.minimum[i],bounds.maximum[i])
                   for i,(n,v) in enumerate(zip(JOINTS[:5],row))]
            controls.append(q)
        if np.max(abs(np.asarray(controls[0])-previous))>.002:raise ValueError('return path discontinuity')
        paths.append((name,np.vstack([previous,controls])));previous=np.asarray(controls[-1])
    points=[];offset=0
    for name,path in paths:
        part,timing=retime(path,bounds,dwell_s=.5 if name=='return_empty' else 0.)
        shifted=[TimedJointPoint(p.offset_ms+offset,p.positions_rad) for p in part]
        points.extend(shifted);offset=shifted[-1].offset_ms
        stages.append(dict(name=name,end_ms=offset,samples=len(part),timing=timing))
    validate_program_winding([CarryStage('empty_return',tuple(points),{},False)],profile)
    return dict(points=[asdict(p) for p in points],stages=stages,bounds=asdict(bounds),
                measured_anchor_rad=anchor.tolist(),duration_s=offset/1000)


def load_empty_return(path):
    data=json.loads(Path(path).read_text())
    if data.get('kind')!='checked_empty_return' or data.get('schema_version')!=1:
        raise ValueError('checked empty return required')
    program=data['program'];proof=data['proof'];model=data['model']
    if proof.get('program_sha256')!=digest(program) or proof.get('model_sha256')!=digest(model):
        raise ValueError('return geometry proof binding mismatch')
    if (proof.get('native_geometry',{}).get('status')!='PASSED' or
        proof.get('observed_empty_required') is not True or proof.get('base_commands')!=0 or
        proof.get('lift_commands')!=0 or data.get('planned_lift_m')!=0.):
        raise ValueError('native checked stationary empty return required')
    bounds=MotionBounds(**{k:tuple(v) for k,v in program['bounds'].items()})
    points=tuple(TimedJointPoint(p['offset_ms'],tuple(p['positions_rad'])) for p in program['points'])
    q=np.asarray([p.positions_rad for p in points]);ticks=np.asarray([p.offset_ms for p in points])
    if (not 2<=len(points)<=20000 or q.shape!=(len(points),12) or not np.isfinite(q).all() or
        np.any(q<bounds.minimum) or np.any(q>bounds.maximum) or not 20<=ticks[0]<=250 or
        np.any(np.diff(ticks)<=0) or np.any(np.diff(ticks)>100) or
        any(type(p.offset_ms)is not int or p.offset_ms%5 for p in points) or
        np.any(q[:,6:]!=q[0,6:]) or
        not np.allclose(q[0],program['measured_anchor_rad'],atol=1e-6,rtol=0)):
        raise ValueError('invalid finite empty return')
    if [s['name'] for s in program['stages']]!=['open_empty','rise_empty','return_empty']:
        raise ValueError('return phase order changed')
    opening_end=program['stages'][0]['end_ms'];opening=q[ticks<=opening_end]
    if np.any(opening[:,:5]!=opening[0,:5]):raise ValueError('arm moved before opening completed')
    if np.any(q[ticks>opening_end,5]!=opening[-1,5]):raise ValueError('empty return changed jaw after opening')
    validate_program_winding([CarryStage('empty_return',points,{},False)],model['profile'])
    return data,points
