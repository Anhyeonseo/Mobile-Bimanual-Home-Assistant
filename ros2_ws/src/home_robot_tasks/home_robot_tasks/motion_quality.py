"""Read-only tracking/hold metrics; never infers PID or mechanical causes."""
import math
import statistics
from .fetch import InvalidTask
from .navigation import number


def analyze_trace(document, *, settling_tolerance_rad=.02, minimum_hold_s=.5):
    if not isinstance(document, dict) or set(document) != {'joint_names', 'samples'}:
        raise InvalidTask('joint_names and samples required')
    names, samples = document['joint_names'], document['samples']
    if not isinstance(names,list) or len(names)!=12 or len(set(names))!=12 or any(not isinstance(n,str) or not n for n in names):
        raise InvalidTask('twelve unique joint names required')
    tolerance=number(settling_tolerance_rad,'settling tolerance')
    hold=number(minimum_hold_s,'minimum hold')
    if not 0<tolerance<=.1 or not 0<hold<=30:
        raise InvalidTask('invalid measurement bounds')
    if not isinstance(samples,list) or not 4<=len(samples)<=200000:
        raise InvalidTask('bounded observation trace required')
    times=[];targets=[];measured=[]
    for s in samples:
        if not isinstance(s,dict) or set(s)!={'time_s','target_rad','measured_rad'}:
            raise InvalidTask('explicit time/target/measured vectors required')
        stamp=number(s['time_s'],'measurement time')
        if stamp<0 or (times and stamp<=times[-1]):raise InvalidTask('strictly increasing source times required')
        for k in ('target_rad','measured_rad'):
            if not isinstance(s[k],list) or len(s[k])!=12:raise InvalidTask('twelve measured joints required')
            for v in s[k]:number(v,k)
        times.append(stamp);targets.append(s['target_rad']);measured.append(s['measured_rad'])
    gaps=[b-a for a,b in zip(times,times[1:])]
    rows=[]
    for axis,name in enumerate(names):
        errors=[m[axis]-t[axis] for m,t in zip(measured,targets)]
        start=len(times)-1
        while start>0 and abs(targets[start-1][axis]-targets[-1][axis])<=1e-6:start-=1
        duration=times[-1]-times[start]
        dense=duration>=hold and max(gaps[start:],default=0)<=.1
        # An arrival transient is excluded from stationary shake amplitude.
        window=[m[axis] for t,m in zip(times,measured) if t>=times[-1]-hold] if dense else []
        tail_bad=[i for i in range(start,len(times)) if abs(errors[i])>tolerance]
        settling=None
        if dense:
            first=(tail_bad[-1]+1) if tail_bad else start
            if first<len(times) and times[-1]-times[first]>=hold:
                settling=times[first]-times[start]
        rows.append(dict(joint=name,rms_tracking_error_rad=math.sqrt(sum(e*e for e in errors)/len(errors)),
            maximum_tracking_error_rad=max(map(abs,errors)),final_hold_duration_s=duration,
            hold_peak_to_peak_rad=max(window)-min(window) if window else None,
            hold_rms_error_rad=math.sqrt(sum((x-targets[-1][axis])**2 for x in window)/len(window)) if window else None,
            settling_time_s=settling,hold_evidence_available=dense))
    return dict(samples=len(times),duration_s=times[-1]-times[0],median_sample_gap_s=statistics.median(gaps),
        maximum_sample_gap_s=max(gaps),settling_tolerance_rad=tolerance,hold_window_s=hold,joints=rows,
        automatic_tuning=False,hardware_commands=0)
