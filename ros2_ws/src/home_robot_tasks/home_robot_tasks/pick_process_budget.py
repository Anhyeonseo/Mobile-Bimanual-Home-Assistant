"""Best-effort CPU headroom for child processes, never real-time authority."""
import os


def background_budget():
    result={'reserved_cpu':None,'cpus':None,'nice':None,'errors':[]}
    if hasattr(os,'sched_getaffinity'):
        try:
            allowed=sorted(os.sched_getaffinity(0))
            if len(allowed)>=3:
                os.sched_setaffinity(0,allowed[:-1])
                result['reserved_cpu']=allowed[-1]
            result['cpus']=sorted(os.sched_getaffinity(0))
        except OSError as error:result['errors'].append(str(error))
    if hasattr(os,'nice'):
        try:result['nice']=os.nice(5)
        except OSError as error:result['errors'].append(str(error))
    return result
