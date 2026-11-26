"""A native SDK call holding the child GIL must not block the serial owner."""
import ctypes
import os
import time
from types import SimpleNamespace as NS

import pytest
from home_robot_tasks.pick_camera_process import PickCameraProcess
from home_robot_tasks import pick_process_budget


class BlockingCamera:
    def __enter__(self):return self
    def __exit__(self,*_):pass
    def next_frame(self):
        # PyDLL deliberately retains the GIL, unlike ctypes.CDLL.
        ctypes.PyDLL(None).usleep(150000)
        return {'color_time_s':123.,'depth_time_s':122.999,'producer_pid':os.getpid()},('color','depth',{'original':True})


def make_blocking_camera():return BlockingCamera()


class FailedCamera:
    def __enter__(self):raise ValueError('camera identity mismatch')
    def __exit__(self,*_):pass


def make_failed_camera():return FailedCamera()


def test_native_capture_stall_is_isolated_and_exposure_is_not_restamped():
    frames=[];ticks=[]
    camera=PickCameraProcess(make_blocking_camera,lambda *args:frames.append(args))
    with camera:
        deadline=time.perf_counter()+2
        while len(frames)<3 and time.perf_counter()<deadline:
            camera.check();ticks.append(time.perf_counter());time.sleep(.005)
        assert len(frames)>=3
        assert len(ticks)>20 and max(b-a for a,b in zip(ticks,ticks[1:]))<.1
        assert camera.runtime_info['pid']!=os.getpid()
        for record,images in frames:
            assert record['producer_pid']==camera.runtime_info['pid']
            assert record['color_time_s']==123. and record['depth_time_s']==122.999
            assert images==('color','depth',{'original':True})
    assert not camera.process.is_alive() and not camera.thread.is_alive()


def test_camera_startup_failure_propagates_and_child_is_joined():
    camera=PickCameraProcess(make_failed_camera,lambda *_:None)
    with pytest.raises(RuntimeError,match='identity mismatch'):
        with camera:pass
    assert not camera.process.is_alive() and not camera.thread.is_alive()


def test_consumer_failure_stops_producer_and_reaches_owner():
    def reject(*_):raise ValueError('scene changed')
    camera=PickCameraProcess(make_blocking_camera,reject)
    with camera:
        deadline=time.perf_counter()+2
        while camera.error is None and time.perf_counter()<deadline:time.sleep(.01)
        with pytest.raises(RuntimeError,match='scene changed'):camera.check()
    assert not camera.process.is_alive()


def test_background_budget_respects_allowed_cpu_mask(monkeypatch):
    masks=[];nice=[];current=[{2,4,7}]
    def set_affinity(pid,cpus):masks.append((pid,set(cpus)));current[0]=set(cpus)
    monkeypatch.setattr(pick_process_budget,'os',NS(sched_getaffinity=lambda _:current[0],
        sched_setaffinity=set_affinity,nice=lambda value:nice.append(value) or 5))
    result=pick_process_budget.background_budget()
    assert masks==[(0,{2,4})] and nice==[5]
    assert result['reserved_cpu']==7 and result['cpus']==[2,4]


def test_restricted_scheduler_is_reported_without_claiming_isolation(monkeypatch):
    def denied(*_):raise PermissionError('restricted')
    monkeypatch.setattr(pick_process_budget,'os',NS(sched_getaffinity=lambda _:{1,2,3},
        sched_setaffinity=denied,nice=denied))
    result=pick_process_budget.background_budget()
    assert result['reserved_cpu'] is None and result['nice'] is None
    assert len(result['errors'])==2
