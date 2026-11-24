"""A slow request-file validation must not stall the single robot-service owner."""
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from home_robot_tasks.pick_backend import PipelineBackend
from home_robot_tasks.pick_service import Context
from home_robot_tasks.pick_worker import ComputeWorker
from tools.lib.pick_compute import PickCompute

class SlowBindingRuntime(PickCompute):
    def __init__(self,options):
        super().__init__(options)
        from home_robot_tasks import carry_bundle
        carry_bundle.load_bundle=lambda *a,**k:(NS(capture_id='frame',source_sha256={'color':'a'}),
                                               dict(target_label='eraser',worker_pid=os.getpid()))
        read=Path.read_text
        def slow_read(path,*args,**kwargs):
            if path.name=='prepared.json':
                until=time.monotonic()+.35
                while time.monotonic()<until:pass
            return read(path,*args,**kwargs)
        Path.read_text=slow_read
    def ping(self):return True

def make_slow_binding(options):return SlowBindingRuntime(options)

def test_execution_binding_keeps_servicing_during_slow_file_validation(tmp_path,monkeypatch):
    snapshot=tmp_path/'frame';snapshot.mkdir()
    (snapshot/'robot_capture.json').write_text(json.dumps({'session_id':'current'}))
    prepared=tmp_path/'prepared.json';prepared.write_text(json.dumps(dict(source_sha256={'color':'a'},label='eraser')))
    ticks=[];dispatch=[]
    backend=PipelineBackend(dict(mode='station'))
    backend.hardware=NS(profile={},session_id='current',execute=lambda ctx,checked:dispatch.append(checked))
    backend.tick=lambda:ticks.append(time.monotonic())
    station=NS(backend=backend,shutdown=threading.Event(),cancelled=threading.Event())
    ctx=Context(station,tmp_path)
    worker=ComputeWorker(make_slow_binding,{})
    backend.worker=worker
    try:
        ctx.wait(worker.submit('ping'))
        ticks.clear()
        original=Path.read_text
        def no_parent_read(path,*args,**kwargs):
            assert path!=prepared,'large observation file decoded on serial owner'
            return original(path,*args,**kwargs)
        monkeypatch.setattr(Path,'read_text',no_parent_read)
        started=time.monotonic()
        backend.execute(ctx,dict(bundle='fixture',prepared=str(prepared),snapshot=str(snapshot),prompt='eraser'))
        assert time.monotonic()-started>=.35 and len(ticks)>=10
        assert max(b-a for a,b in zip(ticks,ticks[1:]))<.25
        assert len(dispatch)==1 and dispatch[0][1]['worker_pid']!=os.getpid()
    finally:worker.close()
