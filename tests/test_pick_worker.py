from contextlib import nullcontext
import hashlib
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.pick_worker import ComputeWorker
from test_pick_service import wait_for


class EchoRuntime:
    def __init__(self,_):self.count=0
    def count_request(self):self.count+=1;return self.count
    def slow(self):time.sleep(.3);return 'late'
    def close(self):pass


def make_echo(options):return EchoRuntime(options)


def test_spawned_worker_reuses_runtime_and_cleans_up_pending_job():
    worker=ComputeWorker(make_echo,{})
    try:
        for expected in (1,2):
            job=worker.submit('count_request')
            wait_for(job.done)
            assert job.result()==expected
        job=worker.submit('slow')
        with pytest.raises(RuntimeError,match='already'):
            worker.submit('count_request')
        worker.close()
        assert not worker.process.is_alive()
        with pytest.raises(RuntimeError,match='unavailable'):
            worker.submit('count_request')
    finally:worker.close()


def test_explicit_graspnet_runtime_loads_weights_once_for_two_inferences(tmp_path,monkeypatch):
    from home_perception.grasp import graspnet
    checkpoint=tmp_path/'weights';checkpoint.write_bytes(b'verified test weights')
    counts={'loads':0,'net':0}
    class Net:
        def __init__(self,**kwargs):counts['net']+=1
        def load_state_dict(self,state,strict):assert state=={'weights':True} and strict
        def to(self,device):assert device=='cuda';return self
        def eval(self):return self
        def __call__(self,value):return value
    def load(*args,**kwargs):counts['loads']+=1;return {'model_state_dict':{'weights':True}}
    fake=NS(load=load,manual_seed=lambda _:None,as_tensor=lambda value,**_:value,
            float32='float32',inference_mode=nullcontext)
    rows=np.ones((3,17))
    monkeypatch.setitem(sys.modules,'torch',fake)
    monkeypatch.setattr(graspnet,'CHECKPOINT_SHA256',hashlib.sha256(checkpoint.read_bytes()).hexdigest())
    monkeypatch.setattr(graspnet,'_baseline',lambda device,audit:(Net,lambda _:[NS(cpu=lambda:NS(numpy=lambda:rows))],object))
    runtime=graspnet.GraspNetRuntime(checkpoint)
    for _ in range(2):assert runtime(np.ones((200,3)),checkpoint)[0] is rows
    assert counts=={'loads':1,'net':1}
    with pytest.raises(ValueError,match='mismatch'):runtime(np.ones((200,3)),tmp_path/'different')
    with pytest.raises(ValueError,match='nonempty'):runtime(np.empty((0,3)),checkpoint)
