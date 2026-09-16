import hashlib
from types import SimpleNamespace
import numpy as np
import pytest
from home_robot_tasks.detector import OnnxDetector

class Session:
    def get_inputs(self):return [SimpleNamespace(shape=[1,3,64,64],type='tensor(float)',name='rgb')]
    def run(self,_,feed):
        image=feed['rgb'];assert image.shape==(1,3,64,64) and image.dtype==np.float32
        assert image.min()>=0 and image.max()<=1
        return [self.output]

def detector(tmp_path):
    p=tmp_path/'fixture.onnx';p.write_bytes(b'synthetic-session-fixture')
    s=Session();s.output=np.array([[[32,33,10],[32,33,10],[20,20,4],[20,20,4],[.9,.8,.1]]],dtype=np.float32)
    d=OnnxDetector(p,hashlib.sha256(p.read_bytes()).hexdigest(),['remote_control'],size=64,session=s)
    return d,s,p

def test_letterbox_nms_and_explicit_model_hash(tmp_path):
    d,s,p=detector(tmp_path)
    result=d(np.zeros((32,64,3),np.uint8));assert len(result)==1
    assert result[0].object_id=='remote_control'
    assert result[0].corners_uv==((22.,6.),(42.,6.),(42.,26.),(22.,26.))
    with pytest.raises(ValueError):OnnxDetector(p,'0'*64,['remote_control'],size=64,session=s)

@pytest.mark.parametrize('fault',('nan','shape','score'))
def test_bad_model_output_fails_closed(tmp_path,fault):
    d,s,p=detector(tmp_path)
    if fault=='nan':s.output[0,0,0]=np.nan
    if fault=='shape':s.output=np.zeros((1,6,3))
    if fault=='score':s.output[0,4,0]=4
    with pytest.raises(ValueError):d(np.zeros((32,64,3),np.uint8))
