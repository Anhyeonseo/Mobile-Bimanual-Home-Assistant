"""Near pick capture remains a single raw exposure with unchanged geometry."""
from types import SimpleNamespace as NS
import json
import numpy as np
import pytest
from home_robot_tasks.pick_web_backend import WebPickBackend

@pytest.mark.parametrize('mode,expected',[('near',8000),('wide',16000),('legacy',None)])
def test_camera_capture_requests_fixed_exposure_and_saves_one_untouched_pair(monkeypatch,tmp_path,mode,expected):
 from home_robot_tasks import camera_evidence,s2_rgbd_acquisition
 calls=[];color=np.full((4,5,3),85,np.uint8);depth=np.arange(20,dtype=np.uint16).reshape(4,5)
 metadata={'depth_frame':{'settings':{'actual_exposure':expected}},'test_geometry':'unchanged'}
 class Camera:
  def __init__(self,profile,reference,**options):
   assert profile=={'camera_device':'same'} and reference=='reference'
   assert options==dict(retain_frame=True,depth_exposure_us=expected)
  def __enter__(self):return self
  def __exit__(self,*args):calls.append('closed')
  def next_frame(self):calls.append('frame');return {'capture_id':'rgbd-test'},(color,depth,metadata)
 monkeypatch.setattr(camera_evidence,'load_capture_profile',lambda root,mode:({'camera_device':'same'},'reference'))
 monkeypatch.setattr(s2_rgbd_acquisition,'Capture',Camera)
 ctx=NS(progress=lambda *a:None,check=lambda:None,folder=tmp_path,artifact=lambda *a:None)
 result=WebPickBackend({'mode':'station'})._camera_capture(ctx,mode)
 from PIL import Image
 from pathlib import Path
 path=Path(result['snapshot'])
 assert calls==['frame','closed']
 np.testing.assert_array_equal(np.asarray(Image.open(path/'depth_aligned.png')),depth)
 assert json.loads((path/'metadata.json').read_text())==metadata
