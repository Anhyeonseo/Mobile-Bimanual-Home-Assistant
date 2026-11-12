import hashlib
import json
from types import SimpleNamespace
import numpy as np
import pytest
from tools.run.check_lds01_driver import RGBDArchive, TeleopCompletion


def header():
    return json.dumps(dict(event='session',firmware=dict(firmware='mobile-base-hold-teleop-v7')))+'\n'


def test_completion_waits_for_full_record_and_preserves_failed_stop(tmp_path):
    path=tmp_path/'drive.jsonl';path.write_text(header())
    watcher=TeleopCompletion(path)
    with path.open('a') as f:f.write('{"event":"finished",')
    assert watcher.poll() is None
    with path.open('a') as f:f.write('"error":"STOP failed"}\n')
    assert watcher.poll()['error']=='STOP failed'
    with pytest.raises(ValueError,match='open'):TeleopCompletion(path)


def test_old_or_truncated_teleop_is_not_a_new_capture(tmp_path):
    path=tmp_path/'drive.jsonl';path.write_text(header())
    watcher=TeleopCompletion(path);path.write_text('')
    with pytest.raises(ValueError,match='truncated'):watcher.poll()
    path.write_text('{"event":"reply"}\n')
    with pytest.raises(ValueError,match='identified'):TeleopCompletion(path)


def test_archive_keeps_native_pixels_and_acquisition_clock(tmp_path):
    archive=RGBDArchive(tmp_path,.5)
    color=np.arange(18,dtype=np.uint8).reshape(2,3,3)
    depth=np.array([[0,65535,123],[400,500,600]],dtype=np.uint16)
    record=dict(capture_id='a',color_time_s=100.,depth_time_s=100.01,
                color_frame={'number':1},depth_frame={'number':2})
    metadata={'depth_scale_m':.001}
    assert archive.add(record,color,depth,metadata)
    assert not archive.add({**record,'depth_time_s':100.4},color,depth,metadata)
    assert archive.add({**record,'depth_time_s':100.6},color,depth,metadata)
    rows=[json.loads(x) for x in (tmp_path/'rgbd/frames.jsonl').read_text().splitlines()]
    assert len(rows)==2 and rows[0]['depth_time_s']==100.01
    file=tmp_path/'rgbd'/rows[0]['file']
    assert hashlib.sha256(file.read_bytes()).hexdigest()==rows[0]['sha256']
    with np.load(file) as data:
        np.testing.assert_array_equal(data['color_bgr'],color)
        np.testing.assert_array_equal(data['native_depth'],depth)
        assert data['native_depth'].dtype==np.uint16
    assert json.loads((tmp_path/'rgbd/metadata.json').read_text())==metadata


def test_disk_reserve_failure_does_not_publish_partial_frame(tmp_path,monkeypatch):
    archive=RGBDArchive(tmp_path,.5)
    monkeypatch.setattr('tools.run.check_lds01_driver.shutil.disk_usage',lambda _:SimpleNamespace(free=100))
    with pytest.raises(RuntimeError,match='reserve'):
        archive.add({'depth_time_s':1},np.zeros((1,1,3)),np.zeros((1,1)),{})
    assert not list((tmp_path/'rgbd').iterdir())
