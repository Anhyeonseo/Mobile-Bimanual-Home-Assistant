import json
from pathlib import Path
import shutil
import numpy as np
import pytest
from home_robot_tasks.d415_geometry import nominal_model

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/home_assistant/d415_official_geometry.json'


def test_official_peripheral_dimensions_and_separate_optical_origins():
    m=nominal_model(CONFIG)
    np.testing.assert_allclose([m['width_m'],m['height_m'],m['depth_m']],[.099,.023,.02005])
    # Independent closed-form chain from the official screw mount frame.
    np.testing.assert_allclose(m['screw_from_color_optical'],
        [[0,0,1,0],[-1,0,0,.035],[0,-1,0,.0115],[0,0,0,1]],atol=1e-14)
    expected=np.eye(4);expected[0,3]=.015
    np.testing.assert_allclose(m['color_optical_from_depth_optical'],expected,atol=1e-14)
    np.testing.assert_allclose(m['rear_top_center_in_color_optical_m'],[.035,-.0115,-.01018],atol=1e-14)
    assert not m['physical_accuracy_verified']


def test_changed_upstream_source_is_rejected(tmp_path):
    shutil.copy(CONFIG,tmp_path/CONFIG.name)
    (tmp_path/'official_sources').mkdir()
    raw=(ROOT/'config/home_assistant/official_sources/realsense_d415.urdf.xacro').read_bytes()
    (tmp_path/'official_sources/realsense_d415.urdf.xacro').write_bytes(raw+b'\n')
    with pytest.raises(ValueError,match='hash mismatch'):nominal_model(tmp_path/CONFIG.name)


def test_official_mesh_bounds_and_provenance():
    m=nominal_model(CONFIG);mesh=json.loads((ROOT/'tools/diagnostics/s2_viewer/d415-mesh.json').read_text())
    assert mesh['source_sha256']==m['mesh_sha256']
    assert mesh['bounds_preserved'] and mesh['original_triangles']==425160
    # STL rounded ends are slightly inside the nominal 99 mm case envelope.
    np.testing.assert_allclose(mesh['span'],[.099,.023,.02005],atol=.0002)
    np.testing.assert_allclose(mesh['min'][1:],[-.0115,-.02005],atol=1e-8)
