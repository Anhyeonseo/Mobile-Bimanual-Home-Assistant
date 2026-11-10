"""A Home change must not silently reuse measured pads from old coordinates."""
import json
from pathlib import Path

import pytest

from tools.diagnostics import replay_s3_grasp as replay


def test_stale_default_measurements_fail_closed_until_nominal_is_explicit(tmp_path, monkeypatch):
    monkeypatch.setattr(replay, 'CONFIG', tmp_path)
    stale = {'schema_version': 1, 'arm': 'left', 'motor_id': 6,
             'section': 'installed_left_front_opposing_contact_faces',
             'status': 'CHECK_PASSED', 'calibration_id': 'old-home',
             'assembly_revision': 'same-assembly'}
    (tmp_path / 'left_pad_gap.observed.json').write_text(json.dumps(stale))
    profile = {'calibration_id': 'new-home', 'assembly_revision': 'same-assembly'}
    with pytest.raises(ValueError, match='verified pad calibration'):
        replay.installed_jaw_for_replay(profile)
    assert replay.installed_jaw_for_replay(profile, nominal_jaw=True) is None
    assert json.loads((tmp_path / 'left_pad_gap.observed.json').read_text()) == stale
    with pytest.raises(ValueError, match='not both'):
        replay.installed_jaw_for_replay(profile, stale, nominal_jaw=True)
