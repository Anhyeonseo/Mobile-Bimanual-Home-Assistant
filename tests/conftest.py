"""Synthetic stationary-lab scenes were designed against the nominal camera/arm profile.

The released profile now carries the measured 2026-11-17 calibration (camera moved ~2 cm, arm zeros,
lift scale). Those fixtures test planning logic, not calibration, so for the whole session they keep using
the archived nominal profile and the jaw record bound to it. Tests of the calibration itself read the
released files directly.
"""
from pathlib import Path
import pytest

ARCHIVE = Path(__file__).resolve().parents[1]/'config/home_assistant/historical/pre-extrinsic-20261117'


@pytest.fixture(autouse=True, scope='session')
def _nominal_profile_for_synthetic_lab():
    try:
        from tools.diagnostics import run_s3_stationary_lab as lab
    except Exception:
        yield
        return
    patch = pytest.MonkeyPatch()
    patch.setattr(lab, 'PROFILE', ARCHIVE/'s2_coordinates.development.json')
    patch.setattr(lab, 'PAD_RECORD', ARCHIVE/'left_pad_gap.observed.json')
    yield
    patch.undo()
