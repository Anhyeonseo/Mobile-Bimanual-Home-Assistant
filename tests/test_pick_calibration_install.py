"""Installation path regressions; real directory copies, no robot or SSH access."""
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from tools.diagnostics.install_pick_calibration import (
    choose_source, choose_destination, install, PREFERRED_RELEASE,
)


@pytest.fixture
def fixture(tmp_path):
    package, releases = tmp_path/'package', tmp_path/'releases'
    source = releases/PREFERRED_RELEASE
    files = {
        'config/home_assistant/example.json': (b'old calibration', b'new calibration'),
        'tools/run/unified_pick.py': (b'original code', b'calibration connection'),
        'tools/diagnostics/extrinsic_capture/apply_measured_calibration.py':
            (None, b'print("loader checked")\n'),
    }
    manifest = dict(files={}, archive_names=['example.json'])
    for name, (before, after) in files.items():
        payload = package/'payload'/name
        payload.parent.mkdir(parents=True, exist_ok=True)
        payload.write_bytes(after)
        if before is not None:
            current = source/name
            current.parent.mkdir(parents=True, exist_ok=True)
            current.write_bytes(before)
        manifest['files'][name] = dict(sha256=hashlib.sha256(after).hexdigest(),
            before_sha256=hashlib.sha256(before).hexdigest() if before is not None else None)
    (package/'overlay.json').write_text(json.dumps(manifest))
    (source/'unrelated-setting.txt').write_text('preserve me')
    return package, releases, source, manifest


def test_default_destination_collision_creates_new_copy_and_preserves_both_existing_trees(fixture):
    package, releases, source, manifest = fixture
    existing = source.with_name(source.name+'-cal1117')
    existing.mkdir(); (existing/'partial-result.txt').write_text('keep this too')
    destination = choose_destination(source)
    result = install(package, choose_source(package, releases, manifest), destination)
    assert result['status'] == 'CALIBRATION_COPY_READY'
    assert destination != existing and destination.is_dir()
    assert (existing/'partial-result.txt').read_text() == 'keep this too'
    assert (source/'config/home_assistant/example.json').read_bytes() == b'old calibration'
    assert (destination/'config/home_assistant/example.json').read_bytes() == b'new calibration'
    assert (destination/'unrelated-setting.txt').read_text() == 'preserve me'
    assert (destination/'config/home_assistant/historical/pre-extrinsic-20261117/example.json').read_bytes() == b'old calibration'


def test_missing_historical_path_discovers_the_unique_exact_match(fixture):
    package, releases, source, manifest = fixture
    actual = source.with_name('actual-pick-release')
    source.rename(actual)
    assert choose_source(package, releases, manifest) == actual


def test_multiple_matching_sources_require_explicit_selection(fixture):
    package, releases, source, manifest = fixture
    actual = source.with_name('first'); source.rename(actual)
    second = releases/'second'; shutil.copytree(actual, second)
    with pytest.raises(ValueError, match='multiple matching installations'):
        choose_source(package, releases, manifest)
    assert choose_source(package, releases, manifest, explicit=second) == second


def test_changed_preferred_source_cannot_silently_fall_back_to_an_older_release(fixture):
    package, releases, source, manifest = fixture
    shutil.copytree(source, releases/'older')
    (source/'tools/run/unified_pick.py').write_text('other work')
    with pytest.raises(ValueError, match='installation differs'):
        choose_source(package, releases, manifest)
    assert (source/'tools/run/unified_pick.py').read_text() == 'other work'


def test_missing_source_is_reported_by_path_and_does_not_create_destination(fixture):
    package, releases, source, manifest = fixture
    missing = releases/'missing'
    with pytest.raises(ValueError, match='source directory not found: .*missing'):
        install(package, missing, releases/'destination')
    assert not (releases/'destination').exists()


def test_explicit_existing_destination_is_preserved(fixture):
    package, releases, source, manifest = fixture
    destination = releases/'existing'; destination.mkdir()
    (destination/'keep').write_text('existing work')
    with pytest.raises(ValueError, match='destination already exists'):
        install(package, source, destination)
    assert list(destination.iterdir()) == [destination/'keep']
    assert (destination/'keep').read_text() == 'existing work'


def test_nested_destination_is_refused_before_copy(fixture):
    package, releases, source, manifest = fixture
    with pytest.raises(ValueError, match='outside the source'):
        install(package, source, source/'nested')
    assert not (source/'nested').exists()


def test_dangling_destination_symlink_is_not_followed(fixture):
    package, releases, source, manifest = fixture
    link = releases/'link'; target = releases/'absent'; link.symlink_to(target)
    with pytest.raises(ValueError, match='destination already exists'):
        install(package, source, link)
    assert link.is_symlink() and not target.exists()


def test_tampered_payload_is_refused_before_copy(fixture):
    package, releases, source, manifest = fixture
    (package/'payload/tools/run/unified_pick.py').write_text('modified payload')
    with pytest.raises(ValueError, match='overlay bytes changed'):
        install(package, source, releases/'new')
    assert not (releases/'new').exists()
