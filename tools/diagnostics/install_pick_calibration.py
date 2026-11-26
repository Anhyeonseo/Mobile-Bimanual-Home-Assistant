#!/usr/bin/env python3
"""Apply a packaged calibration overlay to a NEW runtime copy. No device access.

The adjacent overlay.json binds every replacement to its exact before/after
bytes. Run only from the generated package, not directly from the repository.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

DEFAULT_RELEASES = Path('/home/hyper/robot/releases')
PREFERRED_RELEASE = 's4-unified-pick-20261116'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_files(package, source, manifest):
    """Check replacements without changing the source or creating a destination."""
    if not source.is_dir():
        raise ValueError('source directory not found: '+str(source))
    selected = []
    for name, record in manifest['files'].items():
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('invalid overlay path')
        payload, current = package/'payload'/relative, source/relative
        if digest(payload) != record['sha256']:
            raise ValueError('overlay bytes changed: '+name)
        if record.get('optional_existing') and not current.exists():
            continue  # Older releases do not have the resident pick service.
        if record['before_sha256'] is None:
            if current.exists() and digest(current) != record['sha256']:
                raise ValueError('new file conflicts with installation: '+name)
        elif not current.is_file() or digest(current) != record['before_sha256']:
            raise ValueError('installation differs; no files copied: '+name)
        selected.append(name)
    return selected


def choose_source(package, releases_root, manifest, explicit=None):
    if explicit is not None:
        source = Path(explicit).resolve()
        selected_files(package, source, manifest)
        return source
    releases_root = Path(releases_root).resolve()
    preferred = releases_root/PREFERRED_RELEASE
    if preferred.exists() or preferred.is_symlink():
        # Never silently choose an older release when the known one changed.
        return choose_source(package, releases_root, manifest, explicit=preferred)
    candidates, rejected = set(), []
    if releases_root.is_dir():
        for entry in sorted(releases_root.iterdir()):
            if not entry.is_dir() or '-cal1117' in entry.name:
                continue
            if not (entry/'tools/run/unified_pick.py').is_file():
                continue
            try:
                selected_files(package, entry, manifest)
            except (ValueError, OSError) as error:
                rejected.append(str(entry)+': '+str(error))
            else:
                candidates.add(entry.resolve())
    if len(candidates) == 1:
        return candidates.pop()
    if candidates:
        raise ValueError('multiple matching installations; specify --source:\n'+
                         '\n'.join(map(str, sorted(candidates))))
    raise ValueError('source directory not found: '+str(preferred)+
                     '\nNo compatible pick installation under '+str(releases_root)+
                     ('\n'+'\n'.join(rejected) if rejected else '')+
                     '\nSpecify the actual installation with --source; no files copied.')


def choose_destination(source, explicit=None):
    if explicit is not None:
        return Path(explicit).absolute()
    destination = source.with_name(source.name+'-cal1117')
    if destination.exists() or destination.is_symlink():
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        destination = source.with_name(source.name+'-cal1117-'+stamp+'-'+uuid.uuid4().hex[:8])
    return destination


def install(package, source, destination):
    package, source = (Path(p).resolve() for p in (package, source))
    destination = Path(destination).absolute()
    manifest = json.loads((package/'overlay.json').read_text())
    if not source.is_dir():
        raise ValueError('source directory not found: '+str(source))
    if destination.exists() or destination.is_symlink():
        raise ValueError('destination already exists; preserved without changes: '+str(destination))
    if source == destination.resolve() or source in destination.resolve().parents:
        raise ValueError('destination must be outside the source installation: '+str(destination))
    selected = selected_files(package, source, manifest)
    # Copy symlink contents too, so no write can follow a link back into source.
    shutil.copytree(source, destination, symlinks=False)
    archive = destination/'config/home_assistant/historical/pre-extrinsic-20261117'
    archive.mkdir(parents=True, exist_ok=True)
    for name in manifest['archive_names']:
        old, saved = source/'config/home_assistant'/name, archive/name
        if saved.exists() and digest(saved) != digest(old):
            raise ValueError('existing historical backup differs: '+name)
        shutil.copy2(old, saved)
    for name in selected:
        target = destination/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(package/'payload'/name, target)
    subprocess.run([sys.executable, str(destination/'tools/diagnostics/extrinsic_capture/apply_measured_calibration.py'),
                    '--check-only'], cwd=destination, check=True)
    (destination/'calibration_overlay.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return dict(status='CALIBRATION_COPY_READY', destination=str(destination),
                original_unchanged=True, hardware_commands=0, replaced_files=selected)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, help='explicit original installation; otherwise discover an exact match')
    parser.add_argument('--destination', type=Path, help='new copy; otherwise choose an unused -cal1117 sibling')
    parser.add_argument('--releases-root', type=Path, default=DEFAULT_RELEASES)
    args = parser.parse_args()
    try:
        package = Path(__file__).resolve().parent
        manifest = json.loads((package/'overlay.json').read_text())
        source = choose_source(package, args.releases_root, manifest, args.source)
        destination = choose_destination(source, args.destination)
        print('SOURCE: '+str(source), flush=True)
        print('DESTINATION: '+str(destination), flush=True)
        print(json.dumps(install(package, source, destination), indent=2))
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print('DEPLOYMENT_BLOCKED: '+str(error), file=sys.stderr)
        sys.exit(2)
