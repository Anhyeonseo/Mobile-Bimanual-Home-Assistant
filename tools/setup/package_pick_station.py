"""Build a self-contained station directory for local testing and Jetson staging."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.grasp.graspnet_assets import BASELINE_FILES, CHECKPOINT_SHA256, verify_baseline
from tools.lib.pick_readiness import model_snapshots
from tools.setup.fetch_carry_meshes import FILES as MESHES


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def archive(kit, temporary, parent):
    """Bundle only manifest-listed inputs, even when the kit has been run."""
    target = temporary/'pick-station.tar.gz'
    manifest = kit/'SHA256SUMS'
    with tarfile.open(target, 'w:gz', compresslevel=1) as bundle:
        bundle.add(manifest, 'pick-station/SHA256SUMS', recursive=False)
        for entry in manifest.read_text().splitlines():
            name = entry.split('  ', 1)[1]
            bundle.add(kit/name, 'pick-station/'+name, recursive=False)
    checksum = temporary/'pick-station.tar.gz.sha256'
    checksum.write_text(f'{digest(target)}  {target.name}\n')
    target.replace(parent/target.name)
    checksum.replace(parent/checksum.name)


def build(args, kit):
    # Include the working checkout (also new station modules), honour Git's
    # exclusions and select only runtime directories plus the operator manual.
    paths = ('config', 'ros2_ws/src/home_robot_tasks', 'ros2_ws/src/so101_arm_bridge',
             'ros2_ws/src/so101_description', 'home_perception', 'protocol',
             'tools/lib', 'tools/run', 'tools/diagnostics', 'tools/__init__.py',
             'docs/pick_station_tutorial.md', 'docs/PICK_PIPELINE_INTEGRATION.md',
             'docs/THIRD_PARTY_NOTICES.md', 'LICENSE')
    listing = subprocess.check_output(['git','ls-files','--cached','--others',
                                      '--exclude-standard','-z','--',*paths], cwd=ROOT)
    def copy(source, relative):
        dest = kit/relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
    for name in os.fsdecode(listing).split('\0'):
        if name and (ROOT/name).is_file():
            copy(ROOT/name, name)
    records = json.loads((ROOT/'tools/diagnostics/s2_viewer/meshes.json').read_text())['meshes']
    for name in MESHES:
        source = args.meshes/name
        if digest(source) != records[name]['source_sha256']:
            raise ValueError(f'mobile mesh hash mismatch: {source}')
        copy(source, Path('assets/mobile_meshes')/name)
    baseline = args.artifacts/'graspnet-baseline'
    verify_baseline(baseline)
    for name in BASELINE_FILES:
        copy(baseline/name, Path('artifacts/graspnet-baseline')/name)
    weights = Path('models/checkpoint-rs.tar')
    if digest(args.artifacts/weights) != CHECKPOINT_SHA256:
        raise ValueError('unexpected GraspNet checkpoint digest')
    copy(args.artifacts/weights, Path('artifacts')/weights)
    for relative in model_snapshots(ROOT):
        directory = args.artifacts/relative
        for required in ('model.safetensors', 'config.json', 'preprocessor_config.json'):
            if not (directory/required).is_file():
                raise FileNotFoundError(directory/required)
        for path in sorted(directory.rglob('*')):
            if path.is_file():
                copy(path, Path('artifacts')/relative/path.relative_to(directory))
    for name in ('color.png', 'depth_aligned.png', 'metadata.json'):
        copy(args.snapshot/name, Path('examples/remote/cam')/name)
    manifest = kit/'SHA256SUMS'
    manifest.write_text(''.join(f'{digest(p)}  {p.relative_to(kit)}\n'
                                for p in sorted(kit.rglob('*')) if p.is_file()))
    return digest(manifest)


def main():
    source = Path(os.environ.get('GRASP_SOURCE_REPO', Path.home()/'Documents/GitHub/SO101-Can-Towel-Manipulation'))
    workspace = source/'workspaces/home-assistant'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts', type=Path, default=workspace/'artifacts')
    parser.add_argument('--meshes', type=Path, default=workspace/'tmp/s2-official/AlohaMini/AlohaMini1/simulation/src/Aloha/meshes')
    parser.add_argument('--snapshot', type=Path, default=ROOT/'output/jetson-remote-test-20261117/cam')
    parser.add_argument('--output', type=Path, default=ROOT/'output/pick-kit')
    parser.add_argument('--archive', action='store_true', help='also write a transferable .tar.gz and SHA-256 file')
    args = parser.parse_args()
    parent = args.output.resolve()
    parent.mkdir(exist_ok=True, parents=True)
    with tempfile.TemporaryDirectory(dir=parent) as tmp:
        staging = Path(tmp)/'station'
        identity = build(args, staging)
        final = parent/('station-'+identity[:16])
        if final.exists():
            if digest(final/'SHA256SUMS') != identity:
                raise ValueError(f'existing kit manifest mismatch: {final}')
            subprocess.run(['sha256sum', '--quiet', '-c', 'SHA256SUMS'], cwd=final, check=True)
        else:
            staging.rename(final)
        link = Path(tmp)/'current'
        link.symlink_to(final.name)
        link.replace(parent/'current')
        if args.archive:
            archive(final, Path(tmp), parent)
    print('KIT_READY', final)
    if args.archive:
        print('ARCHIVE_READY', parent/'pick-station.tar.gz')


if __name__ == '__main__':
    main()
