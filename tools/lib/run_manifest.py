"""Small, dependency-free provenance records for local development runs."""
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def provenance():
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=ROOT).decode().strip()
    try:
        names = git('ls-files', '--cached', '--others', '--exclude-standard', '-z').split('\0')
        sources = {name: sha256(ROOT / name) for name in sorted(set(names))
                   if name and (ROOT / name).is_file()}
        head, dirty = git('rev-parse', 'HEAD'), bool(git('status', '--porcelain'))
    except (OSError, subprocess.CalledProcessError):
        head, dirty, sources = None, None, {}
    packages = {}
    for name in ('numpy', 'torch', 'transformers', 'open3d', 'scipy', 'Pillow'):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return {'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'python': sys.version, 'executable': sys.executable, 'platform': platform.platform(),
            'git_head': head, 'git_dirty': dirty, 'source_sha256': sources, 'packages': packages}
