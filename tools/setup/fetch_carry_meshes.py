#!/usr/bin/env python3
"""Fetch only the three pinned Mini 1 collision meshes; verify existing files too."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
FILES = ('base_link.STL', 'vertical_link.STL', 'wheel1.STL')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    geometry = json.loads((ROOT/'config/home_assistant/official_geometry.json').read_text())
    source = next(s for s in geometry['sources'] if s['file'] == 'alohamini1.urdf')
    if source['repository'] != 'liyiteng/AlohaMini':
        raise ValueError('unexpected Mini 1 source repository')
    records = json.loads((ROOT/'tools/diagnostics/s2_viewer/meshes.json').read_text())['meshes']
    args.output.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        path = args.output/name
        if path.exists():
            raw = path.read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/{source['repository']}/{source['commit']}/AlohaMini1/simulation/src/Aloha/meshes/{name}"
            with urlopen(url, timeout=45) as response:
                raw = response.read(16*1024*1024+1)
        if hashlib.sha256(raw).hexdigest() != records[name]['source_sha256']:
            raise ValueError('official mesh hash mismatch: '+name)
        if not path.exists():
            path.write_bytes(raw)
        print('verified', name)


if __name__ == '__main__':
    main()
