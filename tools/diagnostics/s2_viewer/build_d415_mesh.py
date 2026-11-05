#!/usr/bin/env python3
"""Rebuild the D415 display derivative from the pinned official STL."""
import argparse
import hashlib
import json
from pathlib import Path
from build_meshes import ROOT, pack


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mesh',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    manifest=json.loads((ROOT/'config/home_assistant/d415_official_geometry.json').read_text())
    if hashlib.sha256(a.mesh.read_bytes()).hexdigest()!=manifest['mesh']['sha256']:
        raise SystemExit('D415 source mesh hash mismatch')
    a.output.write_text(json.dumps(pack(a.mesh,16000,preserve_bounds=True),separators=(',',':'))+'\n')
