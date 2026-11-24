#!/usr/bin/env python3
"""Package the lift console's Python import closure. No robot access/deployment.

Preserve project paths in a content-addressed directory; configuration remains
in the installed Jetson release and is never included/overwritten by this tool.
Only pyserial is required beyond Python's standard library for this closure.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[2]


def collect(root):
    roots = [root/'tools/run', root/'ros2_ws/src/so101_arm_bridge', root/'ros2_ws/src/home_robot_tasks']
    def locate(name):
        for base in roots:
            path = base.joinpath(*name.split('.'))
            if path.with_suffix('.py').is_file():
                return path.with_suffix('.py'), False
            if (path/'__init__.py').is_file():
                return path/'__init__.py', True
        return None
    modules = {}; files = {}; external = set(); pending = ['unified_lift']
    while pending:
        name = pending.pop()
        if name in modules:
            continue
        found = locate(name)
        if found is None:
            external.add(name.split('.')[0]); continue
        path, is_package = found
        relative = path.relative_to(root).as_posix()
        encoded = path.read_bytes()
        modules[name] = relative; files[relative] = encoded
        if '.' in name:
            pending.append(name.rsplit('.', 1)[0])
        for node in ast.walk(ast.parse(encoded, filename=relative)):
            if isinstance(node, ast.Import):
                pending.extend(item.name for item in node.names)
            elif isinstance(node, ast.ImportFrom):
                module = node.module or ''
                if node.level:
                    package = name if is_package else name.rsplit('.', 1)[0]
                    module = importlib.util.resolve_name('.'*node.level+module, package)
                if module:
                    pending.append(module)
                for item in node.names:
                    child = module+'.'+item.name
                    if item.name != '*' and locate(child):
                        pending.append(child)
    return modules, files, sorted(external)


def build(root, output):
    modules, files, external = collect(root)
    digests = {name:hashlib.sha256(value).hexdigest() for name,value in sorted(files.items())}
    identity = hashlib.sha256(json.dumps(digests, sort_keys=True).encode()).hexdigest()[:12]
    name = 'unified-lift-'+identity
    manifest = dict(schema_version=1, bundle_id=identity, entrypoint='tools/run/unified_lift.py',
                    modules=dict(sorted(modules.items())), files_sha256=digests,
                    external_import_roots=external, includes_configuration=False)
    files['runtime_manifest.json'] = (json.dumps(manifest, indent=2)+'\n').encode()
    folder = output/name
    folder.mkdir(parents=True, exist_ok=True)
    for relative, data in files.items():
        target = folder/relative
        if target.exists() and target.read_bytes()!=data:
            raise ValueError('refusing changed existing bundle file: '+str(target))
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    archive = output/(name+'.tar.gz')
    with tarfile.open(archive, 'w:gz') as tar:
        for relative in sorted(files):
            source = folder/relative
            info = tar.gettarinfo(str(source), arcname=name+'/'+relative)
            info.uid=info.gid=0;info.uname=info.gname='';info.mtime=0
            with source.open('rb') as content:
                tar.addfile(info, content)
    return dict(folder=str(folder), archive=str(archive), bundle_id=identity,
                module_count=len(modules), archive_bytes=archive.stat().st_size)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/jetson-remote-test-20261117/deployment')
    args=parser.parse_args()
    print(json.dumps(build(ROOT,args.output.resolve()), indent=2))
