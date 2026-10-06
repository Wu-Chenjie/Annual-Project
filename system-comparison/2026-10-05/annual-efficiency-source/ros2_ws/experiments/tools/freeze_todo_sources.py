#!/usr/bin/env python3
"""Capture all flight sources and experiment tools before serial validation."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile


def freeze(root, prefix, phase, description):
    root = Path(root).resolve(); prefix = Path(prefix).resolve()
    manifest_file = prefix.with_name(prefix.name+'-source-manifest.json')
    archive_file = prefix.with_name(prefix.name+'-source.tar.gz')
    if manifest_file.exists() or archive_file.exists():
        raise FileExistsError('A frozen snapshot cannot be overwritten')
    folders = ('next_project/core', 'next_project/cpp', 'next_project/maps', 'ros2_ws/src',
               'ros2_ws/experiments', 'docker')
    paths = sorted(p for folder in folders for p in (root/folder).rglob('*')
        if p.is_file() and not any(part in ('__pycache__', '.pytest_cache', 'build', 'install', 'log') for part in p.parts))
    files = {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    manifest = dict(base_commit=commit, captured_at=datetime.now(timezone.utc).isoformat(),
        includes_uncommitted_changes=bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=root)),
        phase=phase, description=description, files=files)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive_file, 'w:gz') as archive:
        for path in paths:
            archive.add(path, arcname=str(path.relative_to(root)), recursive=False)
    manifest['archive_sha256'] = hashlib.sha256(archive_file.read_bytes()).hexdigest()
    manifest_file.write_text(json.dumps(manifest, indent=2)+'\n')
    return dict(manifest=str(manifest_file), archive=str(archive_file), files=len(files),
                archive_sha256=manifest['archive_sha256'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root', required=True);parser.add_argument('--prefix', required=True)
    parser.add_argument('--phase', required=True);parser.add_argument('--description', required=True)
    args=parser.parse_args();print(json.dumps(freeze(args.root,args.prefix,args.phase,args.description)))
