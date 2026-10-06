#!/usr/bin/env python3
"""Verify frozen source bytes and installed flight resources; record binaries."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def verify(root, manifest_file):
    root=Path(root);manifest=json.loads(Path(manifest_file).read_text());files=manifest['files']
    differences=[name for name,digest in files.items() if sha(root/name)!=digest]
    installed=root/'ros2_ws/install/annual_swarm';verified={};binaries={}
    for file in sorted(installed.rglob('*')):
        if not file.is_file() or '__pycache__' in str(file):continue
        name=str(file.relative_to(installed));source=None
        if name.startswith('lib/annual_swarm/legacy/'):
            source='next_project/'+name[len('lib/annual_swarm/legacy/'):]
        elif name.startswith('lib/annual_swarm/') and file.suffix=='.py':
            source='ros2_ws/src/annual_swarm/scripts/'+file.name
        elif name.startswith('share/annual_swarm/maps/'):
            source='next_project/maps/'+name[len('share/annual_swarm/maps/'):]
        elif name.startswith('share/annual_swarm/'):
            relative=name[len('share/annual_swarm/'):];source='ros2_ws/src/annual_swarm/'+relative
            if source not in files and relative.startswith('launch/'):
                source='ros2_ws/src/annual_swarm/scripts/'+file.name
        if source in files:
            digest=sha(file);verified[name]=dict(source=source,sha256=digest)
            if digest!=files[source]:differences.append('installed:'+name)
        elif name in ('lib/annual_swarm/controller_node','lib/annual_swarm/planner_node','lib/libannual_motor_system.so'):
            binaries[name]=sha(file)
    required=['lib/annual_swarm/decentralized_agent_node.py','lib/annual_swarm/view_executor_node.py',
              'lib/annual_swarm/pointcloud_mapping_node.py','share/annual_swarm/config/flight.yaml']
    for name in required:
        if name not in verified:differences.append('unverified required resource:'+name)
    if len(binaries)!=3:differences.append('Missing compiled flight binaries')
    return dict(passed=not differences,frozen_source_files=len(files),verified_installed_files=len(verified),
                differences=differences,files=verified,binaries=binaries,
                scope='Installed source resources match snapshot bytes. Binary hashes identify this build; '
                      'different build paths need not produce byte-identical binaries.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root',required=True);parser.add_argument('--manifest',required=True)
    parser.add_argument('--output',required=True);args=parser.parse_args();result=verify(args.root,args.manifest)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ('passed','frozen_source_files','verified_installed_files','differences')}))
    raise SystemExit(0 if result['passed'] else 1)
