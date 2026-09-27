#!/usr/bin/env python3
"""Freeze an external diagnostic recorder without editing the flight snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def prepare(harness, destination):
    harness = Path(harness); destination = Path(destination); destination.mkdir(parents=True, exist_ok=False)
    for file in harness.glob('*.py'):
        shutil.copy2(file, destination/file.name)
    external = Path(__file__).parent
    names = ['dynamic_backup_fault.launch.py', 'dynamic_backup_probe.py', 'audit_dynamic_backup.py',
             'test_dynamic_backup_probe.py', 'test_dynamic_launch.py']
    for name in names: shutil.copy2(external/name, destination/name)
    recorder = destination/'record_todo_demo.py'; source = recorder.read_text()
    anchor = "command=['ros2','launch','annual_swarm','decentralized_search.launch.py',"
    replacement = "command=['ros2','launch',str(Path(__file__).with_name('dynamic_backup_fault.launch.py')),"
    if source.count(anchor) != 1: raise ValueError('Unknown recorder; do not alter arbitrary launch commands')
    source = source.replace(anchor, replacement)
    # The stock no-fault recorder disables the physical cylinder as well as its
    # scripted motion. This external test needs that model and its Gazebo pose
    # adapter; the diagnostic launch already disables only the stock schedule.
    anchor = "'dynamic_obstacle:=false'"
    if source.count(anchor) != 1: raise ValueError('Unknown recorder cylinder switch')
    source = source.replace(anchor, "'dynamic_obstacle:=true'")
    anchor = "shutil.copy2(__file__,out/'record-run.py')"
    if source.count(anchor) != 1: raise ValueError('Unknown recorder evidence setup')
    source = source.replace(anchor, anchor+"\n    shutil.copy2(Path(__file__).with_name('external-test-manifest.json'),out/'external-test-manifest.json')\n    for name in ('dynamic_backup_fault.launch.py','dynamic_backup_probe.py','audit_dynamic_backup.py'):shutil.copy2(Path(__file__).with_name(name),out/name)")
    recorder.write_text(source)
    manifest = dict(test_case='physical_dynamic_cached_backup', flight_policy='Unmodified frozen combined snapshot',
        difference='Keep the physical cylinder and Gazebo pose adapter enabled even without stock faults. Disable only the stock obstacle schedule; the external injector places one visible cylinder ahead of an active curve with a distinct existing reserve.',
        files={file.name: hashlib.sha256(file.read_bytes()).hexdigest() for file in destination.glob('*.py')})
    (destination/'external-test-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--harness', required=True); parser.add_argument('--destination', required=True)
    args = parser.parse_args(); print(json.dumps(prepare(args.harness, args.destination), indent=2))
