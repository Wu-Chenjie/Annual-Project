#!/usr/bin/env python3
"""Versioned read-only protocol recheck, retaining the original frozen audit."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys


def reaudit(root, revision):
    root = Path(root); revision = Path(revision)
    manifest = json.loads((revision/'manifest.json').read_text())
    for name, digest in manifest['files'].items():
        if hashlib.sha256((revision/name).read_bytes()).hexdigest() != digest: raise ValueError('Auditor revision changed')
    if manifest.get('module_files'):
        import planning_runtime
        import core.exploration
        for module_name,name in manifest['module_files'].items():
            spec=importlib.util.spec_from_file_location(module_name,revision/name)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            sys.modules[module_name]=module;setattr(core.exploration,module_name.rsplit('.',1)[-1],module)
    cleanup = json.loads((root/'process-cleanup.json').read_text())
    if cleanup.get('remaining'): raise ValueError('Wait for verified owned-process cleanup')
    status = json.loads((root/'audit-status.json').read_text())
    previous=status.get('execution_protocol',{}).get('auditor_revision','original-frozen')
    history=root/'protocol-audit-history'/previous;history.mkdir(parents=True,exist_ok=True)
    for name in ('audit-status.json','execution-protocol-audit.json'):
        if (root/name).exists() and not (history/name).exists():shutil.copy2(root/name,history/name)
    original = root/'original-frozen-audit'; original.mkdir(exist_ok=True)
    for name in ['audit-status.json', 'execution-protocol-audit.json']:
        if (root/name).exists() and not (original/name).exists(): shutil.copy2(root/name, original/name)
    spec = importlib.util.spec_from_file_location('versioned_execution_auditor', revision/'audit_execution_protocol.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    packet = module.audit(root)
    status['execution_protocol'] = dict(completed=True, passed=packet['passed'], status=packet['status'], auditor_revision=manifest['revision'])
    status['complete_attempt_accepted'] = (status['outcome']=='COMPLETE' and status['accounting']['completed']
        and status['flight_and_coverage'].get('passed') is True and packet['passed'])
    (root/'audit-status.json').write_text(json.dumps(status, indent=2)+'\n')
    shutil.copy2(revision/'manifest.json', root/'auditor-revision.json')
    return dict(run=root.name, status=packet['status'], errors=packet['errors'],
                legacy_cache_labels=len(packet['legacy_cache_labels_without_execution_provenance']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory'); parser.add_argument('--revision', required=True)
    args = parser.parse_args(); result = reaudit(args.directory, args.revision); print(json.dumps(result))
    raise SystemExit(0 if result['status']=='PASS' else 1)
