#!/usr/bin/env python3
"""Apply a frozen accounting correction without changing flight evidence."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys


def reaudit(root, revision):
    import planning_runtime
    import core.exploration
    root=Path(root);revision=Path(revision);manifest=json.loads((revision/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        if hashlib.sha256((revision/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Frozen accounting correction changed')
    cleanup=json.loads((root/'process-cleanup.json').read_text())
    if cleanup.get('remaining'):raise ValueError('Wait for verified owned-process cleanup')
    status=json.loads((root/'audit-status.json').read_text())
    previous=status.get('accounting',{}).get('auditor_revision','original-frozen')
    history=root/'accounting-audit-history'/previous;history.mkdir(parents=True,exist_ok=True)
    for name in ('todo-audit.json','service-windows.csv','audit-status.json'):
        if (root/name).exists() and not (history/name).exists():shutil.copy2(root/name,history/name)
    original=root/'original-accounting-audit';original.mkdir(exist_ok=True)
    for name in ('todo-audit.json','service-windows.csv','near-pair-samples.csv','audit-status.json'):
        if (root/name).exists() and not (original/name).exists():shutil.copy2(root/name,original/name)
    for module_name,file in manifest['module_files'].items():
        spec=importlib.util.spec_from_file_location(module_name,revision/file)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        sys.modules[module_name]=module;setattr(core.exploration,module_name.rsplit('.',1)[-1],module)
    spec=importlib.util.spec_from_file_location('versioned_accounting_auditor',revision/'audit_todo_evidence.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    try:
        packet=module.audit(root)
        status['accounting']=dict(completed=True,status=packet['status'],auditor_revision=manifest['revision'])
    except Exception as exc:
        status['accounting']=dict(completed=False,error=repr(exc),auditor_revision=manifest['revision'])
        packet=None
    status['complete_attempt_accepted']=(status['outcome']=='COMPLETE' and status['accounting']['completed']
        and status['flight_and_coverage'].get('passed') is True and status['execution_protocol'].get('passed') is True)
    (root/'audit-status.json').write_text(json.dumps(status,indent=2)+'\n')
    shutil.copy2(revision/'manifest.json',root/'accounting-revision.json')
    return dict(run=root.name,completed=status['accounting']['completed'],error=status['accounting'].get('error'),
        windows=packet['service_windows'] if packet else None,
        terminations=packet['service_window_terminations'] if packet else None)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--revision',required=True)
    args=parser.parse_args();result=reaudit(args.directory,args.revision);print(json.dumps(result));raise SystemExit(0 if result['completed'] else 1)
