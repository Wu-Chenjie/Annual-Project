#!/usr/bin/env python3
"""Reaudit completed attempts without replacing their frozen original verdicts."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--study-root', type=Path, required=True)
    parser.add_argument('--auditor-root', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    args = parser.parse_args()
    manifest = json.loads((args.auditor_root/'manifest.json').read_text())
    for name, digest in manifest['files'].items():
        if hashlib.sha256((args.auditor_root/name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Frozen auditor changed: {name}')
    sys.path.insert(0, str(args.auditor_root.resolve()))
    from audit_execution_protocol import check, records

    results = {}
    for directory in sorted(args.study_root.glob('*')):
        if not directory.is_dir() or not (directory/'run-result.json').exists():
            continue
        needed=['summary.json','peer_states.jsonl','execution-evidence.jsonl','command-evidence.jsonl',
                'execution-protocol-audit.json','independent-acceptance.json','todo-audit.json']
        missing=[name for name in needed if not (directory/name).exists()]
        if missing:
            results[directory.name]=dict(original_passed=False,revised_passed=False,
                original_errors=['Original audit unavailable'],revised_errors=[f'Missing audit input: {name}' for name in missing],
                independent_flight_and_coverage_passed=False,accepted_after_revision=False,
                cancelled_by_verified_stop=[])
            continue
        summary = json.loads((directory/'summary.json').read_text())
        revised = check([event for file in directory.glob('drone_*/events.jsonl') for event in records(file)],
                        records(directory/'peer_states.jsonl'), records(directory/'execution-evidence.jsonl'),
                        summary['fleet_size'], summary['simulation_time'], records(directory/'command-evidence.jsonl'),
                        task_end_time=summary.get('finish_time') if summary['status']=='COMPLETE' else None)
        (directory/f'execution-protocol-audit-{args.revision}.json').write_text(json.dumps(revised, indent=2)+'\n')
        original = json.loads((directory/'execution-protocol-audit.json').read_text())
        flight = json.loads((directory/'independent-acceptance.json').read_text())
        run = json.loads((directory/'run-result.json').read_text())
        accounting = json.loads((directory/'todo-audit.json').read_text())
        results[directory.name] = dict(original_passed=original['passed'], revised_passed=revised['passed'],
            original_errors=original['errors'], revised_errors=revised['errors'],
            independent_flight_and_coverage_passed=flight['passed'],
            accounting_status=accounting['status'], run_outcome=run['outcome'],
            accepted_after_revision=bool(revised['passed'] and flight['passed'] and
                                         accounting['status']=='COMPLETE' and run['outcome']=='COMPLETE'),
            cancelled_by_verified_stop=[handoff for handoff in revised['handoffs']
                                        if 'cancelled_by_stop_at' in handoff])
    report = dict(revision=args.revision, manifest=manifest, attempts=results)
    (args.study_root/f'auditor-{args.revision}-recheck.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({name:dict(original=row['original_passed'],revised=row['revised_passed'],
                                stopped_handoffs=len(row['cancelled_by_verified_stop']))
                      for name,row in results.items()},indent=2))
    return 0 if all(row['revised_passed'] for row in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
