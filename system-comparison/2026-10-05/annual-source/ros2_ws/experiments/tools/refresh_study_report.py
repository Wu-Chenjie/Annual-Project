#!/usr/bin/env python3
"""Refresh the frozen reporter from completed independent audit outputs only."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('directory'); parser.add_argument('--harness', required=True)
    args = parser.parse_args(); root = Path(args.directory); harness = Path(args.harness)
    header = json.loads((root/'study-manifest.json').read_text())
    file = harness/'summarize_todo_study.py'; digest = hashlib.sha256(file.read_bytes()).hexdigest()
    if digest != header['harness_manifests'][file.name]: raise ValueError('Use the exact frozen reporter')
    sys.path.insert(0, str(harness))
    spec = importlib.util.spec_from_file_location('frozen_progress_reporter', file)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

    def cached(directory, name, status_key):
        directory = Path(directory); status = json.loads((directory/'audit-status.json').read_text())
        if not status.get(status_key, {}).get('completed'): raise ValueError('Independent audit did not complete')
        return json.loads((directory/name).read_text())

    # Preserve the frozen gate computation. Only replace repeated expensive
    # reconstruction with that same version's already-completed audit outputs.
    module.audit = lambda p: cached(p, 'todo-audit.json', 'accounting')
    module.verify = lambda p: cached(p, 'independent-acceptance.json', 'flight_and_coverage')
    report = module.summarize(root)
    report.update(measurement_source='Frozen completed per-attempt audit outputs; no repeated simulation or verification.',
                  summarizer_sha256=digest)
    (root/'study-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(complete=report['complete'], finished=sum(a['outcome']!='PENDING' for a in report['attempts']),
                         pending=sum(a['outcome']=='PENDING' for a in report['attempts']), comparisons=report['comparisons']), indent=2))


if __name__ == '__main__': main()
