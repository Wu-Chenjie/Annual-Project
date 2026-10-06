#!/usr/bin/env python3
"""Continue a frozen study without replacing stopped infrastructure attempts."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time


RETRYABLE = {'INFRASTRUCTURE_FAILURE', 'INFRASTRUCTURE_INTERRUPTED', 'INTERRUPTED'}


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_manifest(root):
    return {str(p.relative_to(root)): sha(p)
            for folder in ('next_project/core', 'next_project/cpp', 'next_project/maps', 'ros2_ws/src', 'docker')
            for p in sorted((root / folder).rglob('*'))
            if p.is_file() and '__pycache__' not in str(p)}


def histories(root, job, ledger):
    return [root / job] + [root / entry['directory'] for entry in ledger['attempts'] if entry['job'] == job]


def selected(root, job, ledger):
    return histories(root, job, ledger)[-1]


def next_attempt(root, job, ledger):
    current = selected(root, job, ledger)
    if not current.exists():
        return current
    result = current / 'run-result.json'
    if not result.exists():
        raise RuntimeError(f'Unfinished attempt needs review: {current}')
    cleanup = current / 'process-cleanup.json'
    if not cleanup.exists() or read(cleanup).get('remaining') != []:
        raise RuntimeError(f'Unverified process cleanup: {current}')
    if read(result)['outcome'] not in RETRYABLE:
        return None  # Never retry an algorithm failure, timeout or successful sample.
    number = len(histories(root, job, ledger))
    return root / 'infrastructure-retries' / job / f'attempt-{number:03d}'


def validate(header, roots, harness, auditor):
    for variant, expected in header['source_manifests'].items():
        if source_manifest(roots[variant]) != expected:
            raise ValueError(f'Frozen source changed: {variant}')
    actual = {p.name: sha(p) for p in harness.glob('*.py')}
    if actual != header['harness_manifests']:
        raise ValueError('Frozen harness changed')
    manifest = read(auditor / 'manifest.json')
    if manifest.get('flight_policy_changed') is not False:
        raise ValueError('Auditor must be read-only')
    for name, digest in manifest['files'].items():
        if sha(auditor / name) != digest:
            raise ValueError(f'Frozen auditor changed: {name}')


def shell_run(policy, command, log):
    shell = 'source /opt/ros/jazzy/setup.bash && source ' + shlex.quote(str(policy / 'ros2_ws/install/setup.bash'))
    shell += ' && export PYTHONPATH=' + shlex.quote(str(policy / 'ros2_ws/install/annual_swarm/lib/annual_swarm')) + ':${PYTHONPATH:-}'
    shell += ' && ' + shlex.join(command)
    with log.open('a') as stream:
        return subprocess.run(['bash', '-lc', shell], stdout=stream, stderr=stream).returncode


def revised_audit(directory, auditor, revision):
    sys.path.insert(0, str(auditor))
    spec = importlib.util.spec_from_file_location('resume_protocol_auditor', auditor / 'audit_execution_protocol.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    summary = read(directory / 'summary.json')
    result = module.check(
        [event for p in directory.glob('drone_*/events.jsonl') for event in module.records(p)],
        module.records(directory / 'peer_states.jsonl'), module.records(directory / 'execution-evidence.jsonl'),
        summary['fleet_size'], summary['simulation_time'], module.records(directory / 'command-evidence.jsonl'),
        task_end_time=summary.get('finish_time') if summary['status'] == 'COMPLETE' else None)
    write(directory / f'execution-protocol-audit-{revision}.json', result)


def report(root, header, ledger, auditor, revision):
    # The frozen gate computation reads a temporary view of the selected attempts.
    # Revision reporting only reads cached audits, so symlinks cannot overwrite raw evidence.
    sys.path.insert(0, str(auditor))
    spec = importlib.util.spec_from_file_location('resume_reporter', auditor / 'summarize_todo_study.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory(prefix='annual-resume-report-') as temporary:
        view = Path(temporary)
        write(view / 'study-manifest.json', header)
        for job in header['matrix']:
            directory = selected(root, job['job'], ledger)
            if directory.exists():
                (view / job['job']).symlink_to(directory, target_is_directory=True)
        result = module.summarize(view, revision)
        result['infrastructure_attempt_history'] = [
            dict(job=job['job'], directory=str(p.relative_to(root)), selected=p == selected(root, job['job'], ledger),
                 result=read(p / 'run-result.json') if (p / 'run-result.json').exists() else {'outcome': 'RUNNING_OR_INTERRUPTED'})
            for job in header['matrix'] for p in histories(root, job['job'], ledger) if p.exists()]
        result['resume_ledger'] = ledger
        write(root / f'study-report-{revision}-resumed.json', result)
        (root / f'all-attempts-{revision}-resumed.csv').write_bytes((view / f'all-attempts-{revision}.csv').read_bytes())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--study-root', type=Path, required=True)
    parser.add_argument('--baseline-root', type=Path, required=True)
    parser.add_argument('--combined-root', type=Path, required=True)
    parser.add_argument('--harness-root', type=Path, required=True)
    parser.add_argument('--auditor-root', type=Path, required=True)
    parser.add_argument('--revision', default='10e')
    parser.add_argument('--stage', choices=['main-no-fault', 'all'], default='main-no-fault')
    parser.add_argument('--report-only', action='store_true')
    args = parser.parse_args()
    root = args.study_root.resolve()
    header = read(root / 'study-manifest.json')
    roots = {'baseline': args.baseline_root.resolve(), 'combined': args.combined_root.resolve()}
    roots.update({name: Path(value['source_root']) for name, value in header['ablations'].items()})
    harness, auditor = args.harness_root.resolve(), args.auditor_root.resolve()
    validate(header, roots, harness, auditor)
    lock = (root / 'study.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    ledger_path = root / 'resume-attempts.json'
    identity = dict(schema='annual.todo-resume/1', continuation_sha256=sha(__file__),
                    original_manifest_sha256=sha(root / 'study-manifest.json'),
                    auditor_manifest_sha256=sha(auditor / 'manifest.json'), execution_audit_revision=args.revision)
    ledger = read(ledger_path) if ledger_path.exists() else dict(identity, attempts=[])
    if any(ledger.get(k) != v for k, v in identity.items()):
        raise ValueError('Continuation, frozen study or auditor changed')
    write(ledger_path, ledger)
    protocol = header['protocol']
    for job in header['matrix']:
        if args.stage == 'main-no-fault' and not (job['scene'] == 'main' and job['group'] == 'no_fault' and job['variant'] in ('baseline', 'combined')):
            continue
        current = selected(root, job['job'], ledger)
        if (current / 'run-result.json').exists() and read(current / 'run-result.json')['outcome'] == 'COMPLETE':
            revised_audit(current, auditor, args.revision)
        if args.report_only:
            continue
        directory = next_attempt(root, job['job'], ledger)
        if directory is None:
            continue
        validate(header, roots, harness, auditor)
        map_path = roots['combined'] / job['map_file']
        expected = protocol['map']['sha256'] if job['scene'] == 'main' else protocol['targeted_scenes'][job['scene']]['sha256']
        if sha(map_path) != expected:
            raise ValueError('Map changed')
        if directory != root / job['job']:
            ledger['attempts'].append(dict(job=job['job'], directory=str(directory.relative_to(root)),
                                          replaces_infrastructure_attempt=str(current.relative_to(root)), started_wall_unix=time.time()))
            write(ledger_path, ledger)
        directory.parent.mkdir(parents=True, exist_ok=True)
        write(root / 'resume-progress.json', dict(status='RUNNING', job=job['job'], directory=str(directory.relative_to(root)), stage=args.stage))
        policy = roots[job['variant']]
        command = ['python3', str(harness / 'run_todo_experiment.py'), '--observer', str(harness / 'record_observation_evidence.py'),
                   '--output-dir', str(directory), '--seed', str(job['seed']), '--map', str(map_path),
                   '--faults', 'none' if job['group'] == 'no_fault' else 'recovery',
                   '--simulation-limit', str(protocol['simulation_limit_s']), '--wall-limit', str(protocol['wall_limit_s'])]
        exit_code = shell_run(policy, command, root / 'resume-suite.log')
        if not (directory / 'run-result.json').exists():
            raise RuntimeError(f'Harness exit {exit_code} without result; review attempt before resume')
        cleanup = directory / 'process-cleanup.json'
        if not cleanup.exists() or read(cleanup).get('remaining') != []:
            raise RuntimeError('Unverified cleanup; refusing to start another flight')
        if shell_run(roots['combined'], ['python3', str(harness / 'audit_todo_attempt.py'), str(directory)], directory / 'audit.log'):
            raise RuntimeError('Frozen audit failed to run')
        if (directory / 'summary.json').exists():
            revised_audit(directory, auditor, args.revision)
        result = read(directory / 'run-result.json')
        print(json.dumps(dict(job=job['job'], directory=str(directory.relative_to(root)), **result)), flush=True)
        report(root, header, ledger, auditor, args.revision)
        if result['outcome'] in RETRYABLE:
            write(root / 'resume-progress.json', dict(status='INFRASTRUCTURE_REVIEW_REQUIRED', job=job['job']))
            return
    result = report(root, header, ledger, auditor, args.revision)
    main_group = next(c for c in result['comparisons'] if c['scene'] == 'main' and c['group'] == 'no_fault')
    write(root / 'resume-progress.json', dict(status='REPORT_COMPLETE' if args.report_only else 'STAGE_COMPLETE',
                                            stage=args.stage, main_no_fault=main_group, full_matrix_complete=result['complete']))
    print(json.dumps(main_group, indent=2))


if __name__ == '__main__':
    main()
