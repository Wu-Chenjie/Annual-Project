"""Infrastructure retries preserve the original attempt and never cherry-pick policy failures."""
import importlib.util
import json
from pathlib import Path
import shutil
import pytest

spec = importlib.util.spec_from_file_location('continuation', Path(__file__).with_name('resume_todo_study.py'))
continuation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(continuation)


def attempt(path, outcome, remaining=()):
    path.mkdir(parents=True)
    continuation.write(path / 'run-result.json', {'outcome': outcome})
    continuation.write(path / 'process-cleanup.json', {'remaining': list(remaining)})


def test_stopped_attempt_is_preserved_and_retry_failure_cannot_be_retried(tmp_path):
    job = 'main-no_fault-904-combined'
    ledger = {'attempts': []}
    original = tmp_path / job
    attempt(original, 'INFRASTRUCTURE_FAILURE')
    before = (original / 'run-result.json').read_bytes()
    retry = continuation.next_attempt(tmp_path, job, ledger)
    assert retry == tmp_path / 'infrastructure-retries' / job / 'attempt-001'
    ledger['attempts'].append({'job': job, 'directory': str(retry.relative_to(tmp_path))})
    attempt(retry, 'SIMULATION_TIME_LIMIT')
    assert continuation.selected(tmp_path, job, ledger) == retry
    assert continuation.next_attempt(tmp_path, job, ledger) is None
    assert (original / 'run-result.json').read_bytes() == before
    assert len(continuation.histories(tmp_path, job, ledger)) == 2


@pytest.mark.parametrize('outcome', ['COMPLETE', 'FAILED', 'WALL_TIME_LIMIT', 'EXECUTION_FAILURE', 'AGENT_FAILURE'])
def test_algorithm_outcomes_are_never_retried(tmp_path, outcome):
    attempt(tmp_path / 'job', outcome)
    assert continuation.next_attempt(tmp_path, 'job', {'attempts': []}) is None


def test_unfinished_or_unclean_attempt_blocks_flight(tmp_path):
    original = tmp_path / 'job'
    original.mkdir()
    with pytest.raises(RuntimeError, match='Unfinished'):
        continuation.next_attempt(tmp_path, 'job', {'attempts': []})
    continuation.write(original / 'run-result.json', {'outcome': 'INFRASTRUCTURE_FAILURE'})
    continuation.write(original / 'process-cleanup.json', {'remaining': [123]})
    with pytest.raises(RuntimeError, match='cleanup'):
        continuation.next_attempt(tmp_path, 'job', {'attempts': []})


def test_frozen_policy_and_harness_changes_block_resume(tmp_path):
    policy = tmp_path / 'policy'
    script = policy / 'ros2_ws/src/agent.py'
    script.parent.mkdir(parents=True)
    script.write_text('original policy')
    harness = tmp_path / 'harness'
    harness.mkdir()
    (harness / 'run.py').write_text('original harness')
    auditor = tmp_path / 'auditor'
    auditor.mkdir()
    continuation.write(auditor / 'manifest.json', {'flight_policy_changed': False, 'files': {}})
    header = {'source_manifests': {'combined': continuation.source_manifest(policy)},
              'harness_manifests': {'run.py': continuation.sha(harness / 'run.py')}}
    continuation.validate(header, {'combined': policy}, harness, auditor)
    script.write_text('different policy')
    with pytest.raises(ValueError, match='source'):
        continuation.validate(header, {'combined': policy}, harness, auditor)
    script.write_text('original policy')
    (harness / 'run.py').write_text('different harness')
    with pytest.raises(ValueError, match='harness'):
        continuation.validate(header, {'combined': policy}, harness, auditor)


def test_retry_report_keeps_history_and_cannot_hide_failed_safety(tmp_path):
    auditor = tmp_path / 'auditor'
    auditor.mkdir()
    shutil.copyfile(Path(__file__).parents[2] / 'src/annual_swarm/scripts/summarize_todo_study.py',
                    auditor / 'summarize_todo_study.py')
    root = tmp_path / 'study'
    root.mkdir()
    protocol = dict(seeds=list(range(900, 905)), targeted_scenes={}, groups=['no_fault'], ablations=[],
                    gates=dict(tail_median_reduction_min=.2, planning_wait_median_reduction_min=.5,
                               distance_median_increase_max=.1, planning_wall_p95_deadline_s=12.))
    jobs = [dict(job=f'main-no_fault-{seed}-{variant}') for seed in protocol['seeds'] for variant in ('baseline', 'combined')]
    header = dict(protocol=protocol, matrix=jobs)
    ledger = {'attempts': []}
    for job in jobs:
        directory = root / job['job']
        attempt(directory, 'COMPLETE')
        continuation.write(directory / 'summary.json', {'status': 'COMPLETE', 'min_separation_m': 2.})
        accounting = dict(thresholds=dict(t95=10.), tail_90_95_s=2., planning_wait_fleet_s=5.,
                          distances_m={0:10.}, exploration_low_team_yield=1, exploration_zero_team_yield=0,
                          exploration_low_yield_duration_s=1., exploration_total_duration_s=10.,
                          coverage=.95, contacts=0, planning_all_measured_requests_wall_s=dict(p95=1.), moving_handoffs=0)
        continuation.write(directory / 'todo-audit.json', accounting)
        continuation.write(directory / 'independent-acceptance.json', dict(passed=True, gates=dict(zero_contacts=True)))
        continuation.write(directory / 'execution-protocol-audit-10e.json', dict(passed=True, status='PASS'))
    job = 'main-no_fault-904-combined'
    original = root / job
    continuation.write(original / 'run-result.json', {'outcome': 'INFRASTRUCTURE_FAILURE'})
    retry = continuation.next_attempt(root, job, ledger)
    shutil.copytree(original, retry)
    continuation.write(retry / 'run-result.json', {'outcome': 'COMPLETE'})
    continuation.write(retry / 'independent-acceptance.json', dict(passed=False, gates=dict(zero_contacts=False)))
    ledger['attempts'].append({'job': job, 'directory': str(retry.relative_to(root))})
    (root / 'study-report-10e.json').write_text('original report')
    result = continuation.report(root, header, ledger, auditor, '10e')
    comparison = result['comparisons'][0]
    assert comparison['complete'] and not comparison['passed']
    assert comparison['gates']['safety'] is False
    assert comparison['evidence_strength']['verified_pairs'] == 4
    assert len(result['attempts']) == 10
    history = [row for row in result['infrastructure_attempt_history'] if row['job'] == job]
    assert [row['result']['outcome'] for row in history] == ['INFRASTRUCTURE_FAILURE', 'COMPLETE']
    assert [row['selected'] for row in history] == [False, True]
    assert (root / 'study-report-10e.json').read_text() == 'original report'
    assert continuation.read(original / 'run-result.json')['outcome'] == 'INFRASTRUCTURE_FAILURE'
