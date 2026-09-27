"""Unfinished / failed paired trials must not become successful latency claims."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_todo_study import summarize
import summarize_todo_study as reporting


def test_pending_and_all_failed_studies_do_not_pass(tmp_path):
    protocol=dict(seeds=list(range(900,905)),targeted_scenes={},groups=['no_fault'],ablations=[],
        gates=dict(tail_median_reduction_min=.2,planning_wait_median_reduction_min=.5,
                   distance_median_increase_max=.1,planning_wall_p95_deadline_s=12.))
    (tmp_path/'study-manifest.json').write_text(json.dumps(dict(protocol=protocol)))
    pending=summarize(tmp_path)
    assert len(pending['attempts'])==10 and not pending['complete']
    assert not pending['comparisons'][0]['passed']
    for seed in protocol['seeds']:
        for variant in ('baseline','combined'):
            directory=tmp_path/f'main-no_fault-{seed}-{variant}';directory.mkdir()
            (directory/'run-result.json').write_text(json.dumps(dict(outcome='SIMULATION_TIME_LIMIT')))
    failed=summarize(tmp_path)
    assert failed['complete'] and not failed['comparisons'][0]['passed']
    assert all(row['verified_successes']==0 for row in failed['aggregates'])
    assert all(row['successful_only_medians']['t95_s'] is None for row in failed['aggregates'])
    assert all(row['t95_difference_s'] is None for row in failed['comparisons'][0]['paired_results'])


def test_contact_in_failed_attempt_is_not_hidden_by_successful_only_medians(tmp_path,monkeypatch):
    protocol=dict(seeds=list(range(900,905)),targeted_scenes={},groups=['no_fault'],ablations=[],
        gates=dict(tail_median_reduction_min=.2,planning_wait_median_reduction_min=.5,
                   distance_median_increase_max=.1,planning_wall_p95_deadline_s=12.))
    (tmp_path/'study-manifest.json').write_text(json.dumps(dict(protocol=protocol)))
    for seed in protocol['seeds']:
        for variant in ('baseline','combined'):
            directory=tmp_path/f'main-no_fault-{seed}-{variant}';directory.mkdir()
            status='COMPLETE' if seed==900 else 'FAILED'
            (directory/'run-result.json').write_text(json.dumps(dict(outcome=status)))
            (directory/'summary.json').write_text(json.dumps(dict(status=status,min_separation_m=2.)))
    def contact(directory):return directory.name=='main-no_fault-901-combined'
    def fake_audit(directory):
        combined=directory.name.endswith('combined');finished='-900-' in directory.name
        return dict(thresholds=dict(t95=(2. if combined else 10.) if finished else None),tail_90_95_s=2. if combined else 10.,
            planning_wait_fleet_s=10. if combined else 100.,distances_m={0:80. if combined else 100.},
            exploration_low_team_yield=2 if combined else 10,exploration_zero_team_yield=1 if combined else 3,
            exploration_low_yield_duration_s=10. if combined else 50.,exploration_total_duration_s=100.,
            coverage=.95,contacts=int(contact(directory)),planning_all_measured_requests_wall_s=dict(p95=1.),moving_handoffs=0)
    def fake_verify(directory):
        return dict(passed='-900-' in directory.name,gates=dict(zero_contacts=not contact(directory),
            all_truth_streams_recorded=True,final_map_matches='-900-' in directory.name))
    monkeypatch.setattr(reporting,'audit',fake_audit);monkeypatch.setattr(reporting,'verify',fake_verify)
    result=summarize(tmp_path);comparison=result['comparisons'][0]
    assert comparison['gates']['t95'] and comparison['gates']['success_rate']
    assert comparison['gates']['safety'] is False and comparison['passed'] is False
    assert len(result['attempts'])==10
