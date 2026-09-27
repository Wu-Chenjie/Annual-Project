"""Unfinished / failed paired trials must not become successful latency claims."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_todo_study import summarize


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
