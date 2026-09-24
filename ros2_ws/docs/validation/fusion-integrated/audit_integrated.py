"""Audit executed integrated-planning diagnostics, separately from flight safety."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory
plans=[];seen=set()
for line in (root/'peer_states.jsonl').read_text().splitlines():
    state=json.loads(line);f=state.get('fusion',{});wall=f.get('compute_wall_s')
    if wall is None:continue
    key=(state['drone'],state.get('session'),wall)
    if key in seen:continue
    seen.add(key);plans.append(f)
assert plans
assert all(f['route_objective']=='travel_plus_information_latency' for f in plans)
assert all(0<=f['priority_compute']['refined_regions']<=24 for f in plans)
assert all(0<=f['priority_compute']['raycasts']<=24*2*4 for f in plans)
assert any(f['priority_compute']['cached_regions']>0 for f in plans)
assert any(f['priority_compute']['bound_regions']>0 for f in plans)
assert all(0<f['team_observed_cells']<=80*67*14 for f in plans)
commits=[]
for path in root.glob('drone_*/events.jsonl'):
    for line in path.read_text().splitlines():
        event=json.loads(line)
        if event['type']=='path_committed' and event.get('exploration_priority'):commits.append(event['exploration_priority'])
assert commits
assert all(1<=v['aging_multiplier']<=1.5+1e-12 for v in commits)
assert any(v['aging_multiplier']>=1.49 for v in commits)
assert all(v['gain_source'] in ('local_rays','region_bound','remote_proxy') for v in commits)
shares=[(v['aging_multiplier']-1)/v['aging_multiplier'] for v in commits if v['score']>0]
assert max(shares)<=1/3+1e-12
result=dict(passed=True,sampled_distinct_plans=len(plans),scored_commits=len(commits),max_age_multiplier=max(v['aging_multiplier'] for v in commits),
    max_age_fraction_of_positive_score=max(shares),max_refined_regions=max(f['priority_compute']['refined_regions'] for f in plans),
    max_coarse_raycasts=max(f['priority_compute']['raycasts'] for f in plans),
    median_priority_wall_s=float(np.median([f['stage_wall_s']['priority'] for f in plans])),
    max_team_observed_cells=max(f['team_observed_cells'] for f in plans),
    gain_sources={k:sum(v['gain_source']==k for v in commits) for k in ('local_rays','region_bound','remote_proxy')},
    scope='Checks the sampled execution diagnostics and all logged commits. Mathematical route, receipt idempotence/rejoin and unchanged occupancy contracts are covered by the separate test suite. Flight and sensor-map safety require separate audits.')
(root/'integrated-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
