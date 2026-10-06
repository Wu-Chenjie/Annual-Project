"""Descriptive local ownership/workload telemetry, never a global oracle."""
import json,sys
from pathlib import Path
import numpy as np

def summarize(root):
    root=Path(root);end=json.loads((root/'result.json').read_text())['t'];states=[json.loads(l) for l in (root/'peer_states.jsonl').read_text().splitlines() if l.strip()]
    output={}
    for drone in [0,1]:
        rows=sorted((s for s in states if s['drone']==drone and 20<=s['time']<=end),key=lambda s:s['time'])
        changes=0;weight=0.;weighted_workload=0.;previous={}
        for index,state in enumerate(rows):
            owners=state.get('owners',{})
            changes+=sum(rid in previous and previous[rid]!=owner for rid,owner in owners.items())
            previous=owners
            dt=min(.6,max(0.,(rows[index+1]['time'] if index+1<len(rows) else end)-state['time']))
            weight+=dt;weighted_workload+=dt*state.get('workload',0.)
        events=[json.loads(l) for l in (root/f'drone_{drone}/events.jsonl').read_text().splitlines() if l.strip()]
        services=[e for e in events if e['type']=='view_observed' and e['purpose']=='explore']
        output[str(drone)]=dict(local_owner_value_changes=changes,mean_reported_workload=weighted_workload/weight if weight else None,
            explore_completed_services=len(services),online_target_new_cells=sum(e['team_new_cells'] for e in services))
    result=dict(per_uav=output,note='Owner changes count only locally consecutive known owner values, excluding task appearance/disappearance. Counts from two local replicas are not summed as unique global handoffs. Workload is planner estimate, not measured flight duration.')
    (root/'allocation-metrics.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':print(json.dumps(summarize(sys.argv[1]),indent=2))
