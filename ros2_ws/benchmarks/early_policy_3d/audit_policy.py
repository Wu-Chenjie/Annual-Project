import json,sys,collections
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);out=Path(sys.argv[2]);steps=[];assignments=[];counts=collections.Counter();rejects=collections.Counter();historical=collections.Counter();all_events=[]
with (root/'policy-events.jsonl').open() as stream:
 for line in stream:
  e=json.loads(line);counts[e['type']]+=1;all_events.append(e)
  if e['type']=='historical_policy_event':historical[e['event']['type']]+=1
  if e['type']=='policy_step':steps.append(e)
  if e['type']=='historical_policy_event' and e['event']['type']=='task_assigned':assignments.append(e['event'])
  if e['type']=='lease_cancellation_requested':rejects[e['reason']]+=1
s=json.loads((root/'summary.json').read_text());begin=s['start_time'];curve=np.asarray(json.loads((root/'coverage.json').read_text()));tail=curve[curve[:,0]>=begin+300];per=collections.defaultdict(list)
for e in assignments:per[e['drone']].append(e)
trial=s.get('dynamic_trial');response=None
if trial:
 matching=[e for e in all_events if e['type']=='lease_cancellation_requested' and e.get('token')==trial['token'] and e['time']>=trial['start']]
 replacement=[e for e in all_events if e['type']=='central_authorization' and e.get('drone')==trial['drone'] and e['time']>trial['start']]
 response=dict(trial_start=trial['start'],first_cancellation_delay_s=matching[0]['time']-trial['start'] if matching else None,first_replacement_authorization_delay_s=replacement[0]['time']-trial['start'] if replacement else None)
result=dict(historical_event_counts=dict(historical),recorded_backup_switches=historical['backup_switch'],dynamic_response=response,policy_steps=len(steps),median_compute_wall_s=float(np.median([e['compute_wall_s'] for e in steps])),p95_compute_wall_s=float(np.quantile([e['compute_wall_s'] for e in steps],.95)),
 maximum_full_grid_nodes=max(e['stats'].get('nodes',0) for e in steps),event_counts=dict(counts),cancellation_reasons=dict(rejects),
 per_drone={i:dict(assignments=len(es),unique_tasks=len(set(e['task'] for e in es)),assignments_after_300=sum(e['time']>=begin+300 for e in es),
  repeatedly_assigned_tasks=[dict(task=task,count=n) for task,n in collections.Counter(e['task'] for e in es if e['time']>=begin+300).most_common(5)]) for i,es in per.items()},
 coverage_after_300=dict(first_percent=float(100*tail[0,1]),last_percent=float(100*tail[-1,1]),min_percent=float(100*tail[:,1].min()),max_percent=float(100*tail[:,1].max())) if len(tail) else None,
 note='Counts are from recorded historical-policy events. Repeat assignment does not alone prove cause; compute time is wall clock, coverage time is simulation clock. Fixed final adapter only.')
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
