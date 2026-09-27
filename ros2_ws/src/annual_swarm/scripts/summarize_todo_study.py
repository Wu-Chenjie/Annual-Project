#!/usr/bin/env python3
"""Report every scheduled attempt and paired gates; no successful-only verdict."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from audit_todo_evidence import audit
from verify_todo_run import verify


def summarize(root):
    root=Path(root);header=json.loads((root/'study-manifest.json').read_text());protocol=header['protocol']
    scenes=['main']+list(protocol['targeted_scenes']);variants=['baseline','combined']+protocol['ablations']
    rows=[]
    for scene in scenes:
      for group in protocol['groups'] if scene=='main' else ['no_fault']:
       for seed in protocol['seeds']:
        for variant in variants if scene=='main' and group=='no_fault' else ['baseline','combined']:
            name=f'{scene}-{group}-{seed}-{variant}';directory=root/name
            result=json.loads((directory/'run-result.json').read_text()) if (directory/'run-result.json').exists() else dict(outcome='PENDING')
            row=dict(run=name,scene=scene,group=group,seed=seed,variant=variant,outcome=result['outcome'],reason=result.get('reason'))
            try:
                if (directory/'run-result.json').exists() and (directory/'summary.json').exists():
                    a=audit(directory);v=verify(directory);s=json.loads((directory/'summary.json').read_text())
                    row.update(t95_s=a['thresholds']['t95'],tail_s=a['tail_90_95_s'],planning_wait_fleet_s=a['planning_wait_fleet_s'],
                        distance_m=sum(a['distances_m'].values()),low_services=a['exploration_low_team_yield'],zero_services=a['exploration_zero_team_yield'],
                        low_duration_s=a['exploration_low_yield_duration_s'],coverage=a['coverage'],safety_verified=v['passed'],
                        contacts=a['contacts'],planning_p95_s=(a['planning_all_measured_requests_wall_s'] or {}).get('p95'),
                        moving_handoffs=a['moving_handoffs'],minimum_sampled_separation_m=s['min_separation_m'])
                    row['flight_safety_verified']=all(value for key,value in v['gates'].items() if key!='final_map_matches')
                    row['low_duration_ratio']=a['exploration_low_yield_duration_s']/a['exploration_total_duration_s'] if a.get('exploration_total_duration_s',0)>0 else None
                    if s['status']!='COMPLETE':row['t95_s']=None
            except Exception as exc:row['audit_error']=repr(exc)
            rows.append(row)
    fields=sorted({key for row in rows for key in row})
    with (root/'all-attempts.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    def successful(row):return row['outcome']=='COMPLETE' and row.get('safety_verified',False) and row.get('t95_s') is not None
    def median(group,key):
        values=[row[key] for row in group if successful(row) and row.get(key) is not None]
        return float(np.median(values)) if values else None
    aggregates=[];comparisons=[]
    for scene in scenes:
      for group in protocol['groups'] if scene=='main' else ['no_fault']:
        selected=[row for row in rows if row['scene']==scene and row['group']==group]
        for variant in sorted({r['variant'] for r in selected}):
            r=[row for row in selected if row['variant']==variant]
            aggregates.append(dict(scene=scene,group=group,variant=variant,scheduled=len(r),
                finished=sum(row['outcome']!='PENDING' for row in r),verified_successes=sum(successful(row) for row in r),
                success_rate_all_scheduled=sum(successful(row) for row in r)/len(r),
                successful_only_medians={k:median(r,k) for k in ('t95_s','tail_s','planning_wait_fleet_s','distance_m','low_services','zero_services','low_duration_s','low_duration_ratio','planning_p95_s')}))
        b=[r for r in selected if r['variant']=='baseline'];c=[r for r in selected if r['variant']=='combined']
        bm={k:median(b,k) for k in ('t95_s','tail_s','planning_wait_fleet_s','distance_m','low_services','zero_services','low_duration_ratio')};cm={k:median(c,k) for k in bm}
        ready=all(r['outcome']!='PENDING' and 'audit_error' not in r for r in b+c)
        def reduce(key,fraction):
            return None if bm[key] is None or cm[key] is None or bm[key]<=0 else cm[key]<=bm[key]*(1-fraction)
        gates=dict(success_rate=sum(successful(r) for r in c)>=sum(successful(r) for r in b),
            safety=all(r.get('flight_safety_verified',False) for r in c),
            t95=None if bm['t95_s'] is None or cm['t95_s'] is None else cm['t95_s']<bm['t95_s'],
            tail=reduce('tail_s',protocol['gates']['tail_median_reduction_min']),
            planning_wait=reduce('planning_wait_fleet_s',protocol['gates']['planning_wait_median_reduction_min']),
            distance=None if bm['distance_m'] is None or cm['distance_m'] is None else cm['distance_m']<=bm['distance_m']*(1+protocol['gates']['distance_median_increase_max']),
            low_services=None if bm['low_services'] is None or cm['low_services'] is None else cm['low_services']<bm['low_services'],
            zero_services=None if bm['zero_services'] is None or cm['zero_services'] is None else cm['zero_services']<bm['zero_services'],
            low_duration_ratio=None if bm['low_duration_ratio'] is None or cm['low_duration_ratio'] is None else cm['low_duration_ratio']<bm['low_duration_ratio'],
            planning_p95=all(r.get('planning_p95_s') is not None and r['planning_p95_s']<=protocol['gates']['planning_wall_p95_deadline_s'] for r in c))
        pairs=[]
        for seed in protocol['seeds']:
            left=next(r for r in b if r['seed']==seed);right=next(r for r in c if r['seed']==seed)
            pairs.append(dict(seed=seed,baseline_outcome=left['outcome'],combined_outcome=right['outcome'],
                paired_verified_success=successful(left) and successful(right),
                t95_difference_s=right['t95_s']-left['t95_s'] if successful(left) and successful(right) else None))
        differences=[row['t95_difference_s'] for row in pairs if row['paired_verified_success']]
        evidence_strength=dict(verified_pairs=len(differences),t95_improved_pairs=sum(d<0 for d in differences),
            median_difference_s=float(np.median(differences)) if differences else None,
            paired_difference_range_s=[min(differences),max(differences)] if differences else None)
        comparisons.append(dict(scene=scene,group=group,complete=ready,gates=gates,passed=ready and all(v is True for v in gates.values()),evidence_strength=evidence_strength,
            baseline_successful_only_medians=bm,combined_successful_only_medians=cm,paired_results=pairs))
    report=dict(schema='annual.todo-study-report/1',complete=all(r['outcome']!='PENDING' for r in rows),
        attempts=rows,aggregates=aggregates,comparisons=comparisons,
        note='All scheduled attempts remain visible. PENDING is unfinished, never success. Latency medians use verified successes and are explicitly successful-only; failed or censored runs have no invented T95. Development attempts are excluded by protocol.')
    (root/'study-report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args();r=summarize(args.directory)
    print(json.dumps(dict(complete=r['complete'],comparisons=r['comparisons']),indent=2))
