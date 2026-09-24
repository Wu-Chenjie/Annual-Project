#!/usr/bin/env python3
"""Audit a possibly censored baseline run; compare only common 3D metrics."""
import argparse,csv,hashlib,json
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
from core.exploration.voxel_mapping import VoxelTruth,VoxelMap

def records(path):
    with path.open() as f:
        for line in f:
            if line.strip():yield json.loads(line)

def measured(root):
    s=json.loads((root/'summary.json').read_text());begin=s['start_time'];end=s['finish_time'] or s['simulation_time']
    rows=defaultdict(list)
    with (root/'trajectory.csv').open() as f:
        for r in csv.DictReader(f):rows[int(r['drone'])].append([float(r[k]) for k in ('time','x','y','z')])
    movement={}
    for i,values in rows.items():
        a=np.array(values);dt=np.diff(a[:,0]);ds=np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1)
        mask=(a[:-1,0]>=begin)&(a[1:,0]<=end)&(dt>0);dt=dt[mask];ds=ds[mask]
        movement[i]=dict(moving_fraction=float(dt[ds/dt>.05].sum()/dt.sum()),sampled_duration_s=float(dt.sum()),altitude_span_m=float(np.ptp(a[a[:,0]>=begin,3])))
    curve=np.array(json.loads((root/'coverage.json').read_text()));curve[:,0]-=begin
    checkpoints={}
    for t in [300.,362.,1000.,1631.005,2400.]:
        checkpoints[str(t)]=float(curve[np.flatnonzero(curve[:,0]<=t)[-1],1]) if curve[-1,0]>=t else None
    return dict(status=s['status'],coverage=s['coverage'],elapsed_s=end-begin,t90_s=s['t90'],t95_s=s['t95'],distance_m=sum(s['distances_m'].values()),
        contacts=s['contacts_after_takeoff'],minimum_separation_m=s['min_separation_m'],views=sum(s['views'].values()),commits=sum(s['commits'].values()),
        movement=movement,coverage_at_elapsed_s=checkpoints,map_sha256=hashlib.sha256((root/'map.json').read_bytes()).hexdigest(),
        coverage_definition=s['coverage_definition'],truth_free_voxels=s['truth_free_voxels'])

def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('fused',type=Path);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    old,new=measured(a.baseline),measured(a.fused)
    for key in ['map_sha256','coverage_definition','truth_free_voxels']:assert old[key]==new[key],key
    s=json.loads((a.baseline/'summary.json').read_text());saved=np.load(a.baseline/'observed_latest.npz')
    world=VoxelTruth(a.baseline/'map.json');observed=VoxelMap(saved['bounds'],resolution=float(saved['resolution']));observed.state[:]=saved['state']
    coverage=world.coverage_metrics(observed)
    assert np.isclose(coverage['coverage'],old['coverage'],atol=1/58589)
    events=[e for f in a.baseline.glob('drone_*/events.jsonl') for e in records(f)]
    limits=[e['trajectory_limits'] for e in events if e['type']=='path_committed']
    envelope=all(l['speed']<=.601 and l['acceleration']<=.801 and l['jerk']<=2.001 for l in limits)
    candidates=list(records(a.baseline/'candidate_paths.jsonl'));maximum=max((len(c['paths'])-1 for c in candidates),default=0)
    outcome=json.loads((a.baseline/'run-result.json').read_text())
    audit=dict(outcome=outcome,coverage_recomputed=coverage,zero_contacts=old['contacts']==0,minimum_separation_over_point8=old['minimum_separation_m']>.8,
        all_three_moved=all(m['moving_fraction']>0 for m in old['movement'].values()),continuous_reference_limits_passed=envelope,
        max_backups=maximum,event_counts=dict(Counter(e['type'] for e in events)),pause_resumed=s['pause_resumed'],network_resumed=s['network_resumed'],
        restart_recovered=s['restart_recovered'],restart_scope='Per-UAV centralized command client, not a per-UAV planner',
        dynamic_trial=s['dynamic_trial'],dynamic_finished=s['dynamic_finished'],
        common_metric_checks_passed=True,complete_95=outcome['outcome']=='COMPLETE' and old['coverage']>=.95,
        note='Completion is distinct from safe execution and evidence validity. A time-limited run is not a completed mission.')
    (a.output_dir/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    result=dict(early_policy_3d=old,optimized_fused=new,interpretation='One attempt per version; common map, sensing, coverage and motion constraints. Central map sharing, dimensional frontier adaptation and restart semantics differ; not an isolated allocator ablation.')
    (a.output_dir/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(10,5.5),layout='constrained')
    for root,label,color in [(a.baseline,'Early policy, 3D interface adapter','#c56d19'),(a.fused,'Optimized fused pipeline','#087f8c')]:
        summary=json.loads((root/'summary.json').read_text());curve=np.array(json.loads((root/'coverage.json').read_text()));curve[:,0]-=summary['start_time'];curve=curve[curve[:,0]>=0]
        ax.plot(curve[:,0],100*curve[:,1],label=label,color=color,lw=2)
    ax.axhline(95,color='#555555',ls='--',lw=1);ax.set(xlabel='Simulation time after initial scan (s)',ylabel='Observed fully-free voxels (%)',title='Actual Gazebo runs: common 3D sensing and coverage',ylim=(15,100));ax.legend(loc='lower right');ax.grid(alpha=.15)
    fig.supxlabel('Single attempt per version. Centralized sharing and restart semantics differ; no statistical claim.',fontsize=9)
    fig.savefig(a.output_dir/'coverage-comparison.png',dpi=170);plt.close(fig)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
