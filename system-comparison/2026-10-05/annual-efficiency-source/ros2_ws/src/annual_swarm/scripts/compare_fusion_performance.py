#!/usr/bin/env python3
"""Recompute like-for-like metrics from two retained volumetric Gazebo runs.

This compares observed runs, not a statistical claim or an algorithm ablation.
Motion is measured from Gazebo position differences, never from playback speed.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def distribution(values):
    values = np.asarray(values, float)
    return dict(count=len(values), median=float(np.median(values)), p95=float(np.percentile(values,95)),
                maximum=float(np.max(values))) if len(values) else dict(count=0)


def measure(root):
    root=Path(root); s=json.loads((root/'summary.json').read_text())
    assert s['status']=='COMPLETE' and s['mapping_dimensions']==3
    begin,end=s['start_time'],s['finish_time']
    trajectories=defaultdict(list)
    with (root/'trajectory.csv').open() as stream:
        for r in csv.DictReader(stream):
            trajectories[int(r['drone'])].append([float(r[k]) for k in ('time','x','y','z')])
    movement={}
    for i, rows in trajectories.items():
        a=np.asarray(rows);dt=np.diff(a[:,0]);distance=np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1)
        mask=(a[:-1,0]>=begin)&(a[1:,0]<=end)&(dt>0)
        dt=dt[mask];distance=distance[mask];moving=distance/dt>.05
        movement[i]=dict(sampled_duration_s=float(dt.sum()),moving_fraction=float(dt[moving].sum()/dt.sum()),
            moving_mean_speed_m_s=float(distance[moving].sum()/dt[moving].sum()) if moving.any() else 0.,
            all_time_mean_speed_m_s=float(distance.sum()/dt.sum()))
    events=sorted((e for f in root.glob('drone_*/events.jsonl') for e in records(f)),key=lambda e:e['time'])
    counts=Counter(e['type'] for e in events)
    cancelled=Counter(e['reason'] for e in events if e['type']=='lease_cancellation_requested')
    waits=[]; finished={}; prepared=[]
    for e in events:
        key=(e['drone'],e.get('incarnation'))
        if e['type']=='view_observed': finished[key]=e['time']
        elif e['type']=='path_committed' and key in finished: waits.append(e['time']-finished.pop(key))
        elif e['type']=='path_proposed': prepared.append(bool(e.get('prepared_trajectory')))
    last={}; timings=[]; stages=defaultdict(list)
    for state in records(root/'peer_states.jsonl'):
        key=(state['drone'],state.get('session')); fusion=state.get('fusion',{})
        value=fusion.get('compute_wall_s',0.)
        if value and value!=last.get(key):
            last[key]=value;timings.append(value)
            for name,t in fusion.get('stage_wall_s',{}).items():stages[name].append(t)
    return dict(t90_s=s['t90'],t95_s=s['t95'],coverage=s['coverage'],distance_m=sum(s['distances_m'].values()),
                contacts=s['contacts_after_takeoff'],minimum_separation_m=s['min_separation_m'],
                tracking=s['tracking'],
                movement=movement,planning_wall_s=distribution(timings),stage_wall_s={k:distribution(v) for k,v in stages.items()},
                between_views_wait_s=distribution(waits),cancellations=dict(cancelled),event_counts=dict(counts),
                prepared_curves=sum(prepared),proposed_curves=len(prepared),commits=sum(s['commits'].values()),
                views=sum(s['views'].values()),map_sha256=hashlib.sha256((root/'map.json').read_bytes()).hexdigest(),
                coverage_definition=s['coverage_definition'],truth_free_voxels=s['truth_free_voxels'])


def compare(baseline,current):
    a,b=measure(baseline),measure(current)
    for key in ('map_sha256','coverage_definition','truth_free_voxels'):
        assert a[key]==b[key], ('Incompatible experiments',key,a[key],b[key])
    change={k:dict(before=a[k],after=b[k],reduction_percent=100*(1-b[k]/a[k])) for k in ('t90_s','t95_s','distance_m')}
    return dict(baseline=a,current=b,change=change,
        interpretation='One completed run per version; identical scene and coverage denominator. No statistical significance or isolated ablation is claimed. Inspect source manifests for all implementation differences.',
        motion_definition='Time-weighted intervals between Gazebo pose samples inside [start_time, finish_time]; moving means speed > 0.05 m/s.',
        planning_definition='Deduplicated compute_wall_s per agent incarnation. Optimized compute also includes continuous trajectory fitting that previously ran in the ROS callback.',
        wait_definition='Elapsed simulation time from view_observed to the next path_committed in the same incarnation; includes intentional fault recovery and arbitration.')


def figure(result, roots, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors=['#82909e','#087f8c'];labels=['Previous fused run','Optimized fused run']
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for root,color,label in zip(roots,colors,labels):
        s=json.loads((root/'summary.json').read_text());curve=np.asarray(json.loads((root/'coverage.json').read_text()))
        curve=curve[curve[:,0]>=s['start_time']]
        axes[0,0].plot(curve[:,0]-s['start_time'],curve[:,1]*100,color=color,label=label,lw=2)
    axes[0,0].axhline(95,color='#c56d19',ls='--',lw=1)
    axes[0,0].set(xlabel='Simulation time after initial scan (s)',ylabel='Free-voxel coverage (%)',title='Same 3D scene and coverage definition',ylim=(20,100))
    axes[0,0].legend(fontsize=9,loc='lower right')
    values=[result['baseline'],result['current']]
    panels=[(axes[0,1],[v['t95_s'] for v in values],'Time to 95% coverage','Simulation seconds'),
            (axes[1,0],[v['between_views_wait_s']['median'] for v in values],'Median wait between completed views','Simulation seconds'),
            (axes[1,1],[100*np.mean([m['moving_fraction'] for m in v['movement'].values()]) for v in values],
             'Fleet mean fraction of time moving','Time with translation speed > 0.05 m/s (%)')]
    for ax,heights,title,ylabel in panels:
        bars=ax.bar(['Previous','Optimized'],heights,color=colors,width=.55)
        ax.bar_label(bars,fmt='%.2f',padding=5)
        ax.set(title=title,ylabel=ylabel,ylim=(0,max(heights)*1.22))
    fig.suptitle('Fused exploration: recorded Gazebo comparison',fontsize=17,weight='bold')
    fig.supxlabel('One flight per version; not an isolated ablation. Fixed speed limit: 0.6 m/s. All metrics use simulation time.',fontsize=9)
    fig.savefig(output,dpi=160)
    fig.savefig(output.with_suffix('.svg'))
    plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline',type=Path);p.add_argument('current',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--figure',type=Path)
    args=p.parse_args();result=compare(args.baseline,args.current)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    if args.figure:
        args.figure.parent.mkdir(parents=True,exist_ok=True)
        figure(result,(args.baseline,args.current),args.figure)
    print(json.dumps(result['change'],indent=2))


if __name__=='__main__':main()
