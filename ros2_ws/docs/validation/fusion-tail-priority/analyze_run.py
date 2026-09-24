#!/usr/bin/env python3
"""Recorded-run comparison, including an explicitly censored incomplete run."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import tarfile
import numpy as np


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def distribution(values):
    a = np.asarray(values, float)
    return dict(count=len(a), median=float(np.median(a)), p95=float(np.percentile(a,95)), maximum=float(a.max())) if len(a) else dict(count=0)


def measure(root):
    s=json.loads((root/'summary.json').read_text())
    start=s['start_time']; end=s.get('finish_time') or s['simulation_time']
    t90=s.get('t90'); t95=s.get('t95')
    aging_rate=None
    with tarfile.open(root/'recorded-source.tar.gz') as archive:
        names=[n for n in archive.getnames() if n.endswith('/config/exploration_priority.yaml')]
        if names:
            for line in archive.extractfile(names[0]).read().decode().splitlines():
                if line.strip().startswith('aging_per_minute:'):
                    aging_rate=float(line.split(':',1)[1])/60
    phases={'whole':(start,end), 'before_90':(start, start+t90 if t90 is not None else end)}
    if end-start>600:
        phases['after_600']=(start+600,end)
    if t90 is not None:
        phases['tail_90_to_end']=(start+t90,end)
    events=sorted((e for p in root.glob('drone_*/events.jsonl') for e in records(p)),key=lambda e:e['time'])
    pose=defaultdict(list)
    with (root/'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            pose[int(row['drone'])].append([float(row[k]) for k in ('time','x','y','z')])
    pose={k:np.asarray(v) for k,v in pose.items()}
    plans=[]; previous={}
    for state in records(root/'peer_states.jsonl'):
        f=state.get('fusion',{}); wall=f.get('compute_wall_s',0)
        key=(state['drone'],state.get('session'))
        if wall and wall!=previous.get(key):
            plans.append((state['time'],wall,f.get('stage_wall_s',{})))
            previous[key]=wall
    stats={}
    for phase,(begin,finish) in phases.items():
        selected=[e for e in events if begin<=e['time']<=finish]
        counts=Counter(e['type'] for e in selected)
        region_commits=Counter(e['region'] for e in selected if e['type']=='path_committed')
        motion={}
        for drone,a in pose.items():
            dt=np.diff(a[:,0]); ds=np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1)
            mask=(a[:-1,0]>=begin)&(a[1:,0]<=finish)&(dt>0)
            dt=dt[mask]; ds=ds[mask]; moving=ds/dt>.05
            motion[drone]=dict(sampled_duration_s=float(dt.sum()),distance_m=float(ds.sum()),moving_fraction=float(dt[moving].sum()/dt.sum()) if dt.sum() else None)
        timings=[(wall,stages) for t,wall,stages in plans if begin<=t<=finish]
        stages=defaultdict(list)
        for _,parts in timings:
            for k,v in parts.items():stages[k].append(v)
        observed={}; waits=[]
        for e in selected:
            key=(e['drone'],e.get('incarnation'))
            if e['type']=='view_observed':observed[key]=e['time']
            elif e['type']=='path_committed' and key in observed:waits.append(e['time']-observed.pop(key))
        feedback=[e['feedback'] for e in selected if e['type']=='region_service']
        priorities=[e['exploration_priority'] for e in selected if e['type']=='path_committed' and e.get('exploration_priority')]
        aging_shares=[min(1.,aging_rate*p['wait_s']/p['score']) for p in priorities if aging_rate is not None and p.get('score',0)>0]
        stats[phase]=dict(begin=begin,end=finish,duration_s=finish-begin,event_counts=dict(counts),motion=motion,
            planning_wall_s=distribution([x[0] for x in timings]),stage_wall_s={k:distribution(v) for k,v in stages.items()},
            between_views_wait_s=distribution(waits),commits_per_region=dict(region_commits),
            repeated_region_commits=sum(max(0,n-1) for n in region_commits.values()),
            committed_priority=dict(count=len(priorities),remote_proxy_count=sum(p.get('gain_source')=='remote_proxy' for p in priorities),
                predicted_gain_m3=distribution([p['predicted_gain'] for p in priorities]),waiting_s=distribution([p['wait_s'] for p in priorities]),
                aging_fraction_of_score=distribution(aging_shares),aging_over_80_percent_count=sum(v>.8 for v in aging_shares)),
            regional_feedback=dict(count=len(feedback),low_yield=sum(f.get('low_yield_streak',0)>0 for f in feedback),
                zero_gain=sum(f.get('observed_new_cells',0)==0 for f in feedback),
                observed_local_cells=distribution([f['observed_new_cells'] for f in feedback])))
    return dict(status=s['status'],coverage=s['coverage'],observed_elapsed_s=s['simulation_time']-start,t90_s=t90,t95_s=t95,
        tail_90_to_95_s=t95-t90 if t95 is not None and t90 is not None else None,
        time_limit_reached=json.loads((root/'run-result.json').read_text()).get('outcome') if (root/'run-result.json').exists() else None,
        contacts=s['contacts_after_takeoff'],min_separation_m=s['min_separation_m'],distance_m=sum(s['distances_m'].values()),
        coverage_definition=s['coverage_definition'],truth_free_voxels=s['truth_free_voxels'],
        map_sha256=hashlib.sha256((root/'map.json').read_bytes()).hexdigest(),phases=stats)


def figure(result,roots,dest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    labels=['Previous optimized fusion','New regional priority']; colors=['#77889b','#008b8b']
    values=[result['previous'],result['current']]
    for root,label,color,value in zip(roots,labels,colors,values):
        s=json.loads((root/'summary.json').read_text())
        a=np.asarray(json.loads((root/'coverage.json').read_text()))
        a=a[a[:,0]>=s['start_time']];x=a[:,0]-s['start_time']
        axes[0,0].plot(x,a[:,1]*100,label=label,color=color,lw=2)
        if value['t90_s'] is not None:
            mask=x>=value['t90_s']
            axes[0,1].plot(x[mask]-value['t90_s'],a[mask,1]*100,label=label,color=color,lw=2)
        else:
            axes[0,1].text(.04,.08,label+': 90% not reached',transform=axes[0,1].transAxes,color=color,fontsize=10)
    axes[0,0].set(title='Complete recorded coverage history',xlabel='Simulation seconds after initial scan',ylabel='Free-voxel coverage (%)',ylim=(20,100))
    axes[0,1].set(title='Tail aligned at 90% coverage',xlabel='Simulation seconds after reaching 90%',ylabel='Free-voxel coverage (%)',ylim=(89.8,96))
    for ax in axes[0]:ax.axhline(95,color='#c47623',ls='--',lw=1);ax.grid(alpha=.15)
    axes[0,0].legend(fontsize=9,loc='lower right')
    ax=axes[1,0]
    before=[v['t90_s'] or v['observed_elapsed_s'] for v in values]
    tail=[v['tail_90_to_95_s'] if v['tail_90_to_95_s'] is not None else max(0,v['observed_elapsed_s']-(v['t90_s'] or v['observed_elapsed_s'])) for v in values]
    bars=ax.bar(['Previous','New'],before,color=colors,alpha=.55,label='Start to 90% / end')
    ax.bar_label(bars,fmt='%.1f',label_type='center')
    bars=ax.bar(['Previous','New'],tail,bottom=before,color=colors,label='90% to 95% / end')
    ax.bar_label(bars,labels=[f'{t:.1f}' if v['t95_s'] is not None else ('90% not reached' if v['t90_s'] is None else f'{t:.1f} (censored)') for t,v in zip(tail,values)],padding=5)
    ax.set(title='Search and tail duration',ylabel='Simulation seconds',ylim=(0,max(b+t for b,t in zip(before,tail))*1.18));ax.legend(fontsize=9)
    ax=axes[1,1]
    for i,(value,color,label) in enumerate(zip(values,colors,labels)):
        for j,phase in enumerate(('whole','tail_90_to_end')):
            timing=value['phases'].get(phase,{}).get('planning_wall_s',{})
            if timing.get('count',0):
                bar=ax.bar(j+(i-.5)*.33,timing['median'],width=.3,color=color,label=label if j==0 else None)
                ax.bar_label(bar,fmt='%.2f',padding=4)
    ax.set_xticks([0,1],['Whole run','Tail']);ax.set(title='Median planning compute time',ylabel='Wall seconds per recorded planning cycle');ax.margins(y=.2)
    fig.suptitle('Gazebo: regional-priority tail evaluation',fontsize=17,weight='bold')
    fig.supxlabel('Same scene / sensors / limits / fault rules. One run per version; fault realization and asynchronous timing may differ.',fontsize=9)
    fig.savefig(dest,dpi=160);fig.savefig(dest.with_suffix('.svg'));plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('previous',type=Path);p.add_argument('current',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();a=measure(args.previous);b=measure(args.current)
    for k in ('map_sha256','coverage_definition','truth_free_voxels'):assert a[k]==b[k],k
    change={}
    for k in ('t90_s','t95_s','tail_90_to_95_s','distance_m'):
        comparable=b[k] is not None and bool(a[k]) and (k!='distance_m' or (a['t95_s'] is not None and b['t95_s'] is not None))
        change[k]=dict(before=a[k],after=b[k],reduction_percent=100*(1-b[k]/a[k]) if comparable else None)
    result=dict(previous=a,current=b,change=change,
        scope='One native Gazebo run per version, not a statistical significance claim. Equal physical scene, coverage denominator, sensor model, limits and fault-trigger rules. Asynchronous planning and geometry-dependent fault times can differ.',
        phase_note='Tail begins at the first sampled crossing of 90%. Region revisits can be productive and are not automatically wasted work. Local service feedback is not global unique coverage gain. Planning times are deduplicated telemetry samples, not an exhaustive profiler.',
        units='Mission/coverage times: simulation seconds. Planning stage times: wall seconds. Video: 24x wall-clock recording playback.')
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    figure(result,(args.previous,args.current),args.output/'comparison.png')
    print(json.dumps(change,indent=2))


if __name__=='__main__':main()
