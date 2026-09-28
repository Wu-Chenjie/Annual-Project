"""Compare three completed or censored flights without changing denominators."""
from pathlib import Path
import argparse,json
import numpy as np
from analyze_run import measure

p=argparse.ArgumentParser();p.add_argument('optimized',type=Path);p.add_argument('tail',type=Path);p.add_argument('current',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
roots=[a.optimized,a.tail,a.current];names=['Optimized fusion','Tail priority','Integrated improvement']
values=[measure(r) for r in roots]
for field in ('map_sha256','coverage_definition','truth_free_voxels'):
    assert len({v[field] for v in values})==1,field
result=dict(runs=dict(zip(names,values)),comparison_scope='Single run per version. Identical scene, sensors, physical constraints, coverage denominator and fault rules; asynchronous schedules and actual fault times can differ.',changes={})
for name,base in zip(names[:2],values[:2]):
    result['changes'][name]={}
    for k in ('t90_s','t95_s','tail_90_to_95_s','distance_m'):
        before,after=base[k],values[2][k]
        comparable=before is not None and after is not None and before>0 and (k!='distance_m' or values[2]['t95_s'] is not None)
        result['changes'][name][k]=dict(before=before,after=after,reduction_percent=100*(1-after/before) if comparable else None)
(a.output/'three-way-comparison.json').write_text(json.dumps(result,indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained');colors=['#64748b','#db8a2b','#008578']
for root,name,value,color in zip(roots,names,values,colors):
    summary=json.loads((root/'summary.json').read_text());curve=np.asarray(json.loads((root/'coverage.json').read_text()))
    mask=curve[:,0]>=summary['start_time'];x=curve[mask,0]-summary['start_time'];y=curve[mask,1]*100
    axs[0,0].plot(x,y,label=name,color=color,lw=2)
    if value['t90_s'] is not None:
        mask=x>=value['t90_s'];axs[0,1].plot(x[mask]-value['t90_s'],y[mask],color=color,lw=2)
for ax in axs[0]:ax.axhline(95,color='#444',ls='--',lw=.8);ax.grid(alpha=.15)
axs[0,0].set(title='Complete coverage history',xlabel='Simulation seconds after initial scan',ylabel='Observed free voxels (%)',ylim=(20,100));axs[0,0].legend(loc='lower right',fontsize=9)
axs[0,1].set(title='Tail aligned at 90% coverage',xlabel='Simulation seconds after reaching 90%',ylabel='Observed free voxels (%)',ylim=(89.8,96))
x=np.arange(3);before=np.array([v['t90_s'] or v['observed_elapsed_s'] for v in values]);tail=np.array([v['tail_90_to_95_s'] or max(0,v['observed_elapsed_s']-b) for v,b in zip(values,before)])
bar=axs[1,0].bar(x,before,color=colors,alpha=.45,label='Start to 90%');axs[1,0].bar_label(bar,fmt='%.1f',label_type='center')
bar=axs[1,0].bar(x,tail,bottom=before,color=colors,label='90% to 95%');axs[1,0].bar_label(bar,labels=[f'{b+t:.1f}s' if v['t95_s'] is not None else f'>{b+t:.1f}s' for b,t,v in zip(before,tail,values)],padding=5)
axs[1,0].set(xticks=x,xticklabels=['Optimized','Tail priority','Integrated'],ylabel='Simulation seconds',title='Total mission time (lower is better)',ylim=(0,max(before+tail)*1.15));axs[1,0].legend(fontsize=9)
for i,(v,c) in enumerate(zip(values,colors)):
    for j,phase in enumerate(('whole','tail_90_to_end')):
        stats=v['phases'].get(phase,{}).get('planning_wall_s',{})
        if stats.get('count'):
            bar=axs[1,1].bar(j+(i-1)*.25,stats['median'],width=.23,color=c);axs[1,1].bar_label(bar,fmt='%.2f',padding=4,fontsize=9)
axs[1,1].set(xticks=[0,1],xticklabels=['Whole mission','Tail'],ylabel='Wall seconds per planning cycle',title='Median planner computation');axs[1,1].margins(y=.2)
fig.suptitle('Gazebo: joint route, information gain and bounded waiting',fontsize=16,weight='bold')
fig.supxlabel('One flight per version; mission times use the Gazebo clock. Planning times use the wall clock.',fontsize=9)
fig.savefig(a.output/'three-way-comparison.png',dpi=160);fig.savefig(a.output/'three-way-comparison.svg');plt.close(fig)
print(json.dumps(result['changes'],indent=2))
