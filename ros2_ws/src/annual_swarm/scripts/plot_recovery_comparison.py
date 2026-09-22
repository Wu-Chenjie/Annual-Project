#!/usr/bin/env python3
"""Plot recorded comparison results; requires matplotlib, no simulator dependency."""
import argparse
import csv
import json
from pathlib import Path
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

p=argparse.ArgumentParser();p.add_argument('results');args=p.parse_args()
source=Path(args.results).resolve();rows=json.loads(source.read_text())
policies=['cache','replan_single','replan_pool'];labels=['Cached top 5','Replan one A*','Rebuild pool'];colors=['#16877c','#dd8d24','#6b66aa']
fig,axes=plt.subplots(2,3,figsize=(14,8.5));fig.suptitle('Gazebo recorded flights: recovery-policy comparison',fontsize=17)
for ax,key,title in zip(axes[0],['recovery_wall_s','pause_sim_s','flight_sim_s'],['Safe replacement published (wall s)','Reference paused (simulation s)','Flight to HOLDING (simulation s)']):
    values=[[r[key] for r in rows if r['policy']==policy] for policy in policies]
    ax.bar(labels,[statistics.mean(v) for v in values],color=colors,alpha=.75)
    for i,v in enumerate(values):
        ax.scatter([i]*len(v),v,color='black',s=18,zorder=3)
        ax.text(i,max(v)+max(max(x) for x in values)*.04,f'{statistics.mean(v):.3f}',ha='center')
    ax.set_title(title,fontsize=11);ax.set_ylim(0,max(max(x) for x in values)*1.22);ax.grid(axis='y',alpha=.2)
for ax,policy,label,color in zip(axes[1],policies,labels,colors):
    for r in (x for x in rows if x['policy']==policy):
        # Resolve by run directory so copied /workspace results remain portable.
        original=Path(r['summary']);relative=Path(*original.parts[-4:])
        trajectory=source.parent/relative.parent/'trajectory.csv'
        with trajectory.open() as stream:data=[x for x in csv.DictReader(stream) if x['drone']=='0']
        ax.plot([float(x['x']) for x in data],[float(x['y']) for x in data],color=color,alpha=.7,lw=1.6,label=f"Repeat {r['repeat']}")
    for xy in [(7,7),(7,13),(14,10)]:ax.add_patch(Circle(xy,.5,color='#555555'))
    ax.add_patch(Circle((9.5,5),.55,color='#cf3e43'));ax.add_patch(Circle((9.5,5),2.45,fill=False,ls='--',color='#cf3e43',alpha=.5))
    ax.scatter([2,18],[3,16],c=['#16877c','#222222'],marker='x',s=50)
    distances=[r['leader_distance_xy_m'] for r in rows if r['policy']==policy]
    ax.set_title(f'{label}: actual leader XY, mean {statistics.mean(distances):.2f} m',fontsize=11)
    ax.set(xlim=(0,21),ylim=(0,20),xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=8)
counts=[sum(r['policy']==policy for r in rows) for policy in policies]
fig.text(.5,.025,f'Sequential repeats per policy: {counts}; identical initial route and fixed obstacle. Dots = individual runs.\nRecorded odometry, not a Gazebo GUI screenshot. Red dashed circle = required formation clearance.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.065,1,.95));fig.savefig(source.parent/'comparison.png',dpi=180)
