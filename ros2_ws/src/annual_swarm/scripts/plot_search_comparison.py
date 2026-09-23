#!/usr/bin/env python3
"""Plot the three sequential office controls from compare_search.py results."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('result_dir');p.add_argument('--map');p.add_argument('--output',required=True);args=p.parse_args()
root=Path(args.result_dir);records=json.loads((root/'results.json').read_text())
if args.map:records=[r for r in records if Path(r['map']).stem==args.map]
if not records or len({r['map'] for r in records})!=1:p.error('Select one map using --map, for example search_office')
labels={'nearest_frontier':'Nearest frontier','graph_voronoi':'Graph Voronoi','gvp_pairwise':'GVP + pairwise'}
colors={'nearest_frontier':'#dc8b25','graph_voronoi':'#7254ae','gvp_pairwise':'#16877c'}
fig,axes=plt.subplots(1,2,figsize=(13,4.8))
for record in records:
 folder=root/Path(record['summary']).parent.name;summary=json.loads((folder/'summary.json').read_text());coverage=json.loads((folder/'coverage.json').read_text())
 policy=record['policy'];label=labels[policy];color=colors[policy]
 outcome=f"T95 {summary['t95']:.1f} s" if record['passed'] else 'FAILED / TIMEOUT'
 axes[0].plot([v[0]-summary['start_time'] for v in coverage],[v[1]*100 for v in coverage],label=f'{label}: {outcome}',color=color)
 distance=sum(summary['distances_m'].values());axes[1].bar(label,distance,color=color,alpha=.8);axes[1].text(label,distance+2,f'{distance:.1f}',ha='center')
axes[0].axhline(95,color='grey',ls='--',lw=1);axes[0].axvspan(50,80,color='grey',alpha=.12)
axes[0].set(xlabel='Elapsed simulation time (s)',ylabel='Observed planar free-space coverage (%)',ylim=(0,102));axes[0].legend(fontsize=9);axes[0].grid(alpha=.2)
axes[1].set(ylabel='Total actual horizontal flight distance (m)');axes[1].set_ylim(0,axes[1].get_ylim()[1]*1.1);axes[1].grid(axis='y',alpha=.2)
fig.suptitle('Recorded Gazebo allocation-policy comparison',fontsize=15)
fig.text(.5,.02,'One run per policy; identical sensor, controller, path pool and reservations. UAV 1 paused at 50–80 s.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.06,1,.94));fig.savefig(args.output,dpi=180)
