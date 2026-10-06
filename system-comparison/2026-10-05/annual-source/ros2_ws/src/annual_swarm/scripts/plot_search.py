#!/usr/bin/env python3
"""Export actual Gazebo search trajectories and coverage curves (not GUI screenshots)."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
p=argparse.ArgumentParser();p.add_argument('runs',nargs='+');p.add_argument('--output',required=True);args=p.parse_args()
fig,axes=plt.subplots(len(args.runs),2,figsize=(13,5*len(args.runs)),squeeze=False)
for row,folder in enumerate(args.runs):
 root=Path(folder);s=json.loads((root/'summary.json').read_text());mapdata=json.loads((root/'map.json').read_text())
 ax,curve=axes[row];data=list(csv.DictReader((root/'trajectory.csv').open()))
 for obs in mapdata['obstacles']:
  if obs['type']=='aabb':
   lo,hi=obs['min'],obs['max'];ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],color='#627281'))
 for i,color in enumerate(['#16877c','#dc8b25','#7254ae']):
  track=np.array([[float(r[k]) for k in ['x','y']] for r in data if int(r['drone'])==i]);ax.plot(track[:,0],track[:,1],color=color,lw=1.2,label=f'UAV {i}')
  ax.scatter(*track[0],color=color,marker='o',s=35);ax.scatter(*track[-1],color=color,marker='x',s=40)
 if '1.8 m' in mapdata.get('description',''):
  ax.annotate('1.8 m bottleneck',xy=(16,11.3),xytext=(18,13),fontsize=8,color='#b72e39',arrowprops=dict(arrowstyle='->',color='#b72e39'),bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
 ax.set(xlim=(0,24),ylim=(0,20),xlabel='x (m)',ylabel='y (m)',title=f'{root.name}: actual flight paths');ax.set_aspect('equal');ax.legend(loc='upper left',fontsize=8)
 coverage=np.array(json.loads((root/'coverage.json').read_text()));curve.plot(coverage[:,0]-s['start_time'],coverage[:,1]*100,color='#16877c')
 curve.axhline(95,color='grey',ls='--',lw=1)
 for e in s['events']:
  if e['type']=='availability_pause':curve.axvline(e['time']-s['start_time'],color='#dc8b25',ls='--',label='UAV 1 paused')
  if e['type']=='availability_resume':curve.axvline(e['time']-s['start_time'],color='#7254ae',ls='--',label='UAV 1 resumed')
 curve.set(xlabel='Elapsed simulation time (s)',ylabel='Observed free-space coverage (%)',ylim=(0,102),title=f"{s['policy']}: coverage {100*s['coverage']:.2f}%, T95 = {s['t95']} s")
 curve.grid(alpha=.2);curve.legend(loc='lower right',fontsize=9)
fig.suptitle('Recorded Gazebo search: occluded multi-room maps',fontsize=16)
fig.tight_layout(rect=(0,.025,1,.97));fig.text(.5,.01,'Ideal 3.5 m occluded ray sensing; observed-map planning; centralized task coordination. No hidden-map routes.',ha='center',fontsize=10)
fig.savefig(args.output,dpi=170)
