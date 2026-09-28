#!/usr/bin/env python3
"""Independent recorded flight checks; never equate a zero contact count with flight safety."""
import csv, hashlib, json, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]); dest=Path(sys.argv[2])
s=json.loads((root/'summary.json').read_text()); start=s['start_time']; end=s['finish_time'] or s['simulation_time']
trajectories=defaultdict(list)
with (root/'trajectory.csv').open() as stream:
    for row in csv.DictReader(stream):
        if start <= float(row['time']) <= end:
            trajectories[int(row['drone'])].append([float(row[k]) for k in ('time','x','y','z')])
true_pose={}
for drone, rows in trajectories.items():
    a=np.asarray(rows); dt=np.diff(a[:,0]); ds=np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1); ok=dt>0
    true_pose[drone]=dict(samples=len(a),first_time=float(a[0,0]),last_time=float(a[-1,0]),min_height_m=float(a[:,3].min()),max_height_m=float(a[:,3].max()),
        unexpected_landing_samples=int(np.count_nonzero(a[:,3]<.5)),maximum_sample_gap_s=float(dt.max()),maximum_pose_difference_speed_mps=float((ds[ok]/dt[ok]).max()))
errors=defaultdict(list); speed=[]; acceleration=[]
with (root/'tracking.csv').open() as stream:
    for row in csv.DictReader(stream):
        if start<=float(row['time'])<=end:
            errors[int(row['drone'])].append(float(row['error_m']))
            speed.append(np.linalg.norm([float(row[k]) for k in ('vx','vy','vz')]))
            acceleration.append(np.linalg.norm([float(row[k]) for k in ('ax','ay','az')]))
tracking={i:dict(samples=len(e),p95_error_m=float(np.quantile(e,.95)),maximum_error_m=max(e)) for i,e in errors.items()}
result=dict(task_start=start,task_end=end,truth_pose=true_pose,tracking=tracking,max_logged_reference_speed_mps=max(speed,default=0),max_logged_reference_acceleration_mps2=max(acceleration,default=0),
    no_recorded_unexpected_landings=all(v['unexpected_landing_samples']==0 for v in true_pose.values()),all_truth_streams_recorded=set(true_pose)=={0,1,2},
    note='Truth heights exclude takeoff; tracking.csv only logs active references. Sampled checks do not prove continuous collision avoidance. Reference speed and true finite-difference speed are distinct.')
dest.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
