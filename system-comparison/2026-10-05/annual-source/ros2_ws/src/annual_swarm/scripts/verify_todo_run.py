#!/usr/bin/env python3
"""Independent final-map and sampled Gazebo-flight acceptance, for every scene."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import planning_runtime
from core.exploration.voxel_mapping import VoxelMap,VoxelTruth


def verify(directory):
    root=Path(directory);s=json.loads((root/'summary.json').read_text())
    start=s.get('start_time');end=s.get('finish_time') or s['simulation_time']
    poses={i:[] for i in range(s['fleet_size'])};errors={i:[] for i in poses};speed=[];acceleration=[]
    with (root/'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            if start is not None and start<=float(row['time'])<=end:
                poses[int(row['drone'])].append([float(row[k]) for k in ('time','x','y','z')])
    with (root/'tracking.csv').open() as stream:
        for row in csv.DictReader(stream):
            if start is not None and start<=float(row['time'])<=end:
                errors[int(row['drone'])].append(float(row['error_m']))
                speed.append(np.linalg.norm([float(row[k]) for k in ('vx','vy','vz')]))
                acceleration.append(np.linalg.norm([float(row[k]) for k in ('ax','ay','az')]))
    truth_pose={}
    for i,rows in poses.items():
        if len(rows)<2:continue
        a=np.asarray(rows);dt=np.diff(a[:,0]);ds=np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1);ok=dt>0
        truth_pose[i]=dict(samples=len(a),first_time=float(a[0,0]),last_time=float(a[-1,0]),
            min_height_m=float(a[:,3].min()),max_height_m=float(a[:,3].max()),
            unexpected_landing_samples=int(np.count_nonzero(a[:,3]<.5)),maximum_sample_gap_s=float(dt.max()),
            maximum_pose_difference_speed_mps=float((ds[ok]/dt[ok]).max()) if ok.any() else None)
    tracking={i:dict(samples=len(e),p95_error_m=float(np.quantile(e,.95)),maximum_error_m=max(e)) for i,e in errors.items() if e}
    map_result=dict(passed=False,reason='Final sensor map absent; incomplete run retained')
    if (root/'observed_final.npz').exists():
        data=np.load(root/'observed_final.npz');observed=VoxelMap(data['bounds'],resolution=float(data['resolution']))
        observed.state[:]=data['state'];truth=VoxelTruth(root/'map.json',resolution=observed.resolution)
        metrics=truth.coverage_metrics(observed)
        map_result=dict(recomputed=metrics,difference_from_summary=metrics['coverage']-s['coverage'],
            passed=abs(metrics['coverage']-s['coverage'])<1e-12 and metrics['truth_free_voxels']==s['truth_free_voxels'])
    gates=dict(all_truth_streams_recorded=len(truth_pose)==s['fleet_size'],
        no_recorded_unexpected_landings=len(truth_pose)==s['fleet_size'] and all(v['unexpected_landing_samples']==0 for v in truth_pose.values()),
        zero_contacts=s['contacts_after_takeoff']==0,
        sampled_separation=s.get('min_separation_m') is not None and s['min_separation_m']>=1.2,
        reference_speed=max(speed,default=0.)<=.601,reference_acceleration=max(acceleration,default=0.)<=.801,
        final_map_matches=map_result['passed'])
    gates={key:bool(value) for key,value in gates.items()}
    result=dict(passed=all(gates.values()),gates=gates,task_start=start,task_end=end,truth_pose=truth_pose,
        tracking=tracking,max_logged_reference_speed_mps=float(max(speed,default=0.)),
        max_logged_reference_acceleration_mps2=float(max(acceleration,default=0.)),sensor_map=map_result,
        note='Sampled truth and active-reference checks; finite-difference true speed is distinct from reference limits. Incomplete runs need not have a final map.')
    (root/'independent-acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args()
    print(json.dumps(verify(args.directory),indent=2))
