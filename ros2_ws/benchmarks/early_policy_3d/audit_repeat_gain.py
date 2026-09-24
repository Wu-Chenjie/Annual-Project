"""Posthoc probe of recorded repeated goals, not used by the running planner."""
import sys,json,collections,hashlib,math
from pathlib import Path
import numpy as np
from adapter import FrontierVoxelMap,target_heading
from core.exploration.regions import visible_cells
root=Path(sys.argv[1]);out=Path(sys.argv[2]);saved=np.load(root/'observed_latest.npz');runtime=FrontierVoxelMap(saved['bounds']);runtime.state[:]=saved['state'];runtime.rebuild()
summary=json.loads((root/'summary.json').read_text());counts=collections.Counter()
for line in (root/'policy-events.jsonl').read_text().splitlines():
 e=json.loads(line)
 if e['type']=='historical_policy_event':
  v=e['event']
  if v['type']=='task_assigned' and v['time']>=summary['start_time']+300:counts[(v['drone'],v['task'])]+=1
unknown=runtime.points(np.argwhere(runtime.state==-1));rows=[]
for (drone,task),count in counts.most_common(5):
 point=runtime.points([np.unravel_index(task,runtime.shape)])[0];yaw=target_heading(runtime,point,0)
 angles=np.arange(12)*math.pi/6;visible=[len(visible_cells(runtime,point,angle)) for angle in angles]
 rows.append(dict(drone=drone,task=task,assignments_after_300=count,position=point.tolist(),sphere_unknown_voxels_within_2m=int(np.count_nonzero(np.linalg.norm(unknown-point,axis=1)<=2.)),
  adapted_yaw_rad=yaw,finite_fov_predicted_unknown_surface_cells_at_yaw=len(visible_cells(runtime,point,yaw)),best_predicted_unknown_surface_cells_over_12_yaws=max(visible)))
result=dict(recorded_map_sha256=hashlib.sha256((root/'observed_latest.npz').read_bytes()).hexdigest(),map_time=summary['simulation_time'],rows=rows,
 note='Posthoc estimate on the final recorded sensor map using the common visibility routine; no simulator truth or runtime policy changes. Sphere volume counts and ray-surface counts have different units. A proxy comparison is not a causal ablation or an actual measurement of information gained.')
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
