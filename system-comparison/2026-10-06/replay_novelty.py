"""First actual team knowledge/free observation timestamps from raw sensor replay.

Online planned-view feedback and whole sensor-window novelty are different
quantities. Use actual cloud header timestamps, de-duplicate voxel IDs and
credit the lowest aircraft ID at simultaneous observations. Truth filters only
scored free cells; it is never passed to either mapper or flight planner.
"""
import json,sys
from pathlib import Path
import numpy as np
import replay_coverage as replay
from core.exploration.voxel_mapping import VoxelMap,VoxelTruth
context=dict(time=None,drone=None);tracked=[]
class TrackingMap(VoxelMap):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.first_known=np.full(self.state.size,np.inf);self.first_free=self.first_known.copy()
        self.known_source=np.full(self.state.size,-1,int);self.free_source=self.known_source.copy();tracked.append(self)
    def update(self,cells,values):
        super().update(cells,values)
        if context['time'] is None or not len(cells):return
        ids=np.ravel_multi_index(np.asarray(cells,int).T,self.shape);values=np.asarray(values)
        for mask,times,sources in [(values!=-1,self.first_known,self.known_source),(values==0,self.first_free,self.free_source)]:
            selected=np.unique(ids[mask]);t=context['time'];drone=context['drone']
            earlier=(t<times[selected])|((t==times[selected])&(drone<sources[selected]))
            changed=selected[earlier];times[changed]=t;sources[changed]=drone
original_cloud=replay.PointCloudMapper.cloud
def cloud(node,message):
    context.update(time=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,drone=node.id)
    return original_cloud(node,message)
replay.PointCloudMapper.cloud=cloud;replay.VoxelMap=TrackingMap
root=Path(sys.argv[1]);recomputed=replay.replay(root);m=tracked[0];truth=VoxelTruth(root.parent/'map.json')
free=(~truth.occupied).ravel();result=json.loads((root/'result.json').read_text())
events=[json.loads(line) for p in root.glob('drone_*/events.jsonl') for line in p.read_text().splitlines() if line.strip()]
services=[]
for e in sorted((e for e in events if e['type']=='view_observed'),key=lambda e:(e['service_start'],e['drone'])):
    start,end=e['service_start'],e['service_end'];i=e['drone']
    known=(m.first_known>start)&(m.first_known<=end)&(m.known_source==i)
    gained=(m.first_free>start)&(m.first_free<=end)&(m.free_source==i)&free
    services.append(dict(drone=i,token=e['token'],purpose=e['purpose'],region=e['region'],start=start,end=end,
        online_planned_view_team_new_cells=e['team_new_cells'],
        actual_sensor_window_team_new_known_cells=int(known.sum()),actual_sensor_window_team_new_free_cells=int(gained.sum())))
mission=(m.first_free>20.)&(m.first_free<=result['t'])&free
explore=[s for s in services if s['purpose']=='explore']
output=dict(definition='First raw-sensor observed-known and scored observed-free voxel timestamps; actual cloud time; same-time ties credit lower drone ID; each voxel counted once.',
    observation_scope='Whole sensor stream within (service_start, service_end], distinct from online planned-view target cells.',
    mission_new_team_free_cells=int(mission.sum()),mission_new_team_free_volume_m3=float(mission.sum()*.3**3),
    effective_new_free_cells_per_sim_s=float(mission.sum()/max(1e-9,result['t95']-20.)) if result.get('t95') else None,
    explore_completed_service_windows=len(explore),actual_zero_new_known_service_windows=sum(s['actual_sensor_window_team_new_known_cells']==0 for s in explore),
    services=services,coverage_replay=recomputed,
    limitations='Novelty is first evidence in recorded raw-sensor/common-mapper replay, using estimated localization. It differs from task-footprint utility and does not imply all transit or repeated rays are wasteful.')
(root/'actual-novelty.json').write_text(json.dumps(output,indent=2)+'\n')
np.savez_compressed(root/'actual-first-observation.npz',first_known=m.first_known,first_free=m.first_free,known_source=m.known_source,free_source=m.free_source)
print(json.dumps({k:v for k,v in output.items() if k not in ('services','coverage_replay')},indent=2))
