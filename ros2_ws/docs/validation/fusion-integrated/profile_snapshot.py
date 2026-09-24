import json,time,sys
from pathlib import Path
import numpy as np
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.fusion import FusionPlanner
snapshot=np.load(sys.argv[1]);m=VoxelMap(snapshot['bounds'],float(snapshot['resolution']),flight_limits=(.7,3.1))
m.state[:]=snapshot['state'];m.version=1;m.rebuild();p=FusionPlanner(0,m.bounds);rows=[]
for k in range(5):
    begin=time.monotonic()
    p.compute(m,snapshot['position'],float(snapshot['yaw']),None,[],{},[],k+1,2400.+k*2,True,{},{},0.)
    rows.append(dict(wall=time.monotonic()-begin,tasks=len(p.tasks),diagnostics=p.diagnostics))
Path(sys.argv[2]).write_text(json.dumps(rows,indent=2))
print(json.dumps([dict(wall=r['wall'],stages=r['diagnostics']['stage_wall_s']) for r in rows]))
