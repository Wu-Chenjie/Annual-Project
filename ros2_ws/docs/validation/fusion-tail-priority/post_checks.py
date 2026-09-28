"""Retain full-acceptance failure separately from measured flight/map checks."""
import json,subprocess,sys,traceback,hashlib
from pathlib import Path
import numpy as np
import audit_fusion_run
from core.exploration.voxel_mapping import VoxelMap,VoxelTruth

root=Path(sys.argv[1])
s=json.loads((root/'summary.json').read_text())
try:
    audit_fusion_run.audit(root)
    acceptance=dict(passed=True)
except Exception as exc:
    acceptance=dict(passed=False,exception=repr(exc),traceback=traceback.format_exc(),
                    recorded_status=s['status'],coverage=s['coverage'],t90=s['t90'],t95=s['t95'])
(root/'full-acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
flight=Path(__file__).with_name('audit_flight.py')
subprocess.run([sys.executable,str(flight),str(root),str(root/'flight-audit.json')],check=True,stdout=(root/'flight-audit.log').open('w'))
snapshot=root/('observed_final.npz' if (root/'observed_final.npz').exists() else 'observed_latest.npz')
if snapshot.exists():
    data=np.load(snapshot)
    observed=VoxelMap(data['bounds'],resolution=float(data['resolution']))
    observed.state[:]=data['state']
    truth=VoxelTruth(root/'map.json',resolution=observed.resolution)
    measured=truth.coverage_metrics(observed)
    result=dict(snapshot=snapshot.name,recomputed=measured,summary_coverage=s['coverage'],
        difference_from_summary=measured['coverage']-s['coverage'],
        note='observed_latest is captured after the recorder stop condition; the sample may differ from the last summary. observed_final is frozen by the experiment at termination.')
    if snapshot.name=='observed_final.npz':
        assert abs(result['difference_from_summary'])<1e-12
    assert measured['truth_free_voxels']==58589
    (root/'sensor-map-audit.json').write_text(json.dumps(result,indent=2)+'\n')
else:
    (root/'sensor-map-audit.json').write_text(json.dumps(dict(passed=False,reason='No final sensor map retained'),indent=2)+'\n')
print(json.dumps(dict(full_acceptance=acceptance['passed'],status=s['status'],coverage=s['coverage'],sensor_map=snapshot.exists())))
