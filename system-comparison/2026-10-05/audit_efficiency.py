"""Native authorization and phase checks for the paired efficiency run."""
import csv, importlib.util, json, sys
from pathlib import Path
root=Path('/study/trial-annual-efficiency-900-r1')
sys.path.insert(0,'/workspace/ros2_ws/install/annual_swarm/lib/annual_swarm')
import audit_execution_protocol as protocol
result=json.loads((root/'result.json').read_text())
rows=protocol.records
report=protocol.check([e for f in root.glob('drone_*/events.jsonl') for e in rows(f)],
    rows(root/'peer_states.jsonl'),rows(root/'execution-evidence.jsonl'),2,result['t'],
    rows(root/'command-evidence.jsonl'),task_end_time=result['t'] if result['status']=='COVERAGE_TARGET' else None)
(root/'native-protocol-audit.json').write_text(json.dumps(report,indent=2))
# Canonicalize adapter CSV column names for the existing diagnostics tool.
canonical=root/'diagnostic-input';canonical.mkdir(exist_ok=True)
summary=dict(start_time=20.,simulation_time=result['t'],finish_time=result['t'],fleet_size=2,status='COMPLETE')
(canonical/'summary.json').write_text(json.dumps(summary))
with (root/'trajectory.csv').open() as source,(canonical/'trajectory.csv').open('w') as target:
    reader=csv.DictReader(source);writer=csv.DictWriter(target,fieldnames=['time','drone','x','y','z','speed']);writer.writeheader()
    for r in reader:writer.writerow(dict(time=r['t'],drone=r['i'],**{k:r[k] for k in ['x','y','z','speed']}))
for name in ['peer_states.jsonl','execution-evidence.jsonl','drone_0','drone_1']:
    link=canonical/name
    if not link.exists():link.symlink_to(root/name)
spec=importlib.util.spec_from_file_location('diagnostics','/workspace/ros2_ws/experiments/tools/execution_diagnostics.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.diagnose(canonical,root/'execution-diagnostics')
print(json.dumps({k:report[k] for k in ['passed','errors','captured_curve_commands','handoffs']},indent=2))
raise SystemExit(0 if report['passed'] else 1)
