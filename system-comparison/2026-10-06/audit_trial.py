import csv,importlib.util,json,sys
from pathlib import Path
root=Path(sys.argv[1]);installed=Path('/workspace/ros2_ws/install/annual_swarm/lib/annual_swarm')
sys.path.insert(0,str(installed));import audit_execution_protocol as protocol
result=json.loads((root/'result.json').read_text());rows=protocol.records
events=[e for f in root.glob('drone_*/events.jsonl') for e in rows(f)]
report=protocol.check(events,rows(root/'peer_states.jsonl'),rows(root/'execution-evidence.jsonl'),2,result['t'],
    rows(root/'command-evidence.jsonl'),task_end_time=result['t'] if result['status']=='COVERAGE_TARGET' else None)
(root/'native-protocol-audit.json').write_text(json.dumps(report,indent=2)+'\n')
gate_errors=[]
for e in events:
    if e['type']=='handoff_authorized' and 'old_service_target_cells' in e:
        if e['old_service_observed_cells']<e['old_service_required_fraction']*len(e['old_service_target_cells']):gate_errors.append(e['token'])
(root/'handoff-gate-audit.json').write_text(json.dumps(dict(passed=not gate_errors,errors=gate_errors,
    checked=sum(e['type']=='handoff_authorized' and 'old_service_target_cells' in e for e in events),
    limitation='Checks recorded actual-evidence gate; independent native audit separately checks grants, execution and C2/yaw boundaries.'),indent=2)+'\n')
canonical=root/'diagnostic-input';canonical.mkdir(exist_ok=True)
(canonical/'summary.json').write_text(json.dumps(dict(start_time=20.,simulation_time=result['t'],finish_time=result['t'],fleet_size=2,status='COMPLETE' if result['status']=='COVERAGE_TARGET' else result['status'])))
with (root/'trajectory.csv').open() as source,(canonical/'trajectory.csv').open('w') as target:
    reader=csv.DictReader(source);writer=csv.DictWriter(target,fieldnames=['time','drone','x','y','z','speed']);writer.writeheader()
    for r in reader:writer.writerow(dict(time=r['t'],drone=r['i'],**{k:r[k] for k in ['x','y','z','speed']}))
for name in ['peer_states.jsonl','execution-evidence.jsonl','drone_0','drone_1']:
    link=canonical/name
    if not link.exists():link.symlink_to(root/name)
spec=importlib.util.spec_from_file_location('diagnostics','/workspace/ros2_ws/experiments/tools/execution_diagnostics.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.diagnose(canonical,root/'execution-diagnostics')
print(json.dumps(dict(passed=report['passed'],errors=report['errors'],handoffs=report['handoffs'],gate_errors=gate_errors),indent=2))
raise SystemExit(0 if report['passed'] and not gate_errors else 1)
