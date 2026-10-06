import hashlib,json,sys
from pathlib import Path
import numpy as np
from metric_helpers import metrics,service_diagnostics,records
root=Path(__file__).resolve().parent
revised=sys.argv[1] if len(sys.argv)>1 else 'candidate-900'
prefix={'candidate-refit-900':'comparison-refit','candidate-closure-900':'comparison-closure','candidate-publish-900':'comparison-publish'}.get(revised,'comparison')
names=['baseline-900-r1',revised];data={}
for name in names:
    run=root/name
    if not (run/'result.json').exists():
        data[name]=dict(completed=False,error='Missing final result');continue
    d=metrics('annual',run);d['system']=name;d['service_diagnostics']=service_diagnostics(run)
    d['original_native_protocol']=json.loads((run/'native-protocol-audit.json').read_text())
    review=run/'native-protocol-stop-reviewed.json'
    d['native_protocol']=json.loads(review.read_text()) if review.exists() else d['original_native_protocol']
    d['execution_diagnostics']=json.loads((run/'execution-diagnostics/diagnostics.json').read_text())
    d['sensor_gate']=json.loads((run/'handoff-gate-audit.json').read_text())
    data[name]=d
b,a=[data[n] for n in names]
bv=json.loads((root/'baseline-runtime-verification.json').read_text());av=json.loads((root/({'candidate-refit-900':'candidate-refit','candidate-closure-900':'candidate-closure','candidate-publish-900':'candidate-publish'}.get(revised,'candidate-final')+'-runtime-verification.json')).read_text())
protocol=json.loads((root/'protocol.json').read_text())
checks=dict(identical_physics=bv['physics_binaries']==av['physics_binaries'],installed_sources_verified=bv['passed'] and av['passed'],
    frozen_harness_unchanged=all(hashlib.sha256((root/f).read_bytes()).hexdigest()==h for f,h in protocol['frozen_harness'].items()))
if all('result' in d for d in [b,a]):
    checks.update(identical_world=b['checks']['world_sha256']==a['checks']['world_sha256'],identical_denominator=b['result']['truth_free_voxels']==a['result']['truth_free_voxels'])
    for name in names:
        d=data[name]
        checks[name+'_safety']=all([d['valid_completed_trial'],d['result']['contacts']==0,d['native_protocol']['passed'],d['sensor_gate']['passed'],
            d['checks']['reference_speed_cap'],d['checks']['reference_acceleration_cap'],d['checks']['final_coverage_replay_target'],
            not d['checks']['process_cleanup']['ros2']['remaining']])
    successful=b['valid_completed_trial'] and a['valid_completed_trial']
    gates=dict(T95=successful and a['exploration_t95_s']<b['exploration_t95_s'],
        low_speed=successful and a['low_speed_fraction']<b['low_speed_fraction'],
        distance=successful and a['mission_total_distance_m']<=1.1*b['mission_total_distance_m'],safety=all(checks.values()))
else:gates=dict(safety=False,T95=False,low_speed=False,distance=False)
output=dict(runs=data,checks=checks,development_gates=gates,all_development_gates_passed=all(gates.values()),
    scope='One fresh small-map development pair; no multi-seed, formal E01-E10 acceptance, or comparison to historical different-runtime results.')
(root/(prefix+'.json')).write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(dict(checks=checks,gates=gates,metrics={n:{k:data[n].get(k) for k in ['exploration_t95_s','mission_total_distance_m','low_speed_fraction','service_diagnostics']} for n in names}),indent=2))
if all('result' in d for d in [b,a]):
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(11,3.8),layout='constrained')
    for ax,key,label,factor in zip(axes,['exploration_t95_s','mission_total_distance_m','low_speed_fraction'],['Exploration T95 (s)','Fleet mission distance (m)','True speed <0.1 m/s (%)'],[1,1,100]):
        values=[d[key]*factor if d[key] is not None else np.nan for d in [b,a]]
        bars=ax.bar(['Baseline','Revised'],values,color=['#94a3b8','#2563eb']);ax.set_title(label);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        for bar in bars:
            if np.isfinite(bar.get_height()):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height(),f'{bar.get_height():.2f}',ha='center',va='bottom')
    fig.suptitle('Single development pair | seed 900 | identical physics, constraints and sensors')
    fig.savefig(root/(prefix+'.png'),dpi=160);fig.savefig(root/(prefix+'.svg'));plt.close(fig)
