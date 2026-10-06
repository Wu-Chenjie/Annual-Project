"""Paired view-stage microbenchmark; no physics or exploration-time claim."""
import copy, hashlib, importlib.util, json, statistics, sys, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,'/workspace/next_project')
from core.exploration.regions import ObservationPlanner
from core.exploration.voxel_mapping import VoxelMap,VoxelRouter
from core.exploration.service_selection import choose_view_window,materialize_view_window

old_file=Path('/study/annual-efficiency-source/next_project/core/exploration/regions.py')
spec=importlib.util.spec_from_file_location('core.exploration.frozen_efficiency_regions',old_file)
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)


def template(case):
    m=VoxelMap([[0,0,0],[9,9,4]]);m.state[:]=0;m.state[20:,:,:]=-1
    if case=='obstacle':m.state[10:12,10:14,:]=1
    if case=='team_complete':known=np.ones(m.state.size,bool)
    else:known=(m.state!=-1).ravel()
    m.rebuild()
    tasks=[SimpleNamespace(id=i,viewpoints=[np.array(p) for p in points]) for i,points in enumerate([
        [[3.15,2.25,1.65],[4.05,2.25,1.65]],
        [[3.15,4.65,1.65],[4.05,4.65,1.65]],
        [[3.15,6.15,1.65],[4.05,6.15,1.65]]])]
    return m,known,tasks


def run(case,lazy):
    m,known,tasks=template(case);start=np.array([2.25,2.25,1.65]);router=VoxelRouter(m)
    planner=ObservationPlanner() if lazy else old.ObservationPlanner();module=sys.modules[ObservationPlanner.__module__] if lazy else old
    build=module.route_pool;calls=[]
    def counted(*args,**kwargs):
        calls.append(args[3]);return build(*args,**kwargs)
    module.route_pool=counted
    try:
        begin=time.perf_counter();options=[]
        for task in tasks:
            if lazy:
                choice=planner.preview(m,router,start,0.,task,observed_mask=known,history_weight=0.)
                selection=dict(objective=choice['objective'],preview=choice) if choice is not None else None
            else:
                selection=planner.plan(m,router,start,0.,task,1,observed_mask=known,history_weight=0.)
            if selection is not None:options.append((task.id,selection))
        if lazy:
            def materialize(rid,preview):
                return planner.materialize(m,start,0.,tasks[rid],1,preview['preview'],known,history_weight=0.)
            chosen,rejected=materialize_view_window(options,materialize)
        else:chosen=choose_view_window(options);rejected=[]
        elapsed=time.perf_counter()-begin
        if chosen:
            rid,s=chosen;signature=dict(region=rid,yaw=s['yaw'],gain=s['gain'],objective=s['objective'],
                active=s['pool'].active.id,path=s['pool'].active.path.tolist(),backups=[c.id for c in s['pool'].backups])
            assert all(m.safe_path(c.path) for c in [s['pool'].active]+s['pool'].backups)
        else:signature=None
        return elapsed,len(calls),signature,rejected
    finally:module.route_pool=build


def main():
    cases=[]
    for case in ['open_frontier','obstacle','team_complete']:
        run(case,False);run(case,True)
        values={'eager':[],'lazy':[]};pool_calls={};signatures=[]
        for repeat in range(11):
            observed={}
            for lazy in ([False,True] if repeat%2==0 else [True,False]):
                dt,calls,signature,rejected=run(case,lazy);label='lazy' if lazy else 'eager'
                values[label].append(dt);pool_calls[label]=calls;observed[label]=signature
                assert not rejected
            assert observed['eager']==observed['lazy'],(case,observed)
            signatures.append(observed['lazy'])
        med={k:statistics.median(v) for k,v in values.items()}
        cases.append(dict(case=case,iterations=11,wall_s=values,median_wall_s=med,
                          reduction_fraction=1-med['lazy']/med['eager'],pool_constructions=pool_calls,
                          identical_selected_view_and_reserves=True,selection=signatures[-1]))
    output=dict(cases=cases,scope='Three-task view comparison only; same real ray and route code, alternating order, selected goals/heading/gain/objective and active/reserve identities equal. No Gazebo, mission-time or low-speed claim.',
                baseline_regions_sha256=hashlib.sha256(old_file.read_bytes()).hexdigest(),
                tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                tradeoff='Fusion now estimates preview traffic on the safe topology route; final traffic, reserves, geometry, trajectory and leases are checked after materialization. Peer-dependent ranking can differ.')
    Path('/study/preview-benchmark.json').write_text(json.dumps(output,indent=2))
    print(json.dumps([{k:v for k,v in c.items() if k not in ['wall_s','selection']} for c in cases],indent=2))

if __name__=='__main__':main()
