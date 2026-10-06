"""Read-only feasibility probes; never changes the production planner."""
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time

import numpy as np
import scipy
from scipy.optimize import minimize_scalar

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'Annual-Project/next_project'))
from core.exploration.voxel_mapping import VoxelMap, VoxelRouter
from core.exploration.mrdtg import grid_tree, tree_path
from core.planning import continuous_trajectory as ct


def median_time(function, repeats=15):
    function()
    samples=[]
    for _ in range(repeats):
        begin=time.perf_counter();function();samples.append(time.perf_counter()-begin)
    return float(np.median(samples))


def stationary_scale():
    rng=np.random.default_rng(900);cases=120;max_gap=0.;max_gradient_error=0.;max_shape_error=0.
    for index in range(cases):
        points=rng.uniform(-2.,2.,(2+index%3,3))
        durations=rng.uniform(.8,6.,len(points)-1)
        delta=float(rng.uniform(-np.pi,np.pi));curve=ct.interpolate(points,durations,.2,delta)
        T=curve.duration;E=curve.jerk_cost();limits=curve.limits();lam=[0.,.035,.2][index%3]
        lower=max(.01,limits['speed']/.6,math.sqrt(limits['acceleration']/.8),
                  (limits['jerk']/2.)**(1/3),curve.yaw_rate_limit()/.65)
        scale=max(lower,(5*lam*E/T)**(1/6))
        objective=lambda s:s*T+lam*E/s**5
        numerical=minimize_scalar(objective,bounds=(lower,max(lower*2.,scale*2.)),method='bounded',options={'xatol':1e-11})
        gap=abs(objective(scale)-min(objective(lower),numerical.fun))/max(1.,objective(scale))
        max_gap=max(max_gap,gap);assert gap<1e-8
        new=ct.ContinuousTrajectory(curve.durations*scale,curve.coefficients,curve.yaw,curve.yaw_delta,
                                    curve.method,curve.yaw_coefficients)
        a=curve.sample_many(np.linspace(0,T,51));b=new.sample_many(np.linspace(0,new.duration,51))
        max_shape_error=max(max_shape_error,float(np.max(np.abs(a[0]-b[0]))))
        np.testing.assert_allclose(new.jerk_cost(),E/scale**5,rtol=1e-9,atol=1e-10)
        assert new.limits()['speed']<=.6000001 and new.limits()['acceleration']<=.8000001
        assert new.limits()['jerk']<=2.0000001 and new.yaw_rate_limit()<=.6500001
        h=1e-5;z=math.log(scale)
        fd=(objective(math.exp(z+h))-objective(math.exp(z-h)))/(2*h)
        analytic=scale*T-5*lam*E/scale**5
        max_gradient_error=max(max_gradient_error,abs(fd-analytic)/max(1.,abs(analytic)))
    moving=ct.interpolate(np.array([[0.,0.,0.],[2.,0.,0.]]),[8.],0.,.3,
        start_velocity=[.2,0.,0.],start_acceleration=[.03,0.,0.],start_yaw_rate=.1,start_yaw_acceleration=.02)
    scaled=ct.ContinuousTrajectory(moving.durations*2,moving.coefficients,moving.yaw,moving.yaw_delta,
                                   moving.method,moving.yaw_coefficients)
    assert np.linalg.norm(moving.sample(0)[1]-scaled.sample(0)[1])>.09
    assert abs(moving.sample(0)[4]-scaled.sample(0)[4])>.04
    return dict(cases=cases,max_relative_objective_difference=max_gap,max_relative_log_scale_gradient_error=max_gradient_error,
        max_geometry_difference_m=max_shape_error,nonzero_boundary_counterexample=True,
        scope='Fixed shape and segment proportions, zero endpoint derivatives; not per-segment or moving-boundary optimization.')


def direct_trajectory():
    runtime=VoxelMap([[0.,0.,0.],[10.,10.,4.]])
    runtime.state[:]=0;runtime.rebuild();runtime.safe
    original=ct.interpolate;rows=[]
    for distance,heading in [(.3,0.),(.8,.5),(1.8,-.8),(3.5,1.3)]:
        points=np.array([[2.25,2.25,1.65],[2.25+distance,2.25,1.65]])
        calls=[0]
        def counted(*args,**kwargs):
            calls[0]+=1;return original(*args,**kwargs)
        ct.interpolate=counted
        begin=time.perf_counter()
        try:old=ct.optimize_trajectory(points,runtime,0.,heading)
        finally:ct.interpolate=original
        old_time=time.perf_counter()-begin
        begin=time.perf_counter();seed=original(points,[max(distance/.4,.6)],0.,heading)
        limits=seed.limits();minimum=max(limits['speed']/.6,math.sqrt(limits['acceleration']/.8),
            (limits['jerk']/2.)**(1/3),seed.yaw_rate_limit()/.65,.4/seed.duration)
        scale=max(minimum,(5*.035*seed.jerk_cost()/seed.duration)**(1/6))*1.015
        new=ct.ContinuousTrajectory(seed.durations*scale,seed.coefficients,0.,heading,seed.method,seed.yaw_coefficients)
        assert runtime.safe_path(new.path())
        assert new.limits()['speed']<=.601 and new.limits()['acceleration']<=.801
        assert new.limits()['jerk']<=2.001 and new.yaw_rate_limit()<=.651
        new_time=time.perf_counter()-begin
        score=lambda c:c.duration+.035*c.jerk_cost()
        # Compare actual final certified curves, not the optimizer penalty value.
        assert score(new)<=score(old)*1.002
        rows.append(dict(distance_m=distance,yaw_rad=heading,current_interpolation_calls=calls[0],analytic_interpolation_calls=1,
            current_wall_s=old_time,analytic_wall_s=new_time,current_duration_s=old.duration,analytic_duration_s=new.duration,
            current_time_jerk_score=score(old),analytic_time_jerk_score=score(new)))
    return dict(cases=rows,scope='Four static straight routes in a constructed known-free map; host timings, not flight outcomes or universal speedup.')


def voxel_sets():
    rng=np.random.default_rng(901);size=16384;rows=[];checks=0
    for count in [0,1,128,1024,8192]:
        a=np.sort(rng.choice(size,count,replace=False)).astype(np.int32)
        b=np.sort(rng.choice(size,count,replace=False)).astype(np.int32)
        sa=set(map(int,a));sb=set(map(int,b))
        ma=np.zeros(size,bool);mb=ma.copy();ma[a]=True;mb[b]=True
        ba=np.packbits(ma,bitorder='little');bb=np.packbits(mb,bitorder='little')
        diff=np.setdiff1d(a,b,assume_unique=True)
        unpack=np.unpackbits(ba & ~bb,bitorder='little')[:size]
        assert set(map(int,diff))==sa-sb==set(map(int,np.flatnonzero(unpack)));checks+=1
        assert len(sa & sb)==int(np.count_nonzero(ma & mb));checks+=1
        assert len(sa | sb)==int(np.count_nonzero(ma | mb));checks+=1
        rows.append(dict(cells=count,python_set_estimated_bytes=sys.getsizeof(sa)+sum(sys.getsizeof(x) for x in sa),
            sorted_int32_payload_bytes=a.nbytes,whole_grid_bitmap_payload_bytes=ba.nbytes,
            set_difference_median_s=median_time(lambda:sa-sb),array_difference_median_s=median_time(lambda:np.setdiff1d(a,b,assume_unique=True)),
            bitmap_difference_median_s=median_time(lambda:ba & ~bb)))
    return dict(checks=checks,cases=rows,note='Bitmap timing measures bitwise difference only; output enumeration, construction, headers and conversion are excluded. Arrays are not assumed faster for every sparse set.')


def array_trees():
    runtime=VoxelMap([[0.,0.,0.],[9.,9.,4.]])
    runtime.state[:]=0;runtime.rebuild();router=VoxelRouter(runtime)
    start=np.array([2.25,2.25,1.65]);dist,parents=router.search(start,limit=6.+1e-8)
    old_dist,old_parent=grid_tree(runtime,start,router=router)
    reached=np.flatnonzero(np.isfinite(dist));assert len(old_dist)==len(reached)
    for index in reached:
        key=tuple(router.cells[index]);assert old_dist[key]==dist[index]
        assert old_parent.get(key)==(tuple(router.cells[parents[index]]) if parents[index]>=0 else None)
    checked=0
    for index in reached[::max(1,len(reached)//25)]:
        chain=[];node=int(index)
        while node>=0:
            chain.append(router.cells[node]);node=int(parents[node])
        np.testing.assert_array_equal(runtime.points(chain[::-1]),tree_path(runtime,old_parent,tuple(router.cells[index])))
        checked+=1
    dictionary_bytes=sys.getsizeof(old_dist)+sys.getsizeof(old_parent)
    dictionary_bytes+=sum(sys.getsizeof(k)+sum(sys.getsizeof(x) for x in k)+sys.getsizeof(v) for k,v in old_dist.items())
    dictionary_bytes+=sum(sys.getsizeof(k)+sys.getsizeof(v) for k,v in old_parent.items())
    return dict(reached_nodes=len(reached),checked_paths=checked,dictionary_estimated_bytes=dictionary_bytes,
        full_distance_predecessor_array_payload_bytes=dist.nbytes+parents.nbytes,
        dictionary_conversion_median_s=median_time(lambda:grid_tree(runtime,start,router=router)),
        array_reachability_median_s=median_time(lambda:np.flatnonzero(np.isfinite(dist))),
        scope='Retain existing CSR Dijkstra results; distances and reconstructed paths identical. Node IDs require router-generation binding.')


def snapshots():
    local=np.arange(128,dtype=np.int8)
    with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
        filename=Path(directory)/'snapshot.bin'
        writer=np.memmap(filename,dtype=np.int8,mode='w+',shape=local.shape);writer[:]=local;writer.flush();del writer
        snapshot=np.memmap(filename,dtype=np.int8,mode='r',shape=local.shape)
        code='''import json,sys
import numpy as np
a=np.memmap(sys.argv[1],dtype=np.int8,mode='r',shape=(128,))
blocked=False
try:a[0]=99
except ValueError:blocked=True
print(json.dumps(dict(sum=int(a.sum()),readonly=blocked)))
'''
        child=json.loads(subprocess.check_output([sys.executable,'-c',code,str(filename)],text=True))
        assert child==dict(sum=int(snapshot.sum()),readonly=True)
        local[:]=0;assert int(snapshot[1])==1
        del snapshot
    runtime=VoxelMap([[0.,0.,0.],[6.,6.,3.]])
    runtime.state[:]=0;runtime.rebuild();layer=copy.copy(runtime)
    layer.block_paths([[[2.,2.,1.5],[3.,2.,1.5]]])
    assert runtime._navigation_reservations is layer._navigation_reservations
    assert len(runtime._navigation_reservations)==1
    return dict(file_mapped_cross_process_readonly_array=True,snapshot_isolated_from_live_input=True,
        shallow_map_copy_reservation_alias_counterexample=True,
        shared_memory_probe='Named POSIX shared memory was denied by the filesystem sandbox; failed log retained. File-backed read-only mapping verified instead.',
        scope='Shared file mapping prototype works; cKDTree/CSR sharing and snapshot lifetime need separate design.')


def mathematical_bounds():
    views=[{0,1,2},{2,3},{1,4,5},{0,3,5}];checks=0
    value=lambda selected:len(set().union(*(views[i] for i in selected)))
    subsets=[set(i for i in range(4) if mask>>i&1) for mask in range(16)]
    for a,b in itertools.product(subsets,repeat=2):
        if not a<=b:continue
        for v in set(range(4))-b:
            assert value(a|{v})-value(a)>=value(b|{v})-value(b);checks+=1
    # Superset/lower incoming cost alone does not dominate the actual two-view score.
    A=set(range(100));B=set(range(10));C=set(range(10,100))
    score_A=len(A)/99.+.35*len(C-A)/1.
    score_B=len(B)/100.+.35*len(C-B)/1.
    assert B<=A and 99.<100. and score_B>score_A
    rng=np.random.default_rng(902);upper_checks=0
    for _ in range(200):
        sets=[set(map(int,rng.choice(128,int(rng.integers(1,50)),replace=False))) for _ in range(4)]
        times=rng.uniform(.5,20.,4);second_times=rng.uniform(.5,10.,(4,4));exits=rng.uniform(0.,10.,4)
        for i in range(4):
            score=len(sets[i])/times[i]+.35*max([len(sets[j]-sets[i])/second_times[i,j] for j in range(4)])-.002*exits[i]
            # Generous complete-frustum count, certified positive time bounds.
            bound=128/(times[i]*.8)+.35*128/(second_times[i].min()*.8)-.002*(exits[i]*.8)
            assert score<=bound+1e-12;upper_checks+=1
    return dict(fixed_coverage_submodularity_checks=checks,score_upper_bound_checks=upper_checks,
        naive_dominance_counterexample=dict(superset_score=score_A,subset_score=score_B),
        scope='Fixed sets only; valid candidate upper bounds must include lookahead and exit terms. No adaptive-submodularity or current-greedy approximation guarantee established.')


def temporal_and_information():
    def closest(a,start_a,b,start_b,duration=2.):
        lo=max(start_a,start_b);hi=min(start_a+duration,start_b+duration)
        if lo>hi:return float('inf')
        a0,av=map(np.asarray,a);b0,bv=map(np.asarray,b)
        relative=(a0+av*(lo-start_a))-(b0+bv*(lo-start_b));velocity=av-bv
        u=float(np.clip(-np.dot(relative,velocity)/max(np.dot(velocity,velocity),1e-12),0.,hi-lo))
        return float(np.linalg.norm(relative+u*velocity))
    a=([-1.,0.],[1.,0.]);b=([0.,-1.],[0.,1.])
    simultaneous=closest(a,0.,b,0.);separated=closest(a,0.,b,5.)
    assert simultaneous==0. and math.isinf(separated)
    # One unknown door; action costs in seconds, not hidden simulator geometry.
    p_open=.5;detour=6.;direct_open=1.;direct_closed=21.;sense=.5
    prior=min(detour,p_open*direct_open+(1-p_open)*direct_closed)
    after=sense+p_open*min(detour,direct_open)+(1-p_open)*min(detour,direct_closed)
    assert prior-after==2.
    return dict(simultaneous_crossing_detected=True,separated_crossing_unpenalized=True,
        toy_door_value_of_information_s=prior-after,
        scope='Exact linear segment timing and a one-door decision example. Real uncertainty margins, priors, calibration and cost budgets remain unverified.')


def main():
    sources=['core/exploration/regions.py','core/exploration/mrdtg.py','core/exploration/voxel_mapping.py',
        'core/exploration/fusion.py','core/exploration/priority.py','core/planning/continuous_trajectory.py']
    report=dict(environment=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,platform=platform.platform()),
        scope='Local read-only code/prototype feasibility; no production changes or new flights.',
        source_sha256={p:hashlib.sha256((ROOT/'Annual-Project/next_project'/p).read_bytes()).hexdigest() for p in sources})
    for name,function in [('stationary_scaling',stationary_scale),('direct_trajectory',direct_trajectory),
        ('voxel_sets',voxel_sets),('array_trees',array_trees),('snapshots',snapshots),
        ('mathematical_bounds',mathematical_bounds),('temporal_and_information',temporal_and_information)]:
        report[name]=function();print(name,'verified',flush=True)
    report['passed']=True
    (Path(__file__).resolve().parent/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
