"""Build ranked alternatives from one or several original planning algorithms."""
import copy
import numpy as np
from planning_runtime import Runtime,PLANNERS,CostAwareGrid
from core.obstacles import Cylinder
from core.planning.path_quality import Candidate,PathQualityEvaluator,RankedPathPool


def apply_obstacles(runtime,obstacles):
    runtime.field=copy.deepcopy(runtime.static_field)
    for o in obstacles:
        runtime.field.add(Cylinder(np.array(o['center_xy'],float),float(o['radius']),tuple(o['z_range'])))
    data=runtime.static_data.copy()
    points=runtime.grid_points
    for o in obstacles:
        radial=np.linalg.norm(points[:,:2]-np.array(o['center_xy']),axis=1)-float(o['radius'])
        z0,z1=o['z_range'];vertical=np.abs(points[:,2]-(z0+z1)/2)-(z1-z0)/2
        distances=np.minimum(np.maximum(radial,vertical),0)+np.linalg.norm(np.maximum(np.column_stack([radial,vertical]),0),axis=1)
        data|=(distances<runtime.clearance+.18-1e-8).reshape(runtime.grid.shape).astype(np.uint8)
    runtime.grid.data[:]=data
    runtime.search_grid=CostAwareGrid(runtime.grid) if runtime.use_esdf else runtime.grid
    # A new obstacle snapshot invalidates planner-internal cached map state.
    runtime.dstar=None;runtime.window=None
    return runtime


def make_runtime(map_file,obstacles=(),**kwargs):
    runtime=Runtime(map_file,**kwargs);runtime.static_field=copy.deepcopy(runtime.field)
    runtime.static_data=runtime.grid.data.copy()
    runtime.grid_points=np.array([runtime.grid.index_to_world(i) for i in np.ndindex(runtime.grid.shape)])
    if obstacles:apply_obstacles(runtime,obstacles)
    return runtime


def generate_candidates(map_file,start,goal,algorithms,variants,version,obstacles,clearance=1.9,weights=None,generation=0):
    if not algorithms or any(a not in PLANNERS for a in algorithms):raise ValueError('Unknown candidate planner')
    evaluator=PathQualityEvaluator(weights);candidates=[];failures=[]
    start=np.array(start,float);goal=np.array(goal,float)
    direction=goal-start;normal=np.array([-direction[1],direction[0],0.]);normal/=max(np.linalg.norm(normal),1e-9)
    for algorithm in algorithms:
        runtime=make_runtime(map_file,obstacles,algorithm=algorithm,altitude=start[2],clearance=clearance)
        for variant in range(variants):
            runtime.seed=42+variant*97
            try:
                if variant==0:path,_=runtime.plan(start,goal)
                else:
                    # Deterministic search also needs genuinely different routes, not repeated seeds.
                    offset=(-1 if variant%2 else 1)*(2.+2.*((variant-1)//2))
                    waypoint=(start+goal)*.5+normal*offset
                    first,_=runtime.plan(start,waypoint);second,_=runtime.plan(waypoint,goal)
                    path=np.vstack([first,second[1:]])
                quality=evaluator.evaluate(path,runtime)
                candidates.append(Candidate(f'm{version}:g{generation}:{algorithm}:{variant}',algorithm,variant,path,quality,version))
            except Exception as exc:failures.append(dict(planner=algorithm,variant=variant,reason=str(exc)))
    pool=RankedPathPool();pool.rank(candidates)
    return pool,failures,len(candidates)


def regenerate_single(runtime,start,goal,version,weights,generation):
    """Fast baseline: one A* call plus the identical quality/safety gate."""
    runtime.algorithm='astar'
    path,_=runtime.plan(start,goal)
    item=Candidate(f'm{version}:g{generation}:astar:0','astar',0,path,PathQualityEvaluator(weights).evaluate(path,runtime),version)
    pool=RankedPathPool();pool.rank([item])
    return pool,[],1
