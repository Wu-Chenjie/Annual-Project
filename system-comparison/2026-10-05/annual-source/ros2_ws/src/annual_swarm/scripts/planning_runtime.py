"""ROS-independent adapters to the repository's existing planning implementations.

All paths are checked continuously in the same analytic map used to build Gazebo.
No planner substitution is hidden behind an algorithm name.
"""
import sys
from pathlib import Path
import time
import numpy as np

# Installed alongside the ROS executables; source checkout remains usable in tests.
for root in (Path(__file__).parent/'legacy', Path(__file__).resolve().parents[4]/'next_project'):
    if (root/'core').is_dir():
        sys.path.insert(0,str(root)); break
from core.map_loader import load_from_json
from core.obstacles import OccupancyGrid
from core.planning import (AStar, TurnConstrainedAStar, HybridAStar, Dijkstra,
    RRTStar, InformedRRTStar, DStarLite, GNNPlanner, VisibilityGraph,
    WindowReplanner, RiskAdaptiveReplanInterval, DualModeScheduler, CostAwareGrid, FIRIRefiner, TrajectoryOptimizer, MPCFeasibilityEvaluator)

PLANNERS = ('astar','heading_astar','hybrid_astar','dijkstra','rrt_star','informed_rrt_star','dstar_lite','gnn','window')

class PlanarGraph(VisibilityGraph):
    def __init__(self, runtime):
        super().__init__(angular_res=20, buffer_zone=runtime.clearance+0.25)
        self.runtime=runtime
    def _extract_obstacle_vertices(self, obs):
        points=super()._extract_obstacle_vertices(obs)
        unique={}
        for p in points:
            p=p.copy(); p[2]=self.runtime.altitude
            if self.runtime.safe_point(p): unique[tuple(p)]=p
        return list(unique.values())
    def is_visible(self,a,b,field,visible_range=100.):
        return bool(np.linalg.norm(a-b)<=visible_range and self.runtime.safe_path([a,b]))

class CheckedGNN(GNNPlanner):
    # Catmull-Rom in the original base class can cut through obstacles.
    # Keep the actual neural graph search, defer smoothing to the checked stage.
    def smooth(self,path,*args,**kwargs): return path

class WindowBackend:
    """Keep scheduler state across ROS ticks; publish complete checked routes."""
    def __init__(self,runtime,start,goal):
        self.runtime=runtime
        class Local(HybridAStar):
            def smooth(self,path,*args,**kwargs): return path
        class Danger(CheckedGNN):
            def plan(self,start,goal,grid,**kw):
                graph=PlanarGraph(runtime); graph.build(start,goal,runtime.field,100.)
                return super().plan(start,goal,grid,visibility_graph=graph)
        self.scheduler=WindowReplanner(Local(v_z_max=0),runtime.grid,interval=.4,horizon=6.,
            global_planner=InformedRRTStar(max_iter=2500,smooth_method='none'),
            incremental_planner=DStarLite(runtime.grid,start,goal),
            danger_planner=Danger(),dual_mode=DualModeScheduler(),
            adaptive_interval=RiskAdaptiveReplanInterval(),obstacle_field=runtime.field,
            voronoi_region_enabled=True)
        self.goal=goal.copy(); self.path=None
    def plan(self,start,goal,t):
        path=self.scheduler.step(t,start,None,goal)
        events=self.scheduler.get_new_events()
        if path is None:
            if self.path is None: raise ValueError('Window scheduler produced no initial route')
            path=self.path
        path=np.asarray(path,float).copy(); path[:,2]=self.runtime.altitude
        if np.linalg.norm(path[-1]-goal)>.1:
            ref=self.scheduler._global_ref_path
            if ref is None: raise ValueError('Local window has no global continuation')
            closest=int(np.argmin(np.linalg.norm(ref-path[-1],axis=1)))
            path=np.vstack([path,ref[closest:]])
        if not self.runtime.safe_path(path): raise ValueError('Window continuation failed clearance gate')
        self.path=path
        return path,events

class Runtime:
    def __init__(self,map_file,algorithm='astar',altitude=1.5,clearance=1.9,
                 esdf=False,firi=False,trajectory='none',seed=42):
        if algorithm not in PLANNERS: raise ValueError(f'Unknown planner {algorithm}')
        if trajectory not in ('none','moving_average','minimum_jerk','min_snap_proxy','min_jerk_cost'):
            raise ValueError(f'Unknown trajectory method {trajectory}')
        self.field,self.bounds=load_from_json(map_file)
        self.algorithm=algorithm; self.altitude=altitude; self.clearance=clearance
        self.use_esdf=esdf; self.use_firi=firi; self.trajectory_method=trajectory; self.seed=seed
        self.resolution=.25
        origin=self.bounds[0].copy(); origin[2]=altitude
        shape=tuple((np.floor((self.bounds[1,:2]-origin[:2])/self.resolution).astype(int)+1).tolist()+[1])
        self.grid=OccupancyGrid(origin,self.resolution,shape)
        for idx in np.ndindex(shape):
            # Half-cell diagonal margin also protects discretized edge checks.
            self.grid.data[idx]=not self.safe_point(self.grid.index_to_world(idx),margin=.18)
        self.search_grid=CostAwareGrid(self.grid) if esdf else self.grid
        if esdf and algorithm not in ('astar','heading_astar','hybrid_astar'):
            raise ValueError('ESDF soft costs are supported by A* variants only')
        self.dstar=None; self.dstar_goal=None; self.calls=0; self.window=None

    def safe_point(self,p,margin=0.):
        p=np.asarray(p,dtype=float)
        return bool(np.isfinite(p).all() and
            np.all(p[:2]>=self.bounds[0,:2]+self.clearance) and
            np.all(p[:2]<=self.bounds[1,:2]-self.clearance) and
            self.bounds[0,2]+.46<=p[2]<=self.bounds[1,2]-.46 and
            self.field.signed_distance(p)>=self.clearance+margin-1e-8)

    def safe_path(self,path):
        p=np.asarray(path,dtype=float)
        if p.ndim!=2 or p.shape[1]!=3 or not len(p): return False
        for a,b in zip(p[:-1],p[1:]):
            for t in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.04))+1)):
                if not self.safe_point(a+t*(b-a)): return False
        return all(self.safe_point(x) for x in p)

    def check_takeoff(self,start):
        for dy in (0,-1,1):
            for z in np.arange(.1,start[2]+.02,.02):
                p=np.array([start[0],start[1]+dy,z])
                if self.field.signed_distance(p)<.46: raise ValueError('Unsafe takeoff column')

    def plan(self,start,goal,t=0.):
        begin=time.monotonic(); start=np.asarray(start,float); goal=np.asarray(goal,float)
        if abs(goal[2]-self.altitude)>1e-6: raise ValueError('Current mission uses a fixed flight altitude')
        start=start.copy(); start[2]=self.altitude
        for p in (start,goal):
            if not self.safe_point(p) or self.grid.is_occupied(self.grid.world_to_index(p)):
                raise ValueError('Endpoint violates formation clearance')
        factories={'astar':AStar,'heading_astar':TurnConstrainedAStar,'dijkstra':Dijkstra,
            'hybrid_astar':lambda:HybridAStar(v_z_max=0,max_iter=25000),
            'rrt_star':lambda:RRTStar(max_iter=2500,smooth_method='none'),
            'informed_rrt_star':lambda:InformedRRTStar(max_iter=2500,smooth_method='none'),
            'gnn':CheckedGNN}
        reused=False; events=[]
        if self.algorithm=='window':
            if self.window is None or not np.array_equal(goal,self.window.goal):
                self.window=WindowBackend(self,start,goal)
            path,events=self.window.plan(start,goal,t)
        elif self.algorithm=='dstar_lite':
            if self.dstar is None or not np.array_equal(goal,self.dstar_goal):
                self.dstar=DStarLite(self.grid,start,goal); self.dstar_goal=goal.copy()
            else:
                self.dstar.update_start(start); reused=True
            self.dstar.compute_shortest_path(); path=self.dstar.extract_path()
        else:
            planner=factories[self.algorithm](); kw={'seed':self.seed}
            if self.algorithm=='gnn':
                graph=PlanarGraph(self); graph.build(start,goal,self.field,100.)
                kw['visibility_graph']=graph
            path=planner.plan(start,goal,self.search_grid,**kw)
        if path is None or len(path)<2: raise ValueError('Planner did not produce a route')
        path=np.asarray(path,float)[:,:3]; path[:,2]=self.altitude
        path=np.vstack([start,path,goal])
        path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-8]]
        if not self.safe_path(path): raise ValueError('Planner route failed continuous formation-clearance validation')
        report={'algorithm':self.algorithm,'implementation':'next_project/core/planning',
            'window_events':events,'sensor_source':'known static map; Gazebo odometry','dstar_state_reused':reused,'esdf':self.use_esdf,'firi':'disabled','fallbacks':[]}
        if self.use_firi:
            candidate=FIRIRefiner(self.field,min_clearance=self.clearance,mvie_enabled=False).refine(path)
            candidate[:,2]=self.altitude
            if self.safe_path(candidate): path=candidate; report['firi']='accepted (inscribed-ball corridor)'
            else: report['fallbacks'].append('FIRI rejected by continuous clearance gate')
        trajectory=TrajectoryOptimizer(nominal_speed=.5).optimize(path,
            clearance_checker=self.safe_path,method=self.trajectory_method)
        if self.safe_path(trajectory.positions): path=trajectory.positions
        else:
            report['fallbacks'].append('Trajectory rejected by continuous clearance gate')
            trajectory=TrajectoryOptimizer(nominal_speed=.5).optimize(path,method='none')
        if self.window is not None:
            report['window_phase']=self.window.scheduler.phase
            report['window_fallback']=self.window.scheduler.last_fallback_reason
            report['window_interval']=self.window.scheduler._get_current_interval()
        report['trajectory_method']=trajectory.method
        report['trajectory_fallback']=trajectory.fallback_reason
        report['mpc_feasibility_only']=MPCFeasibilityEvaluator(max_speed=1.,max_acceleration=3.).evaluate_trajectory(trajectory).to_dict()
        self.calls+=1; report.update(plan_count=self.calls,wall_seconds=time.monotonic()-begin,points=len(path))
        return path,report
