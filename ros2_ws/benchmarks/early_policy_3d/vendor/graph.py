"""Incremental observed-free-space graph, graph-distance ownership and route reserves.

This sampled graph is an MR-DTG-inspired prototype, not the original MR-DTG.
"""
import heapq
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from core.planning import AStar
from core.planning.base import PlannerError
from core.planning.path_quality import Candidate,PathQualityEvaluator,RankedPathPool

class TopologyGraph:
    def __init__(self,runtime):
        self.runtime=runtime;self.cells=np.argwhere(runtime.safe);self.positions=runtime.points(self.cells)
        self.ids=self.cells[:,0]*runtime.shape[1]+self.cells[:,1]
        self.lookup={tuple(p):i for i,p in enumerate(self.cells)};rows=[];cols=[];weights=[]
        # Four-neighbour edges never cut corners of inflated occupied/unknown cells.
        for i,(x,y) in enumerate(self.cells):
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                j=self.lookup.get((x+dx,y+dy))
                if j is not None:rows.append(i);cols.append(j);weights.append(runtime.resolution)
        self.matrix=csr_matrix((weights,(rows,cols)),shape=(len(self.cells),len(self.cells)))
        self.version=runtime.version
    def node(self,position):
        idx=self.runtime.indices(position);exact=self.lookup.get(tuple(idx))
        if exact is not None:return exact
        if not len(self.cells):return None
        # Attachment is itself collision checked, not an unrestricted nearest snap.
        order=np.argsort(np.linalg.norm(self.positions-position,axis=1))[:12]
        return next((int(i) for i in order if self.runtime.safe_path([position,self.positions[i]])),None)
    def distances(self,positions):
        out=[]
        for p in positions:
            node=self.node(p)
            out.append(np.full(len(self.cells),np.inf) if node is None else dijkstra(self.matrix,indices=node))
        return np.array(out)

def route_pool(runtime,start,goal,task_id,epoch,max_attempts=8):
    """Original A* + penalized graph searches; shared evaluator ranks unique routes."""
    candidates=[];evaluator=PathQualityEvaluator();penalties=np.zeros(runtime.shape)
    start=np.asarray(start).copy();start[2]=runtime.altitude;goal=np.asarray(goal)
    s=tuple(runtime.indices(start));g=tuple(runtime.indices(goal))
    for attempt in range(max_attempts):
        if attempt==0:
            try:path=AStar().plan(runtime.points([s])[0],runtime.points([g])[0],runtime.grid)
            except PlannerError:continue
            if path is None:continue
            path=np.vstack([start,np.asarray(path)[:,:3],goal]);planner='astar'
        else:
            queue=[(0.,0.,s)];cost={s:0.};parent={};found=False
            while queue:
                _,c,u=heapq.heappop(queue)
                if c>cost[u]+1e-8:continue
                if u==g:found=True;break
                for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    v=(u[0]+dx,u[1]+dy)
                    if min(v)<0 or v[0]>=runtime.shape[0] or v[1]>=runtime.shape[1] or not runtime.safe[v]:continue
                    nc=c+runtime.resolution*(1.+penalties[v])
                    if nc<cost.get(v,np.inf):
                        cost[v]=nc;parent[v]=u;heapq.heappush(queue,(nc+runtime.resolution*(abs(v[0]-g[0])+abs(v[1]-g[1])),nc,v))
            if not found:break
            cells=[g]
            while cells[-1]!=s:cells.append(parent[cells[-1]])
            path=np.vstack([start,runtime.points(cells[::-1]),goal]);planner='graph_astar_penalized'
        # Only remove redundant collinear vertices. Keep genuine bends for safety.
        path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-7]]
        if len(path)<2:continue
        directions=np.diff(path,axis=0);directions/=np.maximum(np.linalg.norm(directions,axis=1,keepdims=True),1e-9)
        path=path[np.r_[True,np.linalg.norm(np.diff(directions,axis=0),axis=1)>.01,True]]
        if runtime.safe_path(path):
            candidates.append(Candidate(f'{task_id}:{epoch}:{attempt}',planner,attempt,path,evaluator.evaluate(path,runtime),runtime.version))
        idx=runtime.indices(np.vstack([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.1)+1)) for a,b in zip(path[:-1],path[1:])]))
        penalties[idx[:,0],idx[:,1]]+=3.
    pool=RankedPathPool();pool.rank(candidates)
    return pool
