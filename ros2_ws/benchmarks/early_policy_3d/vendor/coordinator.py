"""Centralized exploration prototype. All routing uses observed map only."""
import copy
import time
import numpy as np
from .mapping import ObservedMap
from .graph import TopologyGraph,route_pool
from .allocation import allocate
from core.planning.path_quality import PathQualityEvaluator

class SearchCoordinator:
    def __init__(self,bounds,policy='gvp_pairwise'):
        self.map=ObservedMap(bounds);self.policy=policy;self.active={};self.pools={};self.routes={};self.epochs={}
        self.previous={};self.events=[];self.visited=[];self.clock=0.;self.sequence=0;self.stats={};self.cooldown={}
        self.pending_transfers={};self.task_positions={};self.assignment_round=0
        self.graph_nodes=set();self.graph_edges=set();self.topology_delta={}
    def release(self,i,reason):
        task=self.active.pop(i,None);self.pools.pop(i,None)
        if task is not None:
            self.events.append(dict(type='task_released',drone=i,task=task,reason=reason,time=self.clock))
            if reason in ('unavailable','blocked'):self.pending_transfers[task]=i
    @staticmethod
    def remainder(position,path):
        if len(path)<2:return np.vstack([position,path])
        a=path[:-1];delta=path[1:]-a
        fraction=np.clip(np.sum((position-a)*delta,axis=1)/np.maximum(np.sum(delta*delta,axis=1),1e-9),0,1)
        projected=a+fraction[:,None]*delta
        k=int(np.argmin(np.linalg.norm(projected-position,axis=1)))
        return np.vstack([position,projected[k],path[k+1:]])
    def reserved_runtime(self,i,positions):
        snapshot=copy.deepcopy(self.map);reservations=[]
        for j,p in positions.items():
            if j==i:continue
            if j in self.pools and self.pools[j].active is not None:
                reservations.append(self.remainder(p,self.pools[j].active.path))
            else:reservations.append([p])
        snapshot.block_paths(reservations)
        return snapshot
    def step(self,positions,available,arrived,observations,t):
        begin=time.monotonic();self.clock=t;commands={}
        for indices,values in observations:self.map.update(indices,values)
        self.map.rebuild()
        for i in list(self.active):
            if i not in available:self.release(i,'unavailable');commands[i]=None;continue
            if arrived.get(i,False):
                task=self.active[i];self.visited.append((self.task_positions[task].copy(),t))
                self.events.append(dict(type='viewpoint_observed',drone=i,task=task,time=t))
                self.release(i,'observed');commands[i]=None
        for i in list(self.active):
            pool=self.pools[i];remaining=self.remainder(positions[i],pool.active.path)
            if not self.map.safe_path(remaining):
                # Validate reserves against observed obstacles AND other committed routes.
                snapshot=self.reserved_runtime(i,positions)
                changed,rejected=pool.revalidate(positions[i],snapshot,PathQualityEvaluator(),self.map.version)
                if pool.active is not None:
                    self.epochs[i]+=1
                    commands[i]=dict(task=self.active[i],epoch=self.epochs[i],map_version=self.map.version,path=pool.active.path.tolist())
                    self.events.append(dict(type='backup_switch',drone=i,task=self.active[i],time=t,rejected=rejected))
                else:
                    self.release(i,'blocked');commands[i]=None
        graph=TopologyGraph(self.map)
        nodes=set(map(int,graph.ids));coo=graph.matrix.tocoo()
        edges={(int(graph.ids[a]),int(graph.ids[b])) for a,b in zip(coo.row,coo.col) if a<b}
        self.sequence+=1
        self.topology_delta=dict(sequence=self.sequence,map_version=self.map.version,resolution=self.map.resolution,
            nodes_added=[dict(id=int(k),position=p.tolist()) for k,p in zip(graph.ids,graph.positions) if int(k) not in self.graph_nodes],
            nodes_removed=sorted(self.graph_nodes-nodes),edges_added=sorted(edges-self.graph_edges),edges_removed=sorted(self.graph_edges-edges))
        self.graph_nodes=nodes;self.graph_edges=edges
        if not len(graph.cells):return commands
        targets=self.map.frontier_targets()
        targets={k:v for k,v in targets.items() if not any(np.linalg.norm(v['position']-p)<.75 and t-when<20 for p,when in self.visited)}
        for i,task in self.active.items():
            point=self.task_positions[task]
            if graph.node(point) is not None:targets[task]=dict(position=point,gain=0)
        # Preserve an interrupted viewpoint until another drone can take ownership.
        for task in list(self.pending_transfers):
            point=self.task_positions[task]
            if graph.node(point) is not None:targets[task]=dict(position=point,gain=0)
        self.task_positions.update({k:v['position'] for k,v in targets.items()})
        drones=sorted(available);tasks=sorted(targets);indices=[graph.node(targets[k]['position']) for k in tasks]
        tasks=[k for k,idx in zip(tasks,indices) if idx is not None];indices=[idx for idx in indices if idx is not None]
        if not drones or not tasks:
            self.stats=dict(nodes=len(graph.cells),edges=graph.matrix.nnz//2,frontiers=len(tasks),map_version=self.map.version)
            return commands
        distance=graph.distances([positions[i] for i in drones]);cost=distance[:,indices]
        tc=graph.distances([targets[k]['position'] for k in tasks])[:,indices]
        pinned={task:i for i,task in self.active.items()}
        self.routes,report=allocate(cost,tc,tasks,drones,self.policy,self.previous,pinned,workload=[1.+targets[k]["gain"]*.01 for k in tasks])
        owners={task:i for i,route in self.routes.items() for task in route}
        self.previous=owners;self.assignment_round+=1
        # Cyclic priority prevents a permanently low-priority UAV from starvation.
        idle=[i for i in drones if i not in self.active]
        idle.sort(key=lambda i:(self.epochs.get(i,0),i))
        planned=False
        for i in idle:
            if planned:break # Bound planning cost per tick; other aircraft continue independently.
            if t<self.cooldown.get(i,0):continue
            route=self.routes.get(i,[])
            transfer=[k for k in self.pending_transfers if k in tasks and self.pending_transfers[k]!=i and np.isfinite(cost[drones.index(i),tasks.index(k)])]
            choices=transfer+[k for k in route if k not in transfer]
            for task in choices[:4]:
                if task in self.active.values():continue
                snapshot=self.reserved_runtime(i,positions)
                if not snapshot.safe_path([positions[i]]) or not snapshot.safe_path([targets[task]['position']]):continue
                epoch=self.epochs.get(i,0)+1
                pool=route_pool(snapshot,positions[i],targets[task]['position'],task,epoch)
                if pool.active is None:continue
                self.epochs[i]=epoch;self.active[i]=task;self.pools[i]=pool
                commands[i]=dict(task=task,epoch=epoch,map_version=self.map.version,path=pool.active.path.tolist())
                self.events.append(dict(type='task_assigned',drone=i,task=task,time=t,backup_count=len(pool.backups)))
                if task in self.pending_transfers:
                    old=self.pending_transfers.pop(task)
                    self.events.append(dict(type='task_transferred' if old!=i else 'task_replanned',old_drone=old,drone=i,task=task,time=t))
                planned=True;break
            if i not in self.active:self.cooldown[i]=t+2.
        self.stats=dict(nodes=len(graph.cells),edges=graph.matrix.nnz//2,frontiers=len(tasks),map_version=self.map.version,
            allocation=report,active={str(k):v for k,v in self.active.items()},round=self.assignment_round,
            wall_seconds=time.monotonic()-begin,backup_counts={str(i):len(p.backups) for i,p in self.pools.items()})
        return commands
