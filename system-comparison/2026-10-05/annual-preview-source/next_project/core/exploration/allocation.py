"""Centralized, versioned GVP ownership with bounded pairwise route improvement."""
import numpy as np


def allocate(drone_cost,task_cost,task_ids,drone_ids,policy='gvp_pairwise',previous=None,pinned=None,workload=None):
    if policy not in ('nearest_frontier','graph_voronoi','gvp_pairwise'):raise ValueError(policy)
    previous=previous or {};pinned=pinned or {};n=len(drone_ids);m=len(task_ids)
    if not n:return {},dict(objective=0.,moves=0,unreachable=list(task_ids))
    d=np.asarray(drone_cost);between=np.asarray(task_cost);owners={};unreachable=[]
    service=np.ones(m) if workload is None else np.asarray(workload,float)
    if service.shape!=(m,) or not np.isfinite(service).all() or np.any(service<0):raise ValueError('Invalid task workload')
    for j,task in enumerate(task_ids):
        if not np.isfinite(d[:,j]).any():unreachable.append(task);continue
        keep=pinned.get(task)
        owners[j]=drone_ids.index(keep) if keep in drone_ids and np.isfinite(d[drone_ids.index(keep),j]) else int(np.argmin(d[:,j]))
    def routes(assignment):
        result={};loads=[]
        for i,drone in enumerate(drone_ids):
            remaining=[j for j,owner in assignment.items() if owner==i];order=[];total=0.
            locked=[j for j in remaining if pinned.get(task_ids[j])==drone]
            while remaining:
                if locked:j=locked.pop(0)
                else:j=min(remaining,key=lambda j:((d[i,j] if not order else between[order[-1],j]),task_ids[j]))
                cost=d[i,j] if not order else between[order[-1],j]
                total+=float(cost)/.6+service[j] # nominal transit + observation work proxy
                order.append(j);remaining.remove(j)
            result[drone]=[task_ids[j] for j in order];loads.append(total)
        return result,loads
    def objective(assignment):
        _,loads=routes(assignment)
        churn=sum(previous.get(task_ids[j],drone_ids[i])!=drone_ids[i] for j,i in assignment.items())
        return max(loads,default=0)+.2*sum(loads)+1.5*churn
    moves=0;initial=objective(owners)
    if policy=='gvp_pairwise':
        # Finite unilateral transfers and pair swaps; each accepted update decreases J.
        for _ in range(12):
            old=objective(owners);best=old;choice=None
            movable=[j for j in owners if task_ids[j] not in pinned]
            for j in movable:
                for i in range(n):
                    if i==owners[j] or not np.isfinite(d[i,j]):continue
                    trial=dict(owners);trial[j]=i;score=objective(trial)
                    if score<best-1.:best=score;choice=trial
            for a,j in enumerate(movable):
                for k in movable[a+1:]:
                    if owners[j]==owners[k]:continue
                    if not np.isfinite(d[owners[k],j]) or not np.isfinite(d[owners[j],k]):continue
                    trial=dict(owners);trial[j],trial[k]=owners[k],owners[j];score=objective(trial)
                    if score<best-1.:best=score;choice=trial
            if choice is None:break
            owners=choice;moves+=1
    result,loads=routes(owners)
    if policy=='nearest_frontier':
        # Exclusive nearest available task per UAV; no future-route optimization.
        result={i:[] for i in drone_ids};remaining=set(owners)
        for j in list(remaining):
            owner=pinned.get(task_ids[j])
            if owner in result:result[owner].append(task_ids[j]);remaining.remove(j)
        for _,i,j in sorted((float(d[i,j]),i,j) for i in range(n) for j in remaining if np.isfinite(d[i,j])):
            if not result[drone_ids[i]] and j in remaining:result[drone_ids[i]]=[task_ids[j]];remaining.remove(j)
    return result,dict(objective=objective(owners),initial_objective=initial,moves=moves,loads=loads,unreachable=unreachable)
