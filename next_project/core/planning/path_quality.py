"""Comparable, sampling-density-independent path quality and ranked reserves."""
from dataclasses import dataclass
import numpy as np
from .trajectory_optimizer import TrajectoryOptimizer
from .mpc_tracker import MPCFeasibilityEvaluator


def resample(path, count=80):
    p=np.asarray(path,float)
    keep=np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-8]
    p=p[keep]
    if len(p)==1:return np.repeat(p,count,axis=0)
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    return np.column_stack([np.interp(np.linspace(0,arc[-1],count),arc,p[:,i]) for i in range(3)])


class PathQualityEvaluator:
    """Safety is a hard gate; lower weighted score is better.

    Kinematic metrics describe a nominal 0.5 m/s reference, not measured tracking.
    Weights and physical normalization scales remain fixed across candidate sets.
    """
    DEFAULT_WEIGHTS={'length':1.,'clearance':.8,'turning':.15,'acceleration':.15,'jerk':.05}
    def __init__(self, weights=None, speed=.5):
        self.weights=dict(self.DEFAULT_WEIGHTS if weights is None else weights)
        if set(self.weights)!=set(self.DEFAULT_WEIGHTS) or any(not np.isfinite(v) or v<0 for v in self.weights.values()):
            raise ValueError('Quality weights require five finite nonnegative entries')
        if sum(self.weights.values())<=0:raise ValueError('At least one weight must be positive')
        self.speed=speed
    def evaluate(self,path,runtime):
        path=np.asarray(path,float)
        if not runtime.safe_path(path): raise ValueError('Unsafe path cannot enter candidate pool')
        length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
        canonical=resample(path,max(3,int(np.ceil(length/.1))+1))
        trajectory=TrajectoryOptimizer(nominal_speed=self.speed,sample_dt=.2).optimize(canonical,method='none')
        distances=np.array([runtime.field.signed_distance(p) for p in canonical])
        clearance=float(np.min(distances))
        # No obstacles is a valid map: represent unbounded obstacle distance by map diagonal.
        if not np.isfinite(clearance):clearance=float(np.linalg.norm(runtime.bounds[1]-runtime.bounds[0]))
        segments=np.diff(canonical,axis=0); unit=segments/np.maximum(np.linalg.norm(segments,axis=1,keepdims=True),1e-8)
        turning=float(np.arccos(np.clip(np.sum(unit[:-1]*unit[1:],axis=1),-1,1)).sum())
        metrics={'length_m':length,'min_clearance_m':clearance,'turning_rad':turning,
            'duration_s':float(trajectory.timestamps[-1]),'max_acceleration_m_s2':trajectory.max_acceleration,
            'mean_jerk_m_s3':trajectory.mean_jerk,'max_curvature':trajectory.max_curvature}
        terms={'length':length/max(float(np.linalg.norm(path[-1]-path[0])),1.),
            'clearance':1./max(clearance-runtime.clearance+.25,.25),
            'turning':turning/np.pi,'acceleration':trajectory.max_acceleration/3.,'jerk':trajectory.mean_jerk/10.}
        metrics['score_terms']=terms
        metrics['score']=float(sum(self.weights[k]*terms[k] for k in self.weights))
        metrics['kinematic_feasibility']=MPCFeasibilityEvaluator(max_speed=1.,max_acceleration=3.).evaluate_trajectory(trajectory).to_dict()
        return metrics


@dataclass
class Candidate:
    id: str
    planner: str
    variant: int
    path: np.ndarray
    quality: dict
    map_version: int
    def metadata(self):
        return dict(id=self.id,planner=self.planner,variant=self.variant,quality=self.quality,map_version=self.map_version)


class RankedPathPool:
    def __init__(self,backup_count=5,diversity_m=.25):
        self.backup_count=backup_count; self.diversity_m=diversity_m; self.active=None; self.backups=[]
    def rank(self,candidates):
        selected=[]
        for item in sorted(candidates,key=lambda c:(c.quality['score'],c.id)):
            if any(float(np.mean(np.linalg.norm(resample(item.path)-resample(other.path),axis=1)))<self.diversity_m for other in selected):continue
            selected.append(item)
            if len(selected)==self.backup_count+1:break
        self.active=selected[0] if selected else None; self.backups=selected[1:]
        return selected
    @staticmethod
    def connect(position,path,runtime):
        """Trim traversed prefix and check the entire joining segment, not just endpoints."""
        position=np.asarray(position,float).copy(); position[2]=runtime.altitude
        path=np.asarray(path,float)
        nearest=int(np.argmin(np.linalg.norm(path-position,axis=1)))
        # A blocked remaining suffix invalidates this reserve. Do not jump over an
        # obstruction simply to salvage it. Validate the unchanged suffix once.
        if not runtime.safe_path(path[nearest:]):return None
        for index in range(nearest,min(nearest+20,len(path))):
            if runtime.safe_path([position,path[index]]):return np.vstack([position,path[index:]])
        return None
    def revalidate(self,position,runtime,evaluator,version):
        active_id=self.active.id if self.active else None
        valid=[]; rejected=[]
        for item in ([self.active] if self.active else [])+self.backups:
            joined=self.connect(position,item.path,runtime)
            if joined is None:rejected.append(item.id);continue
            valid.append(Candidate(item.id,item.planner,item.variant,joined,evaluator.evaluate(joined,runtime),version))
        # Keep a still-safe active route; avoid score-noise induced switching.
        active=next((c for c in valid if c.id==active_id),None)
        alternatives=sorted([c for c in valid if c.id!=active_id],key=lambda c:(c.quality['score'],c.id))
        if active is None and alternatives:active=alternatives.pop(0)
        self.active=active; self.backups=alternatives[:self.backup_count]
        return active_id!=(active.id if active else None),rejected
