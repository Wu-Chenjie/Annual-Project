"""Finite-range occluded sensing. Ground truth is isolated from planning state."""
import json
import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter, convolve
from scipy.spatial import cKDTree
from core.obstacles import OccupancyGrid

class RaySensorWorld:
    def __init__(self,map_file,resolution=.25,altitude=1.5,radius=3.5):
        data=json.loads(open(map_file).read());self.bounds=np.array(data['bounds'],float)
        self.origin=self.bounds[0,:2];self.resolution=resolution;self.radius=radius
        self.shape=tuple(np.ceil((self.bounds[1,:2]-self.origin)/resolution).astype(int))
        xy=self.origin+(np.indices(self.shape).transpose(1,2,0)+.5)*resolution
        self.occupied=np.zeros(self.shape,bool)
        for o in data['obstacles']:
            if o['type']=='aabb' and o['min'][2]<=altitude<=o['max'][2]:
                self.occupied|=np.all((xy>=np.array(o['min'][:2]))&(xy<=np.array(o['max'][:2])),axis=2)
            elif o['type']=='cylinder' and o['z_range'][0]<=altitude<=o['z_range'][1]:
                self.occupied|=np.linalg.norm(xy-o['center_xy'],axis=2)<=o['radius']
        self.static_occupied=self.occupied.copy();self.last_obstacle_version=-1
        angles=np.linspace(0,2*np.pi,720,endpoint=False)
        ranges=np.arange(0,radius+resolution/2,resolution/2)
        self.rays=np.stack([np.cos(angles),np.sin(angles)],axis=1)[:,None,:]*ranges[None,:,None]
    def dynamic_snapshot(self,version,obstacles):
        if version<=self.last_obstacle_version:return
        self.occupied=self.static_occupied.copy()
        xy=self.origin+(np.indices(self.shape).transpose(1,2,0)+.5)*self.resolution
        for o in obstacles:
            if o['z_range'][0]<=1.5<=o['z_range'][1]:
                self.occupied|=np.linalg.norm(xy-o['center_xy'],axis=2)<=o['radius']
        self.last_obstacle_version=version
    def observe(self,position,yaw=None,fov=2*np.pi/3):
        rays=self.rays
        if yaw is not None:
            angles=np.linspace(0,2*np.pi,len(rays),endpoint=False)
            delta=np.arctan2(np.sin(angles-yaw),np.cos(angles-yaw))
            rays=rays[np.abs(delta)<=fov/2]
        cells=np.floor((np.asarray(position)[:2]+rays-self.origin)/self.resolution).astype(int)
        valid=np.all((cells>=0)&(cells<self.shape),axis=2)
        clipped=np.clip(cells,0,np.array(self.shape)-1)
        hit=self.occupied[clipped[:,:,0],clipped[:,:,1]]|~valid
        # Include the first hit, never any cell behind it.
        visible=valid & (np.cumsum(hit,axis=1)-hit==0)
        indices=np.unique(clipped[visible],axis=0)
        return indices,self.occupied[indices[:,0],indices[:,1]].astype(np.int8)
    def coverage(self,observed):
        return float(np.count_nonzero((observed.state==0)&~self.occupied)/np.count_nonzero(~self.occupied))

class ObservedMap:
    def __init__(self,bounds,resolution=.25,altitude=1.5,clearance=.65):
        self.bounds=np.asarray(bounds,float);self.resolution=resolution;self.altitude=altitude;self.clearance=clearance
        self.origin=self.bounds[0,:2];self.shape=tuple(np.ceil((self.bounds[1,:2]-self.origin)/resolution).astype(int))
        self.state=np.full(self.shape,-1,np.int8);self.version=0;self.field=self
        self.rebuild()
    def update(self,indices,values):
        changed=np.any(self.state[indices[:,0],indices[:,1]]!=values)
        self.state[indices[:,0],indices[:,1]]=values
        if changed:self.version+=1
        return bool(changed)
    def rebuild(self):
        # Unknown space and the map boundary are obstacles for navigation.
        free=np.pad(self.state==0,1,constant_values=False)
        self.distance=distance_transform_edt(free)[1:-1,1:-1]*self.resolution-self.resolution*np.sqrt(2)/2
        self.safe=self.distance>=self.clearance
        origin=np.r_[self.origin+self.resolution/2,self.altitude]
        self.grid=OccupancyGrid(origin,self.resolution,(*self.shape,1));self.grid.data[:,:,0]=~self.safe
    def points(self,indices):
        a=self.origin+(np.asarray(indices)+.5)*self.resolution
        return np.column_stack([a,np.full(len(a),self.altitude)])
    def indices(self,points):return np.floor((np.asarray(points)[...,:2]-self.origin)/self.resolution).astype(int)
    def signed_distance(self,point):
        idx=self.indices(point)
        if np.any(idx<0) or np.any(idx>=self.shape):return -1.
        return float(self.distance[tuple(idx)])
    def safe_path(self,path):
        p=np.asarray(path,float)
        if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all():return False
        if np.any(abs(p[:,2]-self.altitude)>.05):return False
        chunks=[p]
        for a,b in zip(p[:-1],p[1:]):chunks.append(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)))
        idx=self.indices(np.vstack(chunks))
        return bool(np.all(idx>=0) and np.all(idx<self.shape) and np.all(self.distance[idx[:,0],idx[:,1]]>=self.clearance))
    def block_paths(self,paths,radius=1.15):
        """Reserve other vehicles' complete committed paths, including hover positions."""
        points=[]
        for path in paths:
            p=np.asarray(path)
            points.extend(p)
            for a,b in zip(p[:-1],p[1:]):points.extend(np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.15)+1)))
        if points:
            indices=np.argwhere(self.safe);coords=self.points(indices)
            blocked=cKDTree(np.array(points)[:,:2]).query(coords[:,:2])[0]<radius
            self.safe[tuple(indices[blocked].T)]=False
            self.distance[tuple(indices[blocked].T)]=0.
            self.grid.data[:,:,0]=~self.safe
    def frontier_targets(self,spacing=2.0,limit=24):
        # Candidate viewpoints are safely behind observed-free/unknown boundaries.
        front=(self.state==0)&maximum_filter(self.state==-1,size=3)
        if not np.any(front) or not np.any(self.safe):return {}
        to_front=distance_transform_edt(~front)*self.resolution
        candidates=np.argwhere(self.safe&(to_front<1.6))
        if not len(candidates):return {}
        size=17;xx,yy=np.ogrid[-8:9,-8:9];kernel=(xx*xx+yy*yy<=64).astype(float)
        gains=convolve((self.state==-1).astype(float),kernel,mode='constant')
        candidates=sorted(candidates,key=lambda i:(-gains[tuple(i)],int(i[0]),int(i[1])))
        chosen={}
        for idx in candidates:
            point=self.points([idx])[0]
            if gains[tuple(idx)]<3:continue
            if any(np.linalg.norm(point-v['position'])<spacing for v in chosen.values()):continue
            key=int(idx[0]*self.shape[1]+idx[1])
            chosen[key]=dict(position=point,gain=float(gains[tuple(idx)]),cell=tuple(idx))
            if len(chosen)>=limit:break
        return chosen
