"""Read-only OBB/AABB overlap test for the physical hull; never used by a planner."""
import numpy as np

def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def intersects(p,q,low,high):
 # Erode hull by 5 mm so numerical grazing is not counted as penetration.
 a=np.array([.315,.315,.075]);lo=np.asarray(low);hi=np.asarray(high);b=(hi-lo)/2;d=(hi+lo)/2-np.asarray(p);R=rotation(q)
 axes=[R[:,i] for i in range(3)]+[np.eye(3)[i] for i in range(3)]
 axes += [np.cross(R[:,i],np.eye(3)[j]) for i in range(3) for j in range(3)]
 for axis in axes:
  if np.dot(axis,axis)<1e-12:continue
  if abs(np.dot(d,axis))>=np.dot(a,np.abs(R.T@axis))+np.dot(b,np.abs(axis)):return False
 return True
