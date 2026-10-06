"""Common bounded reference governor. No environment geometry or task decisions."""
import numpy as np, math

def clipnorm(v,limit):
 n=float(np.linalg.norm(v));return v if n<=limit else v*(limit/n)
class Envelope:
 def __init__(self,p):self.p=np.asarray(p,float);self.v=np.zeros(3);self.yaw=0.
 def step(self,p,v,yaw,dt):
  dt=min(.05,max(.001,dt));goal=np.asarray(p,float);feed=np.asarray(v,float)
  desired=clipnorm(feed+2.*(goal-self.p),.6)
  a=clipnorm((desired-self.v)/dt,.8)
  vn=clipnorm(self.v+a*dt,.6)
  a=(vn-self.v)/dt
  self.p=self.p+(self.v+vn)*(.5*dt);self.v=vn
  delta=math.atan2(math.sin(yaw-self.yaw),math.cos(yaw-self.yaw))
  self.yaw+=max(-.65*dt,min(.65*dt,delta))
  return self.p.tolist(),vn.tolist(),a.tolist(),self.yaw
