#!/usr/bin/env python3
"""Centralized historical policy connected only to live estimated sensor data."""
import copy
import json
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from adapter import FrontierVoxelMap,plan_step,archived
from core.planning.continuous_trajectory import ContinuousTrajectory


class Coordinator(Node):
    def __init__(self):
        super().__init__('early_policy_coordinator')
        self.bounds=json.loads(self.declare_parameter('bounds','[]').value); self.map=FrontierVoxelMap(self.bounds)
        self.output=Path(self.declare_parameter('output_dir','').value); self.output.mkdir(parents=True,exist_ok=True)
        self.log=(self.output/'policy-events.jsonl').open('a'); self.candidates=(self.output/'candidate_paths.jsonl').open('a')
        self.positions={};self.yaws={};self.states={};self.sessions={};self.pending={};self.rejected=set();self.active={};self.core_epochs={}
        self.control_state={};self.future=None;self.submitted=-1e9;self.stats={};self.serial=0;self.calls=0
        self.worker=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs={i:self.create_publisher(String,f'/baseline/command_{i}',q) for i in range(3)}
        self.boot={i:self.create_publisher(String,f'/drone_{i}/map_bootstrap',q) for i in range(3)}
        self.create_subscription(String,'/experiment/control',self.control,q)
        for i in range(3):
            self.create_subscription(String,f'/drone_{i}/observation',lambda m,i=i:self.observe(i,m),q)
            self.create_subscription(String,f'/drone_{i}/peer_state',lambda m,i=i:self.peer(i,m),q)
            self.create_subscription(Odometry,f'/drone_{i}/estimated_odometry',lambda m,i=i:self.odom(i,m),qos_profile_sensor_data)
        self.create_timer(.2,self.heartbeat);self.create_timer(.5,self.tick)

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def event(self,kind,**data):
        self.log.write(json.dumps(dict(type=kind,time=self.now(),**data))+'\n');self.log.flush()
    def odom(self,i,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation;self.positions[i]=np.array([p.x,p.y,p.z])
        self.yaws[i]=float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def peer(self,i,m):
        p=json.loads(m.data)
        if self.sessions.get(i)!=p['session']:
            if i in self.sessions:
                self.cancel(i,'client_session_changed');self.rejected.add(i)
            self.sessions[i]=p['session'];self.event('client_session',drone=i,session=p['session'])
            self.boot[i].publish(String(data=json.dumps(dict(session=p['session']))))
        self.states[i]=p
    def observe(self,i,m):
        if i in self.control_state.get('isolated',[]):return
        p=json.loads(m.data);cells=np.asarray(p['indices'],int).reshape(-1,3);values=np.asarray(p['values'],np.int8)
        self.map.update(cells,values)
        self.pending.update({tuple(c):int(v) for c,v in zip(cells,values)})
    def control(self,m):
        p=json.loads(m.data);old=set(self.control_state.get('isolated',[]));self.control_state=p
        for i in old-set(p.get('isolated',[])):
            self.boot[i].publish(String(data=json.dumps(dict(session=self.sessions.get(i)))))
        for i in set(p.get('paused',[]))|set(p.get('isolated',[]))|set(p.get('restart',[])):
            self.cancel(i,'experiment_control')
        if p.get('done'):
            for i in list(self.active):self.cancel(i,'experiment_complete')
    def cancel(self,i,reason):
        a=self.active.get(i)
        if a and not a.get('retiring'):
            a['retiring']=True;a['reason']=reason;self.rejected.add(i)
            self.event('lease_cancellation_requested',drone=i,reason=reason,token=a['packet']['token'])
    def heartbeat(self):
        t=self.now()
        for i,session in self.sessions.items():
            a=self.active.get(i);p=dict(session=session,time=t,stats=self.stats)
            if a:
                p.update(cancel=a.get('retiring',False),reason=a.get('reason'),packet=None if a.get('retiring') else a['packet'])
            self.pubs[i].publish(String(data=json.dumps(p)))
    def available(self,i):
        p=self.states.get(i,{})
        return bool(p.get('available') and p.get('ready') and p.get('rejoin_ready') and self.now()-p.get('time',-1e9)<3.
                    and not self.active.get(i,{}).get('retiring'))
    def runtime(self,i,recovery=False):
        runtime=copy.deepcopy(self.map)
        reservations=[]
        for j,p in self.positions.items():
            if i==j:continue
            a=self.active.get(j)
            reservations.append(archived.SearchCoordinator.remainder(p,np.array(a['packet']['reservation_path'])) if a else [p])
        runtime.block_paths(reservations)
        if recovery:runtime.clearance=.5;runtime.recovery_yaw=self.yaws[i]
        return runtime
    def tick(self):
        t=self.now()
        if len(self.positions)!=3:return
        self.map.rebuild()
        for i,a in list(self.active.items()):
            state=self.states.get(i,{});intent=state.get('intent') or {}
            if a.get('retiring'):
                if state.get('stopped') and not intent:
                    self.active.pop(i);self.event('reservation_retired',drone=i)
                continue
            if t-a['time']>4. and intent.get('token')!=a['packet']['token']:
                if state.get('completed_core_epoch')==a['packet']['core_epoch'] or a['packet']['recovery']:
                    if state.get('stopped'):self.active.pop(i)
                else:self.cancel(i,'command_not_executed')
                continue
            path=archived.SearchCoordinator.remainder(self.positions[i],np.array(a['packet']['reservation_path']))
            runtime=copy.copy(self.map)
            if a['packet']['recovery']:runtime.clearance=.5;runtime.recovery_yaw=self.yaws[i]
            if not runtime.safe_path(path):self.cancel(i,'path_or_tracking_invalidated')
        if self.future is not None:
            if not self.future.done():return
            result=self.future.result();self.future=None;self.calls+=1
            self.stats=dict(result['stats'],free_cells=int(np.count_nonzero(self.map.safe)))
            self.event('policy_step',compute_wall_s=result['compute_wall_s'],snapshot_time=result['snapshot_time'],stats=self.stats,failures=result['failures'])
            for event in result['events']:self.event('historical_policy_event',event=event)
            for i,paths in result['candidates'].items():
                self.candidates.write(json.dumps(dict(time=t,drone=i,paths=paths))+'\n');self.candidates.flush()
            for i,packet in result['commands'].items():
                if packet is None:continue
                if i in self.active or not self.available(i) or t-result['snapshot_time']>20.:
                    self.rejected.add(i);continue
                runtime=self.runtime(i,packet['recovery'])
                curve=ContinuousTrajectory.from_dict(packet['trajectory']);path=curve.path(.15)
                if (np.linalg.norm(path[0]-self.positions[i])>.2 or not runtime.safe_path(np.vstack([self.positions[i],path]))):
                    self.rejected.add(i);self.event('live_validation_rejected',drone=i);continue
                self.serial+=1;packet.update(token=f'central:{self.serial}',core_epoch=packet['epoch'],reservation_path=path.tolist())
                self.core_epochs[i]=packet['epoch'];self.active[i]=dict(packet=packet,time=t)
                self.event('central_authorization',drone=i,token=packet['token'],task=packet['task'],snapshot_age=t-result['snapshot_time'])
            self.heartbeat()
        if self.control_state.get('done') or t-self.submitted<.5:return
        if len(self.states)!=3 or not all(p.get('ready') for p in self.states.values()):return
        cells=np.array(list(self.pending),int).reshape(-1,3);values=np.array(list(self.pending.values()),np.int8);self.pending.clear()
        arrived={i:p.get('completed_core_epoch',-1)==self.core_epochs.get(i,-2) for i,p in self.states.items()}
        payload=dict(bounds=self.bounds,positions=self.positions,yaws=self.yaws,available=[i for i in range(3) if self.available(i)],
                     observations=[(cells,values)],arrived=arrived,rejected=sorted(self.rejected),busy=list(self.active),time=t)
        self.rejected.clear();self.submitted=t;self.future=self.worker.submit(plan_step,payload)


def main():
    rclpy.init();node=Coordinator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.worker.shutdown(wait=False,cancel_futures=True);node.log.close();node.candidates.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
