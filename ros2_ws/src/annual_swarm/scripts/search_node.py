#!/usr/bin/env python3
"""Observed-map exploration coordinator and explicit ray-sensor simulation adapter."""
import csv
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
import planning_runtime  # installed legacy import path
from core.exploration.mapping import RaySensorWorld
from core.exploration.coordinator import SearchCoordinator

class Search(Node):
    def __init__(self):
        super().__init__('search_coordinator')
        map_file=self.declare_parameter('map_file','').value
        self.world=RaySensorWorld(map_file)
        # Scenario preflight only: reject invalid spawns before any motor commands.
        from core.exploration.mapping import ObservedMap
        check=ObservedMap(self.world.bounds);check.state[:]=self.world.occupied;check.rebuild()
        starts=np.array(json.loads(Path(map_file).read_text())['search_starts'],float)
        if len(starts)!=3 or not all(check.safe_path([p]) for p in starts):raise ValueError('Unsafe search spawn')
        if min(np.linalg.norm(a-b) for i,a in enumerate(starts) for b in starts[i+1:])<1.2:raise ValueError('Search spawns too close')
        del check
        self.policy=self.declare_parameter('policy','gvp_pairwise').value;self.core=SearchCoordinator(self.world.bounds,self.policy)
        self.output=Path(self.declare_parameter('output_dir','/tmp/annual_search').value);self.output.mkdir(parents=True,exist_ok=True)
        (self.output/'map.json').write_text(Path(map_file).read_text())
        self.threshold=self.declare_parameter('coverage_target',.95).value
        self.pause_after=self.declare_parameter('pause_after',0.).value;self.pause_duration=self.declare_parameter('pause_duration',30.).value
        self.positions={};self.execution={};self.available={0,1,2};self.contacts=0;self.airborne=set();self.minimum=None
        self.started=None;self.finished=None;self.pause_started=None;self.pause_resumed=False;self.status='WAITING';self.distance={i:0. for i in range(3)}
        self.coverage=[];self.t95=None;self.t90=None;self.planning_time=[];self.message_bytes=0;self.topology_bytes=0;self.samples={i:0 for i in range(3)}
        self.csv=(self.output/'trajectory.csv').open('w');self.writer=csv.writer(self.csv);self.writer.writerow(['time','drone','x','y','z'])
        self.events=(self.output/'events.jsonl').open('w');self.event_index=0
        self.candidates=(self.output/'candidate_paths.jsonl').open('w')
        self.topology=(self.output/'topology.jsonl').open('w')
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs={i:self.create_publisher(String,f'/drone_{i}/search_path',q) for i in range(3)}
        self.graph_pub=self.create_publisher(String,'/search/topology',q)
        self.snapshot_pub=self.create_publisher(String,'/search/topology_snapshot',q)
        self.diag=self.create_publisher(String,'/search/diagnostics',q);self.control=self.create_publisher(String,'/search/control',q)
        self.create_subscription(String,'/swarm/dynamic_obstacles',self.obstacle,q)
        self.create_subscription(String,'/search/execution',lambda m:setattr(self,'execution',json.loads(m.data)),q)
        for i in range(3):
            self.create_subscription(Odometry,f'/drone_{i}/odometry',lambda m,i=i:self.odom(i,m),qos_profile_sensor_data)
            self.create_subscription(Contacts,f'/drone_{i}/contacts',lambda m,i=i:self.contact(i,m),qos_profile_sensor_data)
        self.create_timer(.5,self.tick)
    def obstacle(self,m):
        data=json.loads(m.data)
        if data.get('frame_id')=='world':self.world.dynamic_snapshot(int(data['version']),data['obstacles'])
    def odom(self,i,m):
        p=m.pose.pose.position;value=np.array([p.x,p.y,p.z]);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if i in self.positions:self.distance[i]+=float(np.linalg.norm(value[:2]-self.positions[i][:2]))
        self.positions[i]=value;self.samples[i]+=1
        if p.z>.5:self.airborne.add(i)
        for j,other in self.positions.items():
            if i!=j:
                d=float(np.linalg.norm(value-other));self.minimum=d if self.minimum is None else min(self.minimum,d)
        self.writer.writerow([t,i,*value])
    def contact(self,i,m):
        for c in m.contacts:
            if 'ground' in c.collision1.name+c.collision2.name and i not in self.airborne:continue
            self.contacts+=1
    def tick(self):
        t=self.get_clock().now().nanoseconds*1e-9
        if len(self.positions)!=3:return
        if not self.execution.get('ready',False):self.save(t);return
        if self.started is None:self.started=t
        if self.finished is not None:self.save(t);return
        if self.execution.get('failure') or self.contacts:
            self.status='FAILED';self.finished=t
        elapsed=t-self.started
        if self.pause_after>0 and elapsed>=self.pause_after and self.pause_started is None:
            self.pause_started=t;self.available.remove(1)
            self.core.events.append(dict(type='availability_pause',drone=1,time=t))
        if self.pause_started is not None and not self.pause_resumed and t-self.pause_started>=self.pause_duration:
            self.available.add(1);self.pause_resumed=True;self.core.events.append(dict(type='availability_resume',drone=1,time=t))
        self.control.publish(String(data=json.dumps(dict(available=sorted(self.available),done=self.finished is not None))))
        if self.finished is not None:self.save(t);return
        observed=[self.world.observe(p) for i,p in self.positions.items() if i in self.available]
        arrived={int(i):bool(v) and self.execution.get('epochs',{}).get(i,0)==self.core.epochs.get(int(i),0) for i,v in self.execution.get('arrived',{}).items()}
        begin=time.monotonic();commands=self.core.step(self.positions,self.available,arrived,observed,t);self.planning_time.append(time.monotonic()-begin)
        for i,packet in commands.items():
            data=json.dumps(packet);self.message_bytes+=len(data.encode());self.pubs[i].publish(String(data=data))
            if packet is not None:
                pool=self.core.pools[i]
                self.candidates.write(json.dumps(dict(time=t,drone=i,task=packet['task'],epoch=packet['epoch'],
                    paths=[dict(**c.metadata(),points=c.path.tolist()) for c in [pool.active]+pool.backups]))+'\n')
        delta=json.dumps(self.core.topology_delta);self.topology_bytes+=len(delta.encode())
        self.graph_pub.publish(String(data=delta));self.topology.write(delta+'\n');self.topology.flush()
        if self.core.sequence%10==1:
            ids=sorted(self.core.graph_nodes);cells=np.array([(k//self.core.map.shape[1],k%self.core.map.shape[1]) for k in ids]).reshape(-1,2)
            snapshot=json.dumps(dict(sequence=self.core.sequence,map_version=self.core.map.version,resolution=self.core.map.resolution,
                nodes=[dict(id=k,position=p.tolist()) for k,p in zip(ids,self.core.map.points(cells))],edges=sorted(self.core.graph_edges)))
            self.topology_bytes+=len(snapshot.encode());self.snapshot_pub.publish(String(data=snapshot))
        value=self.world.coverage(self.core.map);self.coverage.append([t,value]);self.status='SEARCHING'
        if value>=.9 and self.t90 is None:self.t90=elapsed
        if value>=.95 and self.t95 is None:self.t95=elapsed
        if value>=self.threshold:
            self.status='COMPLETE';self.finished=t;self.control.publish(String(data=json.dumps(dict(available=sorted(self.available),done=True))))
        self.save(t)
    def save(self,t):
        report=dict(status=self.status,simulation_time=t,start_time=self.started,finish_time=self.finished,policy=self.policy,
            coverage=self.coverage[-1][1] if self.coverage else 0.,coverage_target=self.threshold,t90=self.t90,t95=self.t95,
            contacts_after_takeoff=self.contacts,min_separation_m=self.minimum,distances_m=self.distance,odometry_samples=self.samples,
            graph=self.core.stats,available=sorted(self.available),pause_resumed=self.pause_resumed,
            events=self.core.events,planner_wall_max=max(self.planning_time,default=0),published_path_payload_bytes=self.message_bytes,published_topology_payload_bytes=self.topology_bytes,
            assumptions='Central shared observed map; 360-degree 3.5 m occluded ray sensor; planar world-frame truth odometry; graph/pairwise heuristic, not RACER/GVP reproduction')
        tmp=self.output/'summary.tmp';tmp.write_text(json.dumps(report,indent=2));tmp.replace(self.output/'summary.json')
        self.diag.publish(String(data=json.dumps(report)))
        for event in self.core.events[self.event_index:]:self.events.write(json.dumps(event)+'\n')
        self.event_index=len(self.core.events);self.events.flush();self.csv.flush();self.candidates.flush()
        (self.output/'coverage.json').write_text(json.dumps(self.coverage))
        # Compact, directly inspectable map state, never the hidden truth occupancy.
        np.savez_compressed(self.output/'observed-map.npz',state=self.core.map.state,origin=self.core.map.origin,resolution=self.core.map.resolution)

def main():
    rclpy.init();node=Search()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.save(node.get_clock().now().nanoseconds*1e-9);node.csv.close();node.events.close();node.candidates.close();node.topology.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
