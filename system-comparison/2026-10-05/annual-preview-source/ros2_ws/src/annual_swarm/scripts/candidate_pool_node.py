#!/usr/bin/env python3
"""Versioned ranked routes, safe backup switching and background replenishment."""
import copy
import json
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from candidate_runtime import make_runtime,generate_candidates,regenerate_single,apply_obstacles,PathQualityEvaluator,RankedPathPool
from core.planning.path_quality import resample

class CandidatePlanning(Node):
    def __init__(self):
        super().__init__('planner')
        def param(k,v):return self.declare_parameter(k,v).value
        self.map_file=param('map_file','');self.start=np.array(param('start',[2.,3.,1.5]));self.goal=np.array(param('goal',[18.,16.,1.5]))
        self.clearance=param('clearance',1.9)
        self.recovery_policy=param('recovery_policy','cache')
        if self.recovery_policy not in ('cache','replan_single','replan_pool'):raise ValueError('Unknown recovery policy')
        self.recovery_pending=None
        self.algorithms=param('candidate_planners','astar,hybrid_astar,rrt_star').split(',')
        self.variants=param('candidate_variants',4)
        if not 1<=self.variants<=32:raise ValueError('candidate_variants must be 1..32')
        self.weights=json.loads(param('quality_weights',json.dumps(PathQualityEvaluator.DEFAULT_WEIGHTS)))
        self.evaluator=PathQualityEvaluator(self.weights)
        self.runtime=make_runtime(self.map_file,altitude=self.start[2],clearance=self.clearance)
        self.runtime.check_takeoff(self.start)
        self.pool=RankedPathPool();self.position=None;self.obstacles=[];self.map_version=0;self.goal_revision=0
        self.pool_executor=ThreadPoolExecutor(max_workers=1);self.future=None;self.pending=True
        self.status='WAITING';self.have_path=False;self.plan_count=0;self.generation=0;self.switches=0;self.events=[]
        self.peak_backups=0;self.initial_backups=None;self.last_snapshot_seq=-1;self.last_refresh=-1e9
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'/swarm/path',qos)
        self.status_pub=self.create_publisher(String,'/swarm/planner_status',qos)
        self.diag_pub=self.create_publisher(String,'/swarm/planner_diagnostics',qos)
        self.pool_pub=self.create_publisher(String,'/swarm/candidate_paths',qos)
        self.create_subscription(Odometry,'/drone_0/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,'/swarm/dynamic_obstacles',self.obstacle_update,qos)
        self.create_subscription(PoseStamped,'/swarm/goal',self.new_goal,1)
        self.create_timer(.1,self.tick)
    def odom(self,msg):
        p=msg.pose.pose.position;self.position=np.array([p.x,p.y,p.z])
    def new_goal(self,msg):
        if msg.header.frame_id!='world':return
        p=msg.pose.position;goal=np.array([p.x,p.y,p.z])
        if abs(goal[2]-self.start[2])>1e-6 or not self.runtime.safe_point(goal):return
        self.goal=goal;self.goal_revision+=1;self.pool=RankedPathPool();self.pending=True
        self.status='PAUSED: new mission goal';self.status_pub.publish(String(data=self.status))
    def obstacle_update(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('frame_id')!='world':raise ValueError('Obstacle frame must be world')
            seq=int(data['version'])
            if seq<=self.last_snapshot_seq:return
            obstacles=data['obstacles']
            for o in obstacles:
                values=np.array([*o['center_xy'],o['radius'],*o['z_range']],float)
                if len(values)!=5 or not np.isfinite(values).all() or o['radius']<=0 or o['z_range'][1]<=o['z_range'][0]:raise ValueError('Invalid cylinder')
        except (KeyError,ValueError,TypeError) as exc:
            self.get_logger().error(str(exc));return
        self.last_snapshot_seq=seq;self.map_version+=1;self.obstacles=obstacles
        # Stop reference progression before any expensive map/path checks. Motors keep hover targets.
        self.status='PAUSED: validating changed obstacles';self.status_pub.publish(String(data=self.status))
        begin=time.monotonic()
        self.recovery_pending=dict(map_version=self.map_version,received_wall=begin,received_sim=self.get_clock().now().nanoseconds*1e-9)
        apply_obstacles(self.runtime,self.obstacles)
        if self.recovery_policy!='cache':
            self.pool=RankedPathPool();self.pending=True
            self.status='PAUSED: baseline replanning without cached routes'
            self.events.append(dict(type='baseline_replan',policy=self.recovery_policy,map_version=self.map_version))
            self.diagnostics();return
        old=self.pool.active.id if self.pool.active else None
        if self.position is not None:
            changed,rejected=self.pool.revalidate(self.position,self.runtime,self.evaluator,self.map_version)
            if self.pool.active is not None:
                if changed:
                    self.switches+=1
                    self.events.append(dict(type='backup_switch',source='cached_backup',old=old,new=self.pool.active.id,
                        map_version=self.map_version,rejected=rejected,validation_wall_s=time.monotonic()-begin))
                self.publish_path(self.pool.active.path)
            else:
                self.status='PAUSED: no safe backup; replanning'
                self.events.append(dict(type='no_safe_backup',map_version=self.map_version,rejected=rejected))
        self.pending=True;self.diagnostics()
    def publish_path(self,path):
        msg=Path();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        for point in path:
            p=PoseStamped();p.header=msg.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,point);p.pose.orientation.w=1.;msg.poses.append(p)
        self.path_pub.publish(msg);self.have_path=True;self.status='READY'
        self.status_pub.publish(String(data=self.status))
        if self.recovery_pending is not None:
            event=self.recovery_pending;self.recovery_pending=None
            self.events.append(dict(type='recovery_ready',policy=self.recovery_policy,map_version=event['map_version'],
                received_sim=event['received_sim'],ready_sim=self.get_clock().now().nanoseconds*1e-9,
                latency_wall_s=time.monotonic()-event['received_wall']))
    def diagnostics(self):
        active=self.pool.active.metadata() if self.pool.active else None
        backups=[x.metadata() for x in self.pool.backups]
        self.peak_backups=max(self.peak_backups,len(backups))
        report=dict(algorithm='portfolio',recovery_policy=self.recovery_policy,goal=self.goal.tolist(),plan_count=self.plan_count,
            map_version=self.map_version,goal_revision=self.goal_revision,active=active,backups=backups,backup_count=len(backups),
            peak_backup_count=self.peak_backups,initial_backup_count=self.initial_backups,
            switch_count=self.switches,events=self.events[-50:],weights=self.weights,
            requested_planners=self.algorithms,variants_per_planner=self.variants,status=self.status,
            obstacle_source='Gazebo scene command acknowledgments; not a perception estimator')
        self.diag_pub.publish(String(data=json.dumps(report)))
        self.pool_pub.publish(String(data=json.dumps(dict(**report,
            active_path=self.pool.active.path.tolist() if self.pool.active else [],
            backup_paths=[dict(id=x.id,points=x.path.tolist()) for x in self.pool.backups]))))
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.future is not None and self.future.done():
            future=self.future;self.future=None
            if self.job_version!=(self.map_version,self.goal_revision):
                self.events.append(dict(type='discard_stale_candidates',version=list(self.job_version)));self.pending=True
            else:
                try:
                    generated,failures,accepted=future.result();self.plan_count+=1
                    position=self.position if self.have_path else self.start
                    generated.revalidate(position,self.runtime,self.evaluator,self.map_version)
                    if self.pool.active is not None:
                        # Replenishment must not replace a safe active route just because scores vary.
                        self.pool.revalidate(position,self.runtime,self.evaluator,self.map_version)
                        active=self.pool.active
                        if active is None:
                            self.pool=generated
                            if self.pool.active:self.publish_path(self.pool.active.path)
                            else:self.status='PAUSED: no safe connection'
                        else:
                            potential=([generated.active] if generated.active else [])+generated.backups+self.pool.backups
                            potential=[x for x in potential if np.mean(np.linalg.norm(resample(x.path)-resample(active.path),axis=1))>=.25]
                            merged=RankedPathPool();self.pool.backups=merged.rank(potential)[:5]
                    else:
                        self.pool=generated
                        if self.pool.active:self.publish_path(self.pool.active.path)
                        else:self.status='PAUSED: no feasible candidate' if self.have_path else 'FAILED: no feasible candidate'
                    if self.initial_backups is None:self.initial_backups=len(self.pool.backups)
                    self.events.append(dict(type='pool_generated',map_version=self.map_version,accepted=accepted,failed=failures))
                except Exception as exc:
                    self.status='PAUSED: '+str(exc) if self.have_path and self.pool.active is None else ('DEGRADED: ' if self.have_path else 'FAILED: ')+str(exc)
            self.last_refresh=now;self.diagnostics()
        if self.position is not None and self.future is None and self.pending:
            self.pending=False;self.generation+=1;self.job_version=(self.map_version,self.goal_revision)
            start=self.position.copy() if self.have_path else self.start.copy();start[2]=self.start[2]
            if self.recovery_policy=='replan_single' and self.map_version>0:
                self.future=self.pool_executor.submit(regenerate_single,copy.deepcopy(self.runtime),start,self.goal.copy(),
                    self.map_version,self.weights,self.generation)
            else:
                self.future=self.pool_executor.submit(generate_candidates,self.map_file,start,self.goal.copy(),
                    self.algorithms,self.variants,self.map_version,json.loads(json.dumps(self.obstacles)),self.clearance,self.weights,self.generation)
        self.status_pub.publish(String(data=self.status))

def main():
    rclpy.init();node=CandidatePlanning()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.pool_executor.shutdown(wait=False,cancel_futures=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
