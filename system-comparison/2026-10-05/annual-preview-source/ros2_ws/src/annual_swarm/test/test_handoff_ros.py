"""Real ROS executor callbacks with controlled authorization / boundary faults."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'next_project'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from std_msgs.msg import String
from rclpy.time import Time
from view_executor_node import ViewExecutor
from core.planning.continuous_trajectory import interpolate


@pytest.mark.parametrize('fault', ['none','missing_ack','stale_own','paused','bad_boundary','cancel_pending'])
def test_executor_handoff_requires_both_leases_and_matching_boundary(fault):
    rclpy.init(args=['--ros-args','-p','start:="[2,2,1.5]"','-p','fleet_starts:="[[2,2,1.5],[2,8,1.5]]"'])
    node=ViewExecutor()
    try:
        old=interpolate(np.array([[2.,2.,1.5],[4.,2.,1.5]]),[12.],0.,.4)
        p,v,a,h,rate=old.sample(8.)
        new=interpolate(np.array([p,[6.,3.,1.5]]),[16.],h,.8-h,start_velocity=v,start_acceleration=a,
                        start_yaw_rate=rate,start_yaw_acceleration=old.yaw_acceleration(8.))
        node.ready=True;node.arrived=False;node.curve=old;node.path=old.path(.15);node.token='old';node.epoch=1;node.curve_time=6.
        node.positions={0:old.sample(6.)[0],1:np.array([2.,8.,1.5])}
        command=dict(epoch=2,token='new',path=new.path(.15).tolist(),yaw=.8,trajectory=new.to_dict(),
                     handoff=dict(from_epoch=1,from_token='old',trajectory_time=8.))
        if fault=='bad_boundary':command['handoff']['trajectory_time']=8.3
        node.command(String(data=json.dumps(command)))
        if fault=='bad_boundary':assert node.pending is None;return
        assert node.pending is not None and node.epoch==1 and node.token=='old'
        old_intent=dict(token='old',committed=True,path=old.path(.15).tolist(),voters=[1])
        new_intent=dict(token='new',committed=True,path=new.path(.15).tolist(),voters=[1])
        node.peers={0:dict(time=10.,intent=old_intent,pending_intent=new_intent),1:dict(time=10.,acks=['old','new'])}
        if fault=='missing_ack':node.peers[1]['acks']=['old']
        if fault=='stale_own':node.peers[0]['time']=0.
        if fault=='paused':node.paused=True
        if fault=='cancel_pending':node.command(String(data=json.dumps(dict(epoch=2,cancel_pending='new'))))
        node.curve_time=7.99;node.ref=old.sample(7.99)[0];node.heading=old.sample(7.99)[3]
        node.positions[0]=node.ref.copy();node.stamps={0:10.,1:10.};node.last=9.98
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.))
        node.tick()
        if fault=='none':
            assert node.epoch==2 and node.token=='new' and node.handoff_count==1
            assert max(node.handoff_continuity.values())<1e-6
            assert np.linalg.norm(node.velocity)>.05 and not node.arrived
        else:
            assert node.epoch==1 and node.token=='old' and node.handoff_count==0
    finally:
        node.destroy_node();rclpy.shutdown()
