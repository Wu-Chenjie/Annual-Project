"""ROS planner fault handling and an actual private-process crash / restart."""
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from multiprocessing import get_context
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'next_project'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from decentralized_agent_node import ExplorationAgent
from core.exploration.planning_budget import PlanningRequest,abort_worker
from core.planning.continuous_trajectory import interpolate


def crash_worker():os._exit(76)


def test_actual_worker_crash_can_be_replaced():
    worker=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'))
    try:
        with pytest.raises(BrokenProcessPool):worker.submit(crash_worker).result(timeout=10.)
    finally:abort_worker(worker)
    replacement=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'))
    try:assert replacement.submit(pow,3,2).result(timeout=10.)==9
    finally:abort_worker(replacement)


def test_wall_watchdog_runs_while_simulation_clock_is_stopped(tmp_path):
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true',
                    '-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.intent=dict(token='authorized',committed=True);node.epoch=10
        node.request=PlanningRequest('frozen-clock',node.fusion.graph.replica.session,10,0,0.,time.monotonic()-13.)
        node.future=SimpleNamespace(done=lambda:False)
        assert node.get_clock().now().nanoseconds==0
        until=time.monotonic()+2.
        while node.future is not None and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.2)
        assert node.future is None
        assert node.intent['token']=='authorized' and node.epoch==10
        assert node.events[-1]['type']=='planning_timeout'
        assert node.get_clock().now().nanoseconds==0
    finally:
        abort_worker(node.worker);node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('fault',['deadline','worker_crash','observation_loss'])
def test_agent_retains_authorized_curve_on_planner_fault_and_holds_on_sensor_loss(tmp_path,fault):
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.now=lambda:100.;node.ready=node.rejoin_ready=node.bootstrapped=True
        node.position=np.array([2.25,2.25,1.65]);node.yaw=0.;node.replica.map.state[:]=0;node.replica.map.rebuild()
        curve=interpolate(np.array([node.position,[4.25,2.25,1.65]]),[12.],0.,0.)
        node.epoch=10;node.intent=dict(epoch=10,token='old',region=7,committed=True,path=curve.path(.15).tolist(),yaw=0.,trajectory=curve.to_dict())
        node.execution=dict(epoch=10,ready=True,arrived=False);node.selection={};node.active=7
        node.observation=dict(time=100. if fault!='observation_loss' else 97.)
        node.request=PlanningRequest('test',node.fusion.graph.replica.session,10,0,99.,time.monotonic()-(13. if fault=='deadline' else 0.))
        if fault=='worker_crash':
            def failed_result():raise BrokenProcessPool('test process failed')
            node.future=SimpleNamespace(done=lambda:True,result=failed_result)
        else:node.future=SimpleNamespace(done=lambda:False)
        node.plan()
        if fault=='observation_loss':
            assert node.intent is None and node.retiring['token']=='old' and node.motion_blocked
            assert node.events[-2]['type']=='lease_cancellation_requested'
        else:
            assert node.intent['token']=='old' and node.intent['committed'] and node.epoch==10
            assert node.future is None and node.available
            assert any(e['type']==('planning_timeout' if fault=='deadline' else 'planning_failed') for e in node.events)
    finally:
        abort_worker(node.worker);node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()
