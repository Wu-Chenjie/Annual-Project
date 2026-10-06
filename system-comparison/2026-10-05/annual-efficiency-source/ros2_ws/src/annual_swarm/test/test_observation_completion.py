"""Endpoint completion requires real, fresh and correctly posed integrated frames."""
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from core.exploration.observation_completion import ObservationCompletion
from core.exploration.mrdtg import observation_grid


def frame(time=10.1, **kwargs):
    return dict(source=0, sensor='gazebo_gpu_lidar', sensor_session='s', sequence=1, time=time,
                sensor_position=[2., 2., 1.5], sensor_yaw=0., pose_time=time,
                integration_completed=True, ray_cell_count=30, point_count=5611, map_version=4,
                shape=[20, 20, 10], origin=[0., 0., 0.], resolution=.3, **kwargs)


def test_actual_frame_can_complete_immediately_without_new_map_cells():
    completion = ObservationCompletion(0)
    assert completion.ingest(frame(), 10.15)
    proof = completion.proof(10.15, 10., [2., 2., 1.5], 0.)
    assert proof and proof['sequence'] == 1
    # Integration on an already-known view need not change the version.
    packet=frame(10.2);packet['sequence']=2
    assert completion.ingest(packet, 10.25)
    assert completion.proof(10.25, 10.16, [2., 2., 1.5], 0.)


@pytest.mark.parametrize('fault', ['before_arrival', 'old_pose', 'wrong_yaw', 'old_frame', 'future_frame',
                                  'wrong_source', 'unintegrated', 'zero_rays', 'duplicate', 'wrong_grid'])
def test_invalid_or_unobserved_target_cannot_complete(fault):
    completion = ObservationCompletion(0); packet = frame()
    grid = observation_grid(SimpleNamespace(shape=[20,20,10], origin=np.zeros(3), resolution=.3))
    if fault=='before_arrival':packet['time']=packet['pose_time']=9.9
    elif fault=='old_pose':packet['pose_time']=9.
    elif fault=='wrong_yaw':packet['sensor_yaw']=.3
    elif fault=='old_frame':packet['time']=packet['pose_time']=8.
    elif fault=='future_frame':packet['time']=packet['pose_time']=11.
    elif fault=='wrong_source':packet['source']=1
    elif fault=='unintegrated':packet['integration_completed']=False
    elif fault=='zero_rays':packet['ray_cell_count']=0
    elif fault=='wrong_grid':packet['shape']=[30,20,10]
    elif fault=='duplicate':
        completion.ingest(packet, 10.15)
        assert not completion.ingest(packet, 10.15)
        assert completion.proof(10.3, 10.2, [2.,2.,1.5], 0.) is None
        return
    completion.ingest(packet, 10.15, grid)
    assert completion.proof(10.15, 10., [2.,2.,1.5], 0., grid) is None


def test_sensor_restart_retires_old_session_without_completing_from_stale_proof():
    completion=ObservationCompletion(0)
    completion.ingest(frame(), 10.15)
    restarted=frame(10.2);restarted['sensor_session']='new'
    assert completion.ingest(restarted, 10.25)
    resurrected=frame(10.3);resurrected['sequence']=2
    assert not completion.ingest(resurrected, 10.35)
    regressed=copy.deepcopy(restarted);regressed.update(time=10.4, pose_time=10.4, sequence=2, map_version=3)
    assert not completion.ingest(regressed,10.45)


def test_real_executor_waits_for_a_post_arrival_frame_and_resets_after_tracking_loss():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.time import Time
    from std_msgs.msg import String
    from view_executor_node import ViewExecutor
    rclpy.init(args=['--ros-args','-p','start:="[2,2,1.5]"','-p','fleet_starts:="[[2,2,1.5]]"'])
    node=ViewExecutor()
    try:
        node.ready=True;node.arrived=False;node.token='v';node.epoch=1
        node.path=np.array([[2.,2.,1.5]]);node.index=1;node.positions={0:node.ref.copy()}
        intent=dict(token='v',committed=True,path=node.path.tolist(),voters=[])
        def tick(t):
            node.stamps={0:t};node.peers={0:dict(time=t,intent=intent)}
            node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=t))
            node.tick()
        tick(10.)
        tick(10.7)
        assert not node.arrived  # Elapsed .65 s alone proves nothing.
        node.positions[0]=np.array([2.2,2.,1.5]);tick(10.8)
        assert node.hold_since is None
        node.observation(String(data=json.dumps(frame(10.75))))
        node.positions[0]=node.ref.copy();tick(10.9)
        assert not node.arrived
        packet=frame(10.95);packet['sequence']=2
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=11.))
        node.observation(String(data=json.dumps(packet)));tick(11.)
        assert node.arrived and node.observation_proof['token']=='v'
        assert node.observation_proof['time']>=node.hold_since
    finally:
        node.destroy_node();rclpy.shutdown()


def test_native_mapper_packet_completes_the_real_executor_after_actual_ray_integration():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField
    from view_executor_node import ViewExecutor
    from pointcloud_mapping_node import PointCloudMapper
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[6,6,3]]"',
                    '-p','start:="[2,2,1.5]"','-p','fleet_starts:="[[2,2,1.5]]"'])
    executor=ViewExecutor();mapper=PointCloudMapper()
    try:
        executor.ready=True;executor.arrived=False;executor.token='v';executor.epoch=1
        executor.path=np.array([[2.,2.,1.5]]);executor.index=1;executor.positions={0:executor.ref.copy()}
        executor.observation_grid=observation_grid(mapper.map)
        executor.stamps={0:10.};executor.peers={0:dict(time=10.,intent=dict(token='v',committed=True,path=executor.path.tolist(),voters=[]))}
        executor.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.))
        executor.tick();assert not executor.arrived
        stamp=Time(seconds=10.1).to_msg()
        odometry=Odometry();odometry.header.stamp=stamp;odometry.pose.pose.orientation.w=1.
        odometry.pose.pose.position.x=2.;odometry.pose.pose.position.y=2.;odometry.pose.pose.position.z=1.5
        mapper.odom(odometry)
        az,el=np.meshgrid(np.linspace(-np.pi/3,np.pi/3,181),np.linspace(-np.pi/3,np.pi/3,31))
        points=4.5*np.column_stack([np.cos(el.ravel())*np.cos(az.ravel()),np.cos(el.ravel())*np.sin(az.ravel()),np.sin(el.ravel())])
        cloud=PointCloud2();cloud.header.stamp=stamp;cloud.height=31;cloud.width=181;cloud.point_step=12;cloud.row_step=181*12
        cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
        cloud.data=points.astype('<f4').tobytes()
        packets=[];mapper.pub=SimpleNamespace(publish=packets.append)
        mapper.cloud(cloud)
        packet=json.loads(packets[-1].data)
        assert packet['map_version']>0 and packet['ray_cell_count']>0
        executor.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.2))
        executor.observation(packets[-1]);executor.stamps={0:10.2};executor.peers[0]['time']=10.2
        executor.tick()
        assert executor.arrived and executor.observation_proof['grid']==executor.observation_grid
        assert executor.observation_proof['map_version']==mapper.map.version
    finally:
        mapper.destroy_node();executor.destroy_node();rclpy.shutdown()
