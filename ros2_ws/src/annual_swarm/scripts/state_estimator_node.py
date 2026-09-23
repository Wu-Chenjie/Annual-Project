#!/usr/bin/env python3
"""Standard IMU + covariance-bearing localization input -> estimated odometry."""
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import planning_runtime
from core.exploration.state_estimation import InertialOdometryFilter


class Estimator(Node):
    def __init__(self):
        super().__init__('state_estimator')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        self.filter = InertialOdometryFilter(); self.attitude = None; self.angular = None
        self.pub = self.create_publisher(Odometry, f'/drone_{self.id}/estimated_odometry', qos_profile_sensor_data)
        self.create_subscription(Imu, f'/drone_{self.id}/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/drone_{self.id}/localization_measurement', self.measurement, qos_profile_sensor_data)

    def measurement(self, m):
        p = m.pose.pose.position; v = m.twist.twist.linear
        variance = [max(1e-8, m.pose.covariance[7*i]) for i in range(3)]+[max(1e-8, m.twist.covariance[7*i]) for i in range(3)]
        q = m.pose.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        world_velocity = rotation.apply([v.x, v.y, v.z])
        self.filter.correct([p.x, p.y, p.z], world_velocity, np.diag(variance))
        if self.attitude is None:
            self.attitude = m.pose.pose.orientation

    def imu(self, m):
        t = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        q = m.orientation
        if np.linalg.norm([q.x, q.y, q.z, q.w]) < .9:
            return
        self.attitude = q; a = m.linear_acceleration
        acceleration = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([a.x, a.y, a.z])+[0., 0., -9.81]
        self.filter.predict(t, acceleration)
        if not self.filter.initialized:
            return
        out = Odometry(); out.header = m.header; out.header.frame_id = 'world'; out.child_frame_id = f'drone_{self.id}/base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, self.filter.x[:3])
        out.pose.pose.orientation = self.attitude
        body_velocity = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply(self.filter.x[3:6])
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, body_velocity)
        out.twist.twist.angular = m.angular_velocity
        for i in range(3):
            out.pose.covariance[7*i] = float(self.filter.P[i, i]); out.twist.covariance[7*i] = float(self.filter.P[i+3, i+3])
        self.pub.publish(out)


def main():
    rclpy.init(); node = Estimator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':main()
