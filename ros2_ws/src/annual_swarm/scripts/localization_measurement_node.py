#!/usr/bin/env python3
"""Simulation-only noisy pose source. Replace this node with real VIO/LIO odometry.

Truth is used only here and in experiment evaluation, not in the estimator.
This does not pretend that noisy truth measurements constitute visual SLAM.
"""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class Measurement(Node):
    def __init__(self):
        super().__init__('localization_measurement')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        self.rng = np.random.default_rng(900+self.id); self.last = -1.
        self.pub = self.create_publisher(Odometry, f'/drone_{self.id}/localization_measurement', qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/drone_{self.id}/odometry', self.odom, qos_profile_sensor_data)

    def odom(self, m):
        t = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if t-self.last < .05:
            return
        self.last = t; out = copy.deepcopy(m)
        p = out.pose.pose.position; v = out.twist.twist.linear
        p.x, p.y, p.z = np.array([p.x, p.y, p.z])+self.rng.normal(0, .007, 3)
        v.x, v.y, v.z = np.array([v.x, v.y, v.z])+self.rng.normal(0, .015, 3)
        for i in range(3):
            out.pose.covariance[7*i] = .007**2; out.twist.covariance[7*i] = .015**2
        self.pub.publish(out)


def main():
    rclpy.init(); node = Measurement()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':main()
