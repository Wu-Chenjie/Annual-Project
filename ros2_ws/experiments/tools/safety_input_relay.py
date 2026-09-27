#!/usr/bin/env python3
"""Forward unchanged odometry except for one logged 3s safety-input outage."""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from native_fault_probe import latest_states


class SafetyRelay(Node):
    def __init__(self):
        super().__init__('independent_safety_input_fault')
        self.output=Path(self.declare_parameter('output_dir','/tmp/unused').value)
        self.started=None;self.ended=False;self.received=0;self.suppressed=0;self.last_input_stamp=None
        self.pub=self.create_publisher(Odometry,'/fault/safety/drone_2/estimated_odometry',qos_profile_sensor_data)
        self.create_subscription(Odometry,'/drone_2/estimated_odometry',self.input,qos_profile_sensor_data)
        self.create_timer(.1,self.check)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def log(self,kind):
        self.output.mkdir(parents=True,exist_ok=True)
        with (self.output/'safety-input-fault.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(type=kind,time=self.now(),received=self.received,suppressed=self.suppressed,
                input_topic='/drone_2/estimated_odometry',output_topic='/fault/safety/drone_2/estimated_odometry',
                affected_node='/drone_0/view_executor',duration_s=3.,last_input_stamp=self.last_input_stamp))+'\n')

    def check(self):
        if self.started is None and self.now()>=130.:
            state=latest_states(self.output).get(0,{})
            if (state.get('intent') or {}).get('committed'):
                self.started=self.now();self.log('safety_input_outage_started')
        if self.started is not None and not self.ended and self.now()-self.started>=3.:
            self.ended=True;self.log('safety_input_outage_ended')

    def input(self,message):
        self.received+=1;self.last_input_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if self.started is not None and 0<=self.now()-self.started<3.:
            self.suppressed+=1;return
        self.pub.publish(message)


def main():
    rclpy.init();node=SafetyRelay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.log('relay_stopped');node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
