#!/usr/bin/env python3
"""Read-only sensor evidence recorder, never an input to agents or executors."""
import gzip
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class EvidenceRecorder(Node):
    def __init__(self):
        super().__init__('observation_evidence_recorder')
        output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value)
        output.mkdir(parents=True, exist_ok=True)
        self.stream = gzip.open(output/'observations.jsonl.gz', 'wt', compresslevel=2)
        self.execution_stream = (output/'execution-evidence.jsonl').open('w')
        self.command_stream = (output/'command-evidence.jsonl').open('w')
        self.identities = set()
        q = QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for i in range(int(self.declare_parameter('fleet_size', 3).value)):
            self.create_subscription(String, f'/drone_{i}/observation', self.observation, q)
            self.create_subscription(String, f'/drone_{i}/view_execution', self.execution, q)
            self.create_subscription(String, f'/drone_{i}/view_command', lambda m,i=i:self.command(i,m), q)

    def command(self, drone, message):
        packet=json.loads(message.data);packet['drone']=drone
        packet['receipt_time']=self.get_clock().now().nanoseconds*1e-9
        packet['receipt_wall_time']=time.monotonic()
        self.command_stream.write(json.dumps(packet)+'\n');self.command_stream.flush()

    def execution(self, message):
        packet = json.loads(message.data)
        packet['receipt_time'] = self.get_clock().now().nanoseconds*1e-9
        packet['receipt_wall_time'] = time.monotonic()
        self.execution_stream.write(json.dumps(packet)+'\n'); self.execution_stream.flush()

    def observation(self, message):
        packet = json.loads(message.data)
        key = (packet['source'], packet.get('sensor_session'), packet['sequence'])
        if key in self.identities:
            return
        self.identities.add(key)
        packet['receipt_time'] = self.get_clock().now().nanoseconds*1e-9
        self.stream.write(json.dumps(packet, separators=(',', ':'))+'\n')
        self.stream.flush()


def main():
    rclpy.init(); node = EvidenceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.close(); node.execution_stream.close(); node.command_stream.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
