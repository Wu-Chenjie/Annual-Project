#!/usr/bin/env python3
"""Read-only latest sensor-union snapshot; never overwrite the finish-time map."""
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String


def main():
    out=Path(sys.argv[1])
    if (out/'observed_final.npz').exists():return
    bounds=json.loads((out/'map.json').read_text())['bounds']
    rclpy.init(args=[]);node=Node('read_only_map_snapshot');latest=[]
    subscription=node.create_subscription(String,'/experiment/observed_map',
        lambda m:latest.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline=time.monotonic()+8.
        while not latest and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
        if not latest:raise RuntimeError('No actual observed-map packet received')
        packet=latest[-1]
        np.savez_compressed(out/'observed_latest.npz',state=np.array(packet['state'],np.int8).reshape(packet['shape']),
                            bounds=bounds,resolution=packet['resolution'])
        (out/'map-snapshot.json').write_text(json.dumps(dict(scope='Latest actual sensor union; incomplete run, not the finish-time acceptance map',
            file='observed_latest.npz',shape=packet['shape']),indent=2)+'\n')
    finally:
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
