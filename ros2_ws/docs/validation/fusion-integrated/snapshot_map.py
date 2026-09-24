"""One-shot measurement after stop condition; never feeds a planner."""
import json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
out=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('recording_final_map_snapshot');received=[]
sub=node.create_subscription(String,'/experiment/observed_map',lambda m:received.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
end=time.monotonic()+8
while not received and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.2)
if not received:raise RuntimeError('No observed-map snapshot received')
data=received[-1];world=json.loads((out/'map.json').read_text())
np.savez_compressed(out/'observed_latest.npz',state=np.asarray(data['state'],dtype=np.int8).reshape(data['shape']),bounds=world['bounds'],resolution=data['resolution'])
(out/'map-snapshot-note.json').write_text(json.dumps(dict(source='/experiment/observed_map',captured_after_stop_condition=True,exact_summary_sample_match_not_guaranteed=True),indent=2))
node.destroy_node();rclpy.shutdown()
