#!/usr/bin/env python3
"""Keep the exact current 3D truth/fault evaluator; label architecture honestly."""
import json
import numpy as np
from pathlib import Path
import exploration_experiment_node as common
from std_msgs.msg import String
_original=common.Experiment.save

def save(self,t):
    _original(self,t)
    np.savez_compressed(self.output/'observed_latest.npz',state=self.observed.state,bounds=self.observed.bounds,resolution=self.observed.resolution)
    path=self.output/'summary.json';s=json.loads(path.read_text())
    s['architecture']='7136591 centralized GVP + pairwise heuristic; dimension-adapted full-grid frontier search'
    s['baseline_adapter']=dict(historical_commit='7136591',mapping='same native 3D sensor/estimator; maps shared centrally',
        graph='6-neighbor full free-voxel graph; not sparse MR-DTG',observation='2m spherical extension of old gain proxy; 12-bin yaw adapter',
        control='common continuous trajectories and executor; central authorization instead of peer quorum',
        restart_scope='actual per-UAV command-client process restart; central planner stays alive, unlike fused per-UAV planner restart')
    tmp=self.output/'summary-adapter.tmp';tmp.write_text(json.dumps(s,indent=2));tmp.replace(path)
    if common.rclpy.ok():self.diag.publish(String(data=json.dumps(s)))
common.Experiment.save=save
if __name__=='__main__':common.main()
