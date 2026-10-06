"""Read-only classification at the exact proposed old-curve boundary."""
import json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'Annual-Project/next_project'))
from core.planning.continuous_trajectory import ContinuousTrajectory

def classify(root):
    root=Path(root)
    events=[json.loads(line) for path in root.glob('drone_*/events.jsonl') for line in path.read_text().splitlines() if line.strip()]
    commands=[json.loads(line) for line in (root/'command-evidence.jsonl').read_text().splitlines() if line.strip()]
    output=[]
    for event in events:
        if event['type']!='handoff_consumed':continue
        proposals=[e for e in events if e['type']=='handoff_proposed' and e['token']==event['token']]
        assert len(proposals)==1
        proposal=proposals[0]
        old=[c for c in commands if c.get('token')==event['old_token'] and 'trajectory' in c]
        assert old and all(c['trajectory']==old[0]['trajectory'] for c in old)
        curve=ContinuousTrajectory.from_dict(old[0]['trajectory'])
        boundary=proposal['boundary']['trajectory_time'];_,v,a,_,_=curve.sample(boundary)
        product=float(np.dot(v,a))
        output.append(dict(token=event['token'],event_time=event['time'],old_curve_boundary_time=boundary,
            actual_event_speed_mps=event['speed'],boundary_reference_speed_mps=float(np.linalg.norm(v)),
            longitudinal_acceleration_product=product,before_deceleration=product>=-1e-7,
            note='Analytic old-curve boundary phase; actual speed is sampled at agent receipt, not exact executor transition.'))
    (root/'handoff-motion-phases.json').write_text(json.dumps(output,indent=2)+'\n')
    return output

if __name__=='__main__':print(json.dumps(classify(sys.argv[1]),indent=2))
