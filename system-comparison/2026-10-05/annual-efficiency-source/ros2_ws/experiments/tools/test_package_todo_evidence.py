import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).parent))
from package_todo_evidence import clock_anchor,lifecycle_pair,late_support


def test_simulation_clip_time_uses_recorded_wall_mapping_and_rejects_extrapolation():
    rows=[dict(drone=0,time=10.,receipt_wall_time=100.),dict(drone=0,time=20.,receipt_wall_time=120.)]
    assert clock_anchor(rows,15.,90.)==20.
    with pytest.raises(ValueError):clock_anchor(rows,25.,90.)


def test_lifecycle_and_support_require_matching_regions_and_actual_followup():
    events=[dict(type='region_low_yield',drone=0,time=1.,region=3),
            dict(type='task_reactivated',drone=0,time=5.,region=4,reason='actual_evidence_changed')]
    assert lifecycle_pair(events) is None
    events.append(dict(type='task_reactivated',drone=0,time=6.,region=3,reason='actual_evidence_changed'))
    assert lifecycle_pair(events)[1]['time']==6.
    assignment=[dict(type='pair_cvrp_committed',time=1.,assignments={'3':0}),
                dict(type='pair_cvrp_committed',time=10.,assignments={'3':1})]
    assert late_support(assignment,5.) is None
    assignment.append(dict(type='path_committed',time=12.,region=3,drone=1))
    assert late_support(assignment,5.)[1]['drone']==1
