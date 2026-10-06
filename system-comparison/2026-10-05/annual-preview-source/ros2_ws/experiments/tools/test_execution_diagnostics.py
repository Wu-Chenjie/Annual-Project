"""Concurrent planning time is a union; missing legacy boundaries are explicit."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from execution_diagnostics import merged,overlap,planning_intervals,phase


def test_overlap_does_not_double_count_concurrent_requests_or_time_outside_mission():
    events=[dict(type=kind,drone=0,time=t,incarnation='s',request_id=token)
            for kind,t,token in [('planning_submitted',1.,'a'),('planning_submitted',2.,'b'),
                                 ('planning_result',4.,'a'),('planning_timeout',6.,'b')]]
    intervals,counts,unmatched=planning_intervals(events,3.,5.)
    assert intervals[0]==[(3.,5.)]
    assert overlap(3.5,4.5,intervals[0])==1.
    assert counts=={'planning_result':1} and unmatched==0


def test_legacy_worker_flag_is_not_invented_request_or_flight_delay():
    intervals,counts,unmatched=planning_intervals([dict(type='planning_result',drone=0,time=3.,total_wall_s=2.)],1.,5.)
    assert not intervals and unmatched==1
    assert phase({'reason':'tracking_view'},{'worker':True})=='execution'
    assert phase({'reason':'idle'},{'worker':True})=='planning_wait'
    assert phase({'reason':'tracking_view'},{'purpose':'transit_reobserve'})=='navigation_reobserve'
