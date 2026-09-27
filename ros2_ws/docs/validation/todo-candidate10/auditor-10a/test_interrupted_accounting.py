"""Raw sensor evidence for completed, cancelled and unfinished tasks."""
import csv
import gzip
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_todo_evidence import audit


def write_jsonl(path, rows):
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


def test_cancelled_navigation_and_task_end_exploration_keep_their_real_gain(tmp_path):
    summary=dict(fleet_size=1,status='COMPLETE',start_time=1.,finish_time=9.,simulation_time=9.,
                 distances_m={'0':.8},coverage=.95,contacts_after_takeoff=0)
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    (tmp_path/'coverage.json').write_text(json.dumps([[1.,.5],[8.,.9],[9.,.95]]))
    drone=tmp_path/'drone_0';drone.mkdir()
    def event(kind,time,token=None,**extra):
        return dict(type=kind,time=time,drone=0,**({'token':token} if token else {}),**extra)
    write_jsonl(drone/'events.jsonl',[
        event('path_committed',1.,'0:s:1',region=3),
        event('view_observed',4.,'0:s:1',region=3,purpose='explore',service_start=1.,service_end=4.),
        event('path_committed',5.,'0:s:2',region=4),
        event('lease_cancellation_requested',7.,region=4,reason='path_or_tracking_invalidated'),
        event('path_committed',8.,'0:s:3',region=5)])
    # Legacy logs carry the transit purpose in the selected planner rather than
    # in the peer intent. They must not be relabelled as zero-gain exploration.
    write_jsonl(drone/'candidates.jsonl',[
        dict(drone=0,epoch=2,region=4,paths=[dict(planner='mrdtg_transit')]),
        dict(drone=0,epoch=3,region=5,paths=[dict(planner='astar')])])
    write_jsonl(tmp_path/'peer_states.jsonl',[
        dict(drone=0,session='s',time=1.,position=[0,0,1.5],fusion={},
             intent=dict(token='0:s:1',region=3,purpose='explore'))])
    write_jsonl(tmp_path/'execution-evidence.jsonl',[
        dict(drone=0,time=t,receipt_time=t,token=token,reason='tracking_view',trajectory_time=.1)
        for t,token in [(2.,'0:s:1'),(6.,'0:s:2'),(8.5,'0:s:3')]])
    with gzip.open(tmp_path/'observations.jsonl.gz','wt') as stream:
        for sequence,(time,cell) in enumerate([(2.,[0,0,0]),(6.,[1,0,0]),(8.5,[2,0,0])]):
            stream.write(json.dumps(dict(source=0,time=time,sequence=sequence,sensor_session='s',
                shape=[3,3,3],origin=[0,0,0],resolution=1.,measured_indices=[cell],measured_values=[0]))+'\n')
    with (tmp_path/'trajectory.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['drone','time','x','y','z']);writer.writeheader()
        writer.writerows(dict(drone=0,time=t,x=t*.1,y=0.,z=1.5) for t in range(1,10))
    result=audit(tmp_path)
    assert result['service_windows']==3 and result['actual_team_new_known_in_windows']==3
    assert result['team_new_known_outside_service_windows']==0
    assert result['purpose_counts']['explore']==2 and result['purpose_counts']['transit_reobserve']==1
    assert result['exploration_low_team_yield']==2 and result['exploration_zero_team_yield']==0
    assert result['exploration_total_duration_s']==4.
    assert result['service_window_terminations']['task_end_censored']==1
    assert result['service_window_terminations']['cancelled:path_or_tracking_invalidated']==1
