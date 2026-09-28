from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.service_windows import recover_service_windows


def event(kind, time, token=None, **fields):
    return dict(type=kind,time=time,drone=0,**({'token':token} if token else {}),**fields)


def execution(time, token):
    return dict(drone=0,time=time,token=token,reason='tracking_view')


def test_cancelled_and_unfinished_services_are_retained_with_completed_ones():
    events=[event('path_committed',1.,'a',region=3),event('view_observed',4.,'a',service_start=1.,service_end=4.),
            event('path_committed',5.,'b',region=7),event('lease_cancellation_requested',8.,reason='obstacle'),
            event('path_committed',9.,'c',region=8)]
    result=recover_service_windows(events,[execution(2.,'a'),execution(6.,'b'),execution(10.,'c')],1.,12.)
    assert [(p['token'],p['start'],p['end']) for p in result['windows']]==[('a',1.,4.),('b',5.,8.),('c',9.,12.)]
    assert result['windows'][1]['termination']=='cancelled:obstacle'
    assert result['windows'][2]['termination']=='task_end_censored'


def test_pending_or_never_executed_curves_are_not_sensor_service():
    events=[event('path_committed',1.,'a'),event('handoff_authorized',3.,'pending'),
            event('handoff_cancelled',4.,'pending'),event('lease_cancellation_requested',5.),
            event('path_committed',6.,'not-executed'),event('lease_cancellation_requested',7.)]
    result=recover_service_windows(events,[execution(2.,'a')],1.,10.)
    assert [p['token'] for p in result['windows']]==['a']
    assert [p['token'] for p in result['never_executed_authorizations']]==['not-executed']


def test_consumed_handoff_boundaries_do_not_overlap_and_can_arrive_late():
    events=[event('path_committed',1.,'a'),event('view_observed',8.3,'a',service_start=1.,service_end=8.),
            event('handoff_consumed',8.3,'b'),event('view_observed',15.,'b',service_start=8.,service_end=15.)]
    rows=recover_service_windows(events,[execution(2.,'a'),execution(9.,'b')],1.,20.)['windows']
    assert [(r['start'],r['end']) for r in rows]==[(1.,8.),(8.,15.)]


def test_restart_and_delayed_cancellation_are_clipped_to_the_task_end():
    events=[event('path_committed',2.,'a'),event('incarnation_ready',6.),
            event('path_committed',8.,'b'),event('lease_cancellation_requested',20.)]
    rows=recover_service_windows(events,[execution(3.,'a'),execution(9.,'b')],1.,12.)['windows']
    assert [(r['start'],r['end']) for r in rows]==[(2.,6.),(8.,12.)]


def test_a_pending_token_cancellation_does_not_end_the_current_service():
    events=[event('path_committed',2.,'a'),event('lease_cancellation_requested',6.,'pending')]
    rows=recover_service_windows(events,[execution(3.,'a')],1.,12.)['windows']
    assert len(rows)==1 and rows[0]['end']==12.
