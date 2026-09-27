"""A commit log alone is not evidence that a cached or moving curve executed."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_execution_protocol import check
from core.planning.continuous_trajectory import interpolate


@pytest.mark.parametrize('fault',['none','missing_command','ambiguous_epoch','before_authorization'])
def test_legacy_epoch_execution_is_checked_against_the_actual_authorized_interval(fault):
    old=interpolate(np.array([[2.,2.,1.5],[4.,2.,1.5]]),[12.],0.,0.)
    events=[dict(type='path_proposed',drone=0,time=1.,token='old'),
            dict(type='path_committed',drone=0,time=2.,token='old',quorum=[1])]
    intent=dict(token='old',epoch=7,region=3,voters=[1],contingency=False)
    commands=[dict(intent,drone=0,trajectory=old.to_dict())]
    execution=dict(drone=0,epoch=7,reason='tracking_view',trajectory_time=.1,
                   receipt_time=1. if fault=='before_authorization' else 3.)
    if fault=='missing_command':commands=[]
    elif fault=='ambiguous_epoch':commands.append(dict(commands[0],token='other'))
    result=check(events,[dict(drone=0,intent=intent)],[execution],2,30.,commands)
    assert result['passed']==(fault=='none')
    if fault=='none':assert result['legacy_executor_epoch_bindings']==1
    assert 'token' not in execution


def trace():
    old=interpolate(np.array([[2.,2.,1.5],[4.,2.,1.5]]),[12.],0.,.4)
    p,v,a,h,r=old.sample(8.)
    new=interpolate(np.array([p,[6.,3.,1.5]]),[16.],h,.8-h,start_velocity=v,start_acceleration=a,
                    start_yaw_rate=r,start_yaw_acceleration=old.yaw_acceleration(8.))
    old_intent=dict(token='old',region=7,voters=[1],contingency=False)
    new_intent=dict(token='new',region=8,voters=[1],contingency=False,handoff=dict(from_token='old',trajectory_time=8.))
    events=[dict(type='path_proposed',drone=0,time=1.,token='old'),
            dict(type='path_committed',drone=0,time=2.,token='old',quorum=[1]),
            dict(type='handoff_proposed',drone=0,time=3.,token='new'),
            dict(type='handoff_authorized',drone=0,time=4.,token='new',quorum=[1]),
            dict(type='reservation_retired',drone=0,time=10.1,token='old')]
    states=[dict(drone=0,intent=old_intent,pending_intent=new_intent)]
    commands=[dict(old_intent,drone=0,trajectory=old.to_dict()),dict(new_intent,drone=0,trajectory=new.to_dict())]
    executions=[dict(token='new',trajectory_time=.1,reason='tracking_view',handoff_from_token='old',handoff_time=10.)]
    return events,states,executions,commands


@pytest.mark.parametrize('fault',['none','missing_command','missing_ack','bad_boundary','never_executed'])
def test_independent_curve_ack_and_executor_checks(fault):
    events,states,executions,commands=trace()
    if fault=='missing_command':commands=[]
    if fault=='missing_ack':events[3]['quorum']=[]
    if fault=='bad_boundary':states[0]['pending_intent']['handoff']['trajectory_time']=8.3
    if fault=='never_executed':executions=[]
    result=check(events,states,executions,2,30.,commands)
    assert result['passed']==(fault=='none')
    if fault=='missing_command':assert result['status']=='INCOMPLETE_OBSERVABILITY'


def test_cancelled_authorized_pending_curve_keeps_old_lease_and_need_not_execute():
    events,states,_,commands=trace();events=events[:-1]
    events.append(dict(type='handoff_cancelled',drone=0,time=5.,token='new'))
    events.append(dict(type='reservation_retired',drone=0,time=20.,token='old'))
    result=check(events,states,[],2,30.,commands)
    assert result['passed'] and result['handoffs'][0]['cancelled_at']==5.


@pytest.mark.parametrize('old_candidate',['other','reserve'])
def test_refitting_active_path_is_not_a_different_cached_backup(old_candidate):
    events,states,executions,commands=trace()
    states[0]['pending_intent']['candidate_id']='reserve'
    events += [dict(type='cached_route_selected',drone=0,time=3.,token='new',candidate='reserve'),
               dict(type='cached_route_switched',drone=0,time=4.,token='new',candidate='reserve',
                    origin=dict(token='old',time=2.5,candidate=old_candidate))]
    result=check(events,states,executions,2,30.,commands)
    assert result['passed']==(old_candidate=='other')


@pytest.mark.parametrize('fault',['no_authorization','after_retirement'])
def test_actual_execution_requires_a_live_authorization(fault):
    events,states,executions,commands=trace()
    executions.append(dict(token='rogue' if fault=='no_authorization' else 'old',drone=0,
                           time=11.,trajectory_time=.2,reason='tracking_view'))
    assert not check(events,states,executions,2,30.,commands)['passed']


def test_absent_command_stream_cannot_receive_a_protocol_pass():
    result=check([],[],[],2,30.)
    assert not result['passed'] and result['status']=='INCOMPLETE_OBSERVABILITY'


def test_same_curve_cannot_be_relabelled_as_another_region_under_the_token():
    events,states,executions,commands=trace()
    commands[0]['region']=99
    assert not check(events,states,executions,2,30.,commands)['passed']


def test_region_only_legacy_cache_label_is_not_execution_proof_or_an_audit_crash():
    events,states,_,commands=trace()
    events=events[:2]+[dict(type='cached_route_switched',drone=0,time=5.,region=7)]
    states=[dict(drone=0,intent=states[0]['intent'])]
    executions=[dict(token='old',drone=0,time=4.,trajectory_time=1.,reason='tracking_view')]
    result=check(events,states,executions,2,30.,commands[:1])
    assert result['passed'] and not result['cached_route_chains']
    assert len(result['legacy_cache_labels_without_execution_provenance'])==1


@pytest.mark.parametrize('metadata',[{},dict(token='new',candidate='reserve')])
def test_modern_cache_metadata_is_mandatory_and_missing_fields_fail_cleanly(metadata):
    events,states,executions,commands=trace()
    events.append(dict(type='cached_route_switched',drone=0,time=5.,region=8,**metadata))
    result=check(events,states,executions,2,30.,commands)
    assert not result['passed'] and 'Incomplete modern cached-route metadata' in result['errors']
