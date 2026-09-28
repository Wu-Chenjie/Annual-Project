from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.execution_identity import bind_execution_identities


def command(token='old',drone=0):
    return dict(drone=drone,epoch=5,token=token,trajectory={'captured':True})


@pytest.mark.parametrize('fault',['none','missing','ambiguous','wrong_drone','modern'])
def test_legacy_epoch_binding_needs_one_actual_command_and_cannot_mask_modern_missing_tokens(fault):
    packet=dict(drone=0,epoch=5,trajectory_time=1.,reason='tracking_view')
    commands=[command()]
    if fault=='missing':commands=[]
    elif fault=='ambiguous':commands.append(command('other'))
    elif fault=='wrong_drone':commands=[command(drone=1)]
    result=bind_execution_identities([packet],commands,modern_required=fault=='modern')
    assert bool(result['errors'])==(fault!='none')
    if fault=='none':assert result['packets'][0]['token']=='old' and result['legacy_bound_packets']==1
    assert 'token' not in packet  # Raw evidence is never changed.


def test_provided_token_must_agree_with_its_epoch_and_idle_needs_no_binding():
    packets=[dict(drone=0,epoch=5,token='wrong',trajectory_time=1.,reason='tracking_view'),
             dict(drone=0,epoch=6,trajectory_time=0.,reason='idle')]
    result=bind_execution_identities(packets,[command()])
    assert result['errors']==['Executor token differs from its captured command epoch']
    assert result['legacy_bound_packets']==0


@pytest.mark.parametrize('fault',['none','moving','no_cancel','early','active_curve','not_arrived'])
def test_legacy_cancel_hold_needs_a_captured_cancel_and_a_stationary_reference(fault):
    packet=dict(drone=0,epoch=6,trajectory_time=10.,reason='tracking_view',receipt_time=12.,
                arrived=True,trajectory_method=None,reference_velocity=[0.,0.,0.],reference_acceleration=[0.,0.,0.])
    cancel=dict(drone=0,epoch=6,cancel=True,receipt_time=11.)
    if fault=='moving':packet['reference_velocity'][0]=.01
    elif fault=='early':packet['receipt_time']=10.
    elif fault=='active_curve':packet['trajectory_method']='minimum_jerk_bspline'
    elif fault=='not_arrived':packet['arrived']=False
    result=bind_execution_identities([packet],[] if fault=='no_cancel' else [cancel])
    assert bool(result['errors'])==(fault!='none')
    assert result['post_cancel_static_hold_packets']==int(fault=='none')
    if fault=='none':assert result['packets'][0]['reason']=='static_hold_after_cancel'
