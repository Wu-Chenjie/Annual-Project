import copy
from refine_stop_audit import proof

def fixture():
    h=dict(old_token='old',token='new',authorized_at=2.)
    events=[dict(type='handoff_proposed',token='new',drone=0,boundary=dict(trajectory_time=10.)),dict(type='lease_cancellation_requested',drone=0,time=3.,reason='geometry')]
    commands=[dict(cancel=True,drone=0,epoch=3,receipt_time=3.,reason='geometry')]
    executions=[dict(drone=0,time=2.9,token='old',epoch=1,trajectory_time=7.)]+[dict(drone=0,time=t,token=None,pending_token=None,epoch=3,reference_velocity=[0.,0.,0.]) for t in [3.1,3.3,3.5,3.7,3.9]]
    states=[dict(drone=0,time=t,intent=dict(token='old',committed=True,retiring=True),retiring_intents=[dict(token='new',committed=True,retiring=True)]) for t in [3.3,3.7,3.9]]
    return h,events,commands,executions,states,4.

def test_proof_requires_cancel_execution_old_prefix_and_held_leases():
    args=fixture();p=proof(*args)
    assert p and not p['consumed'] and p['classification'].startswith('cancelled_before_boundary')
    for fault in ['no_cancel','no_request','late_prefix','new_motion','pending_alive','old_epoch','released_lease','missing_end']:
        a=copy.deepcopy(args)
        if fault=='no_cancel':a[2][0]['cancel']=False
        elif fault=='no_request':a[1].pop()
        elif fault=='late_prefix':a[3][0]['trajectory_time']=10.
        elif fault=='new_motion':a[3][-1]['reference_velocity']=[.1,0.,0.]
        elif fault=='pending_alive':a[3][-1]['pending_token']='new'
        elif fault=='old_epoch':a[2][0]['epoch']=1
        elif fault=='released_lease':a[4][-1]['retiring_intents']=[]
        elif fault=='missing_end':a[3][:]=a[3][:2]
        assert proof(*a) is None,fault
