"""Lightweight execution contracts; clients need no planning/scipy imports."""
def central_execution_lease(states, drone, token, now, seeds, timeout=3.):
    """An actual centralized authorization, not a fabricated peer quorum."""
    state = states.get(drone, {}); intent = state.get('intent') or {}
    return bool(token and state.get('available') and state.get('ready')
                and 0 <= now-state.get('time', -1e9) <= timeout
                and 0 <= now-state.get('central_time', -1e9) <= timeout
                and intent.get('token') == token and intent.get('committed')
                and not intent.get('retiring'))


def view_finished(execution, epoch, duration):
    # 'view_observed' reason lasts one 50 Hz tick; 5 Hz reports may skip it.
    # Completion is the persistent epoch/arrival/trajectory-time contract.
    return bool(execution.get('epoch') == epoch and execution.get('arrived')
                and execution.get('trajectory_time', -1.) >= duration-1e-6)
