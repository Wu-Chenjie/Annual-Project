"""Recover completed and interrupted authorized sensor service windows."""
import bisect


def recover_service_windows(events, executions, task_start, task_end):
    """Cancellation ends the sensor-service window at its request time.

    Braking observations remain outside that task window. Pending handoffs do
    not start a service until actual consumption. Never-executed authorizations
    are disclosed separately rather than counted as sensor service.
    """
    tracking = {}
    for packet in executions:
        if packet.get('reason') == 'tracking_view' and packet.get('token'):
            stamp = packet.get('time', packet.get('receipt_time'))
            if stamp is not None:
                tracking.setdefault((packet['drone'], packet['token']), []).append(stamp)
    for stamps in tracking.values():
        stamps.sort()
    rows = []; inactive = []; active = {}

    def close(drone, end, reason, receipt=None):
        entry = active.pop(drone, None)
        if entry is None:
            return
        commit = entry['commit']; token = commit['token']
        begin = max(task_start, (receipt or {}).get('service_start', entry['start']))
        end = min(task_end, (receipt or {}).get('service_end', end))
        if end < begin:
            return
        clocks = tracking.get((drone, token), [])
        index = bisect.bisect_left(clocks, begin-.05)
        executed = index < len(clocks) and clocks[index] <= end+.05
        record = dict(drone=drone, token=token, region=(receipt or {}).get('region',commit.get('region')), start=begin, end=end,
            termination=reason, receipt=receipt, commit=commit, actual_execution_recorded=executed)
        (rows if executed else inactive).append(record)

    for event in sorted(events, key=lambda e: (e['time'], e['drone'])):
        drone = event['drone']; kind = event['type']
        if kind in ('path_committed', 'handoff_consumed'):
            if event['time'] > task_end:
                continue
            close(drone, event.get('handoff_time', event['time']), 'replaced_without_receipt')
            active[drone] = dict(commit=event, start=event.get('handoff_time', event['time']))
        elif kind == 'view_observed':
            if active.get(drone, {}).get('commit', {}).get('token') == event.get('token'):
                close(drone, event['time'], 'completed', event)
        elif kind == 'lease_cancellation_requested':
            token = event.get('token')
            if not token or active.get(drone, {}).get('commit', {}).get('token') == token:
                close(drone, event['time'], 'cancelled:'+event.get('reason', 'unspecified'))
        elif kind == 'incarnation_ready':
            close(drone, event['time'], 'process_restart')
    for drone in list(active):
        close(drone, task_end, 'task_end_censored')
    return dict(windows=rows, never_executed_authorizations=inactive,
        definition='Actual tracking of an authorized service, from commit/consumption until '
                   'completion, cancellation request, process restart or task end; clipped to '
                   'the common task window. Braking observations are outside the cancelled service.')
