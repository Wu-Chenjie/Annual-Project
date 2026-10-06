"""Bind legacy executor epochs only to uniquely captured curve commands."""
import math


def bind_execution_identities(executions, commands, modern_required=False):
    by_epoch = {}
    cancels = {}
    for command in commands:
        if command.get('trajectory') and command.get('token') and command.get('epoch') is not None:
            by_epoch.setdefault((command['drone'], command['epoch']), set()).add(command['token'])
        if command.get('cancel') and command.get('epoch') is not None:
            cancels[(command['drone'],command['epoch'])]=command.get('receipt_time',command.get('time'))
    packets = []; errors = []; bindings = 0; holds = 0
    for packet in executions:
        normalized = dict(packet)
        actual = packet.get('trajectory_time', 0.) > 0. and packet.get('reason') in (
            'tracking_view', 'observation_dwell', 'view_observed')
        if actual:
            key=(packet.get('drone'),packet.get('epoch'));tokens = by_epoch.get(key, set())
            stamp=packet.get('time',packet.get('receipt_time'));cancel=cancels.get(key)
            def zero_reference(name):
                values=packet.get(name,[])
                return len(values)==3 and all(math.isfinite(x) and abs(x)<1e-9 for x in values)
            static_hold=(not tokens and cancel is not None and stamp is not None and stamp>=cancel-.05
                and packet.get('arrived') is True and packet.get('trajectory_method') is None
                and zero_reference('reference_velocity') and zero_reference('reference_acceleration'))
            if static_hold:
                normalized['reported_reason']=packet['reason'];normalized['reason']='static_hold_after_cancel'
                normalized['identity_source']='captured_cancel_and_zero_reference';holds+=1
                packets.append(normalized);continue
            if not packet.get('token'):
                if modern_required:
                    errors.append('Modern executor omitted its actual token')
                elif len(tokens) == 1:
                    normalized['token'] = next(iter(tokens))
                    normalized['identity_source'] = 'unique_captured_command_epoch'; bindings += 1
                else:
                    errors.append('Missing or ambiguous captured command for legacy executor epoch')
            elif tokens and (len(tokens) != 1 or packet['token'] not in tokens):
                errors.append('Executor token differs from its captured command epoch')
        packets.append(normalized)
    return dict(packets=packets, errors=sorted(set(errors)), legacy_bound_packets=bindings,
                post_cancel_static_hold_packets=holds)
