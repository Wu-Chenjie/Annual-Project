#!/usr/bin/env python3
"""Independent evidence audit for the single fused Gazebo exploration pipeline."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import planning_runtime
from core.exploration.voxel_mapping import VoxelTruth, VoxelMap


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def audit(root):
    root = Path(root); summary = json.loads((root/'summary.json').read_text())
    members = set(range(summary['fleet_size']))
    events = sorted((e for file in root.glob('drone_*/events.jsonl') for e in records(file)), key=lambda e: (e['time'], e['drone']))
    counts = collections.Counter(e['type'] for e in events)
    proposals = {}; committed = {}; intervals = []; methods = collections.Counter()
    for e in events:
        kind, drone = e['type'], e['drone']
        if kind == 'path_proposed':
            assert e['token'] not in proposals, ('reused incarnation token', e)
            proposals[e['token']] = e; methods[e['trajectory_method']] += 1
            limits = e['trajectory_limits']
            assert all(np.isfinite(v) for v in limits.values())
            assert limits['speed'] <= .60001 and limits['acceleration'] <= .80001 and limits['jerk'] <= 2.00001, e
        elif kind == 'path_committed':
            p = proposals[e['token']]
            assert p['time'] <= e['time'] and set(e['quorum']) == set(p['voters'])
            assert p['contingency'] or set(e['quorum']) == members-{drone}, e
            assert e['token'] not in committed
            committed[e['token']] = e
        elif kind in ('view_observed', 'reservation_retired'):
            begin = committed.pop(e.get('token'), None)
            if begin:
                intervals.append(dict(drone=drone, region=begin['region'], start=begin['time'], end=e['time']))
        elif kind == 'incarnation_ready':
            # A new incarnation is admitted only after its independent executor
            # reports a stopped vehicle. Close the old incarnation's reservation.
            for token, begin in list(committed.items()):
                if begin['drone'] == drone and begin.get('incarnation') != e.get('incarnation'):
                    intervals.append(dict(drone=drone, region=begin['region'], start=begin['time'], end=e['time']))
                    del committed[token]
        elif kind == 'pair_cvrp_prepared':
            assert max(e['loads']) <= e['capacity']+1e-6, e
    for begin in committed.values():
        intervals.append(dict(drone=begin['drone'], region=begin['region'], start=begin['time'], end=summary['simulation_time']))
    for k, a in enumerate(intervals):
        for b in intervals[k+1:]:
            assert not (a['drone'] != b['drone'] and a['region'] == b['region']
                        and min(a['end'], b['end'])-max(a['start'], b['start']) > .05), (a, b)
    candidate_count = 0; max_backups = 0; planners = collections.Counter()
    for file in root.glob('drone_*/candidates.jsonl'):
        for c in records(file):
            assert 1 <= len(c['paths']) <= 6 and np.isfinite(c['yaw'])
            candidate_count += 1; max_backups = max(max_backups, len(c['paths'])-1)
            for p in c['paths']:
                assert np.isfinite(p['points']).all()
                planners[p['planner']] += 1
    assert candidate_count
    altitude = {i: [] for i in members}; yaw_travel = {i: 0. for i in members}; previous = {}
    with (root/'trajectory.csv').open() as file:
        for row in csv.DictReader(file):
            i = int(row['drone']); yaw = float(row['yaw']); t = float(row['time'])
            if i in previous:
                delta = yaw-previous[i]; yaw_travel[i] += abs(np.arctan2(np.sin(delta), np.cos(delta)))
            previous[i] = yaw
            if t >= summary['start_time']:
                altitude[i].append(float(row['z']))
    spans = {i: float(np.ptp(z)) for i, z in altitude.items()}
    assert min(yaw_travel.values()) > 6 and max(spans.values()) > .7
    assert summary['status'] == 'COMPLETE' and summary['coverage'] >= .95
    # Recompute the denominator from geometry and the final sensor map. A
    # partially occupied boundary voxel must not be called truth-free merely
    # because its center lies outside a wall. Retain the old metric explicitly.
    snapshot = np.load(root/'observed_final.npz')
    observed = VoxelMap(snapshot['bounds'], resolution=float(snapshot['resolution']))
    observed.state[:] = snapshot['state']
    truth = VoxelTruth(root/'map.json', resolution=observed.resolution)
    measured = truth.coverage_metrics(observed)
    for field in ('coverage', 'legacy_center_free_coverage', 'truth_free_voxels', 'observed_free_voxels'):
        assert abs(summary[field]-measured[field]) < 1e-12, (field, summary[field], measured[field])
    assert summary['contacts_after_takeoff'] == 0 and summary['min_separation_m'] > .8
    assert summary['mapping_dimensions'] == 3 and min(summary['observation_frames'].values()) > 100
    assert all(x > 3 for x in summary['distances_m'].values()) and min(summary['views'].values()) > 0
    assert summary['pause_resumed'] and counts['region_reassigned'] > 0
    assert summary['network_resumed'] and counts['network_recovered'] >= len(members)
    assert summary['restart_recovered'] and counts['process_restart_requested'] == 1
    assert counts['peer_rejoined'] >= len(members)-1
    assert summary['dynamic_trial'] and summary['dynamic_finished']
    dt = summary['dynamic_trial']['start']; target = summary['dynamic_trial']['drone']
    obstacle_responses = [e for e in events if e['drone'] == target and dt <= e['time'] <= dt+24
                          and e['type'] in ('lease_cancellation_requested', 'cached_route_switched')]
    assert obstacle_responses, 'No logged response to the crossing obstacle'
    assert counts['pair_cvrp_committed'] and counts['pair_cvrp_prepared']
    applied_by = {}; pair_members = {}
    for state in records(root/'peer_states.jsonl'):
        tx = state.get('pair_transaction')
        if tx and tx['phase'] == 'applied':
            applied_by.setdefault(tx['token'], set()).add(state['drone'])
            pair_members[tx['token']] = {tx['leader'], tx['follower']}
    jointly_applied = [token for token, seen in applied_by.items() if pair_members[token] <= seen]
    assert jointly_applied, 'No bilateral assignment has recorded application by both peers'
    assert any(f['hgrid_splits'] > 0 and f['handshake_updates'] > 0 and f['delta_applied'] > 0 for f in summary['fusion'].values())
    assert all(g['nodes'] < g['free_cells']/4 for g in summary['graph'].values())
    result = dict(passed=True, events=dict(counts), jointly_applied_pair_transactions=jointly_applied,
                  trajectory_methods=dict(methods), candidate_planners=dict(planners),
                  candidates=candidate_count, max_backups=max_backups, airborne_altitude_span_m=spans,
                  yaw_travel_rad=yaw_travel, overlapping_same_region_leases=0, dynamic_response_events=obstacle_responses,
                  coverage=summary['coverage'], t95_s=summary['t95'], contacts=summary['contacts_after_takeoff'],
                  coverage_definition=measured['coverage_definition'], legacy_center_free_coverage=measured['legacy_center_free_coverage'],
                  min_separation_m=summary['min_separation_m'], truth_tracking=summary['tracking'],
                  summary_sha256=hashlib.sha256((root/'summary.json').read_bytes()).hexdigest(),
                  scope='Observed flight and protocol evidence for this run; not a proof of arbitrary networks or paper benchmark reproduction.')
    (root/'fusion-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory')
    print(json.dumps(audit(parser.parse_args().directory), indent=2))
