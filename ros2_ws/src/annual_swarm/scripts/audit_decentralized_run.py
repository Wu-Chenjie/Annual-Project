#!/usr/bin/env python3
"""Audit recorded decentralized events against flight data, without rerunning a planner."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def audit(directory):
    root = Path(directory); summary = json.loads((root/'summary.json').read_text())
    events = sorted([json.loads(line) for path in root.glob('drone_*/events.jsonl') for line in path.read_text().splitlines()],
                    key=lambda e: (e['time'], e['drone']))
    proposed = {}; open_leases = {}; intervals = []; commits = 0; retired = 0; reassignments = []
    for e in events:
        kind = e['type']; drone = e['drone']
        if kind == 'path_proposed':
            assert e['token'] not in proposed, ('duplicate token', e)
            proposed[e['token']] = e
        elif kind == 'path_committed':
            token = e['token']; assert token in proposed and proposed[token]['time'] <= e['time'], e
            assert set(e['quorum']) == {0, 1, 2}-{drone}, ('incomplete quorum', e)
            assert token not in open_leases, ('duplicate commit', e)
            open_leases[token] = e; commits += 1
        elif kind in ('view_observed', 'reservation_retired'):
            token = e.get('token', f"{drone}:{e.get('epoch')}")
            if token in open_leases:
                begin = open_leases.pop(token)
                intervals.append(dict(drone=drone, region=begin['region'], start=begin['time'], end=e['time'], token=token))
            retired += kind == 'reservation_retired'
        elif kind == 'region_reassigned':
            reassignments.append(e)
    # Still-held leases at the log boundary count as held, not silently released.
    for token, begin in open_leases.items():
        intervals.append(dict(drone=begin['drone'], region=begin['region'], start=begin['time'], end=summary['simulation_time'], token=token))
    overlaps = []
    for i, a in enumerate(intervals):
        for b in intervals[i+1:]:
            if a['drone'] != b['drone'] and a['region'] == b['region'] and min(a['end'], b['end'])-max(a['start'], b['start']) > .01:
                overlaps.append([a, b])
    assert not overlaps, ('overlapping regional execution leases', overlaps)
    candidates = [json.loads(line) for path in root.glob('drone_*/candidates.jsonl') for line in path.read_text().splitlines()]
    max_backups = max((len(p['paths'])-1 for p in candidates), default=0)
    assert max_backups <= 5 and candidates
    assert all(np.isfinite(p['yaw']) and p['paths'] for p in candidates)
    yaw_travel = {i: 0. for i in range(3)}; previous = {}
    with (root/'trajectory.csv').open() as file:
        for row in csv.DictReader(file):
            i = int(row['drone']); yaw = float(row['yaw'])
            if i in previous:
                delta = yaw-previous[i]
                yaw_travel[i] += abs(np.arctan2(np.sin(delta), np.cos(delta)))
            previous[i] = yaw
    assert all(x >= 6 for x in yaw_travel.values()), yaw_travel
    assert summary['status'] == 'COMPLETE' and summary['coverage'] >= .95, summary
    assert summary['contacts_after_takeoff'] == 0 and summary['min_separation_m'] > .8, summary
    assert all(v > 3 for v in summary['distances_m'].values())
    assert all(g['nodes'] < g['free_cells']/4 for g in summary['graph'].values())
    assert summary['pause_resumed'] and reassignments
    result = dict(passed=True, commits_with_full_peer_quorum=commits, overlapping_region_leases=0,
                  confirmed_path_retirements=retired, region_reassignments=reassignments,
                  max_backups=max_backups, actual_yaw_travel_rad=yaw_travel,
                  sparse_ratios={i: g['nodes']/g['free_cells'] for i, g in summary['graph'].items()},
                  summary_sha256=hashlib.sha256((root/'summary.json').read_bytes()).hexdigest(),
                  scope='Recorded lease intervals, quorum identity, candidates, physical yaw, coverage, contacts and separation; not a formal asynchronous-network proof')
    (root/'audit.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('directory'); args = parser.parse_args()
    print(json.dumps(audit(args.directory), indent=2))

if __name__ == '__main__':
    main()
