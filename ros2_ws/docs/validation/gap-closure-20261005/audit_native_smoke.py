#!/usr/bin/env python3
"""Read-only, independent checks for the bounded native smoke (no T95 claim)."""
import argparse
from collections import deque
import gzip
import hashlib
import json
import math
from pathlib import Path


def rows(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def p95(values):
    values = sorted(values)
    x = .95 * (len(values)-1)
    a, b = math.floor(x), math.ceil(x)
    return values[a] + (values[b]-values[a])*(x-a)


def audit(root):
    root = Path(root)
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(message)
    frames = {}
    for packet in rows(root/'observations.jsonl.gz'):
        key = (packet['source'], packet['sensor_session'], packet['sequence'])
        # Keep only small packet metadata, never sensor voxel arrays.
        frames[key] = {k: v for k, v in packet.items()
                       if k not in ('indices', 'values', 'free_indices')}
    commands = {p['token']: p for p in rows(root/'command-evidence.jsonl')
                if p.get('token')}
    completed = []
    timing_count = rolling_count = 0
    consumed = []
    for file in sorted(root.glob('drone_*/events.jsonl')):
        planning = deque(maxlen=64)
        authorization = deque(maxlen=64)
        proposed = {}
        for event in rows(file):
            kind = event['type']
            if kind in ('planning_result', 'planning_failed', 'planning_timeout'):
                if 'total_wall_s' in event:
                    planning.append(event['total_wall_s'])
            elif kind in ('path_proposed', 'handoff_proposed'):
                proposed[event['token']] = event['monotonic_wall_time']
            elif kind in ('path_committed', 'handoff_authorized'):
                start = proposed.pop(event['token'], None)
                if start is not None:
                    authorization.append(event['monotonic_wall_time']-start)
            elif kind in ('handoff_cancelled', 'reservation_retired'):
                proposed.pop(event['token'], None)
            elif kind == 'terminal_observation_completed':
                proof = event['proof']
                key = (event['drone'], proof['sensor_session'], proof['sequence'])
                frame = frames.get(key)
                command = commands.get(event['token'])
                require(frame is not None and command is not None, f'Missing frame/command: {key}')
                if frame is None or command is None:
                    continue
                require(frame['sensor'] == 'gazebo_gpu_lidar' and
                        frame['integration_completed'] is True and
                        frame['ray_cell_count'] > 0 and frame['point_count'] > 0,
                        f'Unintegrated native sensor frame: {key}')
                for k in ('time', 'map_version', 'ray_cell_count', 'shape', 'origin', 'resolution'):
                    require(proof[k] == frame[k], f'Frame proof mismatch {k}: {key}')
                require(proof['position'] == frame['sensor_position'] and
                        proof['yaw'] == frame['sensor_yaw'], f'Pose mismatch: {key}')
                require(abs(frame['pose_time']-frame['time']) <= .15,
                        f'Unsynchronized frame pose: {key}')
                digest = hashlib.sha256(json.dumps(
                    [frame['shape'], frame['origin'], frame['resolution']],
                    separators=(',', ':')).encode()).hexdigest()
                require(digest == proof['grid'] == command['observation_grid'], f'Grid mismatch: {key}')
                require(proof['time'] >= proof['arrived_at'] and
                        0 <= proof['completed_at']-proof['time'] <= 1.5,
                        f'Pre-arrival or stale observation: {key}')
                require(proof['token'] == event['token'] == command['token'] and
                        proof['epoch'] == command['epoch'], f'Wrong task proof: {key}')
                distance = math.dist(proof['position'], command['path'][-1])
                yaw_error = abs(math.atan2(math.sin(proof['yaw']-command['yaw']),
                                           math.cos(proof['yaw']-command['yaw'])))
                require(distance < .07 and yaw_error < .12, f'Wrong endpoint frame: {key}')
                completed.append(dict(drone=event['drone'], token=event['token'],
                                      frame_key=key, proof=proof,
                                      position_error_m=distance, yaw_error_rad=yaw_error))
            elif kind == 'handoff_preparation_timing':
                timing_count += 1
                require(event['planning_samples'] == len(planning) and
                        event['authorization_samples'] == len(authorization), 'Latency window mismatch')
                for name, values, default in [('planning', planning, 12.),
                                               ('authorization', authorization, 2.4)]:
                    estimate = p95(values) if len(values) >= 5 else max(default, max(values, default=0.))
                    require(math.isclose(event[name+'_wall_p95_s'], estimate, abs_tol=1e-8),
                            f'{name} P95 mismatch')
                expected = max(.5, (event['planning_wall_p95_s'] +
                                    event['authorization_wall_p95_s'] + .5)*event['real_time_factor'])
                require(math.isclose(expected, event['lead_sim_s'], abs_tol=1e-8), 'Clock conversion mismatch')
                if event['selected_boundary'] is not None:
                    require(event['selected_boundary']-event['trajectory_progress'] >= event['lead_sim_s']-1e-8,
                            'Handoff boundary truncates the required lead')
                rolling_count += event['planning_source'] == event['authorization_source'] == 'rolling_p95'
            elif kind == 'handoff_consumed':
                consumed.append(event)
    require(bool(completed) and rolling_count > 0 and bool(consumed), 'Missing exercised feature')
    flight = json.loads((root/'independent-acceptance.json').read_text())
    protocol = json.loads((root/'execution-protocol-audit.json').read_text())
    cleanup = json.loads((root/'process-cleanup.json').read_text())
    safety = {k: v for k, v in flight['gates'].items() if k != 'final_map_matches'}
    require(all(safety.values()) and protocol['passed'] and not cleanup['remaining'],
            'Flight, authorization or cleanup gate failed')
    inputs = [root/'summary.json', root/'run-result.json', root/'observations.jsonl.gz',
              root/'command-evidence.jsonl', root/'independent-acceptance.json',
              root/'execution-protocol-audit.json', root/'process-cleanup.json',
              *root.glob('drone_*/events.jsonl')]
    return dict(passed=not errors, errors=errors, completed_observations=len(completed),
                timing_records=timing_count, rolling_p95_records=rolling_count,
                consumed_handoffs=consumed, observation_proofs=completed,
                safety_gates=safety, protocol_passed=protocol['passed'],
                final_map_matches=flight['gates']['final_map_matches'],
                outcome=json.loads((root/'run-result.json').read_text())['outcome'],
                scope='90 simulation-second native smoke; not a completed coverage or T95 experiment',
                auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.directory)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ('observation_proofs', 'consumed_handoffs', 'input_sha256')}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
