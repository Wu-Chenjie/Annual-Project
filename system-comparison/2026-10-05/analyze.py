"""Metrics and plots for one fresh, adapted three-system physical comparison."""
import csv, gzip, hashlib, json, os
from collections import Counter
from pathlib import Path
import numpy as np
os.environ.setdefault('MPLCONFIGDIR', '/tmp/comparison-1005-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE = Path(__file__).resolve().parent
MODES = ['annual', 'racer', 'gvp']
NAMES = ['Annual (current)', 'RACER', 'GVP-MREP']
COLORS = ['#2563eb', '#d97706', '#059669']


def records(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def metrics(mode):
    root = BASE/f'trial-{mode}-900'
    result = json.loads((root/'result.json').read_text())
    samples = list(csv.DictReader((root/'trajectory.csv').open()))
    by_uav = {i: sorted([r for r in samples if int(r['i']) == i], key=lambda r: float(r['t'])) for i in range(2)}
    fleet_seconds = low_seconds = pose_low_seconds = 0.
    distances = {}; gaps = {}; maximum_pose_speed = 0.
    for i, rows in by_uav.items():
        distance = 0.; gaps[str(i)] = 0.
        for a, b in zip(rows, rows[1:]):
            t0, t1 = float(a['t']), float(b['t'])
            start, end = max(20., t0), min(result['t'], t1)
            if not 0 < t1-t0 <= .6:
                if end > start:
                    gaps[str(i)] += end-start
                continue
            if end <= start:
                continue
            dt = end-start
            ds = np.linalg.norm([float(b[k])-float(a[k]) for k in ('x', 'y', 'z')])
            speed = ds/(t1-t0)
            maximum_pose_speed = max(maximum_pose_speed, speed)
            distance += ds*dt/(t1-t0)
            fleet_seconds += dt
            low_seconds += dt*(float(a['speed']) < .1)
            pose_low_seconds += dt*(speed < .1)
        distances[str(i)] = float(distance)
    refs = list(records(root/'executed-commands.jsonl'))
    vmax = max((np.linalg.norm(r['v']) for r in refs), default=0.)
    amax = max((np.linalg.norm(r['a']) for r in refs), default=0.)
    counts = Counter(); first = {}; last = {}
    with gzip.open(root/'sensor-stream.jsonl.gz', 'rt') as stream:
        for line in stream:
            r = json.loads(line); k = r['type']+('_'+str(r['i']) if 'i' in r else '')
            counts[k] += 1; first.setdefault(k, r['t']); last[k] = r['t']
    rates = {k: (n-1)/(last[k]-first[k]) if last[k] > first[k] else 0. for k, n in counts.items()}
    replay = json.loads((root/'coverage-replay.json').read_text())
    valid = result['status'] == 'COVERAGE_TARGET' and not result['geometric_collision_events'] and not result['failure_reason']
    values = dict(system=NAMES[MODES.index(mode)], result=result,
                  valid_completed_trial=valid,
                  exploration_t95_s=result['t95']-20 if valid else None,
                  tail_90_to_95_s=result['t95']-result['t90'] if valid and result['t90'] is not None else None,
                  mission_distance_m=distances, mission_total_distance_m=sum(distances.values()),
                  sampled_fleet_s=fleet_seconds, expected_fleet_s=2*max(0., result['t']-20.),
                  low_speed_fraction=low_seconds/fleet_seconds if fleet_seconds else None,
                  pose_difference_low_speed_fraction=pose_low_seconds/fleet_seconds if fleet_seconds else None,
                  maximum_pose_difference_speed_mps=maximum_pose_speed,
                  maximum_true_velocity_mps=result['max_measured_speed_mps'],
                  low_speed_definition='True Gazebo linear speed <0.1 m/s, time-weighted over t=20 through result time; pose-difference estimate reported separately.',
                  missing_pose_interval_s=gaps,
                  final_distinct_team_free_voxels=round(result['coverage']*result['truth_free_voxels']),
                  final_overlap_free_voxels=result['overlap_free_voxels'],
                  final_overlap_fraction_of_team_free=result['overlap_free_voxels']/round(result['coverage']*result['truth_free_voxels']),
                  overlap_note='Final observed-free set intersection, not a duplicate-service or wasted-work ratio.',
                  coverage_replay=replay,
                  checks=dict(reference_speed_cap=bool(vmax <= .600001),
                              reference_acceleration_cap=bool(amax <= .800001),
                              native_commands_from_both_uavs=result['native_seen'] == [0, 1],
                              recorded_sensor_stream_valid=True,
                              world_sha256=hashlib.sha256((root/'scene/scene.sdf').read_bytes()).hexdigest(),
                              recorded_counts=dict(counts), recorded_sim_hz=rates,
                              final_coverage_replay_difference=replay['difference'],
                              final_coverage_replay_target=replay['recomputed']['coverage'] >= .95,
                              process_cleanup={side: json.loads((root/(side+'-process-cleanup.json')).read_text())
                                               for side in ['ros2']+(['ros1'] if mode != 'annual' else [])}))
    return values


def analyze():
    summaries = [metrics(m) for m in MODES]
    frozen = json.loads((BASE/'frozen-sha256.json').read_text())
    annual = json.loads((BASE/'annual-source-manifest.json').read_text())
    checks = dict(identical_generated_world=len({r['checks']['world_sha256'] for r in summaries}) == 1,
                  identical_truth_denominator=len({r['result']['truth_free_voxels'] for r in summaries}) == 1,
                  frozen_harness_unchanged=all(hashlib.sha256((BASE/p).read_bytes()).hexdigest() == h for p, h in frozen.items()),
                  frozen_annual_source_unchanged=all(hashlib.sha256((BASE/'annual-source'/p).read_bytes()).hexdigest() == h for p, h in annual['files'].items()),
                  all_reference_limits=all(r['checks']['reference_speed_cap'] and r['checks']['reference_acceleration_cap'] for r in summaries),
                  all_cleanup_empty=all(not v['remaining'] for r in summaries for v in r['checks']['process_cleanup'].values()))
    (BASE/'comparison.json').write_text(json.dumps(summaries, indent=2)+'\n')
    (BASE/'validation.json').write_text(json.dumps(checks, indent=2)+'\n')
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'path', 'axes.spines.top': False,
                         'axes.spines.right': False, 'font.size': 10})
    fig, axs = plt.subplots(2, 2, figsize=(12, 10), layout='constrained')
    for m, name, color in zip(MODES, NAMES, COLORS):
        rows = list(records(BASE/f'trial-{m}-900/coverage.jsonl'))
        axs[0, 0].plot([r['t'] for r in rows], [100*r['coverage'] for r in rows], color=color, lw=2, label=name)
    axs[0, 0].axhline(95, color='#64748b', ls='--'); axs[0, 0].axvspan(0, 20, color='#e2e8f0', alpha=.5)
    axs[0, 0].set(xlabel='Simulation time, including 20 s startup (s)', ylabel='Team free-space coverage (%)', ylim=(0, 102), title='Recorded coverage until the stopping condition')
    axs[0, 0].legend(fontsize=9); axs[0, 0].grid(alpha=.15)
    axs[0, 1].bar(NAMES, [r['exploration_t95_s'] or np.nan for r in summaries], color=COLORS)
    axs[0, 1].set(ylabel='T95 minus common 20 s startup (s)', title='Completed exploration time')
    axs[1, 0].bar(NAMES, [r['mission_total_distance_m'] for r in summaries], color=COLORS)
    axs[1, 0].set(ylabel='Fleet distance after startup (m)', title='Actual flight distance, t >= 20 s')
    axs[1, 1].bar(NAMES, [100*r['low_speed_fraction'] for r in summaries], color=COLORS)
    axs[1, 1].set(ylabel='Time at true speed below 0.1 m/s (%)', title='Time-weighted low-speed fraction')
    for ax in axs.flat[1:]:
        ax.grid(axis='y', alpha=.15); ax.set_axisbelow(True)
        for patch in ax.patches:
            height = patch.get_height()
            if np.isfinite(height):
                ax.text(patch.get_x()+patch.get_width()/2, height, f'{height:.1f}', ha='center', va='bottom')
    fig.suptitle('Fresh adapted physical comparison | seed 900 | 2 UAVs\nSame map, LiDAR, estimator, reference governor and motor PID', fontsize=14)
    fig.savefig(BASE/'comparison.png', dpi=160); fig.savefig(BASE/'comparison.svg'); plt.close(fig)
    world = json.loads((BASE/'map.json').read_text())
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), layout='constrained')
    for ax, mode, name in zip(axes, MODES, NAMES):
        for ob in world['obstacles']:
            lo, hi = np.asarray(ob['min']), np.asarray(ob['max'])
            ax.add_patch(Rectangle(lo[:2], *(hi-lo)[:2], facecolor='#cbd5e1', edgecolor='#94a3b8'))
        rows = list(csv.DictReader((BASE/f'trial-{mode}-900/trajectory.csv').open()))
        for i, color in enumerate(['#0e7490', '#a21caf']):
            pts = np.array([[float(r['x']), float(r['y'])] for r in rows if int(r['i']) == i and float(r['t']) >= 20.])
            ax.plot(pts[:, 0], pts[:, 1], color=color, lw=1.2, label=f'UAV {i+1}')
            ax.scatter(*pts[0], marker='^', color=color); ax.scatter(*pts[-1], marker='o', color=color)
        ax.set(title=name, xlabel='x (m), XY projection', ylabel='y (m)', aspect='equal', xlim=(-6.3, 6.3), ylim=(-6.3, 6.3))
        ax.legend(fontsize=8)
    fig.suptitle('Actual flight paths after common startup | low cabinets can be overflown', fontsize=13)
    fig.savefig(BASE/'trajectories.png', dpi=160); fig.savefig(BASE/'trajectories.svg'); plt.close(fig)
    print(json.dumps({'checks': checks, 'metrics': [{k: r[k] for k in ['system', 'exploration_t95_s', 'mission_total_distance_m', 'low_speed_fraction']} for r in summaries]}, indent=2))


if __name__ == '__main__':
    analyze()
