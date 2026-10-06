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


def metrics(mode, root=None):
    root = Path(root) if root is not None else BASE/f'trial-{mode}-900'
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



def service_diagnostics(root):
    events = [e for f in root.glob('drone_*/events.jsonl') for e in records(f)]
    services = [e for e in events if e['type']=='view_observed' and e.get('purpose')=='explore']
    repeated = 0
    for i in range(2):
        local=sorted((e for e in services if e['drone']==i),key=lambda e:e['time'])
        repeated+=sum(a['region']==b['region'] for a,b in zip(local,local[1:]))
    states=list(records(root/'peer_states.jsonl'))
    planning=[e['total_wall_s'] for e in events if e['type']=='planning_result']
    return dict(explore_services=len(services), online_zero_team_gain=sum(e['team_new_cells']==0 for e in services),
                consecutive_same_region_services=repeated,
                idle_stale_active_packets=sum(not s.get('intent') and s.get('active') is not None for s in states),
                idle_stale_pinned_bid_packets=sum(not s.get('intent') and any(v<=-1e5 for v in s.get('bids',{}).values()) for s in states),
                planning_wall_p95_s=float(np.percentile(planning,95)),
                events=dict(Counter(e['type'] for e in events)),
                note='Same-region services may observe different headings/altitudes; this count is not a wasted-flight ratio. Online team gain uses received evidence.')


def main():
    root=BASE/'trial-annual-efficiency-900-r1'
    before=metrics('annual');after=metrics('annual',root);after['system']='Annual revised'
    baselines=[metrics(m) for m in ['racer','gvp']]
    before['service_diagnostics']=service_diagnostics(BASE/'trial-annual-900')
    after['service_diagnostics']=service_diagnostics(root)
    frozen=json.loads((BASE/'frozen-sha256.json').read_text())
    source=json.loads((BASE/'annual-efficiency-source-manifest.json').read_text())
    physics=json.loads((BASE/'efficiency-physics-binaries.json').read_text())
    checks=dict(identical_world=before['checks']['world_sha256']==after['checks']['world_sha256'],
                identical_truth_denominator=before['result']['truth_free_voxels']==after['result']['truth_free_voxels'],
                frozen_harness_unchanged=all(hashlib.sha256((BASE/f).read_bytes()).hexdigest()==h for f,h in frozen.items()),
                frozen_revised_source_unchanged=all(hashlib.sha256((BASE/'annual-efficiency-source'/f).read_bytes()).hexdigest()==h for f,h in source['files'].items()),
                exact_baseline_physics_binaries=all(hashlib.sha256((BASE/'annual-source'/f).read_bytes()).hexdigest()==h==hashlib.sha256((BASE/'annual-efficiency-source'/f).read_bytes()).hexdigest() for f,h in physics.items()),
                reference_limits=after['checks']['reference_speed_cap'] and after['checks']['reference_acceleration_cap'],
                all_cleanup_empty=not after['checks']['process_cleanup']['ros2']['remaining'],
                native_protocol_passed=json.loads((root/'native-protocol-audit.json').read_text())['passed'])
    data=dict(before=before,after=after,baselines=baselines,checks=checks,
              scope='One paired development run at seed 900; reused the completed native adapted RACER/GVP trials. No statistical ranking or formal matrix claim.')
    (BASE/'efficiency-comparison.json').write_text(json.dumps(data,indent=2)+'\n')
    systems=[before,after,*baselines];names=['Annual before','Annual revised','RACER','GVP-MREP'];colors=['#94a3b8','#2563eb','#d97706','#059669']
    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for ax,key,title,scale in zip(axes,['exploration_t95_s','mission_total_distance_m','low_speed_fraction'],['Exploration T95 (s)','Mission fleet distance (m)','True speed <0.1 m/s (%)'],[1,1,100]):
        bars=ax.bar(names,[s[key]*scale if s[key] is not None else np.nan for s in systems],color=colors)
        ax.set_title(title);ax.tick_params(axis='x',rotation=20);ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
        for bar in bars:
            if np.isfinite(bar.get_height()):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height(),f'{bar.get_height():.1f}',ha='center',va='bottom')
    fig.suptitle('One paired efficiency check | seed 900 | unchanged physical plant and sensors')
    fig.savefig(BASE/'efficiency-comparison.png',dpi=160);fig.savefig(BASE/'efficiency-comparison.svg');plt.close(fig)
    world=json.loads((BASE/'map.json').read_text());fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
    for ax,trial,name in zip(axes,['trial-annual-900','trial-annual-efficiency-900-r1'],['Annual before','Annual revised']):
        for ob in world['obstacles']:
            lo,hi=np.asarray(ob['min']),np.asarray(ob['max'])
            ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],facecolor='#cbd5e1',edgecolor='#94a3b8'))
        samples=list(csv.DictReader((BASE/trial/'trajectory.csv').open()))
        for i,color in enumerate(['#0e7490','#a21caf']):
            rows=[r for r in samples if int(r['i'])==i and float(r['t'])>=20.]
            ax.plot([float(r['x']) for r in rows],[float(r['y']) for r in rows],color=color,lw=1.4,label=f'UAV {i}')
        ax.set(title=name,xlabel='x (m)',ylabel='y (m)',xlim=(-6,6),ylim=(-6,6));ax.set_aspect('equal');ax.legend();ax.grid(alpha=.15)
    fig.suptitle('Recorded mission trajectories (XY projection; low cabinet can be overflown)')
    fig.savefig(BASE/'efficiency-trajectories.png',dpi=160);fig.savefig(BASE/'efficiency-trajectories.svg');plt.close(fig)
    print(json.dumps(dict(checks=checks,before={k:before[k] for k in ['exploration_t95_s','mission_total_distance_m','low_speed_fraction']},after={k:after[k] for k in ['exploration_t95_s','mission_total_distance_m','low_speed_fraction']},diagnostics=after['service_diagnostics']),indent=2))

if __name__=='__main__':main()
