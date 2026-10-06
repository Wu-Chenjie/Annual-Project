#!/usr/bin/env python3
"""One independently replaceable exploration agent. No truth map or fleet planner."""
import copy
from dataclasses import fields
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import json
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import planning_runtime  # installed core module location
from core.exploration.decentralized import MapReplica, PeerLedger, remainder, tracking_recovery
from core.exploration.sparse_graph import SparseTopology
from core.exploration.fusion import FusionPlanner, reusable_trajectory, compute_worker
from core.exploration.service_selection import view_gain_retained, live_execution_bids
from core.exploration.priority import PriorityConfig, service_cells, service_result
from core.exploration.team_evidence import known_mask, service_accounting, regional_evidence, team_new_cells, intent_records
from core.exploration.mrdtg import observation_grid
from core.exploration.planning_budget import PlanningRequest, abort_worker
from core.exploration.pairwise import PairExchange
from core.planning.continuous_trajectory import optimize_trajectory, ContinuousTrajectory
from core.planning.handoff import moving_boundary_time
from core.planning.handoff_timing import HandoffTiming
from core.exploration.regions import RegionTasks, ObservationPlanner, insertion_bids, visible_cells
from core.planning.path_quality import PathQualityEvaluator, RankedPathPool, Candidate, resample


def fit_selection(selection, runtime, position, yaw, rid, now, deadline_wall=None, start_boundary=None):
    begin = time.monotonic()
    selection['pool'].revalidate(position, runtime, PathQualityEvaluator(), runtime.version, now=now)
    if selection['pool'].active is None: raise ValueError('No live safe reserve')
    curves = {}; valid = []
    for candidate in [selection['pool'].active]+selection['pool'].backups:
        try:
            curve = optimize_trajectory(
                candidate.path, runtime, yaw, selection['yaw'], speed_limit=.15 if rid < 0 else .6,
                acceleration_limit=.2 if rid < 0 else .8, deadline_wall=deadline_wall,
                preserve_route=candidate is not selection['pool'].active,
                **(start_boundary or {}))
            points=curve.path(.15)
            distances=(runtime.signed_distances(points) if hasattr(runtime,'signed_distances') else
                       np.array([runtime.signed_distance(p) for p in points]))
            clearance = min(float(np.linalg.norm(runtime.bounds[1]-runtime.bounds[0])),float(distances.min()))
            candidate.quality['executed_curve'] = dict(duration_s=curve.duration, limits=curve.limits(),
                yaw_rate=curve.yaw_rate_limit(), jerk_integral=curve.jerk_cost(), method=curve.method,
                minimum_clearance_m=clearance)
            candidate.quality['curve_quality_score'] = curve.duration+.035*curve.jerk_cost()+.8/max(.25,clearance-runtime.clearance+.25)
            if any(np.mean(np.linalg.norm(resample(curve.path(.15))-resample(other.path(.15)),axis=1)) < selection['pool'].diversity_m
                   for other in curves.values()):continue
            curves[candidate.id] = curve; valid.append(candidate)
            if start_boundary is not None: break
        except ValueError:
            continue
        except TimeoutError:
            selection['curve_budget_exhausted'] = True
            break
    selection['curve_alternatives'] = curves
    selection['curve_qualities'] = {c.id:dict(quality=copy.deepcopy(c.quality),map_version=runtime.version,time=now)
                                   for c in valid}
    if not valid:raise ValueError('No validated continuous reserve within the fit budget')
    valid.sort(key=lambda c:(c.quality['curve_quality_score'],c.id))
    selection['pool'].active,selection['pool'].backups=valid[0],valid[1:selection['pool'].backup_count+1]
    selection['trajectory']=curves[valid[0].id];selection['trajectory_candidate']=valid[0].id
    return dict(selection=selection, wall=time.monotonic()-begin, stages=dict(trajectory=time.monotonic()-begin),
                worker_started_wall=begin,worker_finished_wall=time.monotonic())


class ExplorationAgent(Node):
    def __init__(self):
        super().__init__('exploration_agent')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        bounds = json.loads(self.declare_parameter('bounds', '[]').value)
        self.seeds = json.loads(self.declare_parameter('fleet_starts', '[]').value)
        self.network_isolated = set()
        self.replica = MapReplica(bounds, self.id, volumetric=True, flight_limits=(.7, min(3.1, bounds[1][2]-.3))); self.ledger = PeerLedger(self.id, members=tuple(range(len(self.seeds))))
        self.ledger.seeds = self.seeds
        priority_config = PriorityConfig(**{f.name: self.declare_parameter('exploration_priority.'+f.name, f.default).value
                                           for f in fields(PriorityConfig)})
        self.fusion = FusionPlanner(self.id, bounds, priority_config); self.pair = PairExchange(self.id, self.fusion.graph.replica.session)
        self.bootstrapped = False; self.rejoin_ready = False; self.join_stopped_since = None
        self.last_graph_full = -30.; self.graph_bytes = 0; self.last_pair_commits = 0
        self.position = None; self.yaw = 0.; self.observation = None; self.execution = {}
        self.bids = {}; self.tour = []; self.owners = {}; self.workload = 0.
        self.intent = None; self.selection = None; self.active = None; self.epoch = 0
        self.cached_pending = None; self.retiring = None; self.stop_epoch = None; self.stop_since = None; self.speed = 0.
        self.preplanned = None
        self.preparation_token = None
        self.pending_intent = None; self.pending_selection = None; self.retiring_extra = []
        self.pending_service = None
        self.motion_blocked = False
        self.sequence = 0; self.ready = False; self.available = True; self.done = False
        self.cooldown = {}; self.recent = []; self.graph = None; self.tasks = {}
        self.service_feedback = {}
        self.graph_seq = 0; self.acks = []; self.events = []
        self.changed = False; self.last_plan = 0.; self.planner = ObservationPlanner()
        self.parent_geometry_wall_s=0.;self.parent_geometry_count=0;self.parent_geometry_last_wall_s=0.
        self.parent_proposal_wall_s=0.;self.parent_proposal_count=0
        self.output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value)/f'drone_{self.id}'
        self.output.mkdir(parents=True, exist_ok=True)
        previous_events = self.output/'events.jsonl'
        if previous_events.exists():
            for line in previous_events.read_text().splitlines():
                try:
                    e = json.loads(line)
                    if e.get('type') == 'region_service':
                        self.service_feedback[int(e['region'])] = e['feedback']
                    elif e.get('type') == 'region_low_yield':
                        self.service_feedback[int(e['region'])] = dict(stamp=e['time'], defer_until=e['retry_after'], observed_new_cells=e['new_cells'])
                except (ValueError, KeyError):
                    continue
        self.log = (self.output/'events.jsonl').open('a'); self.paths = (self.output/'candidates.jsonl').open('a')
        self.last_map_dump = -20.
        self.view_start_known = 0
        self.view_start_cells = np.empty(0, dtype=int)
        self.view_target_cells = np.empty(0, dtype=int)
        self.view_team_before = np.zeros(self.replica.map.state.size, bool); self.service_start = None
        self.last_service = None
        self.bytes = 0; self.peer_bytes = 0; self.commits = 0; self.observed_views = 0
        # CPU-bound graph/tour/trajectory work must not hold the ROS callback GIL.
        # Spawn avoids inheriting DDS threads or sockets into the planning worker.
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'))
        self.future = None; self.future_epoch = None; self.last_submit = -10.
        self.request = None; self.request_sequence = 0; self.worker_retry_after = 0.
        self.planning_deadline_s = float(self.declare_parameter('planning_deadline_s', 12.).value)
        self.handoff_timing = HandoffTiming(planning_default_s=self.planning_deadline_s)
        self.future_boundary = None
        self.future_kind = 'plan'; self.fit_pending = None
        self.observation_timeout_s = float(self.declare_parameter('observation_timeout_s',1.5).value)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, f'/drone_{self.id}/peer_state', q)
        self.command_pub = self.create_publisher(String, f'/drone_{self.id}/view_command', q)
        self.graph_pub = self.create_publisher(String, f'/drone_{self.id}/topology', q)
        self.bootstrap_pub = self.create_publisher(String, f'/drone_{self.id}/map_bootstrap', q)
        self.sync_pub = self.create_publisher(String, f'/drone_{self.id}/graph_delta', q)
        self.create_subscription(Odometry, f'/drone_{self.id}/estimated_odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, f'/drone_{self.id}/observation', self.sense, q)
        self.create_subscription(String, f'/drone_{self.id}/view_execution', lambda m: setattr(self, 'execution', json.loads(m.data)), q)
        self.create_subscription(String, '/experiment/control', self.control, q)
        for i in range(len(self.seeds)):
            if i != self.id:
                self.create_subscription(String, f'/drone_{i}/peer_state', self.peer, q)
                self.create_subscription(String, f'/drone_{i}/graph_delta', self.sync, q)
        self.create_timer(.4, self.heartbeat)
        self.create_timer(.5, self.plan)
        self.create_timer(.8, self.publish_sync)
        self.create_timer(.1, self.poll_worker, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def poll_worker(self):
        if self.future is None:
            return
        if self.future.done():
            self.plan()
        else:
            self.expire_worker()

    def expire_worker(self):
        # Reserve 250 ms for the watchdog and process teardown within the total
        # request deadline. Simulation slowdown must not delay this wall clock.
        if time.monotonic()-self.request.submitted_wall < max(.1,self.planning_deadline_s-.25):
            return
        self.event('planning_timeout', request_id=self.request.identifier, deadline_wall_s=self.planning_deadline_s,
                   total_wall_s=time.monotonic()-self.request.submitted_wall,
                   continued_token=(self.intent or {}).get('token'))
        abort_worker(self.worker)
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'))
        self.future = None; self.preplanned = None; self.worker_retry_after = time.monotonic()+1.

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def event(self, kind, **values):
        wall = time.monotonic()
        self.handoff_timing.observe_clock(self.now(), wall)
        if kind in ('planning_result', 'planning_timeout', 'planning_failed') and 'total_wall_s' in values:
            self.handoff_timing.record_planning(values['total_wall_s'])
            if getattr(self,'future_kind',None) in ('fit','moving_fit'):
                self.handoff_timing.record_fitting(values['total_wall_s'])
        if kind in ('path_proposed', 'handoff_proposed'):
            self.handoff_timing.proposed(values['token'], wall)
        elif kind in ('path_committed', 'handoff_quorum_ready'):
            self.handoff_timing.authorized(values['token'], wall)
        elif kind in ('handoff_cancelled', 'reservation_retired'):
            self.handoff_timing.cancelled(values['token'])
        event = dict(type=kind, drone=self.id, time=self.now(), monotonic_wall_time=wall,
                     incarnation=self.fusion.graph.replica.session, **values)
        self.events.append(event); self.log.write(json.dumps(event)+'\n'); self.log.flush()

    def odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.position = np.array([p.x, p.y, p.z])
        v = m.twist.twist.linear; self.speed = float(np.linalg.norm([v.x, v.y, v.z]))
        self.yaw = float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def sense(self, m):
        self.observation = json.loads(m.data)
        self.changed |= self.replica.merge(self.observation)

    def update_local_geometry(self):
        if not self.changed:return
        begin=time.monotonic();self.replica.map.rebuild();self.changed=False
        self.parent_geometry_last_wall_s=time.monotonic()-begin
        self.parent_geometry_wall_s+=self.parent_geometry_last_wall_s;self.parent_geometry_count+=1

    def peer(self, m):
        packet = json.loads(m.data)
        if self.id in self.network_isolated or packet['drone'] in self.network_isolated:
            return
        if self.ledger.receive(packet):
            source = packet['drone']; session = packet.get('session')
            current = self.fusion.graph.replica.sessions.get(source)
            if session and current and current != session:
                self.fusion.graph.replica.rejoin(source, session)
                transaction = self.pair.transaction
                if transaction and source in (transaction['leader'], transaction['follower']):
                    self.pair.transaction = None
                    self.event('pair_rejoin_released', peer=source, token=transaction['token'])
                self.event('peer_rejoined', peer=source, session=session)

    def sync(self, m):
        try:
            packet = json.loads(m.data)
            if self.id in self.network_isolated or packet['source'] in self.network_isolated:
                return
            source = packet['source']
            accepted = self.ledger.states.get(source, {}).get('session')
            if accepted and accepted != packet['session']:
                return
            changed = self.fusion.graph.replica.merge(packet)
            if changed:
                self.fusion.graph.rebuild()
        except (ValueError, KeyError, TypeError) as e:
            self.event('graph_packet_rejected', error=str(e))

    def publish_sync(self):
        replica = self.fusion.graph.replica
        peers = list(self.ledger.states.values())
        after = min((int(p.get('graph_received', {}).get(str(self.id), 0)) for p in peers), default=0)
        full = self.now()-self.last_graph_full >= 20. or any(self.id in p.get('graph_missing', []) for p in peers)
        if replica.sequence <= after and not full:
            return
        packet = replica.packet(after, full)
        self.graph_bytes += len(json.dumps(packet, separators=(',', ':')).encode())
        self.publish(self.sync_pub, packet)
        if full:
            self.last_graph_full = self.now()

    def control(self, m):
        control = json.loads(m.data); isolated = set(control.get('isolated', []))
        if isolated != self.network_isolated:
            self.event('network_partition' if isolated else 'network_recovered', isolated=sorted(isolated))
        self.network_isolated = isolated
        if (self.id in control.get('restart', []) and self.bootstrapped
                and control.get('restart_session') == self.fusion.graph.replica.session):
            self.event('process_restart_requested'); self.log.flush(); self.paths.flush(); abort_worker(self.worker); os._exit(75)
        available = self.id not in control.get('paused', [])
        self.done = control.get('done', False)
        if available != self.available:
            self.event('availability_resume' if available else 'availability_pause', region=self.active)
        if (not available or self.done) and (self.intent or self.active is not None):
            self.withdraw('experiment_pause' if not available else 'experiment_complete')
        self.available = available

    def publish(self, pub, packet):
        data = json.dumps(packet, separators=(',', ':')); self.bytes += len(data.encode())
        if pub in (self.state_pub, self.sync_pub):
            self.peer_bytes += len(data.encode())
        pub.publish(String(data=data))

    def withdraw(self, reason):
        previous = self.active
        self.epoch += 1
        if self.intent is not None:
            self.retiring = copy.deepcopy(self.intent); self.retiring['committed'] = True; self.retiring['retiring'] = True
        if self.pending_intent:
            if self.retiring: self.retiring_extra.append(dict(self.pending_intent, retiring=True))
            else: self.retiring = dict(self.pending_intent, retiring=True)
        self.pending_intent = None; self.pending_selection = None
        self.pending_service = None
        self.stop_epoch = self.epoch; self.stop_since = None
        self.publish(self.command_pub, dict(epoch=self.epoch, cancel=True, reason=reason))
        self.intent = None; self.selection = None; self.active = None
        self.preplanned = None
        self.fit_pending = None
        self.event('lease_cancellation_requested', region=previous, reason=reason)

    def state(self):
        intent = copy.deepcopy(self.intent or self.retiring)
        if intent:
            intent.pop('trajectory', None)
        if intent and intent.get('committed'):
            intent['path'] = remainder(self.position, intent['path']).tolist()
        pending = copy.deepcopy(self.pending_intent)
        if pending: pending.pop('trajectory', None)
        # Leases can retire after a worker result was accepted. Recheck at
        # publication so its old execution preference cannot outlive the lease.
        bids = live_execution_bids(self.bids,
            [self.intent,self.pending_intent,self.retiring]+self.retiring_extra)
        if pending and pending.get('committed'): bids[pending['region']] = -1e6
        return dict(drone=self.id, session=self.fusion.graph.replica.session, epoch=self.epoch,
                    rejoin_ready=self.rejoin_ready, stopped=self.speed < .08 and self.execution.get('arrived', False),
                    peer_sessions={i: p.get('session') for i, p in self.ledger.states.items()},
                    sequence=self.sequence, time=self.now(), position=self.position.tolist(), yaw=self.yaw,
                    ready=self.ready, available=self.available and self.rejoin_ready and not self.done and not self.motion_blocked,
                    motion_blocked=self.motion_blocked, bids=bids if self.available and not self.motion_blocked else {},
                    tour=self.tour, workload=self.workload, owners=self.owners, active=self.active,
                    intent=intent, pending_intent=pending, retiring_intents=self.retiring_extra,
                    execution=self.execution, last_service=self.last_service, acks=self.acks, observation=None,
                    fusion=dict(self.fusion.diagnostics, map_clearance=self.replica.map.signed_distance(self.position),
                                worker_running=self.future is not None and not self.future.done(), fit_queued=self.fit_pending is not None,
                                parent_geometry_wall_s=self.parent_geometry_wall_s,parent_geometry_count=self.parent_geometry_count,
                                parent_geometry_last_wall_s=self.parent_geometry_last_wall_s,
                                parent_proposal_wall_s=self.parent_proposal_wall_s,parent_proposal_count=self.parent_proposal_count), graph_connections=self.fusion.connections,
                    graph_received=self.fusion.graph.replica.received, graph_missing=sorted(self.fusion.graph.replica.needs_snapshot),
                    graph_payload_bytes=self.graph_bytes, pair_transaction=self.pair.transaction, pair_commits=self.pair.commits,
                    graph=dict(nodes=len(self.graph.nodes),
                    edges=len(self.graph.edges), free_cells=self.graph.free_cells) if self.graph else {},
                    tasks=[], map_version=self.replica.map.version,
                    payload_bytes=self.bytes, peer_payload_bytes=self.peer_bytes, commits=self.commits,
                    observed_views=self.observed_views, planner_wall_s=self.last_plan)

    def heartbeat(self):
        if self.position is None:
            return
        t = self.now(); self.ready = self.execution.get('ready', False)
        self.handoff_timing.observe_clock(t, time.monotonic())
        if t-self.last_map_dump >= 20.:
            np.savez_compressed(self.output/'map_latest.npz', state=self.replica.map.state,
                bounds=self.replica.map.bounds, resolution=self.replica.map.resolution, position=self.position, yaw=self.yaw)
            self.last_map_dump = t
        if not self.bootstrapped and 'epoch' in self.execution:
            self.epoch = max(self.epoch, int(self.execution['epoch']), int(t*1e6))+1
            self.publish(self.command_pub, dict(epoch=self.epoch, cancel=True, scan=True, reason='incarnation_start'))
            self.bootstrap_pub.publish(String(data=json.dumps(dict(session=self.fusion.graph.replica.session))))
            self.bootstrapped = True
        if self.bootstrapped and not self.rejoin_ready:
            stopped = self.execution.get('epoch') == self.epoch and self.execution.get('arrived') and self.speed < .08
            if stopped:
                if self.join_stopped_since is None:self.join_stopped_since = t
                if t-self.join_stopped_since >= .5:
                    self.rejoin_ready = True
                    self.event('incarnation_ready', epoch=self.epoch)
            else:self.join_stopped_since = None
        if self.retiring:
            stopped = self.execution.get('epoch') == self.stop_epoch and self.execution.get('arrived') and self.speed < .08
            if stopped:
                if self.stop_since is None:
                    self.stop_since = t
                if t-self.stop_since >= .5:
                    self.event('reservation_retired', token=self.retiring['token'], stop_epoch=self.stop_epoch)
                    self.retiring = None
                    self.retiring_extra = []
            else:
                self.stop_since = None
        if self.intent and not self.intent.get('committed') and self.ledger.loses(self.intent):
            self.withdraw('peer_priority')
        self.manage_pending(t)
        self.acks = self.ledger.acknowledge(self.state(), t)
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t, self.intent.get('voters')):
            # Sensing can invalidate an intent while its ACKs are in flight.
            self.update_local_geometry()
            runtime = self.reserved_map()
            if self.intent.get('recovery'):
                runtime.clearance = .5
                runtime.recovery_yaw = self.intent['yaw']
            if not runtime.safe_path(self.intent['path']):
                self.withdraw('intent_invalidated_before_commit')
            elif self.intent.get('purpose','explore')=='explore' and not team_new_cells(
                    visible_cells(runtime,self.intent['path'][-1],self.intent['yaw']),
                    known_mask(runtime,self.fusion.graph.observed_mask(runtime))):
                self.withdraw('latest_team_gain_empty_before_commit')
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t, self.intent.get('voters')):
            self.intent['committed'] = True; self.intent['committed_at'] = t; self.commits += 1
            self.begin_service()
            self.publish(self.command_pub, dict(epoch=self.epoch, path=self.intent['path'], yaw=self.intent['yaw'],
                                               region=self.active, token=self.intent['token'], candidate_id=self.intent['candidate_id'],
                                               trajectory=self.intent.get('trajectory'), observation_grid=self.intent.get('observation_grid')))
            self.event('path_committed', token=self.intent['token'], region=self.active, quorum=self.intent['voters'],
                       predicted_frontier_volume_m3=self.selection['gain'], lookahead=self.selection['lookahead'],
                       exploration_priority=self.selection.get('priority'))
            if self.selection.get('switch_origin'):
                origin=self.selection['switch_origin']
                self.event('cached_route_switched' if origin['candidate']!=self.selection['pool'].active.id else 'cached_route_refitted',
                           region=self.active, token=self.intent['token'],
                           candidate=self.selection['pool'].active.id, origin=self.selection['switch_origin'],
                           actual_start=self.position.tolist(), actual_speed=self.speed, map_version=self.replica.map.version,
                           team_gain_estimate=self.selection['gain'])
        if self.intent and self.intent.get('committed') and self.execution.get('arrived') and self.execution.get('epoch') == self.intent.get('epoch', self.epoch):
            self.event('terminal_observation_completed', token=self.intent['token'], purpose=self.intent.get('purpose','explore'),
                       proof=self.execution.get('observation_proof'))
            self.cancel_pending('current_view_finished')
            self.finish_service('terminal_sensor_dwell')
            self.intent = None; self.selection = None
        if self.intent and not self.intent.get('committed') and t-self.intent['created'] > 6:
            rid = self.active; self.withdraw('reservation_timeout'); self.cooldown[rid] = t+4
        self.sequence += 1
        state = self.state(); owners = self.ledger.owners(state, t)
        for rid, owner in owners.items():
            old = self.owners.get(rid)
            if owner == self.id and old is not None and old != owner and not self.ledger.states.get(old, {}).get('available', True):
                self.event('region_reassigned', region=rid, previous_owner=old, new_owner=owner, reason='peer_unavailable')
        self.owners = owners
        pinned = {int(intent['region']): i for i, p in self.ledger.states.items() for intent in intent_records(p)}
        if self.active is not None:
            pinned[self.active] = self.id
        if self.pending_intent: pinned[self.pending_intent['region']] = self.id
        self.pair.tick(self.ledger.states, owners, t, pinned)
        if self.pair.commits != self.last_pair_commits:
            self.event('pair_cvrp_committed', count=self.pair.commits, assignments=self.pair.overrides)
            self.last_pair_commits = self.pair.commits
        self.publish(self.state_pub, self.state())

    def reserved_map(self):
        runtime = copy.deepcopy(self.replica.map)
        paths = []
        for p in self.ledger.states.values():
            paths.extend([i['path'] for i in intent_records(p)] or [[p['position']]])
        paths.extend(g['path'] for g in self.ledger.grants.values())
        runtime.block_paths(paths, radius=1.25)
        return runtime

    def plan(self):
        if self.position is None or not self.ready or not self.rejoin_ready or not self.available or self.done:
            return
        t = self.now()
        if self.observation is None or not -.1 <= t-self.observation.get('time',-np.inf) <= self.observation_timeout_s:
            if self.intent or self.pending_intent:self.withdraw('observation_timeout')
            self.set_motion_blocked(True,reason='observation_timeout')
            return
        if self.retiring:
            return
        if not self.ledger.fresh(t) and len(self.ledger.states) != len(self.seeds)-1:
            return
        if not self.ledger.fresh(t) and self.intent and not self.intent.get('committed'):
            self.withdraw('proposal_voter_timeout'); return
        self.update_local_geometry()
        if self.intent and self.intent.get('committed'):
            path = remainder(self.position, self.intent['path'])
            recovery = copy.copy(self.replica.map); recovery.clearance = .5
            if self.intent.get('recovery'):
                recovery.recovery_yaw = self.intent['yaw']
            tracking_ok = (len(path) > 1 and np.linalg.norm(path[0]-path[1]) < .3 and recovery.safe_path(path[:2]))
            valid = recovery.safe_path(path) if self.intent.get('recovery') else self.replica.map.safe_path(path[1:]) and tracking_ok
            if not valid:
                rid = self.active; selection = self.selection
                origin = dict(token=self.intent['token'], candidate=self.intent['candidate_id'], time=t, map_version=self.replica.map.version)
                self.withdraw('path_or_tracking_invalidated')
                runtime = self.reserved_map()
                selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version, now=t)
                selection['switch_origin'] = origin
                if selection['pool'].active and rid in self.tasks:
                    self.cached_pending = (rid, selection)
                return
        if self.cached_pending and not self.intent:
            rid, selection = self.cached_pending; self.cached_pending = None
            runtime = self.reserved_map()
            selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version, now=t)
            if selection['pool'].active and self.owners.get(rid) == self.id and self.ledger.can_propose(selection['pool'].active.path, rid, t):
                selection['trajectory'] = None
                self.propose(rid, selection)
        if self.preplanned and not self.intent:
            self.adopt_preplan(t)
        elif self.preplanned and self.intent and self.intent.get('committed'):
            self.adopt_moving_preplan(t)
        if not self.intent and not self.replica.map.safe_path([self.position]):
            runtime = self.reserved_map(); path = tracking_recovery(runtime, self.position, yaw=self.yaw)
            rid = -100-self.id
            if path is not None and self.ledger.can_propose(path, rid, t):
                recovery = copy.copy(runtime); recovery.clearance = .5; recovery.recovery_yaw = self.yaw
                pool = RankedPathPool(); pool.rank([Candidate(f'recovery:{self.epoch+1}', 'tracking_recovery', 0,
                    path, PathQualityEvaluator().evaluate(path, recovery), runtime.version)])
                self.propose(rid, dict(pool=pool, yaw=self.yaw, gain=0., objective=0., lookahead=None))
                if self.intent is not None:
                    self.intent['recovery'] = True
                    self.event('tracking_recovery', goal=path[-1].tolist())
                self.set_motion_blocked(False)
                if self.intent: return
            # A conservative map may offer no certified retreat. Keep sensing,
            # graph replication and task redistribution alive while holding;
            # do not erase unknown cells or retain bids for work we cannot do.
            if path is None: self.set_motion_blocked(True)
        else:
            self.set_motion_blocked(False)
        if self.future is not None:
            if not self.future.done():
                self.expire_worker()
                return
            try:
                result = self.future.result()
            except Exception as e:
                self.future = None; self.event('planning_failed', error=str(e), request_id=self.request.identifier,
                                               total_wall_s=time.monotonic()-self.request.submitted_wall)
                abort_worker(self.worker); self.worker = ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'))
                self.worker_retry_after = time.monotonic()+1.; return
            self.future = None
            rejection = self.request.rejection(self.fusion.graph.replica.session, self.epoch, t)
            self.event('planning_result', request_id=self.request.identifier, compute_wall_s=result['wall'],
                       total_wall_s=time.monotonic()-self.request.submitted_wall,
                       dispatch_queue_wall_s=result.get('worker_started_wall',self.request.submitted_wall)-self.request.submitted_wall,
                       result_delivery_wall_s=time.monotonic()-result.get('worker_finished_wall',time.monotonic()),
                       result_age_sim_s=t-self.request.submitted_sim, phase=self.future_kind,
                       stage_wall_s=result.get('stages', {}) if self.future_kind in ('fit','moving_fit') else result['fusion'].diagnostics.get('stage_wall_s'),
                       rejection=rejection, input_map_version=self.request.map_version, current_map_version=self.replica.map.version)
            if rejection:
                return
            if self.future_kind == 'moving_fit':
                self.preplanned = dict(epoch=self.epoch, position=self.future_boundary['position'], region=self.fit_region,
                    selection=result['selection'], time=self.request.submitted_sim, anticipatory=True,
                    boundary=self.future_boundary, request_id=self.request.identifier)
                if self.intent is not None:
                    self.adopt_moving_preplan(t)
                else:
                    self.adopt_preplan(t)
                return
            if self.future_kind == 'fit':
                rid = self.fit_region
                if not self.intent and not self.retiring and (rid < 0 or self.owners.get(rid)==self.id) and self.ledger.can_propose(result['selection']['pool'].active.path, rid, t):
                    self.propose(rid, result['selection'])
                return
            # Preserve packets received while this private worker snapshot ran.
            remote = self.fusion.graph.replica
            self.fusion = result['fusion']
            self.fusion.graph.replica.remote = remote.remote
            self.fusion.graph.replica.received = remote.received
            self.fusion.graph.replica.sessions = remote.sessions
            self.fusion.graph.replica.needs_snapshot = remote.needs_snapshot
            self.fusion.graph.replica.applied = remote.applied
            self.fusion.graph.rebuild()
            offer = result.get('offer')
            if offer and not self.pair.transaction and not self.motion_blocked:
                base = {r: self.owners.get(r) for r in offer['owners']}
                if all(owner in (self.id, offer['other']) for owner in base.values()):
                    if self.pair.offer(offer['other'], offer['result'], base, t):
                        self.event('pair_cvrp_prepared', other=offer['other'], before=offer['result']['before'], after=offer['result']['after'],
                                   loads=offer['result']['loads'], capacity=offer['result']['capacity'], fixed_loads=offer['result'].get('fixed_loads'))
            self.last_plan = result['wall']; self.graph = result['graph']; self.tasks = result['tasks']
            self.bids, self.tour, self.workload = result['bids'], result['tour'], result['workload']
            self.bids = live_execution_bids(self.bids,
                [self.intent,self.pending_intent,self.retiring]+self.retiring_extra)
            self.graph_seq += 1; snapshot = self.graph.snapshot(); snapshot['sequence'] = self.graph_seq
            snapshot['exploration_priority'] = self.fusion.priority_layer
            self.publish(self.graph_pub, snapshot)
            if self.active not in self.tasks and not self.intent:
                self.active = None
            if self.future_epoch == self.epoch and not self.motion_blocked:
                if not self.intent:
                    for rid in result['rejected']:
                        self.cooldown[rid] = t+6
                        if rid == self.active:
                            self.event('region_yielded', region=rid); self.active = None
                selection = result['selection']; rid = result['selected']
                if selection:
                    self.preplanned = dict(epoch=self.epoch, position=result['position'], region=rid,
                                           selection=selection, time=self.request.submitted_sim, anticipatory=bool(self.intent),
                                           boundary=self.future_boundary, request_id=self.request.identifier)
                    if self.intent:
                        self.event('next_view_ready', region=rid, epoch=self.epoch, compute_wall_s=result['wall'])
                    else:
                        self.adopt_preplan(t)
        if self.fit_pending and self.future is None and not self.intent:
            rid, selection = self.fit_pending; self.fit_pending = None
            runtime = self.reserved_map()
            if rid < 0: runtime.clearance = .5; runtime.recovery_yaw = self.yaw
            self.request_sequence += 1; self.future_epoch = self.epoch; self.future_kind = 'fit'; self.fit_region = rid
            self.request = PlanningRequest(f'{self.id}:fit:{self.request_sequence}', self.fusion.graph.replica.session,
                                          self.epoch, runtime.version, t, time.monotonic(), self.planning_deadline_s)
            self.event('planning_submitted', request_id=self.request.identifier, map_version=runtime.version, epoch=self.epoch, phase='fit')
            self.future = self.worker.submit(fit_selection, selection, runtime, self.position.copy(), self.yaw, rid, t,
                                             deadline_wall=self.request.submitted_wall+self.planning_deadline_s-1.)
            return
        fresh_segment = (self.intent and self.intent.get('committed') and
                         self.execution.get('epoch') == self.intent.get('epoch') and
                         self.preparation_token != self.intent['token'])
        if not fresh_segment and t-self.last_submit < (10. if self.motion_blocked else 2. if self.intent else .25):
            return
        if time.monotonic() < self.worker_retry_after or self.pending_intent:
            return
        if self.intent and (not self.intent.get('committed') or self.intent.get('recovery')):
            return
        # Retain a ready next view rather than continuously refitting it. Refresh
        # periodically as sensing changes; final adoption always checks live data.
        if self.preplanned and t-self.preplanned['time'] < 5.:
            return
        owned = [r for r, owner in self.owners.items() if owner == self.id]
        reservations = [path for p in self.ledger.states.values() for path in
                        ([i['path'] for i in intent_records(p)] or [[p['position']]])]
        reservations.extend(g['path'] for g in self.ledger.grants.values())
        recent = [(p, h) for stamp, p, h in self.recent if t-stamp < 60.]
        active = self.active
        anticipated = dict(position=self.intent['path'][-1], yaw=self.intent['yaw']) if self.intent else None
        self.future_boundary = None
        if self.intent and self.execution.get('epoch') == self.intent.get('epoch'):
            curve = ContinuousTrajectory.from_dict(self.intent['trajectory'])
            progress = self.execution.get('trajectory_time', 0.)
            timing = self.handoff_timing.estimate()
            lead = timing['lead_sim_s']
            handoff_time = moving_boundary_time(curve,progress,lead)
            self.event('handoff_preparation_timing', old_token=self.intent['token'], trajectory_progress=progress,
                       selected_boundary=handoff_time, ready_window_available=handoff_time is not None, **timing)
            if handoff_time is not None:
                p, v, a, h, rate = curve.sample(handoff_time)
                anticipated = dict(position=p.tolist(), yaw=h, start_velocity=v.tolist(), start_acceleration=a.tolist(),
                                   start_yaw_rate=rate, start_yaw_acceleration=curve.yaw_acceleration(handoff_time))
                self.future_boundary = dict(from_epoch=self.intent['epoch'], from_token=self.intent['token'],
                                            trajectory_time=handoff_time, **anticipated)
        self.last_submit = t
        if fresh_segment: self.preparation_token = self.intent['token']
        self.future_epoch = self.epoch
        self.future_kind = 'plan'
        self.request_sequence += 1
        self.request = PlanningRequest(f'{self.id}:{self.request_sequence}', self.fusion.graph.replica.session, self.epoch,
                                       self.replica.map.version, t, time.monotonic(), self.planning_deadline_s)
        self.event('planning_submitted', request_id=self.request.identifier, map_version=self.request.map_version, epoch=self.epoch)
        planner = copy.deepcopy(self.fusion)
        self.future = self.worker.submit(compute_worker, planner, copy.deepcopy(self.replica.map), self.position.copy(), self.yaw,
            active, recent, dict(self.cooldown), reservations, self.epoch, t, True,
            copy.deepcopy(self.ledger.states), dict(self.pair.overrides), dict(self.pair.last_success),
            copy.deepcopy(self.service_feedback), anticipated, not self.motion_blocked,
            own_state=self.state(),
            deadline_wall=self.request.submitted_wall+self.planning_deadline_s-1.)

    def set_motion_blocked(self, blocked, reason=None):
        if blocked == self.motion_blocked:
            return
        self.motion_blocked = blocked
        if blocked:
            self.active = None; self.preplanned = None
        self.event('motion_suspended' if blocked else 'motion_resumed',
                   reason=reason or ('no_certified_recovery' if blocked else 'certified_motion_available'),
                   position=self.position.tolist(), map_clearance=self.replica.map.signed_distance(self.position))

    def finish_service(self, reason, end_time=None, observed_pose=None):
        t = self.now() if end_time is None else end_time
        position,yaw=(self.position.copy(),self.yaw) if observed_pose is None else observed_pose
        peer_end = self.fusion.graph.observed_mask(self.replica.map, exclude_sources=(self.id,))
        clock_fields=self.service_clock_fields(t)
        accounting = service_accounting(self.replica.map, self.view_start_cells, self.view_team_before,peer_end,**clock_fields)
        self.last_service = dict(accounting,
                                 region=self.active, purpose=self.intent.get('purpose','explore'), start=self.service_start, end=t)
        self.observed_views += 1
        new_cells = max(0, int(np.count_nonzero(self.replica.map.state != -1))-self.view_start_known)
        if clock_fields:
            times=self.replica.first_observed;new_cells=int(np.count_nonzero((times>self.service_start)&(times<=t)))
        self.recent.append((t+(240. if new_cells < 3 else 0.), np.asarray(position).copy(), yaw))
        self.event('view_observed', completion_reason=reason, region=self.active, epoch=self.intent.get('epoch',self.epoch), token=self.intent['token'], new_cells=new_cells,
                   observed_volume=new_cells*self.replica.map.resolution**self.replica.map.state.ndim,
                   purpose=self.intent.get('purpose', 'explore'), service_start=self.service_start,service_end=t,
                   service_pose_position=np.asarray(position).tolist(),service_pose_yaw=yaw,
                   service_pose_source='consumed_curve_boundary' if observed_pose is not None else 'estimated_pose_at_feedback',
                   **accounting)
        if self.active is not None and self.active >= 0 and not self.selection.get('transit'):
            gained = accounting['team_new_cells']
            previous = max((self.service_feedback.get(self.active, {}), self.fusion.graph.services.get(self.active, {})),
                           key=lambda v: v.get('stamp', -1.))
            expected_team = int(np.count_nonzero(~self.view_team_before[self.view_start_cells]))
            feedback = service_result(previous, t, gained, expected_team, self.fusion.priority.config)
            evidence = regional_evidence(self.replica.map, self.tasks[self.active].bounds,
                                        self.fusion.graph.observed_mask(self.replica.map)) if self.active in self.tasks else {}
            feedback.update(accounting, expected_team_cells=expected_team, purpose='explore',
                            evidence_signature=evidence.get('signature'), service_start=self.service_start,
                            failed_view=dict(position=np.asarray(position).tolist(),yaw=yaw) if feedback['low_yield_streak'] else {})
            self.service_feedback[self.active] = feedback
            self.cooldown[self.active] = feedback['defer_until']
            if feedback['low_yield_streak']:
                self.event('region_low_yield', region=self.active, new_cells=gained, retry_after=feedback['defer_until'])
            # Write last so restart replay restores streaks and successful resets.
            self.event('region_service', region=self.active, feedback=feedback)
        # Execution pins a region only while its service is live. A successful
        # terminal view must release it too; otherwise the next graph/tour sees
        # a fictitious active service and a permanently preferred bid.
        if self.bids.get(self.active, 0.) <= -1e5:
            self.bids.pop(self.active)
        self.active = None

    def service_clock_fields(self,end):
        return dict(first_observed=self.replica.first_observed,start=self.service_start,end=end) if self.observation and self.observation.get('sensor')=='gazebo_gpu_lidar' else {}

    def begin_service(self, snapshot=None, start_time=None):
        self.view_start_known = int(np.count_nonzero(self.replica.map.state != -1))
        self.view_start_cells = service_cells(self.replica.map, self.tasks.get(self.active), self.intent['path'][-1], self.intent['yaw'])
        self.view_team_before = known_mask(self.replica.map, self.fusion.graph.observed_mask(self.replica.map))
        self.view_target_cells=np.array(sorted(team_new_cells(
            visible_cells(self.replica.map,self.intent['path'][-1],self.intent['yaw']),self.view_team_before)),int)
        self.service_start = self.now()
        if snapshot:
            self.view_start_cells=snapshot['cells'];self.view_team_before=snapshot['team_before'];self.view_start_known=snapshot['known']
        if start_time is not None:self.service_start=start_time

    def cancel_pending(self, reason):
        if self.pending_intent:
            self.publish(self.command_pub, dict(epoch=self.epoch, cancel_pending=self.pending_intent['token']))
            self.event('handoff_cancelled', token=self.pending_intent['token'], reason=reason)
        self.pending_intent = None; self.pending_selection = None
        self.pending_service = None

    def manage_pending(self, t):
        intent = self.pending_intent
        if not intent: return
        if (self.execution.get('epoch') == intent['epoch'] and
                self.execution.get('handoff_from_token') == intent['handoff']['from_token']):
            old = self.intent
            handoff_time=self.execution.get('handoff_time') or t
            boundary=intent['handoff']
            self.finish_service('moving_sensor_service_complete',handoff_time,(boundary['position'],boundary['yaw']))
            self.intent = intent; self.selection = self.pending_selection; self.active = intent['region']
            self.pending_intent = None; self.pending_selection = None; self.begin_service(self.pending_service,handoff_time);self.pending_service=None
            self.event('handoff_consumed', old_token=old['token'], token=intent['token'],
                       continuity=self.execution.get('handoff_continuity'), speed=self.speed)
            self.event('reservation_retired', token=old['token'], reason='executed_prefix_consumed')
            return
        boundary = intent['handoff']
        if not self.intent or self.execution.get('epoch') != boundary['from_epoch']:
            self.cancel_pending('execution_epoch_changed'); return
        progress = self.execution.get('trajectory_time', 0.)
        if intent.get('committed') and progress > boundary['trajectory_time']+.25:
            self.cancel_pending('execution_missed_boundary'); return
        if not intent.get('committed'):
            if progress >= boundary['trajectory_time']-.15 or t-intent['created'] > 6. or self.ledger.loses(intent):
                reason = ('old_service_insufficient_at_boundary' if not self.old_service_ready() else
                          'late_or_conflicting_authorization')
                self.cancel_pending(reason); return
            lifecycle=self.task_lifecycle_reason(intent['region'],self.pending_selection,t)
            if lifecycle:
                self.cancel_pending('latest_task_lifecycle:'+lifecycle);return
            if not self.ledger.quorum(intent['token'], t, intent['voters']):return
            if 'quorum_ready_at' not in intent:
                intent['quorum_ready_at'] = t
                self.event('handoff_quorum_ready', token=intent['token'], old_token=self.intent['token'])
            # Reserve in advance; sending an execution command still requires
            # real old-service observations. Peer promises cannot satisfy this.
            if not self.old_service_ready(): return
            self.update_local_geometry()
            runtime = self.reserved_map()
            if not runtime.safe_path(intent['path']):
                self.cancel_pending('latest_geometry_invalid'); return
            arrival=t+max(0.,boundary['trajectory_time']-progress)+intent['duration']
            cells = team_new_cells(visible_cells(runtime, intent['path'][-1], intent['yaw'])-
                                   self.fusion.priority.committed_cells(runtime,self.ledger.states,t,arrival),
                                   known_mask(runtime, self.fusion.graph.observed_mask(runtime)))
            if intent['purpose'] == 'explore' and not view_gain_retained(
                    len(cells), self.pending_selection.get('planned_team_cells',0)):
                self.cancel_pending('latest_team_gain_collapsed'); return
            intent['committed'] = True; intent['committed_at'] = t; self.commits += 1
            self.pending_service=dict(cells=service_cells(self.replica.map,self.tasks.get(intent['region']),intent['path'][-1],intent['yaw']),
                team_before=known_mask(self.replica.map,self.fusion.graph.observed_mask(self.replica.map)),
                known=int(np.count_nonzero(self.replica.map.state!=-1)))
            self.publish(self.command_pub, dict(epoch=intent['epoch'], path=intent['path'], yaw=intent['yaw'],
                region=intent['region'], token=intent['token'], candidate_id=intent['candidate_id'], trajectory=intent['trajectory'], handoff=boundary,
                observation_grid=intent['observation_grid']))
            self.event('handoff_authorized', token=intent['token'], old_token=self.intent['token'],
                       trajectory_time=boundary['trajectory_time'], quorum=intent['voters'],
                       quorum_ready_at=intent['quorum_ready_at'],
                       sensor_gate_wait_sim_s=t-intent['quorum_ready_at'],
                       old_service_target_cells=self.view_target_cells.tolist(),
                       old_service_observed_cells=int(np.count_nonzero(
                           known_mask(self.replica.map,self.fusion.graph.observed_mask(self.replica.map))[self.view_target_cells])),
                       old_service_required_fraction=.6 if self.intent.get('purpose')=='explore' else 0.,
                       old_service_purpose=self.intent.get('purpose'),
                       old_service_evidence='private_observations_and_received_actual_receipts')

    def old_service_ready(self):
        if self.intent.get('purpose') != 'explore': return True
        actual_known = known_mask(self.replica.map, self.fusion.graph.observed_mask(self.replica.map))
        delivered = int(np.count_nonzero(actual_known[self.view_target_cells]))
        return delivered >= .6*len(self.view_target_cells)

    def refit_moving_preplan(self, t):
        pending=self.preplanned
        if self.future is not None or not self.intent.get('committed') or self.intent.get('recovery'):return
        if pending['epoch'] != self.epoch or t-pending['time']>15.:
            self.preplanned=None;return
        rid=pending['region'];selection=pending['selection']
        if self.task_lifecycle_reason(rid,selection,t):self.preplanned=None;return
        if self.execution.get('epoch')!=self.intent['epoch']:return
        old=ContinuousTrajectory.from_dict(self.intent['trajectory'])
        progress=self.execution.get('trajectory_time',0.);timing=self.handoff_timing.estimate(fitting=True)
        when=moving_boundary_time(old,progress,timing['lead_sim_s'])
        if pending.get('refit_window_logged') != (self.intent['token'],when is not None):
            self.event('handoff_refit_timing',old_token=self.intent['token'],trajectory_progress=progress,
                       selected_boundary=when,ready_window_available=when is not None,**timing)
            pending['refit_window_logged']=(self.intent['token'],when is not None)
        if when is None:return
        p,v,a,h,rate=old.sample(when)
        self.future_boundary=dict(from_epoch=self.intent['epoch'],from_token=self.intent['token'],trajectory_time=when,
                                  position=p.tolist(),yaw=h)
        boundary=dict(start_velocity=v.tolist(),start_acceleration=a.tolist(),start_yaw_rate=rate,
                      start_yaw_acceleration=old.yaw_acceleration(when))
        runtime=self.reserved_map();self.request_sequence+=1;self.future_kind='moving_fit';self.fit_region=rid
        self.future_epoch=self.epoch
        self.request=PlanningRequest(f'{self.id}:moving_fit:{self.request_sequence}',self.fusion.graph.replica.session,
            self.epoch,runtime.version,t,time.monotonic(),self.planning_deadline_s)
        self.event('planning_submitted',request_id=self.request.identifier,map_version=runtime.version,epoch=self.epoch,phase='moving_fit')
        self.future=self.worker.submit(fit_selection,copy.deepcopy(selection),runtime,p,h,rid,t,
            deadline_wall=self.request.submitted_wall+self.planning_deadline_s-1.,start_boundary=boundary)
        self.preplanned=None;self.last_submit=t

    def adopt_moving_preplan(self, t):
        pending = self.preplanned; boundary = pending.get('boundary')
        if self.pending_intent or self.intent.get('recovery'): return
        if not boundary:
            self.refit_moving_preplan(t);return
        if (pending['epoch'] != self.epoch or t-pending['time'] > 15. or
                boundary['from_token'] != self.intent['token'] or
                self.execution.get('trajectory_time', 0.) >= boundary['trajectory_time']-.8):
            self.preplanned = None; return
        if boundary['trajectory_time']-self.execution.get('trajectory_time', 0.) > 5.5: return
        rid = pending['region']; selection = pending['selection']
        if self.owners.get(rid) != self.id: return
        if self.task_lifecycle_reason(rid,selection,t):self.preplanned=None;return
        # Only a proposal is prepared here; real old-service evidence is fenced
        # again immediately before authorization in manage_pending.
        actual_known=known_mask(self.replica.map,self.fusion.graph.observed_mask(self.replica.map))
        curve = selection.get('trajectory')
        if curve is None: self.preplanned = None; return
        runtime = self.reserved_map()
        if not runtime.safe_path(curve.path()) or not self.ledger.can_propose(curve.path(.15), rid, t): return
        if not selection.get('transit'):
            arrival=t+max(0.,boundary['trajectory_time']-self.execution.get('trajectory_time',0.))+curve.duration
            cells=visible_cells(runtime,curve.path()[-1],selection['yaw'])-self.fusion.priority.committed_cells(runtime,self.ledger.states,t,arrival)
            if not view_gain_retained(len(team_new_cells(cells,actual_known)), selection.get('planned_team_cells',0)):
                self.preplanned=None;return
        self.epoch += 1
        self.pending_intent = dict(token=f'{self.id}:{self.fusion.graph.replica.session[:8]}:{self.epoch}', epoch=self.epoch,
            created=t, region=rid, candidate_id=selection['pool'].active.id,
            bounds=self.tasks[rid].bounds if rid in self.tasks else None, committed=False,
            path=curve.path(.15).tolist(), yaw=selection['yaw'], duration=curve.duration, trajectory=curve.to_dict(),
            purpose=selection.get('purpose', 'explore'), handoff=boundary, contingency=False,
            voters=[i for i in self.ledger.members if i != self.id])
        self.pending_intent.update(observation_grid=observation_grid(runtime),
            expected_observation_cells=sorted(visible_cells(runtime,curve.path()[-1],selection['yaw'])))
        self.pending_selection = selection; self.preplanned = None
        self.event('handoff_proposed', token=self.pending_intent['token'], old_token=self.intent['token'],
                   region=rid, boundary=boundary, request_id=pending['request_id'],
                   old_service_ready=self.old_service_ready())
        self.record_candidates(self.pending_intent,selection,'handoff_proposed')

    def adopt_preplan(self, t):
        pending = self.preplanned
        rid = pending['region']; selection = pending['selection']
        if (pending['epoch'] != self.epoch or t-pending['time'] > 60. or
                max(self.cooldown.get(rid, 0), self.fusion.graph.services.get(rid, {}).get('defer_until', 0)) > t or
                self.fusion.graph.regions.get(rid,{}).get('status')=='splitR'):
            self.preplanned = None; return
        if self.owners.get(rid) != self.id:
            return  # Let the next heartbeat settle bids before requesting a lease.
        self.preplanned = None
        candidate = selection['pool'].active
        if candidate is None:
            return
        refresh=(t-pending['time']>15. or np.linalg.norm(pending['position']-self.position)>=.35)
        if refresh:
            selection['trajectory']=None
        # The single proposal gate below performs live geometry/expiry and
        # arrival-dependent team-gain validation. Do not validate the same
        # reserve twice or subtract promises completing after our arrival.
        self.propose(rid, selection)
        if (refresh and self.fit_pending and self.fit_pending[0]==rid and self.fit_pending[1] is selection
                and selection['pool'].active is not None):
            if selection.get('priority'):
                selection['priority']=dict(selection['priority'],estimate_source='cached_task_revalidated_goal',
                    prior_estimate_age_s=t-pending['time'],revalidated_team_gain_m3=selection['gain'])
            self.event('cached_preplan_refit_queued',region=rid,candidate=selection['pool'].active.id,
                prior_request_id=pending['request_id'],prior_age_s=t-pending['time'],
                current_map_version=self.replica.map.version,actual_start=self.position.tolist())
        if self.intent and pending['anticipatory']:
            self.event('next_view_adopted', region=rid, age_s=t-pending['time'])

    def propose(self, rid, selection):
        begin=time.monotonic()
        try:return self._propose(rid,selection)
        finally:
            self.parent_proposal_wall_s+=time.monotonic()-begin;self.parent_proposal_count+=1

    def task_lifecycle_reason(self,rid,selection,now):
        if rid<0:return None
        record=self.fusion.graph.regions.get(rid,{})
        if self.owners.get(rid)!=self.id:return 'ownership_changed'
        if max(self.cooldown.get(rid,0),self.fusion.graph.services.get(rid,{}).get('defer_until',0))>now:
            return 'service_deferred'
        if record.get('status')=='splitR':return 'region_split'
        if not selection.get('transit') and record.get('status')=='deadR':
            runtime=self.replica.map
            services=self.fusion.graph.planning_regions(runtime,now,known_mask(runtime,self.fusion.graph.observed_mask(runtime)))
            if not services.get(rid,{}).get('service_role'):return 'region_complete_without_service'
        return None

    def _propose(self, rid, selection):
        reason=self.task_lifecycle_reason(rid,selection,self.now())
        if reason:
            self.event('task_lifecycle_invalidated',region=rid,reason=reason)
            return
        runtime = self.reserved_map()
        if rid < 0:
            runtime.clearance = .5; runtime.recovery_yaw = self.yaw
        selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version, now=self.now())
        if selection['pool'].active is None:return
        if not self.ledger.can_propose(selection['pool'].active.path,rid,self.now()):return
        if rid>=0 and not selection.get('transit'):
            curve=selection.get('trajectory');arrival=self.now()+(curve.duration if curve else
                np.linalg.norm(np.diff(selection['pool'].active.path,axis=0),axis=1).sum()/.6+1.5)
            cells=(visible_cells(runtime,selection['pool'].active.path[-1],selection['yaw'])-
                self.fusion.priority.committed_cells(runtime,self.ledger.states,self.now(),arrival))
            cells=team_new_cells(cells,known_mask(runtime,self.fusion.graph.observed_mask(runtime)))
            planned = selection.get('planned_team_cells', 0)
            if not view_gain_retained(len(cells), planned):
                self.event('task_gain_invalidated',region=rid,remaining_team_cells=len(cells),
                           planned_team_cells=planned, reason='view_gain_collapsed');return
            selection['gain']=len(cells)*runtime.resolution**runtime.state.ndim
        try:
            trajectory = reusable_trajectory(selection, runtime, self.position, self.yaw) if rid >= 0 else None
            prepared = trajectory is not None
            if trajectory is None:
                if rid < 0 and selection.get('trajectory') and np.linalg.norm(selection['trajectory'].sample(0.)[0]-self.position) < .08:
                    trajectory = selection['trajectory']
                else:
                    self.fit_pending = (rid, selection)
                    self.event('trajectory_fit_queued', region=rid)
                    return
        except ValueError as e:
            self.event('trajectory_rejected', region=rid, error=str(e)); self.cooldown[rid] = self.now()+3.; return
        if not self.ledger.can_propose(trajectory.path(.15), rid, self.now()):
            self.cooldown[rid] = self.now()+3.; return
        self.epoch += 1; self.active = rid; self.selection = selection
        self.intent = dict(token=f'{self.id}:{self.fusion.graph.replica.session[:8]}:{self.epoch}', created=self.now(), region=rid,
                           epoch=self.epoch, candidate_id=selection['pool'].active.id,
                           duration=trajectory.duration, purpose='safety_recovery' if rid < 0 else selection.get('purpose', 'explore'),
                           recovery=rid < 0,
                           navigation_target=selection.get('navigation_target'),navigation_reason=selection.get('navigation_reason'),
                           committed=False, path=trajectory.path(.15).tolist(), yaw=selection['yaw'], trajectory=trajectory.to_dict(),
                           bounds=self.tasks[rid].bounds if rid in self.tasks else None,
                           contingency=not self.ledger.fresh(self.now()),
                           voters=[i for i, p in self.ledger.states.items() if -.1 <= self.now()-p['time'] < 3.],
                           observation_grid=observation_grid(runtime),
                           expected_observation_cells=sorted(visible_cells(runtime,trajectory.path()[-1],selection['yaw'])))
        self.event('path_proposed', token=self.intent['token'], region=rid,
                   tour=self.tour, workload=self.workload, objective=selection['objective'],
                   trajectory_method=trajectory.method, trajectory_limits=trajectory.limits(),
                   trajectory_duration=trajectory.duration, prepared_trajectory=prepared,
                   contingency=self.intent['contingency'], voters=self.intent['voters'])
        if selection.get('switch_origin'):
            self.event('cached_route_selected',region=rid,token=self.intent['token'],candidate=self.intent['candidate_id'])
        if (selection.get('priority') or {}).get('reactivation_reason') not in (None,'new_or_successful_task'):
            self.event('task_reactivated', region=rid, reason=selection['priority']['reactivation_reason'],
                       team_gain_m3=selection['gain'], purpose=self.intent['purpose'])
        self.record_candidates(self.intent,selection,'path_proposed')

    def record_candidates(self,intent,selection,phase):
        pool=selection['pool'];curves=selection.get('curve_alternatives',{});paths=[]
        for candidate in [pool.active]+pool.backups:
            curve=curves.get(candidate.id)
            if candidate is pool.active:curve=ContinuousTrajectory.from_dict(intent['trajectory'])
            if curve is None:continue
            fitted=selection.get('curve_qualities',{}).get(candidate.id,{})
            paths.append(dict(**candidate.metadata(),points=candidate.path.tolist(),trajectory=curve.to_dict(),
                fitted_curve_quality=fitted.get('quality',{}).get('executed_curve'),
                fitted_curve_score=fitted.get('quality',{}).get('curve_quality_score'),
                fit_map_version=fitted.get('map_version'),fit_time=fitted.get('time'),
                curve_validated_map_version=candidate.map_version if candidate is pool.active else fitted.get('map_version'),
                reserve_reuse='Rejoin, revalidate and refit at actual state before a new authorization'))
        self.paths.write(json.dumps(dict(time=self.now(),drone=self.id,epoch=intent['epoch'],region=intent['region'],
            token=intent['token'],phase=phase,purpose=intent['purpose'],active_candidate=intent['candidate_id'],
            yaw=selection['yaw'],gain_m3=selection['gain'],service_cost=selection.get('service_cost'),
            traffic_delay_s=selection.get('traffic_delay_s',0.),paths=paths))+'\n');self.paths.flush()


def main():
    rclpy.init(); node = ExplorationAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        abort_worker(node.worker)
        node.log.close(); node.paths.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
