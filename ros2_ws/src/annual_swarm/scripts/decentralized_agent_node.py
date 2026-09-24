#!/usr/bin/env python3
"""One independently replaceable exploration agent. No truth map or fleet planner."""
import copy
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import json
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import planning_runtime  # installed core module location
from core.exploration.decentralized import MapReplica, PeerLedger, remainder, tracking_recovery
from core.exploration.sparse_graph import SparseTopology
from core.exploration.fusion import FusionPlanner, reusable_trajectory
from core.exploration.pairwise import PairExchange
from core.planning.continuous_trajectory import optimize_trajectory
from core.exploration.regions import RegionTasks, ObservationPlanner, insertion_bids, visible_cells
from core.planning.path_quality import PathQualityEvaluator, RankedPathPool, Candidate


class ExplorationAgent(Node):
    def __init__(self):
        super().__init__('exploration_agent')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        bounds = json.loads(self.declare_parameter('bounds', '[]').value)
        self.seeds = json.loads(self.declare_parameter('fleet_starts', '[]').value)
        self.network_isolated = set()
        self.replica = MapReplica(bounds, self.id, volumetric=True, flight_limits=(.7, min(3.1, bounds[1][2]-.3))); self.ledger = PeerLedger(self.id, members=tuple(range(len(self.seeds))))
        self.ledger.seeds = self.seeds
        self.fusion = FusionPlanner(self.id, bounds); self.pair = PairExchange(self.id, self.fusion.graph.replica.session)
        self.bootstrapped = False; self.rejoin_ready = False; self.join_stopped_since = None
        self.last_graph_full = -30.; self.graph_bytes = 0; self.last_pair_commits = 0
        self.position = None; self.yaw = 0.; self.observation = None; self.execution = {}
        self.bids = {}; self.tour = []; self.owners = {}; self.workload = 0.
        self.intent = None; self.selection = None; self.active = None; self.epoch = 0
        self.cached_pending = None; self.retiring = None; self.stop_epoch = None; self.stop_since = None; self.speed = 0.
        self.preplanned = None
        self.sequence = 0; self.ready = False; self.available = True; self.done = False
        self.cooldown = {}; self.recent = []; self.graph = None; self.tasks = {}
        self.service_feedback = {}
        self.graph_seq = 0; self.acks = []; self.events = []
        self.changed = False; self.last_plan = 0.; self.planner = ObservationPlanner()
        self.output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value)/f'drone_{self.id}'
        self.output.mkdir(parents=True, exist_ok=True)
        previous_events = self.output/'events.jsonl'
        if previous_events.exists():
            for line in previous_events.read_text().splitlines():
                try:
                    e = json.loads(line)
                    if e.get('type') == 'region_low_yield':
                        self.service_feedback[int(e['region'])] = dict(stamp=e['time'], defer_until=e['retry_after'], observed_new_cells=e['new_cells'])
                except (ValueError, KeyError):
                    continue
        self.log = (self.output/'events.jsonl').open('a'); self.paths = (self.output/'candidates.jsonl').open('a')
        self.last_map_dump = -20.
        self.view_start_known = 0
        self.bytes = 0; self.peer_bytes = 0; self.commits = 0; self.observed_views = 0
        # CPU-bound graph/tour/trajectory work must not hold the ROS callback GIL.
        # Spawn avoids inheriting DDS threads or sockets into the planning worker.
        self.worker = ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'))
        self.future = None; self.future_epoch = None; self.last_submit = -10.
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

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def event(self, kind, **values):
        event = dict(type=kind, drone=self.id, time=self.now(), incarnation=self.fusion.graph.replica.session, **values)
        self.events.append(event); self.log.write(json.dumps(event)+'\n'); self.log.flush()

    def odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.position = np.array([p.x, p.y, p.z])
        v = m.twist.twist.linear; self.speed = float(np.linalg.norm([v.x, v.y, v.z]))
        self.yaw = float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def sense(self, m):
        self.observation = json.loads(m.data)
        self.changed |= self.replica.merge(self.observation)

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
            self.event('process_restart_requested'); self.log.flush(); self.paths.flush(); os._exit(75)
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
        if self.intent is not None and self.intent.get('committed'):
            self.retiring = copy.deepcopy(self.intent); self.retiring['committed'] = True; self.retiring['retiring'] = True
        self.stop_epoch = self.epoch; self.stop_since = None
        self.publish(self.command_pub, dict(epoch=self.epoch, cancel=True, reason=reason))
        self.intent = None; self.selection = None; self.active = None
        self.preplanned = None
        self.event('lease_cancellation_requested', region=previous, reason=reason)

    def state(self):
        intent = copy.deepcopy(self.intent or self.retiring)
        if intent:
            intent.pop('trajectory', None)
        if intent and intent.get('committed'):
            intent['path'] = remainder(self.position, intent['path']).tolist()
        return dict(drone=self.id, session=self.fusion.graph.replica.session, epoch=self.epoch,
                    rejoin_ready=self.rejoin_ready, stopped=self.speed < .08 and self.execution.get('arrived', False),
                    peer_sessions={i: p.get('session') for i, p in self.ledger.states.items()},
                    sequence=self.sequence, time=self.now(), position=self.position.tolist(), yaw=self.yaw,
                    ready=self.ready, available=self.available and self.rejoin_ready and not self.done, bids=self.bids if self.available else {},
                    tour=self.tour, workload=self.workload, owners=self.owners, active=self.active,
                    intent=intent, acks=self.acks, observation=None,
                    fusion=dict(self.fusion.diagnostics, map_clearance=self.replica.map.signed_distance(self.position),
                                worker_running=self.future is not None and not self.future.done()), graph_connections=self.fusion.connections,
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
            else:
                self.stop_since = None
        if self.intent and not self.intent.get('committed') and self.ledger.loses(self.intent):
            self.withdraw('peer_priority')
        self.acks = self.ledger.acknowledge(self.state(), t)
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t, self.intent.get('voters')):
            # Sensing can invalidate an intent while its ACKs are in flight.
            if self.changed:
                self.replica.map.rebuild(); self.changed = False
            runtime = self.reserved_map()
            if self.intent.get('recovery'):
                runtime.clearance = .5
                runtime.recovery_yaw = self.intent['yaw']
            if not runtime.safe_path(self.intent['path']):
                self.withdraw('intent_invalidated_before_commit')
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t, self.intent.get('voters')):
            self.intent['committed'] = True; self.commits += 1
            self.view_start_known = int(np.count_nonzero(self.replica.map.state != -1))
            self.publish(self.command_pub, dict(epoch=self.epoch, path=self.intent['path'], yaw=self.intent['yaw'],
                                               region=self.active, token=self.intent['token'], trajectory=self.intent.get('trajectory')))
            self.event('path_committed', token=self.intent['token'], region=self.active, quorum=self.intent['voters'],
                       predicted_frontier_volume_m3=self.selection['gain'], lookahead=self.selection['lookahead'])
        if self.intent and self.intent.get('committed') and self.execution.get('arrived') and self.execution.get('epoch') == self.epoch:
            self.observed_views += 1
            new_cells = max(0, int(np.count_nonzero(self.replica.map.state != -1))-self.view_start_known)
            self.recent.append((t+(240. if new_cells < 3 else 0.), self.position.copy(), self.intent['yaw']))
            self.event('view_observed', region=self.active, epoch=self.epoch, token=self.intent['token'], new_cells=new_cells,
                       observed_volume=new_cells*self.replica.map.resolution**self.replica.map.state.ndim)
            if self.active is not None and self.active >= 0 and new_cells < 30:
                # A noisy wall fringe can remain an apparent frontier after a
                # successful view. Measured low yield ends the current regional
                # service, allowing the coverage tour to advance before retry.
                self.cooldown[self.active] = t+600.
                self.service_feedback[self.active] = dict(stamp=t, defer_until=t+600., observed_new_cells=new_cells)
                self.event('region_low_yield', region=self.active, new_cells=new_cells, retry_after=t+600.)
                self.active = None
            # Retain the regional service priority while selecting its next view.
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
        pinned = {int(p['active']): i for i, p in self.ledger.states.items() if p.get('active') is not None and p.get('intent')}
        if self.active is not None:
            pinned[self.active] = self.id
        self.pair.tick(self.ledger.states, owners, t, pinned)
        if self.pair.commits != self.last_pair_commits:
            self.event('pair_cvrp_committed', count=self.pair.commits, assignments=self.pair.overrides)
            self.last_pair_commits = self.pair.commits
        self.publish(self.state_pub, self.state())

    def reserved_map(self):
        runtime = copy.deepcopy(self.replica.map)
        paths = []
        for p in self.ledger.states.values():
            paths.append(p['intent']['path'] if p.get('intent') else [p['position']])
        paths.extend(g['path'] for g in self.ledger.grants.values())
        runtime.block_paths(paths, radius=1.25)
        return runtime

    def plan(self):
        if self.position is None or not self.ready or not self.rejoin_ready or not self.available or self.done:
            return
        t = self.now()
        if self.retiring:
            return
        if not self.ledger.fresh(t) and len(self.ledger.states) != len(self.seeds)-1:
            return
        if not self.ledger.fresh(t) and self.intent and not self.intent.get('committed'):
            self.withdraw('proposal_voter_timeout'); return
        if self.changed:
            self.replica.map.rebuild(); self.changed = False
        if self.intent and self.intent.get('committed'):
            path = remainder(self.position, self.intent['path'])
            recovery = copy.copy(self.replica.map); recovery.clearance = .5
            if self.intent.get('recovery'):
                recovery.recovery_yaw = self.intent['yaw']
            tracking_ok = (len(path) > 1 and np.linalg.norm(path[0]-path[1]) < .3 and recovery.safe_path(path[:2]))
            valid = recovery.safe_path(path) if self.intent.get('recovery') else self.replica.map.safe_path(path[1:]) and tracking_ok
            if not valid:
                rid = self.active; selection = self.selection
                self.withdraw('path_or_tracking_invalidated')
                runtime = self.reserved_map()
                selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version)
                if selection['pool'].active and rid in self.tasks:
                    self.cached_pending = (rid, selection)
                return
        if self.cached_pending and not self.intent:
            rid, selection = self.cached_pending; self.cached_pending = None
            runtime = self.reserved_map()
            selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version)
            if selection['pool'].active and self.owners.get(rid) == self.id and self.ledger.can_propose(selection['pool'].active.path, rid, t):
                self.propose(rid, selection); self.event('cached_route_switched', region=rid)
        if self.preplanned and not self.intent:
            self.adopt_preplan(t)
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
            return
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result = self.future.result()
            except Exception as e:
                self.future = None; self.event('planning_failed', error=str(e)); return
            self.future = None
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
            if offer and not self.pair.transaction:
                base = {r: self.owners.get(r) for r in offer['owners']}
                if all(owner in (self.id, offer['other']) for owner in base.values()):
                    if self.pair.offer(offer['other'], offer['result'], base, t):
                        self.event('pair_cvrp_prepared', other=offer['other'], before=offer['result']['before'], after=offer['result']['after'],
                                   loads=offer['result']['loads'], capacity=offer['result']['capacity'], fixed_loads=offer['result'].get('fixed_loads'))
            self.last_plan = result['wall']; self.graph = result['graph']; self.tasks = result['tasks']
            self.bids, self.tour, self.workload = result['bids'], result['tour'], result['workload']
            self.graph_seq += 1; snapshot = self.graph.snapshot(); snapshot['sequence'] = self.graph_seq
            self.publish(self.graph_pub, snapshot)
            if self.active not in self.tasks and not self.intent:
                self.active = None
            if self.future_epoch == self.epoch:
                if not self.intent:
                    for rid in result['rejected']:
                        self.cooldown[rid] = t+6
                        if rid == self.active:
                            self.event('region_yielded', region=rid); self.active = None
                selection = result['selection']; rid = result['selected']
                if selection:
                    self.preplanned = dict(epoch=self.epoch, position=result['position'], region=rid,
                                           selection=selection, time=t, anticipatory=bool(self.intent))
                    if self.intent:
                        self.event('next_view_ready', region=rid, epoch=self.epoch, compute_wall_s=result['wall'])
                    else:
                        self.adopt_preplan(t)
        if t-self.last_submit < 2.:
            return
        if self.intent and (not self.intent.get('committed') or self.intent.get('recovery')):
            return
        # Retain a ready next view rather than continuously refitting it. Refresh
        # periodically as sensing changes; final adoption always checks live data.
        if self.preplanned and t-self.preplanned['time'] < 5.:
            return
        owned = [r for r, owner in self.owners.items() if owner == self.id]
        reservations = [p['intent']['path'] if p.get('intent') else [p['position']] for p in self.ledger.states.values()]
        reservations.extend(g['path'] for g in self.ledger.grants.values())
        recent = [(p, h) for stamp, p, h in self.recent if t-stamp < 60.]
        active = self.active
        anticipated = dict(position=self.intent['path'][-1], yaw=self.intent['yaw']) if self.intent else None
        self.last_submit = t
        self.future_epoch = self.epoch
        planner = copy.deepcopy(self.fusion)
        self.future = self.worker.submit(planner.compute, copy.deepcopy(self.replica.map), self.position.copy(), self.yaw,
            active, recent, dict(self.cooldown), reservations, self.epoch, t, True,
            copy.deepcopy(self.ledger.states), dict(self.pair.overrides), dict(self.pair.last_success),
            copy.deepcopy(self.service_feedback), anticipated)

    def adopt_preplan(self, t):
        pending = self.preplanned
        rid = pending['region']; selection = pending['selection']
        if (pending['epoch'] != self.epoch or t-pending['time'] > 15. or
                max(self.cooldown.get(rid, 0), self.fusion.graph.services.get(rid, {}).get('defer_until', 0)) > t or
                np.linalg.norm(pending['position']-self.position) >= .35):
            self.preplanned = None; return
        if self.owners.get(rid) != self.id:
            return  # Let the next heartbeat settle bids before requesting a lease.
        self.preplanned = None
        runtime = self.reserved_map()
        selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version)
        candidate = selection['pool'].active
        if candidate is None:
            return
        if pending['anticipatory'] and not selection.get('transit'):
            cells = visible_cells(runtime, candidate.path[-1], selection['yaw'])
            if len(cells) < 5:
                return
            selection['gain'] = len(cells)*runtime.resolution**runtime.state.ndim
        if self.ledger.can_propose(candidate.path, rid, t):
            self.propose(rid, selection)
            if self.intent and pending['anticipatory']:
                self.event('next_view_adopted', region=rid, age_s=t-pending['time'])

    def propose(self, rid, selection):
        runtime = self.reserved_map()
        if rid < 0:
            runtime.clearance = .5; runtime.recovery_yaw = self.yaw
        try:
            trajectory = reusable_trajectory(selection, runtime, self.position, self.yaw) if rid >= 0 else None
            prepared = trajectory is not None
            if trajectory is None:
                trajectory = optimize_trajectory(selection['pool'].active.path, runtime, self.yaw, selection['yaw'],
                                             speed_limit=.15 if rid < 0 else .6, acceleration_limit=.2 if rid < 0 else .8)
        except ValueError as e:
            self.event('trajectory_rejected', region=rid, error=str(e)); self.cooldown[rid] = self.now()+3.; return
        if not self.ledger.can_propose(trajectory.path(.15), rid, self.now()):
            self.cooldown[rid] = self.now()+3.; return
        self.epoch += 1; self.active = rid; self.selection = selection
        self.intent = dict(token=f'{self.id}:{self.fusion.graph.replica.session[:8]}:{self.epoch}', created=self.now(), region=rid,
                           committed=False, path=trajectory.path(.15).tolist(), yaw=selection['yaw'], trajectory=trajectory.to_dict(),
                           bounds=self.tasks[rid].bounds if rid in self.tasks else None,
                           contingency=not self.ledger.fresh(self.now()),
                           voters=[i for i, p in self.ledger.states.items() if -.1 <= self.now()-p['time'] < 3.])
        self.event('path_proposed', token=self.intent['token'], region=rid,
                   tour=self.tour, workload=self.workload, objective=selection['objective'],
                   trajectory_method=trajectory.method, trajectory_limits=trajectory.limits(),
                   trajectory_duration=trajectory.duration, prepared_trajectory=prepared,
                   contingency=self.intent['contingency'], voters=self.intent['voters'])
        pool = selection['pool']
        self.paths.write(json.dumps(dict(time=self.now(), drone=self.id, epoch=self.epoch, region=rid,
            yaw=selection['yaw'], gain_m2=selection['gain'], paths=[dict(**p.metadata(), points=p.path.tolist())
            for p in [pool.active]+pool.backups]))+'\n'); self.paths.flush()


def main():
    rclpy.init(); node = ExplorationAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        node.log.close(); node.paths.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
