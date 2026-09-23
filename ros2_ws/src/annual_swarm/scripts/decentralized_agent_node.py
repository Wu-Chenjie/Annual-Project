#!/usr/bin/env python3
"""One independently replaceable exploration agent. No truth map or fleet planner."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
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
from core.exploration.regions import RegionTasks, ObservationPlanner, insertion_bids
from core.planning.path_quality import PathQualityEvaluator, RankedPathPool, Candidate


class ExplorationAgent(Node):
    def __init__(self):
        super().__init__('exploration_agent')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        bounds = json.loads(self.declare_parameter('bounds', '[]').value)
        self.replica = MapReplica(bounds, self.id); self.ledger = PeerLedger(self.id)
        self.position = None; self.yaw = 0.; self.observation = None; self.execution = {}
        self.bids = {}; self.tour = []; self.owners = {}; self.workload = 0.
        self.intent = None; self.selection = None; self.active = None; self.epoch = 0
        self.cached_pending = None; self.retiring = None; self.stop_epoch = None; self.stop_since = None; self.speed = 0.
        self.sequence = 0; self.ready = False; self.available = True; self.done = False
        self.cooldown = {}; self.recent = []; self.graph = None; self.tasks = {}
        self.graph_seq = 0; self.acks = []; self.events = []
        self.changed = False; self.last_plan = 0.; self.planner = ObservationPlanner()
        self.output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value)/f'drone_{self.id}'
        self.output.mkdir(parents=True, exist_ok=True)
        self.log = (self.output/'events.jsonl').open('w'); self.paths = (self.output/'candidates.jsonl').open('w')
        self.bytes = 0; self.commits = 0; self.observed_views = 0
        self.worker = ThreadPoolExecutor(max_workers=1); self.future = None; self.future_epoch = None; self.last_submit = -10.
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, f'/drone_{self.id}/peer_state', q)
        self.command_pub = self.create_publisher(String, f'/drone_{self.id}/view_command', q)
        self.graph_pub = self.create_publisher(String, f'/drone_{self.id}/topology', q)
        self.sync_pub = self.create_publisher(String, f'/drone_{self.id}/map_sync', q)
        self.create_subscription(Odometry, f'/drone_{self.id}/odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, f'/drone_{self.id}/observation', self.sense, q)
        self.create_subscription(String, f'/drone_{self.id}/view_execution', lambda m: setattr(self, 'execution', json.loads(m.data)), q)
        self.create_subscription(String, '/experiment/control', self.control, q)
        for i in range(3):
            if i != self.id:
                self.create_subscription(String, f'/drone_{i}/peer_state', self.peer, q)
                self.create_subscription(String, f'/drone_{i}/map_sync', self.sync, q)
        self.create_timer(.4, self.heartbeat)
        self.create_timer(.5, self.plan)
        self.create_timer(6., self.publish_sync)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def event(self, kind, **values):
        event = dict(type=kind, drone=self.id, time=self.now(), **values)
        self.events.append(event); self.log.write(json.dumps(event)+'\n'); self.log.flush()

    def odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.position = np.array([p.x, p.y, 1.5])
        v = m.twist.twist.linear; self.speed = float(np.linalg.norm([v.x, v.y, v.z]))
        self.yaw = float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def sense(self, m):
        self.observation = json.loads(m.data)
        self.changed |= self.replica.merge(self.observation)

    def peer(self, m):
        packet = json.loads(m.data)
        if self.ledger.receive(packet) and packet.get('observation'):
            self.changed |= self.replica.merge(packet['observation'])

    def sync(self, m):
        data = json.loads(m.data)
        ids = np.asarray(data['ids'], int); stamps = np.asarray(data['stamps'], float); sources = np.asarray(data['sources'], int)
        if not len(ids):
            return
        newer = (stamps > self.replica.stamps.flat[ids]) | ((stamps == self.replica.stamps.flat[ids]) & (sources > self.replica.sources.flat[ids]))
        ids, stamps, sources = ids[newer], stamps[newer], sources[newer]
        values = np.asarray(data['values'], np.int8)[newer]
        self.changed |= self.replica.map.update(np.column_stack(np.unravel_index(ids, self.replica.map.shape)), values)
        self.replica.stamps.flat[ids] = stamps; self.replica.sources.flat[ids] = sources

    def publish_sync(self):
        m = self.replica.map; ids = np.flatnonzero(m.state != -1)
        packet = dict(drone=self.id, time=self.now(), ids=ids.tolist(), values=m.state.flat[ids].tolist(),
                      stamps=self.replica.stamps.flat[ids].tolist(), sources=self.replica.sources.flat[ids].tolist())
        self.publish(self.sync_pub, packet)

    def control(self, m):
        control = json.loads(m.data); available = self.id not in control.get('paused', [])
        self.done = control.get('done', False)
        if available != self.available:
            self.event('availability_resume' if available else 'availability_pause', region=self.active)
        if (not available or self.done) and (self.intent or self.active is not None):
            self.withdraw('experiment_pause' if not available else 'experiment_complete')
        self.available = available

    def publish(self, pub, packet):
        data = json.dumps(packet, separators=(',', ':')); self.bytes += len(data.encode()); pub.publish(String(data=data))

    def withdraw(self, reason):
        previous = self.active
        self.epoch += 1
        if self.intent is not None and self.intent.get('committed'):
            self.retiring = copy.deepcopy(self.intent); self.retiring['committed'] = True; self.retiring['retiring'] = True
        self.stop_epoch = self.epoch; self.stop_since = None
        self.publish(self.command_pub, dict(epoch=self.epoch, cancel=True, reason=reason))
        self.intent = None; self.selection = None; self.active = None
        self.event('lease_cancellation_requested', region=previous, reason=reason)

    def state(self):
        intent = copy.deepcopy(self.intent or self.retiring)
        if intent and intent.get('committed'):
            intent['path'] = remainder(self.position, intent['path']).tolist()
        return dict(drone=self.id, sequence=self.sequence, time=self.now(), position=self.position.tolist(), yaw=self.yaw,
                    ready=self.ready, available=self.available and not self.done, bids=self.bids if self.available else {},
                    tour=self.tour, workload=self.workload, owners=self.owners, active=self.active,
                    intent=intent, acks=self.acks, observation=self.observation, graph=dict(nodes=len(self.graph.nodes),
                    edges=len(self.graph.edges), free_cells=self.graph.free_cells) if self.graph else {},
                    tasks=[t.descriptor() for t in self.tasks.values()], map_version=self.replica.map.version,
                    payload_bytes=self.bytes, commits=self.commits, observed_views=self.observed_views, planner_wall_s=self.last_plan)

    def heartbeat(self):
        if self.position is None:
            return
        t = self.now(); self.ready = self.execution.get('ready', False)
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
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t):
            # Sensing can invalidate an intent while its ACKs are in flight.
            if self.changed:
                self.replica.map.rebuild(); self.changed = False
            runtime = self.reserved_map()
            if self.intent.get('recovery'):
                runtime.clearance = .5
            if not runtime.safe_path(self.intent['path']):
                self.withdraw('intent_invalidated_before_commit')
        if self.intent and not self.intent.get('committed') and self.ledger.quorum(self.intent['token'], t):
            self.intent['committed'] = True; self.commits += 1
            self.publish(self.command_pub, dict(epoch=self.epoch, path=self.intent['path'], yaw=self.intent['yaw'],
                                               region=self.active, token=self.intent['token']))
            self.event('path_committed', token=self.intent['token'], region=self.active, quorum=sorted(self.ledger.states),
                       gain_m2=self.selection['gain'], lookahead=self.selection['lookahead'])
        if self.intent and self.intent.get('committed') and self.execution.get('arrived') and self.execution.get('epoch') == self.epoch:
            self.observed_views += 1
            self.recent.append((t, self.position.copy(), self.intent['yaw']))
            self.event('view_observed', region=self.active, epoch=self.epoch)
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
        self.publish(self.state_pub, state)

    def reserved_map(self):
        runtime = copy.deepcopy(self.replica.map)
        paths = []
        for p in self.ledger.states.values():
            paths.append(p['intent']['path'] if p.get('intent') else [p['position']])
        paths.extend(g['path'] for g in self.ledger.grants.values())
        runtime.block_paths(paths, radius=1.25)
        return runtime

    @staticmethod
    def compute(runtime, position, yaw, owned, active, recent, cooldown, reservations, epoch, now, plan_view):
        begin = time.monotonic()
        runtime.rebuild(); graph = SparseTopology(runtime); tasks = RegionTasks(runtime).tasks
        feasible = {r: task for r, task in tasks.items() if cooldown.get(r, 0) <= now}
        bids, tour, workload, _ = insertion_bids(position, owned, feasible, graph, active)
        choices = ([active] if active in feasible else [])+[r for r in tour if r != active]
        local = copy.deepcopy(runtime); local.block_paths(reservations, radius=1.25)
        local_graph = SparseTopology(local) if plan_view else None; planner = ObservationPlanner()
        if not plan_view:
            choices = []
        selection = None; selected = None; rejected = []
        for rid in choices:
            if rid not in feasible:
                continue
            selection = planner.plan(local, local_graph, position, yaw, feasible[rid], epoch+1, recent)
            if selection:
                selected = rid; break
            rejected.append(rid)
        return dict(graph=graph, tasks=tasks, bids=bids, tour=tour, workload=workload,
                    selection=selection, selected=selected, rejected=rejected, wall=time.monotonic()-begin,
                    position=position, version=runtime.version)

    def plan(self):
        if self.position is None or not self.ready or not self.available or self.done:
            return
        t = self.now()
        if self.retiring:
            return
        if not self.ledger.fresh(t):
            if self.intent:
                self.withdraw('peer_timeout')
            return
        if self.changed:
            self.replica.map.rebuild(); self.changed = False
        if self.intent and self.intent.get('committed'):
            path = remainder(self.position, self.intent['path'])
            recovery = copy.copy(self.replica.map); recovery.clearance = .5
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
        if not self.intent and not self.replica.map.safe_path([self.position]):
            runtime = self.reserved_map(); path = tracking_recovery(runtime, self.position)
            rid = -100-self.id
            if path is not None and self.ledger.can_propose(path, rid, t):
                recovery = copy.copy(runtime); recovery.clearance = .5
                pool = RankedPathPool(); pool.rank([Candidate(f'recovery:{self.epoch+1}', 'tracking_recovery', 0,
                    path, PathQualityEvaluator().evaluate(path, recovery), runtime.version)])
                self.propose(rid, dict(pool=pool, yaw=self.yaw, gain=0., objective=0., lookahead=None))
                self.intent['recovery'] = True
                self.event('tracking_recovery', goal=path[-1].tolist())
            return
        if self.future is not None:
            if not self.future.done():
                return
            result = self.future.result(); self.future = None
            self.last_plan = result['wall']; self.graph = result['graph']; self.tasks = result['tasks']
            self.bids, self.tour, self.workload = result['bids'], result['tour'], result['workload']
            self.graph_seq += 1; snapshot = self.graph.snapshot(); snapshot['sequence'] = self.graph_seq
            self.publish(self.graph_pub, snapshot)
            if self.active not in self.tasks and not self.intent:
                self.active = None
            if self.future_epoch == self.epoch and not self.intent:
                for rid in result['rejected']:
                    self.cooldown[rid] = t+6
                    if rid == self.active:
                        self.event('region_yielded', region=rid); self.active = None
                selection = result['selection']; rid = result['selected']
                # Work completed in the background is untrusted until current map,
                # ownership, position and every live reservation have been checked.
                if selection and self.owners.get(rid) == self.id and np.linalg.norm(result['position']-self.position) < .35:
                    runtime = self.reserved_map()
                    selection['pool'].revalidate(self.position, runtime, PathQualityEvaluator(), runtime.version)
                    if selection['pool'].active and self.ledger.can_propose(selection['pool'].active.path, rid, t):
                        self.propose(rid, selection)
        if t-self.last_submit < 2.:
            return
        owned = [r for r, owner in self.owners.items() if owner == self.id]
        reservations = [p['intent']['path'] if p.get('intent') else [p['position']] for p in self.ledger.states.values()]
        reservations.extend(g['path'] for g in self.ledger.grants.values())
        recent = [(p, h) for stamp, p, h in self.recent if t-stamp < 20.]
        # An executing aircraft updates allocation costs but need not generate an
        # unrequested replacement view. The epoch fence rejects late worker output.
        active = self.active
        self.last_submit = t
        self.future_epoch = self.epoch
        self.future = self.worker.submit(self.compute, copy.deepcopy(self.replica.map), self.position.copy(), self.yaw,
            owned, active, recent, dict(self.cooldown), reservations, self.epoch, t, not bool(self.intent))

    def propose(self, rid, selection):
        self.epoch += 1; self.active = rid; self.selection = selection
        self.intent = dict(token=f'{self.id}:{self.epoch}', created=self.now(), region=rid,
                           committed=False, path=selection['pool'].active.path.tolist(), yaw=selection['yaw'])
        self.event('path_proposed', token=self.intent['token'], region=rid,
                   tour=self.tour, workload=self.workload, objective=selection['objective'])
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
