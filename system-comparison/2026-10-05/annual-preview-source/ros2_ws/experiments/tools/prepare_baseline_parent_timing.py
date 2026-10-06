#!/usr/bin/env python3
"""Add parent geometry timing only to an immutable early-policy copy."""
import argparse
import ast
import difflib
from pathlib import Path


def adapt(root, patch):
    file = Path(root)/'ros2_ws/src/annual_swarm/scripts/decentralized_agent_node.py'
    old = file.read_text(); new = old
    if 'timed_baseline_compute' not in old:
        raise ValueError('Apply the frozen baseline seed/sensor/timing adapter first')
    if 'parent_geometry_wall_s' in old:
        raise ValueError('Parent timing adapter was already applied')
    anchor = 'self.changed = False; self.last_plan = 0.; self.planner = ObservationPlanner()'
    if new.count(anchor) != 1: raise ValueError('Unknown baseline constructor')
    new = new.replace(anchor, anchor+'\n        self.parent_geometry_wall_s=0.;self.parent_geometry_count=0;self.parent_geometry_last_wall_s=0.\n        self.parent_proposal_wall_s=0.;self.parent_proposal_count=0')
    anchor = '    def peer(self, m):'
    method = ('    def update_local_geometry(self):\n'
              '        if not self.changed:return\n'
              '        begin=time.monotonic();self.replica.map.rebuild();self.changed=False\n'
              '        self.parent_geometry_last_wall_s=time.monotonic()-begin\n'
              '        self.parent_geometry_wall_s+=self.parent_geometry_last_wall_s;self.parent_geometry_count+=1\n\n')
    if new.count(anchor) != 1: raise ValueError('Unknown baseline callback layout')
    new = new.replace(anchor, method+anchor)
    for indent in (8, 12):
        anchor = ' '*indent+'if self.changed:\n'+' '*(indent+4)+'self.replica.map.rebuild(); self.changed = False'
        if new.count(anchor) != 1: raise ValueError('Unknown baseline geometry rebuild site')
        new = new.replace(anchor, ' '*indent+'self.update_local_geometry()')
    anchor = 'worker_running=self.future is not None and not self.future.done()), graph_connections=self.fusion.connections,'
    if new.count(anchor) != 1: raise ValueError('Unknown baseline diagnostics')
    new = new.replace(anchor, 'worker_running=self.future is not None and not self.future.done(),\n'
        '                                parent_geometry_wall_s=self.parent_geometry_wall_s,parent_geometry_count=self.parent_geometry_count,\n'
        '                                parent_geometry_last_wall_s=self.parent_geometry_last_wall_s,\n'
        '                                parent_proposal_wall_s=self.parent_proposal_wall_s,parent_proposal_count=self.parent_proposal_count), graph_connections=self.fusion.connections,')
    anchor='    def propose(self, rid, selection):'
    wrapper=('    def propose(self, rid, selection):\n'
        '        begin=time.monotonic()\n'
        '        try:return self._propose(rid,selection)\n'
        '        finally:\n'
        '            self.parent_proposal_wall_s+=time.monotonic()-begin;self.parent_proposal_count+=1\n\n'
        '    def _propose(self, rid, selection):')
    if new.count(anchor)!=1:raise ValueError('Unknown baseline proposal method')
    new=new.replace(anchor,wrapper)
    ast.parse(new)
    Path(patch).write_text(''.join(difflib.unified_diff(old.splitlines(keepends=True),new.splitlines(keepends=True),
        fromfile='a/ros2_ws/src/annual_swarm/scripts/decentralized_agent_node.py',tofile='b/ros2_ws/src/annual_swarm/scripts/decentralized_agent_node.py')))
    file.write_text(new)


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root',required=True);parser.add_argument('--patch',required=True)
    args=parser.parse_args();adapt(args.root,args.patch)
