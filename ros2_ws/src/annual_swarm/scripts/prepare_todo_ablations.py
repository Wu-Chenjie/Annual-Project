#!/usr/bin/env python3
"""Create auditable offline source mutants; no deployed algorithm mode flags."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def replace(root,file,old,new,count=1):
    path=root/file;source=path.read_text()
    if source.count(old)!=count:raise ValueError(f'Mutation anchor count changed: {file}: {old}')
    changed=source.replace(old,new,count);path.write_text(changed)
    return ''.join(difflib.unified_diff(source.splitlines(True),changed.splitlines(True),fromfile=file,tofile=file))


def main():
    p=argparse.ArgumentParser();p.add_argument('--source-root',required=True);p.add_argument('--output-root',required=True);p.add_argument('--build',action='store_true')
    args=p.parse_args();source=Path(args.source_root).resolve();output=Path(args.output_root).resolve();output.mkdir(parents=True,exist_ok=False)
    fusion='next_project/core/exploration/fusion.py';agent='ros2_ws/src/annual_swarm/scripts/decentralized_agent_node.py'
    priority='next_project/core/exploration/priority.py';graph='next_project/core/exploration/mrdtg.py'
    mutations={
        'continuous_handoff':[(agent,"if self.intent and self.execution.get('epoch') == self.intent.get('epoch'):","if False and self.intent and self.execution.get('epoch') == self.intent.get('epoch'):")],
        'task_route_objective':[(fusion,'rewards=rewards,','rewards=None,'),(fusion,'latency_weight=self.priority.config.route_latency_weight','latency_weight=0.'),(fusion,'reward_weights=[rewards[r] for r in ids]','reward_weights=None')],
        'incremental_cache':[(fusion,'global _WORKER_PLANNER\n','global _WORKER_PLANNER\n    _WORKER_PLANNER = None\n')],
        'task_lifecycle':[(agent,"feedback = service_result(previous, t, gained, expected_team, self.fusion.priority.config)\n", "feedback = service_result(previous, t, gained, expected_team, self.fusion.priority.config)\n            feedback['defer_until']=t;feedback['low_yield_streak']=0\n")],
        'team_new_accounting':[(graph,"grid = observation_grid(runtime); blocks = self.coverage.get(grid, {})\n", "grid = observation_grid(runtime); blocks = {}\n        for source,value in self.replica.values():\n            if source==self.source and value['kind']=='observed_cells' and value['grid']==grid:\n                blocks[value['block']]=blocks.get(value['block'],0)|int(value['bits'],16)\n"),
            (agent,"gained = accounting['team_new_cells']","gained = accounting['local_new_cells']")],
        'arrival_support_prediction':[(priority,'def committed_cells(self, runtime, peers, now, arrival=None):', 'def committed_cells(self, runtime, peers, now, arrival=None):\n        return frozenset()'),
            (priority,"if any(intent.get('region')==rid", "if False and any(intent.get('region')==rid")]
    }
    descriptions=dict(task_lifecycle='Disable low-yield retirement/reactivation penalties. Actual observed-union and collision checks remain.',
                      incremental_cache='Recreate private planner caches on every request; same geometry and information rules.',
                      team_new_accounting='Local-novelty planning and service rewards; raw independent team evidence remains recorded.')
    index={}
    for name,changes in mutations.items():
        root=output/name;root.mkdir()
        for folder in ('next_project','ros2_ws/src','ros2_ws/experiments','docker'):
            shutil.copytree(source/folder,root/folder,ignore=shutil.ignore_patterns('__pycache__','build','install','log'))
        patch=''
        for file,old,new in changes:
            count=2 if name=='task_route_objective' and old.startswith('latency_weight=') else 1
            patch+=replace(root,file,old,new,count)
        (root/'ablation.patch').write_text(patch)
        subprocess.run(['python3','-m','compileall','-q',str(root/'next_project/core'),str(root/'ros2_ws/src/annual_swarm/scripts')],check=True)
        manifest={str(file.relative_to(root)):hashlib.sha256(file.read_bytes()).hexdigest() for folder in ('next_project','ros2_ws/src')
                  for file in sorted((root/folder).rglob('*')) if file.is_file() and '__pycache__' not in str(file)}
        (root/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        index[name]=dict(source_root=str(root),description=descriptions.get(name,'Remove only the named contribution; keep fused pipeline and safety gates.'),
                         patch_sha256=hashlib.sha256(patch.encode()).hexdigest())
        if args.build:
            with (root/'build.log').open('w') as log:
                subprocess.run(['colcon','build','--executor','sequential','--cmake-args','-DCMAKE_BUILD_TYPE=Release'],cwd=root/'ros2_ws',stdout=log,stderr=log,check=True)
    (output/'ablation-index.json').write_text(json.dumps(index,indent=2)+'\n')


if __name__=='__main__':main()
