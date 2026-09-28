#!/usr/bin/env python3
"""Serialized paired-seed study; failures are records, never silently skipped."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


def source_manifest(root):
    files = [p for folder in ('next_project/core','next_project/maps','ros2_ws/src') for p in (root/folder).rglob('*')
             if p.is_file() and '__pycache__' not in str(p)]
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}


def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline-root',required=True);p.add_argument('--combined-root',required=True)
    p.add_argument('--protocol',required=True);p.add_argument('--output-root',required=True)
    p.add_argument('--ablation-root',required=True,help='Frozen offline mutants with ablation-index.json')
    p.add_argument('--harness-root',required=True,help='Frozen read-only recorder / run harness tools')
    p.add_argument('--stop-after',type=int,default=0,help='Bounded experiment stage; unfinished jobs remain pending on resume')
    args=p.parse_args(); protocol=json.loads(Path(args.protocol).read_text())
    if len(protocol['seeds'])<5:raise ValueError('Formal paired study requires at least 5 seeds')
    roots={'baseline':Path(args.baseline_root).resolve(),'combined':Path(args.combined_root).resolve()}
    ablations=json.loads((Path(args.ablation_root)/'ablation-index.json').read_text())
    if set(ablations)!=set(protocol['ablations']):raise ValueError('Missing preregistered ablation')
    roots.update({name:Path(value['source_root']).resolve() for name,value in ablations.items()})
    output=Path(args.output_root).resolve();output.mkdir(parents=True,exist_ok=True)
    lock=(output/'study.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    frozen={variant:source_manifest(root) for variant,root in roots.items()}
    harness_root=Path(args.harness_root).resolve()
    harness_files={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in harness_root.glob('*.py')}
    evidence=dict(protocol=protocol,source_manifests=frozen,ablations=ablations,harness_manifests=harness_files,
                  orchestrator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),started_wall_unix=time.time(),jobs=[])
    header=output/'study-manifest.json'
    if header.exists():
        previous=json.loads(header.read_text())
        if previous['protocol']!=protocol or previous['source_manifests']!=frozen or previous['harness_manifests']!=harness_files or previous['orchestrator_sha256']!=evidence['orchestrator_sha256']:
            raise ValueError('Resume requires identical protocol and frozen policy sources')
        evidence=previous
    else:header.write_text(json.dumps(evidence,indent=2)+'\n')
    maps=[('main',protocol['map']['file'])]+[(name,value['file']) for name,value in protocol['targeted_scenes'].items()]
    matrix=[(scene,map_file,group,seed,variant) for scene,map_file in maps
            for group in (protocol['groups'] if scene=='main' else ['no_fault'])
            for seed in protocol['seeds'] for variant in ('baseline','combined')]
    matrix += [('main',protocol['map']['file'],'no_fault',seed,variant)
               for seed in protocol['seeds'] for variant in protocol['ablations']]
    evidence['matrix']=[dict(scene=a,map_file=b,group=c,seed=d,variant=e,job=f'{a}-{c}-{d}-{e}') for a,b,c,d,e in matrix]
    header.write_text(json.dumps(evidence,indent=2)+'\n')
    harness=harness_root/'run_todo_experiment.py'
    observer=harness_root/'record_observation_evidence.py'
    for scene,map_file,group,seed,variant in matrix:
        if args.stop_after and sum((output/j['job']/'run-result.json').exists() for j in evidence['matrix'])>=args.stop_after:
            (output/'progress.json').write_text(json.dumps(dict(status='STAGE_COMPLETE',total_jobs=len(matrix),
                finished_jobs=sum((output/j['job']/'run-result.json').exists() for j in evidence['matrix'])),indent=2)+'\n')
            return
        name=f'{scene}-{group}-{seed}-{variant}'; directory=output/name
        if (directory/'run-result.json').exists():
            cleanup=directory/'process-cleanup.json'
            if not cleanup.exists() or json.loads(cleanup.read_text()).get('remaining'):
                raise RuntimeError('Resume requires verified cleanup for every completed attempt')
            if not any(job['job']==name for job in evidence['jobs']):
                evidence['jobs'].append(dict(job=name,**json.loads((directory/'run-result.json').read_text())))
                header.write_text(json.dumps(evidence,indent=2)+'\n')
            continue
        if directory.exists():
            cleanup=directory/'process-cleanup.json'
            if not cleanup.exists() or json.loads(cleanup.read_text()).get('remaining'):
                raise RuntimeError('Interrupted attempt needs owned-process cleanup before resume')
            (directory/'run-result.json').write_text(json.dumps(dict(outcome='INFRASTRUCTURE_INTERRUPTED',reason='Unfinished attempt found on resume'),indent=2)+'\n')
            continue
        root=roots[variant]
        if source_manifest(root)!=frozen[variant]:raise ValueError('Frozen source changed during study')
        if {f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in harness_root.glob('*.py')}!=harness_files:
            raise ValueError('Frozen observer/harness changed during study')
        map_path=roots['combined']/map_file
        expected=protocol['map']['sha256'] if scene=='main' else protocol['targeted_scenes'][scene]['sha256']
        if hashlib.sha256(map_path.read_bytes()).hexdigest()!=expected:raise ValueError('Map changed')
        status=dict(job=name,total_jobs=len(matrix),completed_jobs=sum((output/n/'run-result.json').exists() for n in
            [f'{a}-{c}-{d}-{e}' for a,b,c,d,e in matrix]),started_wall_unix=time.time())
        (output/'progress.json').write_text(json.dumps(status,indent=2)+'\n')
        command=['python3',str(harness),'--observer',str(observer),'--output-dir',str(directory),'--seed',str(seed),
                 '--map',str(map_path),'--faults','none' if group=='no_fault' else 'recovery',
                 '--simulation-limit',str(protocol['simulation_limit_s']),'--wall-limit',str(protocol['wall_limit_s'])]
        shell='source /opt/ros/jazzy/setup.bash && source '+shlex.quote(str(root/'ros2_ws/install/setup.bash'))
        shell+=' && export PYTHONPATH='+shlex.quote(str(root/'ros2_ws/install/annual_swarm/lib/annual_swarm'))+':${PYTHONPATH:-}'
        shell+=' && '+shlex.join(command)
        with (output/'suite.log').open('a') as log:
            completed=subprocess.run(['bash','-lc',shell],stdout=log,stderr=log)
        if not (directory/'run-result.json').exists():
            directory.mkdir(parents=True,exist_ok=True)
            (directory/'run-result.json').write_text(json.dumps(dict(outcome='INFRASTRUCTURE_FAILURE',reason=f'Harness exit {completed.returncode} without result'),indent=2)+'\n')
        result=json.loads((directory/'run-result.json').read_text())
        cleanup=directory/'process-cleanup.json'
        if not cleanup.exists() or json.loads(cleanup.read_text()).get('remaining'):
            raise RuntimeError('Missing or failed process cleanup; do not start another sample')
        # Audit only after all owned flight processes exit. Both policies use
        # this same frozen analytic auditor, not their respective planner code.
        audit_root=roots['combined']
        audit_shell='source /opt/ros/jazzy/setup.bash && source '+shlex.quote(str(audit_root/'ros2_ws/install/setup.bash'))
        audit_shell+=' && export PYTHONPATH='+shlex.quote(str(audit_root/'ros2_ws/install/annual_swarm/lib/annual_swarm'))+':${PYTHONPATH:-}'
        audit_shell+=' && '+shlex.join(['python3',str(harness_root/'audit_todo_attempt.py'),str(directory)])
        with (directory/'audit.log').open('w') as log:
            subprocess.run(['bash','-lc',audit_shell],stdout=log,stderr=log,check=True)
        print(json.dumps(dict(job=name,**result)),flush=True)
        evidence['jobs'].append(dict(job=name,**result));header.write_text(json.dumps(evidence,indent=2)+'\n')
        if result['outcome'].startswith('INFRASTRUCTURE'):
            (output/'progress.json').write_text(json.dumps(dict(status='INFRASTRUCTURE_REVIEW_REQUIRED',job=name,
                total_jobs=len(matrix),finished_jobs=len(evidence['jobs'])),indent=2)+'\n')
            return
    (output/'progress.json').write_text(json.dumps(dict(status='COMPLETE',total_jobs=len(matrix)),indent=2)+'\n')


if __name__=='__main__':main()
