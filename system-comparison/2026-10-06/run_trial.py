"""One bounded native development trial; never starts the formal matrix."""
import json, os, subprocess, sys, time
from pathlib import Path
root=Path(__file__).resolve().parent
name=sys.argv[1]
if name not in ('baseline-900','baseline-900-r1','candidate-900','candidate-refit-900','candidate-closure-900','candidate-publish-900'):raise ValueError('Trial must be predeclared in protocol')
out=root/name;out.mkdir(exist_ok=False)
command=['docker','--context','colima','exec']
for k,v in dict(COMPARISON_MODE='annual',COMPARISON_OUTPUT='/study/'+name,COMPARISON_LIMIT='300',
    OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LP_NUM_THREADS='2',ANNUAL_EXPERIMENT_SEED='900',
    ROS_DOMAIN_ID='74',GZ_PARTITION='todo_1006_'+name,PYTHONDONTWRITEBYTECODE='1',
    PYTHONPYCACHEPREFIX='/tmp/todo-pycache-'+name,LIBGL_ALWAYS_SOFTWARE='1').items():command+=['-e',k+'='+v]
command+=['annual-todo-1006','bash','-lc','source /workspace/ros2_ws/install/setup.bash && exec python3 -u /study/supervisor.py ros2 annual']
with (out/'supervisor-host.log').open('w') as log:
    process=subprocess.Popen(command,stdout=log,stderr=log)
    begin=time.monotonic();last=0
    try:
        while process.poll() is None:
            elapsed=time.monotonic()-begin
            if elapsed-last>=20:
                last=elapsed
                if (out/'progress.json').exists():
                    d=json.loads((out/'progress.json').read_text());print(json.dumps({k:d.get(k) for k in ('t','coverage','contacts','positions')}),flush=True)
                else:print('waiting for simulator',round(elapsed),flush=True)
            if elapsed>1200:
                (out/'STOP').touch();break
            time.sleep(.5)
    finally:
        (out/'STOP').touch()
        try:process.wait(timeout=30)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        (out/'host-exit.json').write_text(json.dumps(dict(exit_code=process.returncode,wall_s=time.monotonic()-begin),indent=2))
print('finished',name,'result_present=',(out/'result.json').exists(),flush=True)
raise SystemExit(0 if (out/'result.json').exists() else 1)
