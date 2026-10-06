import os,sys,subprocess,time,signal
from pathlib import Path
side,mode=sys.argv[1:3];out=Path(os.environ['COMPARISON_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
children=[];logs=[]
def start(args,name,stdio=False):
 log=open(out/(name+'.log'),'w');logs.append(log)
 p=subprocess.Popen(args,stdin=sys.stdin if stdio else subprocess.DEVNULL,stdout=sys.stdout if stdio else log,stderr=log,start_new_session=True);children.append(p);return p
try:
 if side=='ros1':
  master=start(['roscore','-p','11328'],'roscore');time.sleep(2)
  launch=start(['roslaunch','/study/'+mode+'.launch'],'planner');time.sleep(2)
  bridge=start(['python3','-u','/study/bridge_ros1.py',mode],'transport-ros1',True)
 else:
  bridge=start(['python3','-u','/study/bridge_ros2.py'],'observer',True)
  launch=start(['ros2','launch','/study/common.launch.py'],'simulator')
 while not (out/'STOP').exists() and not (out/'result.json').exists():
  if bridge.poll() is not None:break
  if launch.poll() is not None:(out/(side+'-launch-failed.txt')).write_text(str(launch.returncode));break
  time.sleep(.2)
finally:
 for p in reversed(children):
  try:os.killpg(p.pid,signal.SIGINT)
  except ProcessLookupError:pass
 time.sleep(3)
 for p in children:
  try:os.killpg(p.pid,signal.SIGKILL)
  except ProcessLookupError:pass
 for p in children:
  try:p.wait(timeout=2)
  except subprocess.TimeoutExpired:pass

 from experiment_processes import terminate_partition
 result=terminate_partition(os.environ["GZ_PARTITION"])
 (out/(side+"-process-cleanup.json")).write_text(__import__("json").dumps(result,indent=2))
