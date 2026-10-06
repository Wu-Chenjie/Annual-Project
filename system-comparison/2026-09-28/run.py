import subprocess,os,sys,time,threading,json,shlex
from pathlib import Path
root=Path(__file__).resolve().parent
mode=sys.argv[1];name=sys.argv[2];limit=sys.argv[3] if len(sys.argv)>3 else '300';out=root/name
out.mkdir(exist_ok=False)
procs=[];files=[];counts={}
def run(side):
 ctx='colima-annual-fusion' if side=='ros2' else 'colima-racer-gvp';container='comparison-'+side+'-0928'
 env={'COMPARISON_MODE':mode,'COMPARISON_OUTPUT':'/comparison/'+name,'COMPARISON_LIMIT':limit,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','LP_NUM_THREADS':'2','ANNUAL_EXPERIMENT_SEED':'900','ROS_DOMAIN_ID':'73','GZ_PARTITION':'comparison_'+name,'ROS_MASTER_URI':'http://localhost:11328','ROS_HOSTNAME':'localhost','PYTHONDONTWRITEBYTECODE':'1','PYTHONPYCACHEPREFIX':'/tmp/comparison-pycache-'+name}
 setup='/workspace/ros2_ws/install/setup.bash' if side=='ros2' else '/work/racer_ws/devel/setup.bash' if mode=='racer' else '/comparison/gvp_ws/devel/setup.bash'
 cmd=['docker','--context',ctx,'exec','-i']
 for k,v in env.items():cmd+=['-e',k+'='+v]
 cmd += [container,'bash','-lc','source '+shlex.quote(setup)+' && exec python3 -u /comparison/supervisor.py '+side+' '+mode]
 f=(out/(side+'-supervisor.log')).open('wb');files.append(f)
 p=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=f,bufsize=65536);procs.append(p);return p

def pump(src,dst,key):
 counts[key]=0
 for line in iter(src.stdout.readline,b''):
  if not line.startswith(b'{'):continue
  counts[key]+=1
  if dst is not None:
   try:dst.stdin.write(line);dst.stdin.flush()
   except (BrokenPipeError,ValueError,OSError):break

p1=run('ros1') if mode!='annual' else None
if p1:time.sleep(5)
p2=run('ros2')
threads=[threading.Thread(target=pump,args=(p2,p1,'ros2_to_ros1'),daemon=True)]
if p1:threads.append(threading.Thread(target=pump,args=(p1,p2,'ros1_to_ros2'),daemon=True))
for t in threads:t.start()
start=time.monotonic();last=0
try:
 while p2.poll() is None:
  if (out/'result.json').exists() or (out/'STOP').exists():break
  elapsed=time.monotonic()-start
  if elapsed-last>=20:
   last=elapsed
   f=out/'progress.json'
   if f.exists():
    d=json.loads(f.read_text());print(json.dumps({k:d.get(k) for k in ['mode','t','coverage','positions','native_seen','contacts','max_measured_speed_mps']}),flush=True)
   else:print('waiting for simulator',round(elapsed),flush=True)
  if elapsed>min(2400,120+float(limit)*15):(out/'STOP').touch();break
  time.sleep(.5)
finally:
 (out/'STOP').touch()
 for p in procs:
  try:p.wait(timeout=20)
  except subprocess.TimeoutExpired:p.terminate()
 (out/'transport-counts.json').write_text(json.dumps(dict(counts=counts,exit_codes=[p.poll() for p in procs]),indent=2))
 for f in files:f.close()
print('finished',name,'result', (out/'result.json').exists(),flush=True)
