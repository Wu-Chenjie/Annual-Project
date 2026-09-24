#!/usr/bin/env python3
"""Record exactly one baseline attempt, including incomplete/failed outcomes."""
import argparse,hashlib,json,os,signal,subprocess,sys,tarfile,time,uuid
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--simulation-limit',type=float,default=2400.);p.add_argument('--wall-limit',type=float,default=14400.);p.add_argument('--speed',type=float,default=24.)
    args=p.parse_args();out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    here=Path(__file__).resolve().parent;root=here.parents[2]
    files=[]
    for folder in ['next_project/core','next_project/cpp','next_project/maps','ros2_ws/src/annual_swarm','ros2_ws/benchmarks/early_policy_3d']:
        files.extend(x for x in (root/folder).rglob('*') if x.is_file() and not {'__pycache__','.pytest_cache'} & set(x.parts))
    manifest={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)}
    (out/'source-manifest.json').write_text(json.dumps(manifest,indent=2))
    with tarfile.open(out/'recorded-source.tar.gz','w:gz') as tar:
        for f in files:tar.add(f,arcname=f.relative_to(root))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LP_NUM_THREADS='2',DISPLAY=':99',LIBGL_ALWAYS_SOFTWARE='1',QT_X11_NO_MITSHM='1',PYTHONDONTWRITEBYTECODE='1',
        GZ_PARTITION='annual_early_'+uuid.uuid4().hex,ROS_DOMAIN_ID=str(20+int(uuid.uuid4().hex[:4],16)%180))
    env['PYTHONPYCACHEPREFIX']='/tmp/'+env['GZ_PARTITION']+'_pycache'
    (out/'transport.json').write_text(json.dumps({k:env[k] for k in ('GZ_PARTITION','ROS_DOMAIN_ID')},indent=2))
    (out/'run-configuration.json').write_text(json.dumps(dict(historical_commit='7136591',runtime_base='620d1aa',arguments=vars(args),cpu_count=os.cpu_count(),
        comparison='Same 3D scene, sensing, estimator, coverage and trajectory constraints. Central map sharing and authorization retained. Restarted endpoint is a command client, not a per-UAV planner.'),indent=2))
    servers=[]
    if not Path('/tmp/.X11-unix/X99').exists():
        servers.append(subprocess.Popen(['Xvfb',':99','-screen','0','1920x1080x24','-ac'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
        for _ in range(100):
            if Path('/tmp/.X11-unix/X99').exists():break
            time.sleep(.1)
        servers.append(subprocess.Popen(['openbox'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
    log=(out/'launch.log').open('w');proc=None;recorder=None;result={};last=None
    start=time.monotonic()
    try:
        recorder=subprocess.Popen(['ffmpeg','-y','-loglevel','warning','-f','x11grab','-framerate','10','-video_size','1920x1080','-i',env['DISPLAY'],'-c:v','libx264','-preset','ultrafast','-crf','23','-threads','1','-pix_fmt','yuv420p',str(out/'gazebo-rviz-raw.mp4')],env=env,stdout=log,stderr=log)
        proc=subprocess.Popen(['ros2','launch',str(here/'benchmark.launch.py'),f'output_dir:={out}'],env=env,stdout=log,stderr=log,start_new_session=True)
        arranged=set();holding=None
        while time.monotonic()-start<args.wall_limit:
            if proc.poll() is not None:raise RuntimeError('Launch exited')
            if len(arranged)<2:
                for name,selector,geo in [('gazebo',['--name','^Gazebo Sim$'],[0,40,800,1000]),('rviz',['--class','rviz'],[800,40,1120,1000])]:
                    if name in arranged:continue
                    found=subprocess.run(['xdotool','search','--onlyvisible',*selector],env=env,capture_output=True,text=True)
                    if found.returncode==0 and found.stdout.strip():
                        wid=found.stdout.splitlines()[-1];subprocess.run(['xdotool','windowmove',wid,*map(str,geo[:2]),'windowsize',wid,*map(str,geo[2:])],env=env,check=True);arranged.add(name)
                if time.monotonic()-start>120 and len(arranged)!=2:raise RuntimeError('Native GUI windows missing')
            file=out/'summary.json'
            if file.exists():
                try:s=json.loads(file.read_text())
                except (json.JSONDecodeError,FileNotFoundError):time.sleep(.2);continue
                last=s
                if time.time()-file.stat().st_mtime>90:raise RuntimeError('Telemetry stopped')
                if s['status']=='FAILED':result=dict(outcome='FAILED',reason=s.get('failure_reason'));break
                if s['status']=='COMPLETE':
                    if holding is None:holding=s['simulation_time']
                    if s['simulation_time']-holding>=3.:result=dict(outcome='COMPLETE');break
                if s.get('start_time') is not None and s['simulation_time']-s['start_time']>=args.simulation_limit:
                    result=dict(outcome='SIMULATION_TIME_LIMIT',reason='Did not reach 95% within the preregistered limit');break
                if s['simulation_time']>50 and len(s.get('agent_ages',{}))!=3:raise RuntimeError('A command client failed to start')
                if max(s.get('agent_ages',{}).values(),default=0)>15.:raise RuntimeError('Command client heartbeat stopped')
            elif time.monotonic()-start>180:raise RuntimeError('No experiment telemetry')
            time.sleep(1)
        if not result:result=dict(outcome='WALL_TIME_LIMIT')
    except Exception as exc:
        result=dict(outcome='INFRASTRUCTURE_FAILURE',reason=repr(exc))
    finally:
        result.update(wall_elapsed_s=time.monotonic()-start,last_coverage=last.get('coverage') if last else None,last_simulation_time=last.get('simulation_time') if last else None)
        (out/'run-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        if recorder and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT);recorder.wait(timeout=30)
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        # Limit cleanup to the unique transport partition of this attempt.
        marker=('GZ_PARTITION='+env['GZ_PARTITION']).encode()
        owned=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                if marker in (entry/'environ').read_bytes().split(b'\0'):owned.append(int(entry.name))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        for sig in (signal.SIGTERM,signal.SIGKILL):
            for pid in owned:
                try:os.kill(pid,sig)
                except ProcessLookupError:pass
            time.sleep(.3)
        for server in reversed(servers):server.terminate()
        log.close()
    final=out/'early-policy-3d.mp4'
    title=f'EARLY CENTRALIZED POLICY + COMMON 3D GAZEBO SENSORS | {args.speed:g}x recording speed'
    filters=f"setpts=PTS/{args.speed},drawbox=x=0:y=0:w=iw:h=38:color=0x121923:t=fill,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='{title}':x=16:y=9:fontsize=20:fontcolor=white"
    subprocess.run(['ffmpeg','-y','-loglevel','warning','-i',str(out/'gazebo-rviz-raw.mp4'),'-vf',filters,'-r','30','-c:v','libx264','-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(final)],check=True)
    probe=subprocess.check_output(['ffprobe','-v','quiet','-show_format','-show_streams','-of','json',str(final)],text=True)
    (out/'video-probe.json').write_text(probe)
    print(final,flush=True)
if __name__=='__main__':main()
