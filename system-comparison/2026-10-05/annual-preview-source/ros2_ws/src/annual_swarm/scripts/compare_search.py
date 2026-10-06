#!/usr/bin/env python3
"""Sequential real-Gazebo search comparison; failures remain explicit in results."""
import argparse,json,subprocess,sys
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--maps',nargs='+',required=True)
    p.add_argument('--policies',nargs='+',default=['nearest_frontier','graph_voronoi','gvp_pairwise']);p.add_argument('--timeout',type=float,default=900)
    args=p.parse_args();root=Path(args.output_dir).resolve();root.mkdir(parents=True,exist_ok=False);results=[]
    for map_file in args.maps:
        for policy in args.policies:
            output=root/(Path(map_file).stem+'_'+policy)
            command=[sys.executable,str(Path(__file__).with_name('search_smoke.py')),'--map',map_file,'--policy',policy,'--pause-after','50','--timeout',str(args.timeout),'--output-dir',str(output)]
            print('RUN',output.name,flush=True)
            with (root/(output.name+'.log')).open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            summary=json.loads((output/'summary.json').read_text()) if (output/'summary.json').exists() else {}
            results.append(dict(map=map_file,policy=policy,passed=result.returncode==0,summary=str(output/'summary.json'),
                coverage=summary.get('coverage'),t90=summary.get('t90'),t95=summary.get('t95'),contacts=summary.get('contacts_after_takeoff'),min_separation_m=summary.get('min_separation_m')))
            (root/'results.json').write_text(json.dumps(results,indent=2));print(results[-1],flush=True)
    if not all(x['passed'] for x in results):raise SystemExit('One or more runs failed; inspect retained logs')
if __name__=='__main__':main()
