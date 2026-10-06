import hashlib,json,sys
from pathlib import Path
study=Path('/study');root=Path('/workspace');tag=sys.argv[1]
manifest=json.loads((study/(tag+'-source-manifest.json')).read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
errors=[]
for rel,digest in manifest['files'].items():
    p=root/rel
    if not p.exists() or sha(p)!=digest:errors.append('source:'+rel)
installed={};prefix=root/'ros2_ws/install/annual_swarm'
for folder,dest in [('next_project/core','lib/annual_swarm/legacy/core'),('next_project/maps','share/annual_swarm/maps'),('ros2_ws/src/annual_swarm/config','share/annual_swarm/config'),('ros2_ws/src/annual_swarm/launch','share/annual_swarm/launch')]:
    for source in (root/folder).rglob('*'):
        if not source.is_file() or '__pycache__' in source.parts:continue
        target=prefix/dest/source.relative_to(root/folder)
        if not target.exists() or sha(target)!=sha(source):errors.append('install:'+str(target))
        else:installed[str(target.relative_to(prefix))]=sha(target)
for target in (prefix/'lib/annual_swarm').glob('*.py'):
    source=root/'ros2_ws/src/annual_swarm/scripts'/target.name
    if not source.exists() or sha(source)!=sha(target):errors.append('install:'+str(target))
    else:installed[str(target.relative_to(prefix))]=sha(target)
binaries={str(p.relative_to(prefix)):sha(p) for p in (prefix/'lib/annual_swarm/controller_node',prefix/'lib/libannual_motor_system.so')}
report=dict(passed=not errors,errors=errors,source_files=len(manifest['files']),installed_files=len(installed),installed_sha256=installed,physics_binaries=binaries)
(study/(tag+'-runtime-verification.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('passed','errors','source_files','installed_files','physics_binaries')},indent=2))
raise SystemExit(0 if report['passed'] else 1)
