import hashlib,json
from pathlib import Path

root=Path('/tmp/annual-integrated-20260924')
out=Path('/tmp/annual-priority-integrated-v1')
prefix=root/'ros2_ws/install/annual_swarm'
manifest=json.loads((out/'source-manifest.json').read_text())['files']
verified={}
for installed in prefix.rglob('*'):
    if not installed.is_file() or '__pycache__' in installed.parts:
        continue
    relative=installed.relative_to(prefix).as_posix()
    source=None
    if relative.startswith('lib/annual_swarm/legacy/core/'):
        source='next_project/'+relative.removeprefix('lib/annual_swarm/legacy/')
    elif relative.startswith('lib/annual_swarm/') and installed.suffix=='.py':
        source='ros2_ws/src/annual_swarm/scripts/'+installed.name
    elif relative.startswith('share/annual_swarm/'):
        asset=relative.removeprefix('share/annual_swarm/')
        if asset=='launch/scene.py':
            source='ros2_ws/src/annual_swarm/scripts/scene.py'
        elif asset.startswith('maps/'):
            source='next_project/'+asset
        elif asset.split('/')[0] in ('launch','config','models','worlds'):
            source='ros2_ws/src/annual_swarm/'+asset
    if source is not None:
        assert source in manifest,source
        actual=hashlib.sha256(installed.read_bytes()).hexdigest()
        assert actual==manifest[source], (source,'installed hash differs')
        verified[relative]=dict(source=source,sha256=actual)
assert any(n.endswith('exploration/priority.py') for n in verified)
assert any(n.endswith('config/exploration_priority.yaml') for n in verified)
assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in manifest.items())
result=dict(passed=True,verified_installed_files=len(verified),frozen_source_files=len(manifest),files=verified)
(out/'installed-source-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='files'}))
