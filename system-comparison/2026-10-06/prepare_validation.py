from pathlib import Path
import hashlib, json, shutil
root=Path(__file__).resolve().parent
previous=root.parent/'2026-10-05'
harness=('common.launch.py','common_mapper.py','bridge_ros2.py','supervisor.py','experiment_processes.py','motion_envelope.py','geometry_check.py','replay_coverage.py','map.json')
for name in harness:
    shutil.copy2(previous/name,root/name)
old=json.loads((previous/'protocol.json').read_text())
old.update(status='fixed_before_followup_flights', systems=['Annual baseline view-preview','Annual handoff-service-corridor'],
    repetitions=1,trial_seed=900,scored_runs=['baseline-900','candidate-900'],flight_order=['baseline-900','candidate-900'],
    limit_wall_seconds=1200, comparison_to_previous='Fresh baseline and candidate pair on one container/image; historical flights are not paired here.',
    acceptance=dict(T95='candidate < baseline',low_speed='candidate < baseline; measured v < 0.1m/s after t=20',
        distance='candidate <= 1.1*baseline',planning='report P95; do not hide regression',
        safety='zero contacts and geometric hits; native execution authorization, exact reference derivatives and raw sensor coverage audits pass'),
    scope='Two development runs only; no formal 70-run matrix or statistical acceptance.',
    frozen_harness={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in harness})
for name in ('annual_source_archive_sha256','annual_source_files','prior_test'):old.pop(name,None)
(root/'protocol.json').write_text(json.dumps(old,indent=2)+'\n')
