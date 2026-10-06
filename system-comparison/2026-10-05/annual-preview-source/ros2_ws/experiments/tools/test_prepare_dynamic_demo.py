"""The diagnostic recorder must leave the physical Gazebo obstacle spawned."""
from pathlib import Path
import shutil

from prepare_dynamic_demo import prepare


def test_nofault_recorder_keeps_cylinder_and_uses_external_schedule(tmp_path):
    harness=tmp_path/'harness';harness.mkdir()
    source=Path(__file__).resolve().parents[2]/'src/annual_swarm/scripts/record_todo_demo.py'
    shutil.copy2(source,harness/source.name)
    out=tmp_path/'external'
    prepare(harness,out)
    recorder=(out/'record_todo_demo.py').read_text()
    assert "'dynamic_obstacle:=true'" in recorder
    assert "'dynamic_obstacle:=false'" not in recorder
    assert "dynamic_backup_fault.launch.py" in recorder
    assert "pause_after:=0" in recorder and "network_after:=0" in recorder
