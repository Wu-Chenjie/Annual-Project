"""An abandoned experiment cannot pollute the next paired run."""
import os
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import experiment_processes as processes


def test_process_partition_is_exact_and_excludes_self_and_zombies(tmp_path):
    for pid,partition,state in [(100,'annual_todo_a','S'),(101,'annual_todo_ab','S'),
                                (102,'annual_todo_a','Z'),(os.getpid(),'annual_todo_a','S')]:
        p=tmp_path/str(pid);p.mkdir();(p/'environ').write_bytes(('GZ_PARTITION='+partition+'\0').encode())
        (p/'status').write_text('State:\t'+state+' (example)\n')
    assert processes.partition_processes('annual_todo_a',tmp_path)=={100}


def test_pid_reused_by_another_partition_is_never_signaled(monkeypatch):
    scans=iter([{100},set(),set(),set()]);signals=[]
    monkeypatch.setattr(processes,'partition_processes',lambda _:next(scans))
    monkeypatch.setattr(processes.os,'kill',lambda pid,sig:signals.append((pid,sig)))
    monkeypatch.setattr(processes.time,'sleep',lambda _:None)
    assert processes.terminate_partition('annual_todo_a')['remaining']==[]
    assert not signals
