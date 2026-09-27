"""Exact ownership and start-time fences prevent signals to unrelated processes."""
import signal
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent))
import native_fault_probe as probe


def test_pid_reuse_or_argument_change_prevents_signal(monkeypatch):
    original=dict(pid=77,start_ticks='100',arguments=['python3','owned.py'])
    sent=[];monkeypatch.setattr(probe.os,'kill',lambda *args:sent.append(args))
    for changed in (None,dict(original,start_ticks='101'),dict(original,arguments=['unrelated'])):
        monkeypatch.setattr(probe,'process',lambda *args,value=changed:value)
        assert not probe.send(original,'exact_partition',signal.SIGSTOP)
    assert not sent
    monkeypatch.setattr(probe,'process',lambda *args:original)
    assert probe.send(original,'exact_partition',signal.SIGSTOP)
    assert sent==[(77,signal.SIGSTOP)]


def test_only_correct_namespace_and_spawn_child_are_selected(monkeypatch,tmp_path):
    script=str(tmp_path/'ros2_ws/install/annual_swarm/lib/annual_swarm/decentralized_agent_node.py')
    node=dict(pid=10,parent=1,arguments=['python3',script,'__ns:=/drone_0'])
    worker=dict(pid=11,parent=10,arguments=['python3','-c','from multiprocessing.spawn import spawn_main'])
    tracker=dict(pid=12,parent=10,arguments=['python3','-c','from multiprocessing.resource_tracker import main'])
    other=dict(pid=21,parent=20,arguments=worker['arguments'])
    monkeypatch.setattr(probe,'processes',lambda partition:[node,worker,tracker,other])
    assert probe.target('owned',tmp_path,0,True)==worker
    assert probe.target('owned',tmp_path,1,True) is None
    assert probe.target('owned',tmp_path/'different',0,True) is None
