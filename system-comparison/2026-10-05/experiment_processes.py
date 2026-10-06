"""Clean up only processes in one explicitly owned Gazebo transport partition."""
import os
from pathlib import Path
import signal
import time


def partition_processes(partition, proc_root=Path('/proc')):
    marker=('GZ_PARTITION='+partition).encode();owned=set()
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
        try:
            if marker in (entry/'environ').read_bytes().split(b'\0'):
                if 'State:\tZ' not in (entry/'status').read_text():owned.add(int(entry.name))
        except (OSError,PermissionError):pass
    return owned


def terminate_partition(partition, grace_s=.5):
    terminated=[]
    for sig in (signal.SIGTERM,signal.SIGKILL):
        for pid in partition_processes(partition):
            # Recheck ownership before signaling: the PID may have been reused.
            if pid not in partition_processes(partition):continue
            try:os.kill(pid,sig);terminated.append(dict(pid=pid,signal=int(sig)))
            except ProcessLookupError:pass
        time.sleep(grace_s)
    return dict(partition=partition,signals=terminated,remaining=sorted(partition_processes(partition)))
