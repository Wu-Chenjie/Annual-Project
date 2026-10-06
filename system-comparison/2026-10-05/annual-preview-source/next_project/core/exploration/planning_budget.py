"""Clock-domain explicit request fences shared by worker and ROS integration."""
from dataclasses import dataclass
import time


@dataclass(frozen=True)
class PlanningRequest:
    identifier: str
    incarnation: str
    epoch: int
    map_version: int
    submitted_sim: float
    submitted_wall: float
    deadline_s: float = 12.
    max_sim_age_s: float = 15.

    def rejection(self, incarnation, epoch, simulation_time, wall_time=None):
        wall_time = time.monotonic() if wall_time is None else wall_time
        if incarnation != self.incarnation: return 'incarnation_changed'
        if epoch != self.epoch: return 'epoch_changed'
        if wall_time-self.submitted_wall > self.deadline_s: return 'wall_deadline'
        if not -.1 <= simulation_time-self.submitted_sim <= self.max_sim_age_s: return 'simulation_age'
        return None


def abort_worker(worker):
    """Terminate a hung private planner, never a ROS/control process."""
    processes = list((getattr(worker, '_processes', None) or {}).values())
    worker.shutdown(wait=False, cancel_futures=True)
    for process in processes:
        if process.is_alive(): process.terminate()
    for process in processes:
        process.join(timeout=.1)
        if process.is_alive(): process.kill()
