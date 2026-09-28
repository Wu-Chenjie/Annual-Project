"""Safety and causality preconditions for the external physical fault."""
import numpy as np
import pytest
from dynamic_backup_probe import select_point, last_pool


class Line:
    duration = 20.
    def __init__(self, points):
        self.points = np.asarray(points, float)
    def sample(self, time):
        return (self.points[0]+(self.points[1]-self.points[0])*time/self.duration,)
    def path(self, dt):
        return self.points[0]+(self.points[1]-self.points[0])*np.linspace(0., 1., 100)[:, None]


def inputs():
    return dict(active=Line([[3., 3., 1.5], [9., 3., 1.5]]), progress=0.,
        reserves=[('reserve', Line([[3., 3., 1.5], [3., 9., 1.5]]))], position=[3., 3., 1.5],
        yaw=0., fleet_positions=[[3., 3., 1.5], [10., 10., 1.5], [12., 12., 1.5]],
        occupied_xy=np.empty((0, 2)), bounds=np.array([[0., 0., 0.], [20., 20., 4.]]))


def test_visible_active_curve_has_a_genuinely_unblocked_reserve():
    point = select_point(**inputs())
    assert point and point['reserve_candidate'] == 'reserve'
    assert 2.5 <= point['visible_distance_m'] <= 3.8
    assert point['reserve_center_clearance_m'] > 1.45


def test_external_fault_accepts_ahead_points_inside_the_real_lidar_field():
    values=inputs();values['yaw']=-.95
    assert select_point(**values) is not None
    values['yaw']=-1.08
    assert select_point(**values) is None


@pytest.mark.parametrize('fault', ['shared_route', 'wall', 'uav', 'behind', 'finished'])
def test_unsafe_or_nondivergent_fault_is_not_injected(fault):
    values = inputs()
    if fault == 'shared_route': values['reserves'] = [('same', values['active'])]
    elif fault == 'wall': values['occupied_xy'] = np.column_stack([np.arange(5., 8., .1), np.full(30, 3.)])
    elif fault == 'uav': values['fleet_positions'].append([6., 3., 1.5])
    elif fault == 'behind': values['yaw'] = np.pi
    elif fault == 'finished': values['progress'] = 17.
    assert select_point(**values) is None


def test_pool_reader_uses_the_executing_token_and_tolerates_partial_tail(tmp_path):
    file = tmp_path/'candidates.jsonl'
    file.write_text('{"token":"actual","epoch":1}\n{"token":"pending","epoch":2}\n{"token":')
    assert last_pool(file, 'actual')['epoch'] == 1
    assert last_pool(file, 'unknown') is None


def test_replacement_distance_is_reported_only_for_the_same_destination():
    from audit_dynamic_backup import reference_distance_change
    from core.planning.continuous_trajectory import interpolate
    old=interpolate(np.array([[2.,2.,1.5],[6.,2.,1.5]]),[20.],0.,0.)
    progress=8.;position=old.sample(progress)[0]
    direct=interpolate(np.array([position,[6.,2.,1.5]]),[16.],0.,0.)
    detour=interpolate(np.array([position,[4.,4.,1.5],[6.,2.,1.5]]),[12.,12.],0.,0.)
    other=interpolate(np.array([position,[6.,5.,1.5]]),[16.],0.,0.)
    assert abs(reference_distance_change(old,progress,direct)['reference_distance_change_m'])<1e-9
    assert reference_distance_change(old,progress,detour)['reference_distance_change_m']>1.
    assert reference_distance_change(old,progress,other)['reference_distance_change_m'] is None
    with pytest.raises(ValueError):reference_distance_change(old,21.,direct)
