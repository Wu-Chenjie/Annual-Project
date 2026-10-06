"""Bounded view selection and stale utility checks, independent of leases."""
import math


def choose_view_window(options, improvement=1.15):
    """Keep tour order unless another nearby service offers clearly more utility.

    Options contain only feasible tasks from the tour's bounded prefix. Transit
    keeps its order: its usefulness cannot be compared to local ray information.
    """
    if not options:
        return None
    first = options[0]
    if first[1].get('transit'):
        return first
    useful = [o for o in options if not o[1].get('transit')]
    best = max(useful, key=lambda o: o[1]['objective'] /
               (1.+o[1].get('traffic_delay_s', 0.)/1.5))
    def score(option):
        return option[1]['objective']/(1.+option[1].get('traffic_delay_s', 0.)/1.5)
    return best if score(best) > improvement*score(first) else first


def view_gain_retained(remaining, planned, fraction=.2):
    """Replan a collapsed estimate without excluding genuinely small tail views.

    No absolute cell floor: a fresh one-cell task can still finish exploration.
    This is a utility check; it never authorizes motion or marks cells observed.
    """
    return remaining > 0 and remaining >= max(1, math.ceil(fraction*planned))
