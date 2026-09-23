#!/usr/bin/env python3
"""Plot measured evidence; never synthesize or substitute a Gazebo flight video."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def summarize(root, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root, output = Path(root), Path(output); output.mkdir(parents=True, exist_ok=True)
    summary = json.loads((root/'summary.json').read_text()); start = summary['start_time'] or 0.
    coverage = np.asarray(json.loads((root/'coverage.json').read_text()))
    rows = np.genfromtxt(root/'trajectory.csv', delimiter=',', names=True)
    tracking = {i: [] for i in range(summary['fleet_size'])}
    with (root/'tracking.csv').open() as stream:
        for row in csv.DictReader(stream):
            tracking[int(row['drone'])].append(float(row['error_m']))
    transactions = {}; applied_by = {}
    with (root/'peer_states.jsonl').open() as stream:
        for line in stream:
            state = json.loads(line); tx = state.get('pair_transaction')
            if tx and tx['phase'] == 'applied':
                transactions[tx['token']] = tx
                applied_by.setdefault(tx['token'], set()).add(state['drone'])
    transactions = {token: tx for token, tx in transactions.items()
                    if {tx['leader'], tx['follower']} <= applied_by[token]}
    colors = ['#e85d35', '#159c6d', '#3c82c9']
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    ax = axes[0, 0]; ax.plot(coverage[:, 0]-start, 100*coverage[:, 1], color='#20465e', lw=2)
    ax.axhline(95, ls='--', color='#6c777d', lw=1); ax.set(xlabel='Simulation time (s)', ylabel='Observed free-space coverage (%)', ylim=(0, 101))
    if summary.get('network_started'):
        t = summary['network_started']-start; ax.axvspan(t, t+18, color='#d49332', alpha=.2, label='Planning link outage')
    if summary.get('dynamic_trial'):
        t = summary['dynamic_trial']['start']-start; ax.axvspan(t, t+22, color='#c7474c', alpha=.2, label='Crossing obstacle')
    if summary.get('restart_trial'):
        ax.axvline(summary['restart_trial']['start']-start, color='#875eb0', ls=':', label='Agent restart')
    ax.legend(loc='lower right', fontsize=8)
    for i, color in enumerate(colors):
        own = rows[rows['drone'] == i]; own = own[own['time'] >= start]
        axes[0, 1].plot(own['time'][::20]-start, own['z'][::20], color=color, label=f'UAV {i}', lw=1)
        errors = np.sort(tracking[i])
        if len(errors):
            axes[1, 0].plot(errors, np.arange(1, len(errors)+1)/len(errors), color=color, label=f'UAV {i}')
    axes[0, 1].set(xlabel='Simulation time (s)', ylabel='Actual altitude (m)'); axes[0, 1].legend()
    axes[1, 0].set(xlabel='Truth/reference tracking error (m)', ylabel='Empirical cumulative probability'); axes[1, 0].legend()
    finite = [tx for tx in transactions.values() if tx.get('before') is not None and np.isfinite(tx['before'])]
    x = np.arange(len(finite)); ax = axes[1, 1]
    ax.bar(x-.18, [tx['before'] for tx in finite], width=.36, label='Initial allocation', color='#a6b3bc')
    ax.bar(x+.18, [tx['after'] for tx in finite], width=.36, label='Committed pair allocation', color='#237885')
    ax.set(xlabel='Unique pair transaction', ylabel='Window route objective (s)'); ax.legend(fontsize=8)
    fig.suptitle('Fused exploration: measured flight and in-pipeline allocation comparison', fontsize=14)
    fig.savefig(output/'fusion-evidence.png', dpi=180); fig.savefig(output/'fusion-evidence.svg'); plt.close(fig)
    (output/'pair-transactions.json').write_text(json.dumps(list(transactions.values()), indent=2)+'\n')
    return dict(unique_committed_pair_transactions=len(transactions), comparison_scope='Same pair window before/after fused refinement; initial allocation can violate capacity. Not an end-to-end RACER/GVP benchmark.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('directory'); p.add_argument('--output', required=True)
    a = p.parse_args(); print(json.dumps(summarize(a.directory, a.output), indent=2))
