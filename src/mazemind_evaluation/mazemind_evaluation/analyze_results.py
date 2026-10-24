"""Aggregate experiment results into CSV, a Markdown summary and comparison plots.

    ros2 run mazemind_evaluation analyze_results results/easy_v1
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Sequence

from mazemind_evaluation.metrics import summarize, wilson_interval

STRATEGY_ORDER = ['random', 'nearest', 'weighted']
CSV_FIELDS = [
    'run_id', 'strategy', 'seed', 'rows', 'cols', 'final_status', 'exit_success',
    'time_to_first_map_s', 'time_to_exit_s', 'exploration_time_s', 'exit_detect_time_s',
    'distance_gt_m', 'distance_odom_m', 'distance_ratio', 'solution_length_m', 'coverage',
    'goals_sent', 'goals_failed', 'goals_preempted', 'goals_resolved_early', 'recoveries',
    'collision_events', 'near_collision_events', 'min_clearance_m',
    'loc_err_mean_m', 'loc_err_max_m', 'exit_detection_error_m', 'startup_retries', 'wall_time_s',
    'reason',
]


def load_runs(campaign_dir: str) -> List[dict]:
    runs = []
    for path in sorted(glob.glob(os.path.join(campaign_dir, 'runs', '*.json'))):
        if '.attempt' in os.path.basename(path):
            continue      # failed startup attempts, kept as evidence; see startup_retries
        with open(path, encoding='utf-8') as fh:
            runs.append(json.load(fh))
    return runs


def flatten(r: dict) -> dict:
    maze = r.get('maze', {})
    loc = r.get('localization_error_m') or {}
    det = r.get('exit_detection') or {}
    row = {k: r.get(k) for k in CSV_FIELDS}
    row.update(seed=maze.get('seed'), rows=maze.get('rows'), cols=maze.get('cols'),
               solution_length_m=maze.get('solution_length_m'),
               loc_err_mean_m=loc.get('mean'), loc_err_max_m=loc.get('max'),
               exit_detection_error_m=det.get('error_m'))
    return row


def write_csv(rows: Sequence[dict], path: str) -> None:
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _fmt(stats: dict, unit: str = '', scale: float = 1.0, digits: int = 1) -> str:
    if not stats['n']:
        return 'n/a'
    if stats['n'] == 1:      # spread is undefined for a single run
        return f"{stats['mean'] * scale:.{digits}f}{unit} (n=1)"
    return (f"{stats['mean'] * scale:.{digits}f} ± {stats['std'] * scale:.{digits}f}{unit} "
            f"(median {stats['median'] * scale:.{digits}f}, n={stats['n']})")


def summarize_by_strategy(rows: Sequence[dict]) -> Dict[str, dict]:
    groups: Dict[str, List[dict]] = defaultdict(list)
    for row in rows:
        groups[row['strategy']].append(row)
    out = {}
    for strategy in sorted(groups, key=lambda s: STRATEGY_ORDER.index(s) if s in STRATEGY_ORDER else 99):
        g = groups[strategy]
        ok = [r for r in g if r['exit_success']]
        lo, hi = wilson_interval(len(ok), len(g))
        statuses: Dict[str, int] = defaultdict(int)
        for r in g:
            statuses[r['final_status']] += 1
        out[strategy] = {
            'n': len(g), 'successes': len(ok), 'success_rate': len(ok) / len(g) if g else 0.0,
            'success_ci95': [lo, hi], 'statuses': dict(statuses),
            # distance / time are reported over successful runs only (a failed run's
            # distance is not comparable: it may have stopped early or timed out)
            'distance_success': summarize([r['distance_gt_m'] for r in ok]),
            'time_to_exit': summarize([r['time_to_exit_s'] for r in ok]),
            'distance_ratio': summarize([r['distance_ratio'] for r in ok]),
            'distance_all': summarize([r['distance_gt_m'] for r in g]),
            'coverage_all': summarize([r['coverage'] for r in g]),
            'goals_failed': summarize([r['goals_failed'] for r in g]),
            'recoveries': summarize([r['recoveries'] for r in g]),
            'collisions': summarize([r['collision_events'] for r in g]),
            'near_collisions': summarize([r['near_collision_events'] for r in g]),
            'exit_detection_error': summarize([r['exit_detection_error_m'] for r in g]),
            'loc_err_mean': summarize([r['loc_err_mean_m'] for r in g]),
            'startup_retries': sum(r['startup_retries'] or 0 for r in g),
        }
    return out


def paired_comparisons(rows: Sequence[dict]) -> List[dict]:
    """Per-seed paired differences between strategies (same maze, both runs successful).

    Uses the Wilcoxon signed-rank test (two-sided) when SciPy is available; with ~10
    seeds this has little power, so effect sizes and win counts matter more than p.
    """
    by = {(r['seed'], r['strategy']): r for r in rows}
    strategies = [s for s in STRATEGY_ORDER if any(r['strategy'] == s for r in rows)]
    out = []
    for i, a in enumerate(strategies):
        for b in strategies[i + 1:]:
            seeds = sorted({r['seed'] for r in rows})
            pairs = [(by[(s, a)], by[(s, b)]) for s in seeds
                     if (s, a) in by and (s, b) in by
                     and by[(s, a)]['exit_success'] and by[(s, b)]['exit_success']]
            if not pairs:
                continue
            entry = {'a': a, 'b': b, 'n': len(pairs)}
            for key, metric in (('distance', 'distance_gt_m'), ('time', 'time_to_exit_s')):
                diffs = [x[metric] - y[metric] for x, y in pairs]
                p = None
                try:
                    from scipy.stats import wilcoxon
                    if any(d != 0 for d in diffs):
                        p = float(wilcoxon(diffs).pvalue)
                except ImportError:
                    pass
                entry[key] = {'mean_diff': sum(diffs) / len(diffs),
                              'a_better': sum(d < 0 for d in diffs), 'p_wilcoxon': p}
            out.append(entry)
    return out


def markdown_paired(paired: Sequence[dict]) -> str:
    if not paired:
        return ''
    lines = ['', '## Paired comparison (same maze seed, both runs reached the exit)', '',
             '| A vs B | n | Distance A − B (m) | A shorter | p | Time A − B (s) | A faster | p |',
             '|---|---|---|---|---|---|---|---|']
    for e in paired:
        d, t = e['distance'], e['time']
        fmt_p = lambda p: 'n/a' if p is None else f'{p:.3f}'  # noqa: E731
        lines.append(f"| {e['a']} vs {e['b']} | {e['n']} | {d['mean_diff']:+.1f} | "
                     f"{d['a_better']}/{e['n']} | {fmt_p(d['p_wilcoxon'])} | {t['mean_diff']:+.1f} | "
                     f"{t['a_better']}/{e['n']} | {fmt_p(t['p_wilcoxon'])} |")
    lines.append('')
    lines.append('p: two-sided Wilcoxon signed-rank test on the per-seed differences.')
    return '\n'.join(lines) + '\n'


def markdown_summary(summary: Dict[str, dict], campaign: str) -> str:
    lines = [f'# Results: {campaign}', '',
             'Generated by `analyze_results` from the run JSON files in `runs/`. '
             'Values are mean ± sample std (median, n). Distance and time-to-exit use only '
             'successful runs; other metrics use all runs.', '',
             '| Strategy | Exit success | Distance to exit (m) | Time to exit (s) | Coverage at end (%) '
             '| Failed goals | Recoveries | Contact events | Near-collisions (<5 cm) |',
             '|---|---|---|---|---|---|---|---|---|']
    for s, d in summary.items():
        lo, hi = d['success_ci95']
        lines.append(
            f"| {s} | {d['successes']}/{d['n']} ({100 * d['success_rate']:.0f} %, "
            f"95 % CI {100 * lo:.0f}–{100 * hi:.0f} %) | {_fmt(d['distance_success'])} "
            f"| {_fmt(d['time_to_exit'])} | {_fmt(d['coverage_all'], scale=100)} "
            f"| {_fmt(d['goals_failed'])} | {_fmt(d['recoveries'])} | {_fmt(d['collisions'])} "
            f"| {_fmt(d['near_collisions'])} |")
    lines += ['', '## Terminal statuses', '']
    for s, d in summary.items():
        lines.append(f"- **{s}**: " + ', '.join(f'{k}: {v}' for k, v in sorted(d['statuses'].items()))
                     + f" (infrastructure startup retries: {d['startup_retries']})")
    lines += ['', 'Startup retries re-run attempts in which the ROS/Nav2 stack never became ready '
              '(the robot never moved); the failed attempts are kept in `runs/*.attempt*.json`.']
    return '\n'.join(lines) + '\n'


def make_plots(rows: Sequence[dict], summary: Dict[str, dict], out_dir: str) -> List[str]:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    strategies = list(summary)
    colors = {'random': '#8c8c8c', 'nearest': '#3a78c2', 'weighted': '#e07b26'}
    paths = []

    def boxplot(ax, key, title, ylabel, only_success=False, scale=1.0):
        data = []
        for s in strategies:
            vals = [r[key] * scale for r in rows if r['strategy'] == s and r[key] is not None
                    and (r['exit_success'] or not only_success)]
            data.append(vals)
        ax.boxplot(data, labels=[f'{s}\n(n={len(v)})' for s, v in zip(strategies, data)],
                   showmeans=True, widths=0.5)
        for i, (s, vals) in enumerate(zip(strategies, data), start=1):
            ax.scatter([i] * len(vals), vals, color=colors.get(s, 'k'), alpha=0.7, zorder=3, s=18)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(axis='y', alpha=0.3)

    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    ax = axes[0, 0]
    rates = [summary[s]['success_rate'] * 100 for s in strategies]
    err = [[100 * (summary[s]['success_rate'] - summary[s]['success_ci95'][0]) for s in strategies],
           [100 * (summary[s]['success_ci95'][1] - summary[s]['success_rate']) for s in strategies]]
    ax.bar(strategies, rates, yerr=err, capsize=6, color=[colors.get(s, 'k') for s in strategies])
    for i, s in enumerate(strategies):
        ax.text(i, 2, f"{summary[s]['successes']}/{summary[s]['n']}", ha='center', color='white',
                fontweight='bold')
    ax.set_ylim(0, 105)
    ax.set_title('Exit success rate (95 % Wilson CI)')
    ax.set_ylabel('%')
    boxplot(axes[0, 1], 'distance_gt_m', 'Distance travelled to exit (successful runs)', 'm', True)
    boxplot(axes[0, 2], 'time_to_exit_s', 'Time to exit (successful runs)', 's (sim)', True)
    boxplot(axes[1, 0], 'coverage', 'Map coverage at end of run', '%', scale=100)
    boxplot(axes[1, 1], 'goals_failed', 'Failed navigation goals per run', 'count')
    boxplot(axes[1, 2], 'recoveries', 'Nav2 recovery behaviours per run', 'count')
    fig.suptitle(f'MazeMind strategy comparison ({len(rows)} runs)', fontsize=14)
    fig.tight_layout()
    path = os.path.join(out_dir, 'comparison.png')
    fig.savefig(path, dpi=120)
    plt.close(fig)
    paths.append(path)

    # Paired view: distance per seed, one line per strategy
    seeds = sorted({r['seed'] for r in rows if r['seed'] is not None})
    if seeds:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for s in strategies:
            ys = []
            for seed in seeds:
                match = [r for r in rows if r['strategy'] == s and r['seed'] == seed]
                ys.append(match[0]['distance_gt_m'] if match and match[0]['exit_success'] else None)
            ax.plot(seeds, [y if y is not None else float('nan') for y in ys], 'o-',
                    color=colors.get(s, 'k'), label=s)
            for x, y in zip(seeds, ys):
                if y is None:
                    ax.scatter([x], [0], marker='x', color=colors.get(s, 'k'))
        ax.set_xlabel('maze seed')
        ax.set_ylabel('distance to exit (m)')
        ax.set_title('Distance to exit per maze (x on axis = run did not reach the exit)')
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        path = os.path.join(out_dir, 'distance_per_seed.png')
        fig.savefig(path, dpi=120)
        plt.close(fig)
        paths.append(path)
    return paths


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign_dir')
    args = p.parse_args(argv)
    runs = load_runs(args.campaign_dir)
    if not runs:
        print(f'no runs found in {args.campaign_dir}/runs', file=sys.stderr)
        return 1
    rows = [flatten(r) for r in runs]
    write_csv(rows, os.path.join(args.campaign_dir, 'results.csv'))
    summary = summarize_by_strategy(rows)
    with open(os.path.join(args.campaign_dir, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, indent=2)
    paired = paired_comparisons(rows)
    with open(os.path.join(args.campaign_dir, 'paired.json'), 'w', encoding='utf-8') as fh:
        json.dump(paired, fh, indent=2)
    md = markdown_summary(summary, os.path.basename(os.path.normpath(args.campaign_dir)))
    md += markdown_paired(paired)
    with open(os.path.join(args.campaign_dir, 'summary.md'), 'w', encoding='utf-8') as fh:
        fh.write(md)
    plots = make_plots(rows, summary, args.campaign_dir)
    print(md)
    print('wrote results.csv, summary.json, summary.md, ' + ', '.join(os.path.basename(x) for x in plots))
    return 0


if __name__ == '__main__':
    sys.exit(main())
