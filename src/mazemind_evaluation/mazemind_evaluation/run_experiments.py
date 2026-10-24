"""Automated experiment campaigns.

Every run launches a fresh simulation (the most reliable "reset": no state
leaks between runs), waits for the evaluator to write its result and shut
the launch down, then cleans up the whole process group.

    ros2 run mazemind_evaluation run_experiments --campaign easy_v1 \
        --difficulty easy --seeds 0 1 2 3 4 --strategies random nearest weighted

Runs whose result file already exists are skipped, so an interrupted
campaign can be resumed with the same command.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from typing import List, Optional, Sequence


def run_id_for(difficulty: str, seed: int, strategy: str) -> str:
    return f'{difficulty}_s{seed:03d}_{strategy}'


def _kill_group(proc: subprocess.Popen, grace: float = 15.0) -> None:
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def _sweep_strays() -> None:
    """Gazebo can outlive its parent if the launch is killed hard. Remove only
    simulator processes running a MazeMind-generated world."""
    subprocess.run(['pkill', '-f', r'ign gazebo.*mazemind/worlds/maze_'], check=False)


def is_startup_failure(result: dict) -> bool:
    """The stack never became ready (e.g. Nav2 lifecycle bringup hung): infrastructure, not algorithm."""
    if result.get('goals_sent', 0) != 0:
        return False
    if result.get('final_status') != 'SIMULATION_ERROR':
        return False
    reason = result.get('reason', '')
    return 'startup timeout' in reason or 'no evaluator result' in reason


def run_one(args: argparse.Namespace, seed: int, strategy: str, out_dir: str) -> dict:
    """Run with up to ``args.startup_retries`` retries for startup failures.

    Failed attempts are kept as ``<run_id>.attemptN.json`` / ``.log`` and counted in
    the final result's ``startup_retries`` field, so retries are visible in the data.
    """
    run_id = run_id_for(args.difficulty, seed, strategy)
    result_path = os.path.join(out_dir, 'runs', f'{run_id}.json')
    if os.path.exists(result_path) and not args.force:
        print(f'[skip] {run_id} (result exists)')
        with open(result_path, encoding='utf-8') as fh:
            return json.load(fh)
    for attempt in range(args.startup_retries + 1):
        result = run_attempt(args, seed, strategy, out_dir)
        result['startup_retries'] = attempt
        if not is_startup_failure(result) or attempt == args.startup_retries:
            break
        print(f'[retry] {run_id}: startup failure ({result.get("reason", "")[:80]}); '
              f'keeping attempt {attempt + 1} as evidence')
        for ext, folder in (('.json', 'runs'), ('.log', 'logs')):
            src = os.path.join(out_dir, folder, run_id + ext)
            if os.path.exists(src):
                os.replace(src, os.path.join(out_dir, folder, f'{run_id}.attempt{attempt + 1}{ext}'))
    with open(result_path, 'w', encoding='utf-8') as fh:
        json.dump(result, fh, indent=2)
    return result


def run_attempt(args: argparse.Namespace, seed: int, strategy: str, out_dir: str) -> dict:
    run_id = run_id_for(args.difficulty, seed, strategy)
    result_path = os.path.join(out_dir, 'runs', f'{run_id}.json')

    os.makedirs(os.path.join(out_dir, 'logs'), exist_ok=True)
    log_path = os.path.join(out_dir, 'logs', f'{run_id}.log')
    partition = f'mazemind_{os.getpid()}_{run_id}'
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(args.domain_id)
    env['IGN_PARTITION'] = partition
    env['GZ_PARTITION'] = partition
    cmd = ['ros2', 'launch', 'mazemind_bringup', 'mazemind.launch.py',
           f'difficulty:={args.difficulty}', f'seed:={seed}', f'strategy:={strategy}',
           'headless:=true', 'rviz:=false', 'shutdown_on_finish:=true',
           f'time_limit:={args.time_limit}', f'run_id:={run_id}', f'results_dir:={out_dir}',
           f'render_mode:={args.render_mode}']
    if args.rows:
        cmd.append(f'rows:={args.rows}')
    if args.cols:
        cmd.append(f'cols:={args.cols}')

    print(f'[run ] {run_id} ...', flush=True)
    t0 = time.time()
    with open(log_path, 'w', encoding='utf-8') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True)
        wall_limit = args.time_limit * args.wall_factor + 180
        try:
            proc.wait(timeout=wall_limit)
        except subprocess.TimeoutExpired:
            print(f'[warn] {run_id}: wall-clock limit {wall_limit:.0f}s exceeded; killing')
        _kill_group(proc)
    _sweep_strays()
    time.sleep(args.cooldown)

    if os.path.exists(result_path):
        with open(result_path, encoding='utf-8') as fh:
            result = json.load(fh)
    else:
        # The stack failed before the evaluator could report: record it honestly.
        result = {'run_id': run_id, 'strategy': strategy, 'final_status': 'SIMULATION_ERROR',
                  'reason': f'no evaluator result; see logs/{run_id}.log', 'exit_success': False,
                  'maze': {'seed': seed}, 'wall_time_s': time.time() - t0}
        os.makedirs(os.path.dirname(result_path), exist_ok=True)
        with open(result_path, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, indent=2)
    print(f"[done] {run_id}: {result['final_status']} exit_success={result.get('exit_success')} "
          f"dist={result.get('distance_gt_m') or 0:.1f}m wall={time.time() - t0:.0f}s", flush=True)
    return result


def make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Run MazeMind exploration experiments.')
    p.add_argument('--campaign', required=True, help='results sub-directory name')
    p.add_argument('--out', default=os.environ.get('MAZEMIND_RESULTS', 'results'),
                   help='results root directory (default ./results)')
    p.add_argument('--difficulty', default='easy')
    p.add_argument('--rows', type=int)
    p.add_argument('--cols', type=int)
    p.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2, 3, 4])
    p.add_argument('--strategies', nargs='+', default=['random', 'nearest', 'weighted'])
    p.add_argument('--time-limit', type=float, default=480.0, help='simulated seconds per run')
    p.add_argument('--wall-factor', type=float, default=2.0)
    p.add_argument('--render-mode', default='auto')
    p.add_argument('--domain-id', type=int, default=77)
    p.add_argument('--cooldown', type=float, default=3.0)
    p.add_argument('--force', action='store_true', help='re-run even if a result exists')
    p.add_argument('--startup-retries', type=int, default=2,
                   help='re-run attempts whose stack never became ready (kept as evidence)')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = make_parser().parse_args(argv)
    out_dir = os.path.abspath(os.path.join(os.path.expanduser(args.out), args.campaign))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'campaign.json'), 'w', encoding='utf-8') as fh:
        json.dump({k: v for k, v in vars(args).items()}, fh, indent=2)
    results: List[dict] = []
    # Interleave strategies per seed so time-dependent effects affect all equally.
    for seed in args.seeds:
        for strategy in args.strategies:
            results.append(run_one(args, seed, strategy, out_dir))
    ok = sum(1 for r in results if r.get('exit_success'))
    print(f'campaign {args.campaign}: {ok}/{len(results)} runs reached the exit. '
          f'Analyse with: ros2 run mazemind_evaluation analyze_results {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
