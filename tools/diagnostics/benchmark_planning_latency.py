#!/usr/bin/env python3
"""Paired offline planning ablation on identical saved candidates; no inference/execution."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'),
               str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.benchmark_grasp_feasibility import validate, save, verify_inputs


def summary(rows):
    result = {}
    for mode in sorted({r['mode'] for r in rows}):
        selected = [r for r in rows if r['mode'] == mode]
        successes = [r for r in selected if r['report']['status'] == 'OFFLINE_ROUTE_FOUND']
        times = [r['report']['first_feasible_validation_ms'] for r in successes]
        search_times = [r['report']['validation_ms'] for r in selected if 'validation_ms' in r['report']]
        result[mode] = dict(runs=len(selected), successes=len(successes),
            median_first_complete_route_ms=statistics.median(times) if times else None,
            median_search_wall_ms=statistics.median(search_times) if search_times else None,
            errors=sum(r['report']['status'] in ('VALIDATION_ERROR', 'INVALID_INITIAL_STATE') for r in selected))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--mobile-mesh-dir', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--repeats', type=int, choices=range(1, 11), default=3)
    p.add_argument('--maximum-candidates', type=int, choices=range(1, 33), default=8)
    p.add_argument('--budget', type=float, default=5.)
    p.add_argument('--attempts', type=int, choices=(1, 2), default=2)
    p.add_argument('--request-window', type=int, choices=range(1,33), default=8)
    p.add_argument('--verify-batch', action='store_true', help='slow reference equivalence check; not a performance run')
    p.add_argument('--modes', nargs='+', choices=('baseline', 'pipelined', 'batch', 'parallel'),
                   default=['baseline', 'pipelined'])
    a = p.parse_args()
    if not 0 < a.budget <= 5: p.error('budget must be in (0, 5]')
    if a.output_dir.exists(): p.error('output directory exists; choose a new directory')
    prepared = json.loads(a.prepared.read_text())
    verify_inputs(a.snapshot, prepared)
    prepared['group']['refined_grasps'] = prepared['group']['refined_grasps'][:a.maximum_candidates]
    a.output_dir.mkdir(parents=True)
    save(a.output_dir/'input.json', dict(prepared_path=str(a.prepared.resolve()),
        prepared_sha256=hashlib.sha256(a.prepared.read_bytes()).hexdigest(),
        candidate_count=len(prepared['group']['refined_grasps']), modes=a.modes,
        scope='binding + first complete pick/lift/carry route; no physical execution',
        budget_s=a.budget, attempts=a.attempts, repeats=a.repeats,
        request_window=a.request_window, reference_verification=a.verify_batch))
    rows = []
    for repeat in range(a.repeats):
        # Rotate ordering to reduce systematic warm-cache bias.
        shift = repeat % len(a.modes)
        for mode in a.modes[shift:]+a.modes[:shift]:
            directory = a.output_dir/f'run-{repeat+1}'/mode
            print(f'Run {repeat+1}: {mode}', flush=True)
            try:
                report = validate(prepared, directory, a.mobile_mesh_dir, a.model_dir,
                    acceleration=mode, planning_budget_s=a.budget, planning_attempts=a.attempts,
                    verify_batch=a.verify_batch, request_window=a.request_window)
            except Exception as exc:
                report = dict(status='VALIDATION_ERROR', reason=str(exc), hardware_commands=0)
            save(directory/'report.json', report)
            rows.append(dict(repeat=repeat+1, mode=mode, report=report))
            save(a.output_dir/'summary.json', dict(summary=summary(rows), runs=rows,
                motion_authorized=False, physical_grasp_success_verified=False,
                notes=['OMPL stochastic; compare success counts as well as successful-run latency.',
                       'Setup excluded from first-route time; stored separately in each report.',
                       'Parallel uses two one-attempt pipelines; budgets are per pipeline, not CPU-equivalent.',
                       'Sampled collision checking retained; no continuous collision proof.']))
            print(report['status'], report.get('first_feasible_validation_ms'), flush=True)
    if any(r['report']['status'] == 'VALIDATION_ERROR' for r in rows):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
