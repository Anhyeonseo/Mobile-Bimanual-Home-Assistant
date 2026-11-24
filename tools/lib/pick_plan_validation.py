"""Shared offline admission for web picks and saved-capture regressions.

Reuse the existing carry program builder, native wire-geometry validator and
fault simulations. This module has no hardware writer.
"""
from dataclasses import asdict
import json
from pathlib import Path


def validate_pick_plan(report_path, prepared, mobile_mesh_dir, output, *, session_factory=None):
    from tools.diagnostics.plan_capture_carry import serial
    from tools.diagnostics.prepare_carry_execution import load_inputs, validate_native
    from tools.diagnostics.carry_stream_sim import simulate
    from home_robot_tasks.carry_program import manifest

    report_path, prepared, output = Path(report_path), Path(prepared), Path(output)
    if json.loads(report_path.read_text()).get('status') != 'OFFLINE_ROUTE_FOUND':
        raise ValueError('complete route required before execution validation')
    program, inputs = load_inputs(report_path, prepared)
    output.mkdir(parents=True, exist_ok=False)
    summary = manifest(program)
    kwargs = {} if session_factory is None else {'session_factory': session_factory}
    summary['native_geometry'] = validate_native(
        program, inputs, Path(mobile_mesh_dir), output/'moveit', **kwargs)
    faults = (None, 'no_follow', 'slip', 'empty_close', 'timeout', 'dispatch',
              'stale', 'cancel', 'base_moved', 'stop_unconfirmed')
    summary['rehearsals'] = [simulate(program, fault=fault) for fault in faults]
    summary['simulation_passed'] = [r['state'] for r in summary['rehearsals']] == (
        ['HOLDING'] + ['FAILED']*6 + ['CANCELLED', 'FAILED', 'STOPPING'])
    if not summary['simulation_passed']:
        raise ValueError('execution rehearsal failed')
    summary['status'] = 'OFFLINE_PREFLIGHT_PASSED'
    for name, data in (('report.json', summary), ('program.json', asdict(program))):
        (output/name).write_text(json.dumps(data, default=serial, indent=2, allow_nan=False)+'\n')
    return summary
