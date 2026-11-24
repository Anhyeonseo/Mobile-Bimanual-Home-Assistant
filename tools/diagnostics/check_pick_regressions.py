#!/usr/bin/env python3
"""Check saved top-pick scenes through planning, wire geometry and stop simulations.

No serial ports, live camera, torque commands or hardware execution. Model paths
and calibration come from --root; each input keeps its capture identity. Exit
nonzero if any case fails. This does not assert physical grasp success.
"""
from pathlib import Path
import sys,json,time,argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
parser.add_argument('--prepared',type=Path,action='append',required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--mobile-mesh-dir',type=Path)
options=parser.parse_args()
ROOT=options.root.resolve()
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.plan_capture_carry import plan,serial
from tools.lib.pick_plan_validation import validate_pick_plan
from tools.lib.pick_planning import station_plan_options,save_plan_options
P=options.output.resolve();P.mkdir(parents=True,exist_ok=True)
cases=[p.resolve() for p in options.prepared]
mesh=options.mobile_mesh_dir or ROOT/'assets/mobile_meshes'
summary=[]
for index,prepared in enumerate(cases):
 data=json.loads(prepared.read_text());name=data['capture_id'];out=P/'regression'/f'{index+1:02d}-{name}'
 row=dict(capture_id=name,prepared=str(prepared),hardware_commands=0,status='RUNNING');summary.append(row)
 (P/'regression-summary.json').write_text(json.dumps(summary,indent=2));print(name,'planning',flush=True)
 started=time.monotonic()
 try:
  args=station_plan_options(prepared,out/'planning',mesh)
  save_plan_options(args,out/'planning-options.json')
  report=plan(args);row.update(plan_status=report['status'],planning_states=report.get('checked_states'))
  if report['status']!='OFFLINE_ROUTE_FOUND':raise ValueError(report['status']+': '+str(report.get('reason',report.get('plan',{}).get('failures'))))
  row['candidate']=report['plan']['candidate_id'];print(name,'wire/native checks',flush=True)
  proof=validate_pick_plan(out/'planning/report.json',prepared,args.mobile_mesh_dir,out/'proof')
  native=proof['native_geometry'];rehearsals=proof['rehearsals']
  (out/'proof.json').write_text(json.dumps(proof,default=serial,indent=2))
  row.update(status='PASSED',native_states=sum(s['checked_states'] for s in native['stages']),
   independent_obstacle_rejected=native['independent_obstacle_rejected'],fault_scenarios=len(rehearsals))
 except Exception as exc:
  row.update(status='FAILED',reason=str(exc))
 row['seconds']=time.monotonic()-started
 (P/'regression-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(row),flush=True)
if any(r['status']!='PASSED' for r in summary):raise SystemExit(1)
