#!/usr/bin/env python3
"""Local pick-stage monitor and hardware-free persistent-loop rehearsal.

demo: test the requested lifecycle, cancellation and web UI without devices.
watch: observe unified_pick.py's real state.json; no control commands.
replay: real inference and resident MoveIt on a bound saved capture; no devices.
live: fresh capture, real inference/planning, checked stationary left pick.
"""
import argparse
from datetime import datetime
import math
import json
from pathlib import Path
import signal
import sys
import threading

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'),
               str(ROOT/'ros2_ws/src/so101_arm_bridge'), str(ROOT/'tools/run')]

from home_robot_tasks.pick_demo import DemoBackend
from home_robot_tasks.pick_service import PickStation
from home_robot_tasks.pick_watch import WatchedPick
from home_robot_tasks.pick_web import create_server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('demo', 'watch', 'replay', 'live', 'station'), default='demo')
    parser.add_argument('--port', type=int, default=18768)
    parser.add_argument('--output', type=Path, help='new directory for demo request reports')
    parser.add_argument('--watch', type=Path, help='unified_pick output directory containing state.json')
    parser.add_argument('--demo-image', type=Path, help='optional saved reference image, explicitly labelled as not live')
    parser.add_argument('--demo-step-seconds', type=float, default=.8)
    parser.add_argument('--demo-fail-stage', choices=('capture', 'detect', 'candidates', 'plan', 'execute', 'verify'))
    parser.add_argument('--snapshot', type=Path, help='replay: archived RGB-D folder')
    parser.add_argument('--robot-report', type=Path, help='replay: robot evidence from this exact exposure')
    parser.add_argument('--artifacts-dir', type=Path, default=ROOT/'artifacts', help='existing model cache and pinned GraspNet artifacts')
    parser.add_argument('--mobile-mesh-dir', type=Path, help='directory of original calibrated mobile STL meshes')
    parser.add_argument('--candidate-source', choices=('so101','graspnet','graspnet-first','graspnet-so101','flat-rule'))
    parser.add_argument('--device', choices=('auto','cpu','cuda'), default='auto')
    parser.add_argument('--camera-mode', choices=('wide','legacy','near'))
    parser.add_argument('--lift-height-model',choices=('field_effective','encoder_no_sag'),
                        help='station defaults to encoder_no_sag; field_effective explicitly restores historical 630 mm compensation')
    parser.add_argument('--maximum-grasps', type=int, choices=range(1,33), default=8)
    parser.add_argument('--stage-timeout-seconds', type=float,
                        help='overall stage deadline; 0 waits until completion (station default), other modes default to 300')
    parser.add_argument('--serial-port', default='/dev/ttyACM0')
    parser.add_argument('--baseline', type=Path, help='live: approved unified-controller baseline JSON')
    parser.add_argument('--resume-from', type=Path, help='station: failed pre-execution bottom-to-top request to resume explicitly')
    parser.add_argument('--top-baseline', type=Path, help='station: baseline recorded for the top-reference firmware')
    parser.add_argument('--execute', action='store_true', help='authorize supported hardware startup; live mode only')
    parser.add_argument('--supported-empty-bottom', action='store_true')
    parser.add_argument('--lift-target-um', type=int, default=0, help='live preparation stays within the existing 0..10000um range')
    parser.add_argument('--plan-only', action='store_true', help='live: hold/capture/plan but do not dispatch an arm route')
    parser.add_argument('--check', action='store_true', help='read-only dependency check; no web, models or devices started')
    args = parser.parse_args(argv)
    if args.lift_height_model is None:
        from home_robot_tasks.pick_calibration import STATION_HEIGHT_MODEL
        args.lift_height_model = STATION_HEIGHT_MODEL if args.mode == 'station' else 'field_effective'
    if args.candidate_source is None:
        args.candidate_source = 'graspnet' if args.mode == 'station' else 'so101'
    if args.stage_timeout_seconds is None:
        args.stage_timeout_seconds = 0 if args.mode == 'station' else 300
    if not 1 <= args.port <= 65535:
        parser.error('port must be within 1..65535')
    if not math.isfinite(args.demo_step_seconds) or not 0 <= args.demo_step_seconds <= 60:
        parser.error('demo step duration must be within 0..60 seconds')
    if not math.isfinite(args.stage_timeout_seconds) or args.stage_timeout_seconds<0:
        parser.error('nonnegative finite stage timeout required; use 0 to wait until completion')
    if args.mode!='live' and (args.execute or args.supported_empty_bottom or args.plan_only or (args.baseline and args.mode!='station') or args.lift_target_um):
        parser.error('hardware options require --mode live')
    if args.mode in ('live','replay','station'):
        if not args.mobile_mesh_dir or (not args.check and not args.mobile_mesh_dir.is_dir()):
            parser.error('--mobile-mesh-dir is required for real planning')
        if args.demo_image or args.demo_fail_stage or args.watch:
            parser.error('pipeline modes do not accept demo/watch inputs')
    if args.mode=='live':
        if not args.check and (not args.execute or not args.supported_empty_bottom or not args.baseline or not args.baseline.is_file()):
            parser.error('live requires --execute --supported-empty-bottom --baseline <existing JSON>')
        if args.snapshot or args.robot_report or not 0<=args.lift_target_um<=10000:
            parser.error('live requires new capture and supported 0..10000um lift startup')
        args.camera_mode=args.camera_mode or 'wide'
    if args.resume_from is not None and args.mode!='station':
        parser.error('--resume-from requires station mode')
    if args.mode=='station':
        if args.snapshot or args.robot_report:
            parser.error('station requests require a fresh capture')
        args.camera_mode = args.camera_mode or 'wide'
    if args.mode=='replay':
        if not args.check and (not args.snapshot or not args.robot_report or not args.robot_report.is_file()):
            parser.error('replay requires --snapshot and --robot-report from the same exposure')
        if not args.check and any(not (args.snapshot/name).is_file() for name in ('color.png','depth_aligned.png','metadata.json')):
            parser.error('replay snapshot requires RGB, aligned depth and metadata')
    if args.check:
        if args.mode not in ('live','replay'):
            parser.error('--check is for live or replay dependencies')
        from tools.lib.pick_readiness import check
        result=check(vars(args))
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result['status']=='DEPENDENCIES_PRESENT' else 2
    if args.mode == 'watch':
        if not args.watch or args.output or args.demo_image or args.demo_fail_stage:
            parser.error('watch requires --watch and does not accept demo options or --output')
        station = WatchedPick(args.watch)
    else:
        if args.watch:
            parser.error('--watch requires --mode watch')
        if args.demo_image and (not args.demo_image.is_file() or args.demo_image.suffix.lower() not in ('.jpg', '.jpeg', '.png')):
            parser.error('--demo-image must be an existing PNG or JPEG')
        output = args.output or ROOT/'output/pick-station'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        output.mkdir(parents=True, exist_ok=False)
        if args.mode=='demo':
            backend=DemoBackend(step_seconds=args.demo_step_seconds,image=args.demo_image,fail_stage=args.demo_fail_stage)
        else:
            from home_robot_tasks.pick_backend import PipelineBackend
            options={key:str(value) if isinstance(value,Path) else value for key,value in vars(args).items()}
            if args.mode == 'station':
                from home_robot_tasks.pick_web_backend import WebPickBackend
                if args.resume_from is not None:
                    from home_robot_tasks.pick_resume import ResumePickBackend
                    backend=ResumePickBackend(options)
                else:
                    backend = WebPickBackend(options)
            else:
                backend = PipelineBackend(options)
        station = PickStation(backend, output, stage_timeout_s=args.stage_timeout_seconds)
    try:
        server = create_server(station, args.port)
    except OSError as exc:
        parser.exit(2, f'로컬 웹 포트를 열 수 없습니다: {exc}\n')
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopping.set())
    try:
        station.start()
        thread = threading.Thread(target=server.serve_forever, name='pick-monitor-http', daemon=True)
        thread.start()
        print(f'Pick Station: http://127.0.0.1:{server.server_port}  mode={args.mode}', flush=True)
        print('실제 장치 연결 모드입니다.' if args.mode in ('live', 'station') else '실제 로봇에 명령을 보내지 않습니다.', flush=True)
        while not stopping.wait(.2):
            pass
    finally:
        server.shutdown()
        server.server_close()
        station.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
