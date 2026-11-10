#!/usr/bin/env python3
"""Single-owner D415 filming and offline bag-to-snapshot export. No motor access."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import select
import shutil
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import write_json


def intrinsics(frame):
    i = frame.profile.as_video_stream_profile().get_intrinsics()
    return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.ppx,
                ppy=i.ppy, coeffs=list(i.coeffs), model=str(i.model))


def frame_info(frame):
    return dict(number=frame.get_frame_number(), timestamp_ms=frame.get_timestamp(),
                timestamp_domain=str(frame.get_frame_timestamp_domain()))


def record(args, rs):
    """Match the established near-field profile; keep native depth in the bag."""
    args.output.parent.mkdir(parents=True, exist_ok=True)
    estimated = (640*480*3 + 640*360*2)*30*args.seconds*1.2 + 1024**3
    if shutil.disk_usage(args.output.parent).free < estimated:
        raise ValueError(f'녹화 여유 공간 {estimated / 1024**3:.1f}GiB 이상 필요')
    devices = [d for d in rs.context().query_devices() if 'D415' in d.get_info(rs.camera_info.name)]
    if len(devices) != 1:
        raise ValueError('D415 한 대가 필요합니다. 기존 카메라 프로그램도 종료하세요.')
    device = devices[0]
    usb = device.get_info(rs.camera_info.usb_type_descriptor)
    if not usb.startswith('3'):
        raise ValueError('USB 3 연결이 필요합니다.')
    args.output.mkdir(exist_ok=False)
    metadata = dict(schema_version=1, purpose='presentation_rgbd_recording',
        status='starting', robot_commands=0, raw_depth_unaligned_unfiltered=True,
        robot_pose_recorded=False, maximum_seconds=args.seconds, usb=usb,
        requested_profiles=dict(color=[640,480,30,'bgr8'], depth=[640,360,30,'z16']))
    write_json(args.output/'recording.json', metadata)
    config = rs.config()
    config.enable_device(device.get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    config.enable_stream(rs.stream.depth,640,360,rs.format.z16,30)
    config.enable_record_to_file(str((args.output/'recording.bag').resolve()))
    pipeline = rs.pipeline()
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    previous = {s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    active = False
    counts = dict(color=0,depth=0)
    last_numbers = {}
    start = time.monotonic()
    try:
        profile = pipeline.start(config)
        active = True
        metadata.update(status='recording', started_at_utc=datetime.now(timezone.utc).isoformat(),
                        depth_scale_m=profile.get_device().first_depth_sensor().get_depth_scale())
        write_json(args.output/'recording.json', metadata)
        print('RGB-D 녹화 시작 · 종료: Enter 또는 Ctrl+C · 다른 카메라 도구를 함께 열지 마세요.',flush=True)
        start = time.monotonic(); last_print = -1
        while not stopped and time.monotonic()-start < args.seconds:
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                sys.stdin.readline(); break
            frames = pipeline.wait_for_frames(2000)
            for name,frame in (('color',frames.get_color_frame()),('depth',frames.get_depth_frame())):
                if frame and frame.get_frame_number() != last_numbers.get(name):
                    counts[name] += 1; last_numbers[name] = frame.get_frame_number()
                    if counts[name] == 1:
                        metadata['first_'+name] = frame_info(frame)
                        metadata[name+'_intrinsics'] = intrinsics(frame)
                    metadata['last_'+name] = frame_info(frame)
            elapsed = int(time.monotonic()-start)
            if elapsed != last_print:
                print(f'{elapsed:03d}s | RGB 수신 {counts["color"]} | Depth 수신 {counts["depth"]}',flush=True)
                last_print = elapsed
        metadata['status'] = 'recorded_unverified' if all(counts.values()) else 'empty'
    except BaseException as exc:
        metadata.update(status='interrupted', error=str(exc))
        raise
    finally:
        try:
            if active:
                pipeline.stop()
        finally:
            for sig,handler in previous.items():
                signal.signal(sig,handler)
            metadata.update(finished_at_utc=datetime.now(timezone.utc).isoformat(),
                            elapsed_s=time.monotonic()-start, received_frames=counts,
                            full_playback_verified=False)
            write_json(args.output/'recording.json',metadata)
    print('저장:',args.output/'recording.bag')
    print('녹화 프레임을 추출·재생해 영상과 깊이를 확인하세요.')


def extract(args, rs):
    """Sequential offline playback. Never falls back to a connected camera."""
    import cv2
    import numpy as np
    bag = args.bag.resolve(strict=True)
    if args.output.exists():
        raise ValueError('출력 폴더가 이미 있습니다. 새 경로를 지정하세요.')
    config = rs.config()
    config.enable_device_from_file(str(bag), repeat_playback=False)
    pipeline = rs.pipeline()
    active = False
    first = None
    try:
        profile = pipeline.start(config); active = True
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)
        duration = playback.get_duration().total_seconds()
        if args.at >= duration:
            raise ValueError(f'요청 시각이 녹화 길이 {duration:.2f}초를 벗어났습니다.')
        align = rs.align(rs.stream.color)
        deadline = time.monotonic()+max(60,duration*3)
        while time.monotonic() < deadline:
            try:
                frames = pipeline.wait_for_frames(3000)
            except RuntimeError:
                if playback.current_status() == rs.playback_status.stopped:
                    raise ValueError('해당 시각까지 동기화된 RGB-D 프레임이 없습니다.')
                raise
            color,depth = frames.get_color_frame(),frames.get_depth_frame()
            if not color or not depth:
                continue
            if first is None:
                first = color.get_timestamp()
            offset = (color.get_timestamp()-first)/1000
            if offset < args.at:
                continue
            if color.get_frame_timestamp_domain() != depth.get_frame_timestamp_domain():
                raise ValueError('컬러·깊이의 시각 기준이 다릅니다.')
            skew = abs(color.get_timestamp()-depth.get_timestamp())
            if skew > 40:
                continue
            aligned = align.process(frames)
            ac,ad = aligned.get_color_frame(),aligned.get_depth_frame()
            if not ac or not ad:
                continue
            rgb = np.asanyarray(ac.get_data()).copy()
            fmt = color.profile.format()
            if fmt == rs.format.rgb8:
                rgb = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            elif fmt != rs.format.bgr8:
                raise ValueError('지원하지 않는 컬러 형식')
            raw = np.asanyarray(ad.get_data()).copy()
            if raw.dtype != np.uint16:
                raise ValueError('원본 16비트 깊이가 필요합니다.')
            info = frame_info(color)
            # Only an epoch-based SDK timestamp can supply capture UTC.
            epoch = info['timestamp_ms']/1000
            capture_utc = (datetime.fromtimestamp(epoch,timezone.utc).isoformat()
                if ('global_time' in info['timestamp_domain'] or 'system_time' in info['timestamp_domain'])
                and 946684800 < epoch < 4102444800 else None)
            e = depth.profile.get_extrinsics_to(color.profile)
            metadata = dict(coordinate_frame='color_camera_optical_frame',
                measurement_depth_file='depth_aligned.png',alignment='depth to color via rs.align',
                captured_at_utc=capture_utc, extracted_at_utc=datetime.now(timezone.utc).isoformat(),
                color_intrinsics=intrinsics(color),native_depth_intrinsics=intrinsics(depth),
                aligned_depth_intrinsics=intrinsics(ad),
                depth_scale_m=profile.get_device().first_depth_sensor().get_depth_scale(),
                depth_to_color=dict(rotation_column_major=list(e.rotation),translation_m=list(e.translation)),
                color_frame=info,depth_frame=frame_info(depth),
                recording_source=dict(path=str(bag),bytes=bag.stat().st_size,
                    modified_ns=bag.stat().st_mtime_ns,requested_offset_s=args.at,
                    offset_from_first_decoded_color_s=offset,first_decoded_color_timestamp_ms=first,
                    rgb_depth_skew_ms=skew),robot_transform=None,robot_pose_recorded=False)
            args.output.mkdir(parents=True,exist_ok=False)
            for name,image in (('color.png',rgb),('depth_aligned.png',raw)):
                if not cv2.imwrite(str(args.output/name),image):
                    raise OSError('이미지 저장 실패: '+name)
            write_json(args.output/'metadata.json',metadata)
            write_json(args.output/'export.json',dict(status='RGBD_FRAME_EXPORTED',
                robot_commands=0,source_sha256={name:hashlib.sha256((args.output/name).read_bytes()).hexdigest()
                    for name in ('color.png','depth_aligned.png','metadata.json')},
                coordinate_frame='color_camera_optical_frame',robot_pose_recorded=False))
            print(f'추출 완료: {offset:.3f}초 · {args.output} · 카메라 좌표, 로봇 자세 기록 없음')
            return
        raise TimeoutError('녹화 재생 제한 시간 초과')
    finally:
        if active:
            pipeline.stop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    record_parser = sub.add_parser('record',help='실제 D415를 점유하여 컬러+깊이 녹화')
    record_parser.add_argument('--output',type=Path,required=True)
    record_parser.add_argument('--seconds',type=int,default=180)
    export_parser = sub.add_parser('extract',help='bag에서 재분석용 프레임 추출; 실물 카메라 접근 없음')
    export_parser.add_argument('--bag',type=Path,required=True)
    export_parser.add_argument('--at',type=float,required=True)
    export_parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args(argv)
    if args.command == 'record' and not 1 <= args.seconds <= 1800:
        parser.error('녹화 시간은 1~1800초')
    if args.command == 'extract' and (not math.isfinite(args.at) or args.at < 0):
        parser.error('추출 시각은 0 이상의 유한한 초 단위 값')
    try:
        import pyrealsense2 as rs
        (record if args.command == 'record' else extract)(args,rs)
        return 0
    except (ValueError,OSError,RuntimeError,ImportError) as exc:
        print('중단:',exc,file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
