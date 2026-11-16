#!/usr/bin/env python3
"""Analyze a recorded wheel inventory or teleop JSONL without opening a serial port."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.s4_base_preparation import review_capture, review_teleop_capture

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--lidar-command-campaign',type=Path,help='reuse independent masked lidar trajectories and original teleop ACKs for command-speed candidates')
    p.add_argument('--teleop',action='store_true',help='review a completed powered teleop session instead of read-only inventory')
    p.add_argument("--sensor-capture",type=Path,help="also review synchronized LDS/native RGB-D archive")
    p.add_argument("--mapping-output",type=Path,help="new directory for offline sensor comparison")
    a=p.parse_args()
    if a.lidar_command_campaign:
        if a.capture or a.teleop or a.sensor_capture or a.mapping_output:p.error('command-speed review is separate from encoder-unit review')
        from home_robot_tasks.s4_base_preparation import review_lidar_command_scale
        sys.path.insert(0,str(ROOT/'tools/run'))
        from pulse_mobile_base import PATTERNS
        result=review_lidar_command_scale(a.lidar_command_campaign,PATTERNS)
        with a.output.open('x') as stream:json.dump(result,stream,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k not in ('windows','source_sha256')}))
        return
    if not a.capture:p.error('--capture or --lidar-command-campaign required')
    if a.sensor_capture and (not a.teleop or a.mapping_output is None):
        p.error("sensor review requires --teleop and --mapping-output")
    if a.mapping_output and not a.sensor_capture:p.error("--mapping-output requires --sensor-capture")
    result=(review_teleop_capture if a.teleop else review_capture)([json.loads(line) for line in a.capture.read_text().splitlines() if line.strip()])
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    if a.sensor_capture:
        from home_robot_tasks.s4_recorded_mapping import review_recorded
        result=review_recorded(ROOT,a.capture,a.sensor_capture,a.mapping_output)
    print(json.dumps(result))

if __name__=='__main__':main()
