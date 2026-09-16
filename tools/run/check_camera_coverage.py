#!/usr/bin/env python3
"""Report candidate camera blind regions across a travel corridor (no hardware)."""
import argparse
import json
from pathlib import Path
import math
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.camera_coverage import frustum_coverage


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera-height',type=float,default=.6)
    p.add_argument('--pitch-down-deg',type=float,default=15.)
    p.add_argument('--body-height',type=float,default=1.)
    p.add_argument('--radius',type=float,default=.3)
    p.add_argument('--camera-x',type=float,default=0.)
    p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480)
    p.add_argument('--fx',type=float,default=502.3);p.add_argument('--fy',type=float,default=659.4)
    p.add_argument('--cx',type=float,default=319.5);p.add_argument('--cy',type=float,default=239.5)
    p.add_argument('--minimum-depth',type=float,default=.45)
    a=p.parse_args()
    if not .05<=a.radius<=1 or not .1<=a.body_height<=3:p.error('bounded body dimensions required')
    points=[(x,y,z) for x in (.2,.4,.6,1.,1.5,2.) for y in (-a.radius,0,a.radius) for z in (0.02,a.body_height/2,a.body_height)]
    visible=frustum_coverage(points,height_m=a.camera_height,pitch_down_rad=math.radians(a.pitch_down_deg),camera_x_m=a.camera_x,
        width=a.width,height=a.height,fx=a.fx,fy=a.fy,cx=a.cx,cy=a.cy,minimum_depth_m=a.minimum_depth,maximum_depth_m=4.)
    print(json.dumps(dict(mode='simulation',hardware_commands=0,dimensions_measured=False,
        note='Geometric samples only; defaults approximate 65x40 degree depth FOV. Use actual CameraInfo. Occlusion, material, motion and detection not tested.',
        camera_height_m=a.camera_height,pitch_down_deg=a.pitch_down_deg,sampled=len(points),in_frustum=sum(visible),
        blind_points_m=[q for q,v in zip(points,visible) if not v]),indent=2))


if __name__=='__main__':main()
