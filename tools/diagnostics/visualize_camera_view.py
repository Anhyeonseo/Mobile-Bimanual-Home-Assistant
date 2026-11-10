#!/usr/bin/env python3
"""Read-only FOV illustration from recorded SDK intrinsics and fixed mount model."""
import argparse,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks')]
from home_robot_tasks.s2_geometry import load_geometry
from tools.diagnostics.s2_visualize import bundle

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    config=ROOT/'config/home_assistant';read=lambda n:json.loads((config/n).read_text())
    profile=read('s2_coordinates.development.json');near=read('camera_device.near.observed.json');hd=read('camera_device.observed.json')
    sdk=near['sdk_calibration'];rgb=sdk['color_intrinsics'];depth=sdk['native_depth_intrinsics']
    # Only the current 640x480 color intrinsics were read from this device.
    # The wider RGB comparison is explicitly nominal, never a new calibration.
    wide=dict(width=1280,height=720,fx=1280/(2*math.tan(math.radians(69/2))),
        fy=720/(2*math.tan(math.radians(42/2))),ppx=640,ppy=360)
    modes=[dict(label='현재 · RGB 640×480 / 깊이 640×360',rgb=rgb,depth=depth,min_z_m=.24,estimated=False),
           dict(label='깊이만 1280×720로 변경',rgb=rgb,depth=hd['sdk_calibration']['native_depth_intrinsics'],min_z_m=.45,estimated=False),
           dict(label='RGB 1280×720 광각 · 예상',rgb=wide,depth=depth,min_z_m=.24,estimated=True)]
    robot=bundle(load_geometry(config/'official_geometry.json'),profile)
    robot['display'].update(inspection_lighting=True,highlight_platform=True)
    data=dict(id='d415-fov-20261108',robot=robot,camera=profile['base_from_optical'],ground=profile['ground_z_m'],
        color_from_depth=sdk['color_optical_from_depth_optical'],baseline_m=.055,display_depth_m=3.,modes=modes,
        hardware_commands=0,camera_settings_changed=False,occlusion_modelled=False,
        source=dict(profile='config/home_assistant/s2_coordinates.development.json',
            intrinsics='config/home_assistant/camera_device.near.observed.json',
            datasheet='https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf',
            min_z_section='4.6',stereo_overlap_sections=['4.3','4.4','4.5'],nominal_rgb_hv_deg=[69,42]))
    here=Path(__file__).parent;content=(here/'camera_viewer/template.html').read_text()
    parts={'__DATA__':json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c'),
        '__KINEMATICS__':(here/'s2_viewer/viewer.js').read_text(),'__RENDERER__':(here/'s2_viewer/mesh-renderer.js').read_text(),
        '__GEOMETRY__':(here/'camera_viewer/geometry.js').read_text(),'__VIEWER__':(here/'camera_viewer/viewer.js').read_text()}
    for k,v in parts.items():content=content.replace(k,v)
    if len(content.encode())>1_000_000:raise ValueError('inline visualization size budget exceeded')
    a.output.write_text(content);print(a.output)

if __name__=='__main__':main()
