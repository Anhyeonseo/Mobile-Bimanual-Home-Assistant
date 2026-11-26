"""Table-height checks based on the native, route-bound stock finger proof."""
import json
import math

# Only the open-gripper inspection stops above the work surface.
CHECKPOINT_CLEARANCE_M = .030


class TableClearanceRejected(ValueError):
    def __init__(self, stage, side, value, minimum):
        self.diagnostics = dict(stage=stage, finger=side, clearance_m=value, required_m=minimum)
        super().__init__(f'{stage} · {side} 손가락 예상 상판 여유 {value*1000:.1f} mm · '
                         f'필요 여유 {minimum*1000:.0f} mm 초과')


def require_table_clearance(proof, *, pregrasp_check):
    surfaces = proof['native_geometry']['stock_finger_surfaces']
    if (surfaces['approach_mode'] != 'vertical'
            or surfaces['basis'] != 'verified_official_STL_surface_vertices'):
        raise ValueError('수직 접근 경로의 손가락 표면 검사가 필요합니다.')
    minimum = CHECKPOINT_CLEARANCE_M if pregrasp_check else 0.0
    stages = ('open', 'pregrasp') if pregrasp_check else ('open', 'pregrasp', 'approach', 'close', 'probe', 'lift')
    rows = {r['stage']: r for r in surfaces['stages']}
    for stage in stages:
        for side in ('fixed', 'moving'):
            value = rows[stage]['minimum_by_finger'][side]['clearance_m']
            if not math.isfinite(value):
                raise ValueError('유효한 손가락 여유 측정값이 필요합니다.')
            if value <= minimum:
                raise TableClearanceRejected(stage, side, value, minimum)


def save_overlay(folder, bundle):
    """Project planned contact faces at the open pregrasp onto the second photo."""
    import numpy as np
    from PIL import Image, ImageDraw
    from tools.diagnostics.run_s3_stationary_lab import model
    from .s2_geometry import JOINTS, model_from_control

    def read(path):
        return json.loads(path.read_text())

    program = read(bundle/'program.json')
    profile = read(bundle/'model.json')['profile']
    stage = program['stages'][1]
    if stage['name'] != 'pregrasp':
        raise ValueError('pregrasp stage required')
    arm = model(program['planned_lift_m'], profile=profile)
    q = [model_from_control(v, profile['arms']['left']['alignment'][name])
         for v, name in zip(stage['points'][-1]['positions_rad'][:6], JOINTS, strict=True)]
    stock = arm.fk(q[:5])[0] @ arm.tcp_from_stock
    axis = -stock[:3, 0]
    fixed = stock[:3, 3] + axis * arm.jaw.fixed_face_m
    gap = np.interp(q[5], arm.jaw.angles, arm.jaw.gaps)
    points = np.stack([fixed, fixed + axis * gap])
    camera = np.linalg.inv(profile['base_from_optical'])
    optical = points @ camera[:3, :3].T + camera[:3, 3]
    if np.any(optical[:, 2] <= 0):
        raise ValueError('planned fingers outside the camera view')
    intr = read(folder/'metadata.json')['color_intrinsics']
    pixels = optical[:, :2]/optical[:, 2, None] * [intr['fx'], intr['fy']] + [intr['ppx'], intr['ppy']]
    image = Image.open(folder/'color.png').convert('RGB')
    draw = ImageDraw.Draw(image)
    for (x,y), color in zip(pixels, ('tomato','cyan'), strict=True):
        draw.ellipse((x-5,y-5,x+5,y+5),outline=color,width=2)
    draw.rectangle((0,0,285,20), fill='black')
    draw.text((4,4),'Planned faces: red fixed / cyan moving',fill='white')
    destination = folder/'planned-fingers.png'
    image.save(destination)
    return destination
