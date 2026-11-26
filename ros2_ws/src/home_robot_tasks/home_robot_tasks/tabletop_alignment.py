"""Evaluate the captured world relative to a translated base (offline only)."""
from dataclasses import replace
import numpy as np


def translated_observation(delta_xy, profile, review, group, boxes, payload):
    delta = np.asarray(delta_xy, dtype=float)
    if delta.shape != (2,) or not np.isfinite(delta).all() or np.linalg.norm(delta) > .150000001:
        raise ValueError('relative base translation outside the 15 cm trial envelope')
    offset = np.array([-delta[0], -delta[1], 0.])
    shift = np.eye(4)
    shift[:3, 3] = offset
    optical_shift = np.asarray(profile['base_from_optical'])[:3, :3].T @ offset
    grasps = [dict(g, translation_m=(g['translation_m'] + optical_shift).tolist())
              for g in group['refined_grasps']]
    points = dict(review['base_points_m'])
    for field in ('center_xyz_m', 'surface_xyz_m'):
        if field in points:
            points[field] = (offset + points[field]).tolist()
    return (dict(review, base_points_m=points), dict(group, refined_grasps=grasps),
            [replace(box, base_pose=shift @ box.base_pose) for box in boxes],
            replace(payload, base_from_object=shift @ payload.base_from_object))


def preferred_offset(samples):
    """Prefer the interior of the widest consecutive sampled reachable stretch."""
    if samples.get(0.) is True:
        return 0.
    runs = []
    run = []
    for distance, ok in sorted(samples.items()):
        if ok:
            run.append(distance)
        elif run:
            runs.append(run)
            run = []
    if run:
        runs.append(run)
    if not runs:
        return None
    def score(run):
        midpoint = (run[0] + run[-1]) / 2
        return (round(run[-1] - run[0], 8), -abs(midpoint))
    best = max(runs, key=score)
    mid = (best[0] + best[-1]) / 2
    return min(best, key=lambda x: (round(abs(x-mid), 8), abs(x)))

def scene_sweep(goal_boxes, delta_xy):
    """Sample the measured scene against the stationary robot posture along translation."""
    delta = np.asarray(delta_xy, dtype=float)
    steps = max(1, int(np.ceil(np.linalg.norm(delta) / .01)))
    for fraction in np.linspace(0, 1, steps + 1):
        shift = np.eye(4)
        shift[:2, 3] = delta * (1 - fraction)
        yield fraction, [replace(box, base_pose=shift @ box.base_pose) for box in goal_boxes]


def projected_target_visible(profile, payload, intrinsics):
    from itertools import product
    from .assistant.view_geometry import view_check
    vertices = (np.asarray(payload.mesh['vertices']) if payload.mesh is not None
                else np.asarray(list(product((-0.5, 0.5), repeat=3))) * payload.size_m)
    pose = payload.base_from_object
    points = vertices @ pose[:3, :3].T + pose[:3, 3]
    return view_check(points, profile['base_from_optical'], intrinsics).valid


def draw_comparison(summary, center_xy, path):
    """Top view in the original robot frame. Dots represent base origins."""
    from PIL import Image, ImageDraw, ImageFont
    image = Image.new('RGB', (760, 620), 'white')
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', 17)
        title = ImageFont.truetype('DejaVuSans.ttf', 24)
    except OSError:
        font = title = ImageFont.load_default()
    xy = np.asarray([center_xy, [0, 0], *[s['base_delta_xy_m'] for s in summary['samples']]])
    scale = min(490 / max(.4, np.ptp(xy[:, 1])), 340 / max(.4, np.ptp(xy[:, 0])))
    mid = (xy.max(axis=0) + xy.min(axis=0)) / 2
    def pixel(point):
        x, y = np.asarray(point) - mid
        return (round(380-y*scale), round(300-x*scale))
    draw.text((25, 19), 'Tabletop pick  /  tested base positions', font=title, fill='#163044')
    draw.text((25, 58), 'Top view   |   forward up, robot left to the left', font=font, fill='#405466')
    px, py = pixel(center_xy)
    draw.rectangle((px-6, py-6, px+6, py+6), fill='#282e38')
    draw.text((px+12, py-10), 'Object', font=font, fill='#282e38')
    for sample in summary['samples']:
        x, y = pixel(sample['base_delta_xy_m'])
        ok = sample['status'] in ('OFFLINE_ROUTE_FOUND', 'HYPOTHETICAL_ROUTE_FOUND')
        color = '#137863' if ok else '#b64c39'
        draw.ellipse((x-6, y-6, x+6, y+6), fill=color)
        if sample['offset_mm'] == summary['preferred_tested_offset_mm']:
            draw.ellipse((x-11, y-11, x+11, y+11), outline=color, width=2)
        draw.text((x, y+18), f"{sample['offset_mm']:+g}", font=font, fill=color, anchor='ma')
    draw.text((25, 545), 'Offsets in mm   |   green: route found   red: rejected   ring: selected', font=font, fill='#334955')
    draw.text((25, 575), 'Saved observation and robot model', font=font, fill='#55666d')
    image.save(path)
