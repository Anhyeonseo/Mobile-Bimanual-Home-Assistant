"""Build the cabinet route's upper travel envelope from this session's measured arms."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import ConvexHull

from home_robot_tasks import s2_geometry as geom
from home_robot_tasks.camera_evidence import load_capture_profile
from tools.diagnostics.review_s4_sensor_capture import footprint_kernel, slab_footprints, voxel_body_centres
from tools.diagnostics.s2_viewer.build_meshes import read_stl


def hull(points):
    return points[ConvexHull(points).vertices]


def padded_outline(parts, clearance):
    """Nav2 accepts one perimeter. Trace the union of padded convex parts."""
    import cv2
    from home_robot_tasks.nav2_trial import offset_convex_footprint
    # Cover both subpixel rasterization and contour simplification errors.
    step = .001
    padded = [np.asarray(offset_convex_footprint(poly, clearance + 3*step)) for poly in parts]
    points = np.vstack(padded)
    origin = points.min(0) - 2*step
    width, height = np.ceil((points.max(0)-origin)/step).astype(int) + 3
    mask = np.zeros((height, width), np.uint8)
    for poly in padded:
        cv2.fillConvexPoly(mask, np.rint((poly-origin)/step).astype(np.int32), 1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 1:
        raise ValueError('상단 차체와 양팔 외곽을 하나의 주행 형상으로 연결할 수 없습니다.')
    outline = cv2.approxPolyDP(contours[0], 1., True).reshape(-1, 2)
    return (origin + step*outline).tolist()


def measured_arms(root, raw, height, profile, surfaces):
    config = root/'config/home_assistant'
    geometry = geom.load_geometry(config/'official_geometry.json')
    manifest = json.loads((root/'tools/diagnostics/s2_viewer/meshes.json').read_text())
    bounds = []
    for side in ('left', 'right'):
        joints = dict(zip(geom.JOINTS, raw[side], strict=True))
        if any(type(v) is not int or not 0 <= v < 4096 for v in joints.values()):
            raise ValueError('양팔 엔코더 관측이 필요합니다.')
        arm = profile['arms'][side]
        frames = geom.arm_transforms(geometry, geom.model_angles(joints, arm['alignment']))
        mount = np.array(arm['base_from_arm_at_lift_zero'])
        mount[:3, 3] += np.asarray(profile['lift']['axis_base']) * height
        clouds = []
        for visual in manifest['arm_visuals']:
            mesh = visual['mesh']
            points, data = read_stl(root/'ros2_ws/src/so101_description/meshes'/mesh)
            if hashlib.sha256(data).hexdigest() != manifest['meshes'][mesh]['source_sha256']:
                raise ValueError('팔 형상 메시 해시 불일치')
            tf = mount @ frames[visual['link']] @ np.asarray(visual['origin'])
            clouds.append(points @ tf[:3, :3].T + tf[:3, 3])
        for cloud in clouds:
            triangles = cloud.reshape(-1, 3, 3).copy()
            triangles[:, :, 2] -= profile['ground_z_m']
            surfaces.append(triangles)
        points = np.vstack(clouds)
        z = points[:, 2] - profile['ground_z_m']
        bounds.append((hull(points[:, :2]), z.min(), z.max()))
    return bounds


def write_saved_scene(path, voxels, surfaces, boxes, clearance):
    """Keep original map voxel squares and cut the robot meshes at their heights."""
    bands = []
    for level in np.unique(voxels[:, 2]):
        low, high = level*.05-clearance, (level+1)*.05+clearance
        polygons = slab_footprints(surfaces, max(0., low), high) if high > 0 else []
        polygons += [xy for xy, bottom, top in boxes if low <= top and high >= bottom]
        if polygons:
            bands.append((level, polygons, voxels[voxels[:, 2] == level, :2]))
    if not bands:
        raise ValueError('저장 깊이와 로봇 높이 단면을 구성할 수 없습니다.')
    with path.open('w') as stream:
        # Covers DWB cache bin rounding for the measured AlohaMini footprint.
        radius = max(np.linalg.norm(p, axis=1).max() for _, polys, _ in bands for p in polys)
        cache_margin = np.sqrt(2)*.001 + radius*np.deg2rad(.125)
        print('cabinet-depth-v1', .05, clearance+cache_margin, len(bands), file=stream)
        for level, polygons, cells in bands:
            print(level, len(polygons), len(cells), file=stream)
            for poly in polygons:
                # An edge exactly tangent to a slab still has finite extent.
                if len(poly) < 3:
                    poly = hull((poly[:, None, :] + np.array([[0, .0001], [.0001, 0],
                                                              [0, -.0001], [-.0001, 0]])).reshape(-1, 2))
                print(len(poly), *poly.ravel(), file=stream)
            np.savetxt(stream, cells, fmt='%d')
    return bands


def build(root, archive, output, raw, height, calibration_id):
    from tools.run.cabinet_navigation import recorded_file
    config = root/'config/home_assistant'
    profile, _ = load_capture_profile(root, 'near')
    if profile['calibration_id'] != calibration_id or not np.isfinite(height) or height <= 0:
        raise ValueError('상단 보정 기준을 확인할 수 없습니다.')
    route = json.loads((config/'cabinet_navigation.json').read_text())
    saved = json.loads(recorded_file(route, 'map', archive).read_text())
    with np.load(recorded_file(route, 'area', archive), allow_pickle=False) as file:
        area = dict(file)
    with np.load(recorded_file(route, 'depth_voxels', archive), allow_pickle=False) as file:
        voxels = file['voxels'][file['occupied']]
    grid = np.reshape(saved['data'], (saved['height'], saved['width']))
    resolution, origin = saved['resolution'], np.asarray(saved['origin'][:2])
    fixed = json.loads((config/'navigation_body.fixed.json').read_text())
    fixed_xy = np.asarray(fixed['footprint_xy_m'])
    margin = fixed['clearance_m']
    mesh_name = 'base_link.STL'
    mesh = root/'assets/mobile_meshes'/mesh_name
    if not mesh.is_file():
        mesh = root/next(p for p in fixed['source_mesh_sha256'] if Path(p).name == mesh_name)
    vertices, data = read_stl(mesh)
    expected = next(v for p, v in fixed['source_mesh_sha256'].items() if Path(p).name == mesh_name)
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError('차체 메시 해시 불일치')
    base = np.asarray(profile['base_from_cad'])
    chassis = vertices @ base[:3, :3].T + base[:3, 3]
    chassis[:, 2] -= profile['ground_z_m']
    surfaces = [chassis.reshape(-1, 3, 3)]
    parts = measured_arms(root, raw, height, profile, surfaces)
    boxes = []
    # Include the raised carriage and both arm mounts at their new heights.
    displacement = np.array(profile['lift']['axis_base']) * height
    for part in fixed['parts']:
        if part['name'] == mesh_name:
            continue
        raised = part['name'] == 'vertical_link.STL' or 'mounting base' in part['name']
        shift = displacement if raised else 0.
        low, high = np.array(part['minimum_m']) + shift, np.array(part['maximum_m']) + shift
        xy = np.array([[low[0],low[1]], [high[0],low[1]], [high[0],high[1]], [low[0],high[1]]])
        boxes.append((xy, low[2], high[2]))
        if raised:
            parts.append(boxes[-1])
    footprints = [fixed_xy, *(xy for xy, _, _ in parts)]
    polygon = hull(np.vstack(footprints))
    top = max(fixed['height_above_floor_m'], *(hi for _, _, hi in parts))
    # Navigation depth endpoints are in base_link. Mesh limits above are
    # floor-relative. Keep the fixed assembly conservatively covered at all
    # of its heights; folded arms and the carriage each have their own band.
    floor_z = profile['ground_z_m']
    bands_base = [[fixed['minimum_m'][2]+floor_z, fixed['maximum_m'][2]+floor_z],
                  *[[float(lo+floor_z), float(hi+floor_z)] for _, lo, hi in parts]]
    body = dict(mode='measured_upper_travel', footprint_xy_m=polygon.tolist(), clearance_m=margin,
                base_footprint_xy_m=hull(chassis[:,:2]).tolist(),
                self_mask_xy_m=fixed_xy.tolist(), height_above_floor_m=top,
                component_footprints=[xy.tolist() for xy in footprints], component_z_base_m=bands_base,
                padded_outline_xy_m=padded_outline(footprints, margin),
                lift_position_m=height, calibration_id=calibration_id, raw=raw)
    bands = [(xy, np.unique(voxels[((voxels[:, 2]+1)*.05 > lo-margin) &
                    (voxels[:, 2]*.05 < hi+margin), :2], axis=0)) for xy, lo, hi in parts]
    for index, layer in enumerate(area['feasible_by_yaw']):
        yaw = index * 2*np.pi / len(area['feasible_by_yaw'])
        kernel = footprint_kernel(footprints, resolution, yaw, margin)
        layer &= ~binary_dilation((grid >= 65) | (grid < 0), structure=kernel, border_value=1)
        for xy, occupied in bands:
            layer &= ~voxel_body_centres(occupied, [xy], voxel_size=.05, grid_shape=grid.shape,
                                         origin=origin, resolution=resolution, yaw=yaw, clearance=margin)
    area['navigation_footprint_xy_m'] = polygon
    area['review_free_by_yaw'] &= area['feasible_by_yaw']
    output.mkdir(parents=True, exist_ok=True)
    write_saved_scene(output/'saved-depth.scene', voxels, surfaces, boxes, margin)
    body['saved_depth_scene'] = 'saved-depth.scene'
    # Sections for newly observed columns include even levels with no historical depth returns.
    sections = {}
    for level in range(int(np.floor(fixed['minimum_m'][2]/.05)), int(np.ceil(top/.05))):
        lo, hi = level*.05-margin, (level+1)*.05+margin
        polys = slab_footprints(surfaces, max(0., lo), hi)
        polys += [xy for xy, bottom, end in boxes if lo <= end and hi >= bottom]
        if polys:
            sections[level] = [xy.tolist() for xy in polys]
    (output/'upper-sections.json').write_text(json.dumps(sections, allow_nan=False))
    body['ground_z_m'] = floor_z
    np.savez_compressed(output/'upper-area.npz', **area)
    (output/'upper-body.json').write_text(json.dumps(body, allow_nan=False))
    return body


if __name__ == '__main__':
    folder = Path(sys.argv[1])
    observed = json.loads((folder/'upper-posture.json').read_text())
    build(Path.cwd(), Path(sys.argv[2]), folder, **observed)
