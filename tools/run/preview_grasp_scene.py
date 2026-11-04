#!/usr/bin/env python3
"""Show a saved target cloud and SO-ARM101 meshes in one virtual 3D scene.

Default: interactive window. --export-only saves geometry without a display.
--render saves preview.png and exits (requires a working graphics context).
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]

import numpy as np
from tools.lib.grasp_preview import (JOINTS, candidate_meshes, joint_positions,
                                     load_layout, load_robot, pose_matrix, transformed)
from tools.lib.run_manifest import sha256, write_json


class PreviewScene:
    def __init__(self, objects, grasps, layout):
        import open3d as o3d
        self.layout = deepcopy(layout)
        self.robot = load_robot(ROOT / 'ros2_ws/src/so101_description', layout['arm'],
                                ROOT / 'config/bimanual_operational_limits.json')
        self.robot.link_transforms(joint_positions(layout))
        self.meshes = self.robot.visual_meshes()
        self.objects_path, self.grasps_path = Path(objects).resolve(), Path(grasps).resolve()
        objects_report = json.loads((self.objects_path / 'objects.json').read_text())
        grasp_report = json.loads(self.grasps_path.read_text())
        self.grasp_report = grasp_report
        self.ik_result = None
        if Path(grasp_report['objects_path']).resolve() != self.objects_path:
            raise ValueError('saved grasps refer to a different objects directory')
        target = next((obj for obj in objects_report['objects']
                       if obj['index'] == grasp_report['target']['index'] and obj['status'] == 'OK'), None)
        if target is None or target['label'] != grasp_report['target']['label']:
            raise ValueError('matching accepted target cloud required')
        target_path = (self.objects_path / target['cloud']).resolve()
        if not target_path.is_relative_to(self.objects_path):
            raise ValueError('target cloud must be inside objects directory')
        self.target = o3d.io.read_point_cloud(str(target_path))
        self.workspace = o3d.io.read_point_cloud(str(self.objects_path / 'workspace.ply'))
        if self.target.is_empty() or self.workspace.is_empty():
            raise ValueError('target and workspace clouds must not be empty')
        if not np.isfinite(np.asarray(self.target.points)).all() or not np.isfinite(np.asarray(self.workspace.points)).all():
            raise ValueError('non-finite point cloud')
        self.target = self.target.voxel_down_sample(.0015)
        self.workspace = self.workspace.voxel_down_sample(.004)
        self.target.paint_uniform_color([.015, .65, .045])
        self.workspace.paint_uniform_color([.32, .36, .40])
        self.candidates = candidate_meshes(grasp_report)
        self.label, self.grasp_count = target['label'], len(grasp_report['grasps'])
        self.root_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(.08)
        self.camera_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(.055)
        self.tcp_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(.035)

    def ik_current(self):
        if self.ik_result is None:
            return False
        checked = self.ik_result['layout_at_check']
        # Open3D sliders round values through float32. Ignore only rounding
        # far below the pose acceptance thresholds, including after Reset.
        return (checked['arm'] == self.layout['arm']
                and np.allclose(checked['camera_translation_m'], self.layout['camera_translation_m'], rtol=0, atol=1e-7)
                and np.allclose(checked['camera_rpy_degrees'], self.layout['camera_rpy_degrees'], rtol=0, atol=1e-5))

    def check_ik(self, settings):
        from tools.lib.grasp_ik import evaluate_candidates
        snapshot = deepcopy(self.layout)
        result = evaluate_candidates(self.robot, snapshot, self.grasp_report['grasps'], settings)
        result['layout_at_check'] = snapshot
        self.ik_result = result
        return result

    def apply_selected_ik(self):
        if not self.ik_current() or self.ik_result['selected_index'] is None:
            raise ValueError('a current passing IK candidate is required')
        selected = next(c for c in self.ik_result['candidates'] if c['candidate_index'] == self.ik_result['selected_index'])
        updated = deepcopy(self.layout)
        for name, angle in selected['joint_positions_rad'].items():
            joint = name.removeprefix(self.layout['arm'] + '_').removesuffix('_joint')
            updated['joint_positions_degrees'][joint] = float(np.degrees(angle))
        self.robot.link_transforms(joint_positions(updated))
        self.layout = updated

    def candidate_color_key(self):
        return tuple(c['status'] for c in self.ik_result['candidates']) if self.ik_current() else None

    def candidate_geometry(self):
        if not self.ik_current():
            return self.candidates
        colors = [[.02, .7, .12] if c['kinematically_feasible'] else [.95, .035, .02]
                  for c in self.ik_result['candidates']]
        return candidate_meshes(self.grasp_report, colors)

    def transforms(self):
        links = self.robot.link_transforms(joint_positions(self.layout))
        camera = pose_matrix(self.layout['camera_translation_m'], self.layout['camera_rpy_degrees'])
        return links, camera

    def geometries(self):
        links, camera = self.transforms()
        geometry = {f'robot/{name}': transformed(mesh, links[name]) for name, mesh in self.meshes.items()}
        geometry.update(workspace=transformed(self.workspace, camera),
                        target=transformed(self.target, camera),
                        root_axes=deepcopy(self.root_axes),
                        camera_axes=transformed(self.camera_axes, camera),
                        tcp_axes=transformed(self.tcp_axes, links[self.layout['arm'] + '_gripper_frame_link']))
        if not self.candidates.is_empty():
            geometry['candidates'] = transformed(self.candidate_geometry(), camera)
        return geometry

    def statistics(self):
        links, camera = self.transforms()
        tcp = links[self.layout['arm'] + '_gripper_frame_link'][:3, 3]
        target = camera[:3, :3] @ self.target.get_center() + camera[:3, 3]
        return {'target_center_m': target.tolist(), 'tcp_position_m': tcp.tolist(),
                'manual_tcp_to_target_m': float(np.linalg.norm(tcp - target))}

    def export(self, output):
        import open3d as o3d
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        geometry = self.geometries()
        robot = o3d.geometry.TriangleMesh()
        for name, mesh in geometry.items():
            if name.startswith('robot/'):
                robot += mesh
        for name, value in (('robot', robot), ('workspace', geometry['workspace']), ('target', geometry['target'])):
            writer = o3d.io.write_point_cloud if isinstance(value, o3d.geometry.PointCloud) else o3d.io.write_triangle_mesh
            if not writer(str(output / (name + '.ply')), value):
                raise RuntimeError('failed to export ' + name)
        if 'candidates' in geometry:
            if not o3d.io.write_triangle_mesh(str(output / 'candidates.ply'), geometry['candidates']):
                raise RuntimeError('failed to export candidates')
        else:
            (output / 'candidates.ply').unlink(missing_ok=True)
        (output / 'robot.urdf').write_text(self.robot.xml, encoding='utf-8')
        write_json(output / 'layout.json', self.layout)
        write_json(output / 'report.json', {
            'schema_version': 1, 'mode': 'virtual_preview', 'reference_frame': self.robot.root,
            'camera_transform_measured': False, 'robot_feasibility_checked': False,
            'ik_checked_for_current_target': self.ik_current(),
            'collision_checked': False, 'hardware_commands': 0, 'physical_task_completed': False,
            'target': self.label, 'candidate_count': self.grasp_count,
            'objects_directory': str(self.objects_path), 'grasps_sha256': sha256(self.grasps_path),
            'robot_model': 'repository SO-ARM101 standalone visual URDF',
            'gripper_angle_kind': 'URDF visualization angle, not calibrated servo/jaw gap',
            **self.statistics()})
        if self.ik_result is not None:
            write_json(output / 'ik_report.json', {**self.ik_result, 'matches_current_target': self.ik_current()})
        else:
            (output / 'ik_report.json').unlink(missing_ok=True)


def material(point_cloud=False):
    from open3d.visualization import rendering
    result = rendering.MaterialRecord()
    result.shader = 'defaultUnlit' if point_cloud else 'defaultLit'
    result.point_size = 3.0
    return result


def view_parameters(geometries):
    # Background depth can contain distant outliers. Frame the actual subject;
    # leave workspace points unchanged rather than using them to zoom far out.
    focus = [g for name, g in geometries.items() if name.startswith('robot/') or name in ('target', 'candidates', 'tcp_axes')]
    minimum = np.min([g.get_min_bound() for g in focus], axis=0)
    maximum = np.max([g.get_max_bound() for g in focus], axis=0)
    center = (minimum + maximum) / 2
    extent = max(float(np.max(maximum - minimum)), .35)
    eye = center + np.array([.85, -1.15, .95]) * extent
    return center, eye


def render_preview(scene, output):
    import open3d as o3d
    from open3d.visualization import rendering
    from PIL import Image, ImageDraw, ImageFont
    renderer = rendering.OffscreenRenderer(1400, 1000)
    renderer.scene.set_background([.96, .97, .98, 1.])
    renderer.scene.scene.enable_sun_light(True)
    renderer.scene.scene.set_sun_light([-.5, -.3, -1.], [1., 1., 1.], 60000)
    geometry = scene.geometries()
    for name, value in geometry.items():
        renderer.scene.add_geometry(name, value, material(isinstance(value, o3d.geometry.PointCloud)))
    center, eye = view_parameters(geometry)
    renderer.setup_camera(55., center, eye, [0., 0., 1.], .005, 10.)
    # The Vulkan backend may return an empty first readback while geometry
    # uploads are pending. Render another frame and refuse a blank export.
    for _ in range(3):
        pixels = np.asarray(renderer.render_to_image()).copy()
        if np.any(pixels):
            break
    else:
        raise RuntimeError('renderer returned only blank frames')
    preview = Image.fromarray(pixels)
    draw = ImageDraw.Draw(preview)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 22) if Path(font_path).exists() else ImageFont.load_default()
    small = ImageFont.truetype(font_path, 17) if Path(font_path).exists() else font
    draw.rectangle([0, 0, 1400, 100], fill='#152334')
    draw.text((25, 16), f'SO-ARM101 + {scene.label} / VIRTUAL LAYOUT', font=font, fill='white')
    if scene.ik_current():
        selected = scene.ik_result['selected_index']
        if selected is not None:
            candidate = next(c for c in scene.ik_result['candidates'] if c['candidate_index'] == selected)
            caption = f'IK PASS #{selected}: position {candidate["position_error_m"]*1000:.2f} mm / orientation {candidate["orientation_error_degrees"]:.2f} deg. Collision/path NOT checked.'
        else:
            caption = 'No IK solution within configured tolerances. Collision/path NOT checked.'
    else:
        caption = 'Camera placement is illustrative. Pose IK not checked. Collision/path NOT checked.'
    draw.text((25, 56), caption, font=small, fill='#ffc36b')
    draw.rectangle([0, 946, 1400, 1000], fill='#152334')
    draw.text((25, 960), 'Yellow: robot   Green: target   Grasp: orange=unchecked / green=IK pass / red=IK fail   Gray: workspace',
              font=small, fill='white')
    preview.save(Path(output) / 'preview.png')


class PreviewWindow:
    def __init__(self, scene, output, ik_settings=None):
        import open3d as o3d
        from open3d.visualization import gui, rendering
        self.scene, self.output, self.initial = scene, output, deepcopy(scene.layout)
        self.ik_settings = ik_settings
        self.checking = False
        self._candidate_state = scene.candidate_color_key()
        self.syncing = False
        self.window = gui.Application.instance.create_window('SO-ARM101 + target | Pose IK (VIRTUAL)', 1480, 980)
        self.view = gui.SceneWidget()
        self.view.scene = rendering.Open3DScene(self.window.renderer)
        self.view.scene.set_background([.96, .97, .98, 1.])
        self.view.scene.scene.enable_sun_light(True)
        self.view.scene.scene.set_sun_light([-.5, -.3, -1.], [1., 1., 1.], 60000)
        for name, value in scene.geometries().items():
            self.view.scene.add_geometry(name, value, material(isinstance(value, o3d.geometry.PointCloud)))
        em = self.window.theme.font_size
        self.panel = gui.ScrollableVert(.3 * em, gui.Margins(em, em, em, em))
        for text in ('VIRTUAL LAYOUT / meters', 'Pose IK only; collision/path not checked',
                     'Yellow: SO-ARM101 | Green: target', 'Orange: generic grasp | Gray: workspace',
                     'Mouse: rotate / pan / zoom'):
            self.panel.add_child(gui.Label(text))
        self.check_button = gui.Button('Check candidate IK')
        self.check_button.set_on_clicked(self.check_ik)
        self.panel.add_child(self.check_button)
        self.apply_button = gui.Button('Show selected IK pose')
        self.apply_button.set_on_clicked(self.apply_ik)
        self.panel.add_child(self.apply_button)
        self.ik_status = gui.Label('IK not checked.')
        self.panel.add_child(self.ik_status)
        self.panel.add_fixed(.6 * em)
        self.panel.add_child(gui.Label('ARM JOINTS (degrees)'))
        self.sliders, self.value_labels = {}, {}
        for name in JOINTS:
            bounds = np.degrees(scene.robot.bounds[f'{scene.layout["arm"]}_{name}_joint'])
            self.add_slider('joint/' + name, name, bounds[0], bounds[1], scene.layout['joint_positions_degrees'][name])
        self.panel.add_fixed(.6 * em)
        self.panel.add_child(gui.Label('VIRTUAL CAMERA -> ARM ROOT'))
        for i, axis in enumerate(('X (m)', 'Y (m)', 'Z (m)')):
            self.add_slider('xyz/' + str(i), axis, -1., 1.5, scene.layout['camera_translation_m'][i])
        for i, axis in enumerate(('Roll (deg)', 'Pitch (deg)', 'Yaw (deg)')):
            self.add_slider('rpy/' + str(i), axis, -180., 180., scene.layout['camera_rpy_degrees'][i])
        self.panel.add_fixed(.5 * em)
        row = gui.Horiz(.3 * em)
        for text, callback in (('Reset layout', self.reset), ('Fit view', self.fit)):
            button = gui.Button(text)
            button.set_on_clicked(callback)
            row.add_child(button)
        self.panel.add_child(row)
        save = gui.Button('Save layout + geometry')
        save.set_on_clicked(self.save)
        self.panel.add_child(save)
        self.status = gui.Label('')
        self.panel.add_child(self.status)
        self.panel.add_child(gui.Label('Gripper slider = visual jaw angle.'))
        self.window.add_child(self.view)
        self.window.add_child(self.panel)
        self.window.set_on_layout(self.layout)
        self.fit()
        self.update_status()

    def add_slider(self, key, name, lower, upper, value):
        from open3d.visualization import gui
        label = gui.Label(f'{name}: {value:.3f}')
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(float(lower), float(upper))
        slider.double_value = value
        slider.set_on_value_changed(lambda current, k=key, n=name: self.change(k, n, current))
        self.panel.add_child(label)
        self.panel.add_child(slider)
        self.sliders[key], self.value_labels[key] = slider, (label, name)

    def layout(self, context):
        from open3d.visualization import gui
        rect = self.window.content_rect
        panel_width = 350
        self.view.frame = gui.Rect(rect.x, rect.y, rect.width - panel_width, rect.height)
        self.panel.frame = gui.Rect(rect.get_right() - panel_width, rect.y, panel_width, rect.height)

    def change(self, key, name, value):
        if self.syncing:
            return
        kind, field = key.split('/')
        if kind == 'joint':
            self.scene.layout['joint_positions_degrees'][field] = float(value)
        else:
            self.scene.layout['camera_translation_m' if kind == 'xyz' else 'camera_rpy_degrees'][int(field)] = float(value)
        self.value_labels[key][0].text = f'{name}: {value:.3f}'
        self.refresh()

    def refresh(self):
        # Uploaded geometry already has the initial transform. Rebase updates
        # against that initial transform to avoid applying an origin twice.
        current_links, current_camera = self.scene.transforms()
        current = self.scene.layout
        self.scene.layout = self.initial
        initial_links, initial_camera = self.scene.transforms()
        self.scene.layout = current
        color_key = self.scene.candidate_color_key()
        if color_key != self._candidate_state:
            self.view.scene.remove_geometry('candidates')
            if not self.scene.candidates.is_empty():
                self.view.scene.add_geometry('candidates', transformed(self.scene.candidate_geometry(), initial_camera), material())
            self._candidate_state = color_key
        for link in self.scene.meshes:
            self.view.scene.set_geometry_transform('robot/' + link, current_links[link] @ np.linalg.inv(initial_links[link]))
        for name in ('workspace', 'target', 'camera_axes', 'candidates'):
            if self.view.scene.has_geometry(name):
                self.view.scene.set_geometry_transform(name, current_camera @ np.linalg.inv(initial_camera))
        tcp = self.scene.layout['arm'] + '_gripper_frame_link'
        self.view.scene.set_geometry_transform('tcp_axes', current_links[tcp] @ np.linalg.inv(initial_links[tcp]))
        self.update_status()
        self.view.force_redraw()

    def update_status(self):
        distance = self.scene.statistics()['manual_tcp_to_target_m']
        self.status.text = f'TCP-to-target cloud center: {distance:.3f} m'
        self.apply_button.enabled = not self.checking and self.scene.ik_current() and self.scene.ik_result['selected_index'] is not None
        if self.checking:
            self.ik_status.text = 'Checking bounded pose IK...'
        elif self.scene.ik_current():
            selected = self.scene.ik_result['selected_index']
            if selected is not None:
                candidate = next(c for c in self.scene.ik_result['candidates'] if c['candidate_index'] == selected)
                self.ik_status.text = (f'IK PASS: candidate {selected}\n'
                                       f'Position: {candidate["position_error_m"]*1000:.2f} mm\n'
                                       f'Orientation: {candidate["orientation_error_degrees"]:.2f} deg\n'
                                       'Collision / path / jaw fit not checked.')
            else:
                reason = self.scene.ik_result['candidates'][0]['status'] if self.scene.ik_result['candidates'] else 'NO_CANDIDATES'
                self.ik_status.text = 'No passing IK candidate.\n' + reason
        elif self.scene.ik_result is not None:
            self.ik_status.text = 'Camera layout changed. Check IK again.'
        else:
            self.ik_status.text = 'IK not checked.'

    def check_ik(self):
        if self.checking:
            return
        from open3d.visualization import gui
        from tools.lib.grasp_ik import evaluate_candidates, load_settings
        settings = self.ik_settings or load_settings(ROOT / 'config/grasp_ik.virtual.json')
        snapshot = deepcopy(self.scene.layout)
        self.checking = True
        self.check_button.enabled = False
        self.update_status()

        def finish(result, error):
            self.checking = False
            self.check_button.enabled = True
            if error is not None:
                self.update_status()
                self.ik_status.text = 'IK error: ' + str(error)
                return
            result['layout_at_check'] = snapshot
            self.scene.ik_result = result
            write_json(Path(self.output) / 'ik_report.json', {**result, 'matches_current_target': self.scene.ik_current()})
            self.refresh()

        def worker():
            try:
                result = evaluate_candidates(self.scene.robot, snapshot, self.scene.grasp_report['grasps'], settings)
                error = None
            except Exception as caught:
                result, error = None, caught
            gui.Application.instance.post_to_main_thread(self.window, lambda: finish(result, error))
        threading.Thread(target=worker, daemon=True).start()

    def sync_sliders(self):
        self.syncing = True
        for key, slider in self.sliders.items():
            kind, name = key.split('/')
            value = (self.scene.layout['joint_positions_degrees'][name] if kind == 'joint'
                     else self.scene.layout['camera_translation_m' if kind == 'xyz' else 'camera_rpy_degrees'][int(name)])
            slider.double_value = value
            label, title = self.value_labels[key]
            label.text = f'{title}: {value:.3f}'
        self.syncing = False

    def apply_ik(self):
        self.scene.apply_selected_ik()
        self.sync_sliders()
        self.refresh()
        self.scene.export(self.output)

    def reset(self):
        self.scene.layout = deepcopy(self.initial)
        self.sync_sliders()
        self.refresh()
        self.fit()

    def fit(self):
        import open3d as o3d
        geometry = self.scene.geometries()
        points = np.asarray([point for name, value in geometry.items()
                             if name.startswith('robot/') or name in ('target', 'candidates', 'tcp_axes')
                             for point in (value.get_min_bound(), value.get_max_bound())])
        bounds = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
        center, eye = view_parameters(geometry)
        self.view.setup_camera(55., bounds, center)
        self.view.look_at(center, eye, [0., 0., 1.])

    def save(self):
        self.scene.export(self.output)
        self.status.text = 'Saved layout.json and scene geometry.'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objects', type=Path, default=ROOT / 'output/perception-pipeline-gpu/objects')
    parser.add_argument('--grasps', type=Path, default=ROOT / 'output/perception-pipeline-gpu/grasps/grasps.json')
    parser.add_argument('--layout', type=Path, help='Default: saved output/layout.json, otherwise repository virtual layout')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/grasp-preview')
    parser.add_argument('--export-only', action='store_true', help='Save geometry/layout without opening a window')
    parser.add_argument('--render', action='store_true', help='Render preview.png and exit; requires graphics access')
    parser.add_argument('--check-ik', action='store_true', help='Evaluate saved candidates with bounded pose IK')
    parser.add_argument('--show-ik', action='store_true', help='Check IK and show a passing joint solution; no execution')
    parser.add_argument('--ik-settings', type=Path, default=ROOT / 'config/grasp_ik.virtual.json')
    args = parser.parse_args(argv)
    try:
        saved_layout = args.output / 'layout.json'
        layout = args.layout or (saved_layout if saved_layout.is_file() else ROOT / 'config/grasp_preview.virtual.json')
        scene = PreviewScene(args.objects, args.grasps, load_layout(layout))
        if args.check_ik or args.show_ik:
            from tools.lib.grasp_ik import load_settings
            result = scene.check_ik(load_settings(args.ik_settings))
            print(f'Pose IK: {result["accepted_count"]}/{result["candidate_count"]} pass; selected={result["selected_index"]}', flush=True)
            if args.show_ik and result['selected_index'] is not None:
                scene.apply_selected_ik()
        scene.export(args.output)
        print(f'Virtual scene: {scene.label}, {scene.grasp_count} grasp(s)\nOutput: {args.output.resolve()}\nCamera placement is virtual; collision/path/jaw fit not checked.', flush=True)
        if args.render:
            render_preview(scene, args.output)
            print(f'Preview: {(args.output / "preview.png").resolve()}', flush=True)
        elif not args.export_only:
            from open3d.visualization import gui
            gui.Application.instance.initialize()
            from tools.lib.grasp_ik import load_settings
            window = PreviewWindow(scene, args.output, load_settings(args.ik_settings))
            gui.Application.instance.run()
        return 0
    except (OSError, ValueError, KeyError, ImportError, RuntimeError) as error:
        parser.exit(2, f'Preview error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
