"""Read-only environment report; no model download, socket or device opening."""
import importlib.util
import ast
from pathlib import Path


def model_snapshots(root):
    """Relative cache directories used by both the station and its offline kit."""
    for source in ('grounding_rgbd.py', 'sam_rgbd.py'):
        tree = ast.parse((root/'tools/lib'/source).read_text())
        values = {node.targets[0].id: ast.literal_eval(node.value) for node in tree.body
                  if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                  and node.targets[0].id in ('MODEL_ID', 'MODEL_REVISION')}
        yield (Path('models/huggingface')/('models--'+values['MODEL_ID'].replace('/', '--'))
               /'snapshots'/values['MODEL_REVISION'])


def check(options):
    root=Path(__file__).resolve().parents[2]
    modules=['numpy','scipy','PIL','cv2','torch','torchvision','transformers','yaml','rclpy','moveit_msgs','pyrealsense2']
    if options['mode']=='live':modules+=['serial']
    missing=[name for name in modules if importlib.util.find_spec(name) is None]
    artifacts=Path(options.get('artifacts_dir') or root/'artifacts')
    files=[]
    for snapshot in model_snapshots(root):
        path=artifacts/snapshot
        if not path.is_dir():files.append(str(path))
    if options.get('candidate_source') in ('graspnet','graspnet-first','graspnet-so101'):
        for path in (artifacts/'models/checkpoint-rs.tar',artifacts/'graspnet-baseline'):
            if not path.exists():files.append(str(path))
    meshes=Path(options['mobile_mesh_dir'])
    for name in ('base_link.STL','vertical_link.STL','wheel1.STL'):
        if not (meshes/name).is_file():files.append(str(meshes/name))
    return dict(status='DEPENDENCIES_PRESENT' if not missing and not files else 'SETUP_REQUIRED',
                missing_python_modules=missing,missing_local_paths=files,
                model_cache_contents_fully_validated=False,devices_opened=False,hardware_commands=0,
                robot_execution_verified=False,
                note='Dependency presence only. Calibration hashes, model contents and live feedback are checked at startup.')
