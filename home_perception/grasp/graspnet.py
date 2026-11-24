"""GraspNet target proposals; legacy generic-gripper checks are audit data only."""
import hashlib
from pathlib import Path
from threading import Lock
import sys
import types

import numpy as np
from scipy.spatial import cKDTree

from home_perception.geometry.plane import backproject
from home_perception.grasp.handoff import CONVENTION, PRODUCER_REVISION, candidate_from_row
from home_perception.grasp.refine import SUPPORT_MARGIN_M, refine
from home_perception.grasp.proposals import select as select_proposals

from home_perception.grasp.graspnet_assets import BASELINE_REVISION, CHECKPOINT_SHA256, verify_baseline

ARTIFACTS = Path(__file__).resolve().parents[2] / 'artifacts'
CHECKPOINT = ARTIFACTS / 'models/checkpoint-rs.tar'
_import_lock = Lock()


def _baseline(device='cuda', audit=True):
    import torch

    root = ARTIFACTS / 'graspnet-baseline'
    verify_baseline(root)
    if device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA was requested but is unavailable')
    # The baseline imports its training-only nearest-neighbour extension at module import.
    # The team's 1-based torch implementation avoids a second compiled extension.
    def knn(ref, query, k=1):
        indices = torch.cdist(query.transpose(1, 2), ref.transpose(1, 2)).topk(k, largest=False).indices
        return (indices + 1).transpose(1, 2).contiguous()

    knn_module = types.ModuleType('knn_modules')
    knn_module.knn = knn
    # The reference repository uses unqualified imports. Bind its modules only
    # while loading, so a CPU worker and a native CUDA runtime can coexist.
    names = ('knn_modules', 'pointnet2', 'pointnet2._ext', 'graspnet', 'backbone',
             'modules', 'loss', 'pointnet2_modules', 'pointnet2_utils', 'pytorch_utils',
             'loss_utils', 'label_generation', 'collision_detector')
    with _import_lock:
        original_path = sys.path[:]
        previous = {name: sys.modules.pop(name) for name in names if name in sys.modules}
        try:
            sys.path[:0] = [str(root / name) for name in ('models', 'utils', 'pointnet2')]
            if device == 'cpu':
                from home_perception.grasp import pointnet2_cpu
                package = types.ModuleType('pointnet2')
                package.__path__ = []
                package._ext = pointnet2_cpu
                sys.modules.update({'pointnet2': package, 'pointnet2._ext': pointnet2_cpu})
            else:
                sys.path.insert(0, str(ARTIFACTS / 'graspnet-runtime'))
            sys.modules['knn_modules'] = knn_module
            from graspnet import GraspNet, pred_decode
            ModelFreeCollisionDetector = None
            if audit:
                from collision_detector import ModelFreeCollisionDetector
        finally:
            sys.path[:] = original_path
            for name in names:
                sys.modules.pop(name, None)
            sys.modules.update(previous)
    return GraspNet, pred_decode, ModelFreeCollisionDetector


class GraspNetRuntime:
    """Explicit process-local model lifetime. Construct once, infer many times."""
    def __init__(self, checkpoint=CHECKPOINT, *, device='cuda', audit=True):
        import torch
        if device not in ('cpu', 'cuda'):
            raise ValueError('expected cpu or cuda')
        self.device = device
        self.checkpoint = Path(checkpoint).resolve()
        with self.checkpoint.open('rb') as file:
            if hashlib.sha256(file.read()).hexdigest() != CHECKPOINT_SHA256:
                raise ValueError('unexpected RealSense GraspNet checkpoint digest')
            file.seek(0)
            GraspNet, self.decode, self.detector = _baseline(device, audit)
            net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                          cylinder_radius=.05, hmin=-.02, hmax_list=[.01,.02,.03,.04], is_training=False)
            net.load_state_dict(torch.load(file, map_location='cpu', weights_only=True)['model_state_dict'], strict=True)
        self.net = net.to(device).eval()

    def __call__(self, points, checkpoint=CHECKPOINT):
        import torch
        if Path(checkpoint).resolve() != self.checkpoint:
            raise ValueError('runtime/checkpoint mismatch; construct a new runtime')
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise ValueError('finite nonempty XYZ point cloud required')
        selected = np.random.default_rng(0).choice(len(points), 20000, replace=len(points) < 20000)
        torch.manual_seed(0)
        cloud = torch.as_tensor(points[selected][None], dtype=torch.float32, device=self.device)
        with torch.inference_mode():
            rows = self.decode(self.net({'point_clouds': cloud}))[0].cpu().numpy()
        return rows, self.detector


def infer(points, checkpoint):
    # Historical one-shot API; resident applications explicitly own the runtime.
    return GraspNetRuntime(checkpoint)(points, checkpoint)


def associate(rows, intr, mask, target):
    """Require both a projection inside the current mask and measured target points within 3 cm."""
    centers = rows[:, 13:16]
    valid = np.flatnonzero((centers[:, 2] > 0) & (rows[:, 1] > 0) & (rows[:, 3] > 0))
    uv = np.rint(centers[valid, :2] / centers[valid, 2:] * [intr['fx'], intr['fy']] + [intr['ppx'], intr['ppy']])
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < mask.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < mask.shape[0])
    u, v = uv[inside].astype(int).T
    valid = valid[inside][mask[v, u]]
    distances = cKDTree(target).query(centers[valid])[0]
    return rows[valid[distances <= .03]]


def collision_filter(rows, scene, detector_type):
    if not len(rows):
        return rows
    detector = detector_type(scene, voxel_size=.005)
    keep = []
    for start in range(0, len(rows), 32):
        chunk = rows[start:start+32]
        # ModelFreeCollisionDetector uses just these five GraspGroup properties.
        group = types.SimpleNamespace(translations=chunk[:, 13:16], rotation_matrices=chunk[:, 4:13].reshape(-1, 3, 3),
                                      heights=chunk[:, 2], depths=chunk[:, 3], widths=chunk[:, 1])
        collision, empty = detector.detect(group, approach_dist=.05, collision_thresh=.01, return_empty_grasp=True)
        keep.extend(~(collision | empty))
    return rows[np.asarray(keep, dtype=bool)]


def rank_for_jaw(filtered, target, plane, height, opening_range):
    # A wider grasp at the same pose must not suppress a feasible opening during duplicate removal.
    original, rejected = refine(filtered, target, plane, height)
    low, high = opening_range
    fits = (filtered[:, 1] >= low) & (filtered[:, 1] <= high)
    fitted, _ = refine(filtered[fits], target, plane, height)
    outside = original[(original[:, 1] < low) | (original[:, 1] > high)]
    return np.vstack((fitted, outside)), rejected


def candidates(depth, intr, masks, index, scene, plane, height, opening_range, checkpoint=CHECKPOINT, *, section_points=None, audit_enabled=True, inference=None):
    from home_perception.geometry.workspace import workspace_mask

    valid = depth > 0
    target = backproject(depth, intr, valid & masks[index])
    workspace = backproject(depth, intr, valid & workspace_mask(depth, intr, plane, masks))
    if not len(target):
        raise ValueError('selected object has no measured depth')
    points = workspace[np.linalg.norm(workspace - target.mean(axis=0), axis=1) <= .15]
    if not len(points):
        raise ValueError('no measured points in the target sampling region')
    raw, detector = (inference or infer)(points, checkpoint)
    rows = raw[np.isfinite(raw).all(axis=1)]
    associated = associate(rows, intr, masks[index], target)
    audit = dict(enabled=bool(audit_enabled), authoritative=False)
    if audit_enabled:
        filtered = collision_filter(associated, scene, detector)
        kept, rejected = rank_for_jaw(filtered, target, plane, height, opening_range)
        audit.update(collision_and_empty_filtered=len(filtered), refined=len(kept), rejected_by_check=rejected)
    if section_points is None:
        section_points = target[plane.height(target) > SUPPORT_MARGIN_M]
    proposals, selection = select_proposals(associated, section_points, plane, opening_range, centered_contacts=True)
    reason = ('graspnet_proposals_require_robot_check' if proposals else 'no_target_associated' if not len(associated)
              else 'no_observed_robot_width_proposal')
    return dict(tcp_convention=CONVENTION, producer_revision=PRODUCER_REVISION, baseline_revision=BASELINE_REVISION,
                checkpoint_sha256=CHECKPOINT_SHA256, candidate_reason=reason, proposal_selection=selection,
                legacy_parallel_jaw_audit=audit,
                counts=dict(generated=len(raw), target_associated=len(associated),
                            refined=len(proposals)),
                refined_grasps=proposals)
