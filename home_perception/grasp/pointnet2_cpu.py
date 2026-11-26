"""CPU inference operators for the pinned GraspNet PointNet++ backbone."""
import numpy as np
import torch
from scipy.spatial import cKDTree


def furthest_point_sampling(xyz, npoint):
    batches = xyz.detach().numpy()
    indices = np.zeros((len(batches), npoint), dtype=np.int32)
    for points, selected in zip(batches, indices):
        eligible = np.einsum('ij,ij->i', points, points) > 1e-3
        nearest = np.where(eligible, np.float32(1e10), np.float32(-1))
        for i in range(1, npoint):
            delta = points - points[selected[i - 1]]
            squared = np.einsum('ij,ij->i', delta, delta)
            np.minimum(nearest, squared, out=nearest)
            selected[i] = nearest.argmax()
    return torch.from_numpy(indices)


def group_points(features, indices):
    # B,C,N and B,M[,K] -> B,C,M[,K]
    batches = torch.arange(len(features)).reshape((-1,) + (1,) * (indices.ndim - 1))
    return features.transpose(1, 2)[batches, indices.long()].movedim(-1, 1).contiguous()


gather_points = group_points


def three_nn(unknown, known):
    distances, indices = [], []
    for queries, points in zip(unknown.detach().numpy(), known.detach().numpy()):
        d, i = cKDTree(points).query(queries, k=3)
        distances.append(np.square(d))
        indices.append(np.where(np.isfinite(d), i, 0))
    return (torch.from_numpy(np.asarray(distances, dtype=np.float32)),
            torch.from_numpy(np.asarray(indices, dtype=np.int32)))


def three_interpolate(features, indices, weight):
    return (group_points(features, indices) * weight.unsqueeze(1)).sum(dim=-1)


def _query(centers, xyz, radius, nsample, rotations=None, hmin=0., hmax=0.):
    # Keep the first qualifying input indices and pad with the first match, as
    # in the reference kernels. Query one center at a time to bound Jetson RAM.
    output = np.zeros((*centers.shape[:2], nsample), dtype=np.int32)
    bound = np.hypot(radius, max(abs(hmin), abs(hmax)))
    for batch, (points, queries) in enumerate(zip(xyz.detach().numpy(), centers.detach().numpy())):
        tree = cKDTree(points)
        for j, center in enumerate(queries):
            nearby = np.asarray(tree.query_ball_point(center, bound + 1e-6, return_sorted=True), dtype=np.intp)
            delta = points[nearby] - center
            if rotations is None:
                keep = np.einsum('ij,ij->i', delta, delta) < radius ** 2
            else:
                local = delta @ rotations[batch, j]
                keep = ((local[:, 0] > hmin) & (local[:, 0] < hmax)
                        & ((local[:, 1] ** 2 + local[:, 2] ** 2) < radius ** 2))
            matches = nearby[keep][:nsample]
            if len(matches):
                output[batch, j].fill(matches[0])
                output[batch, j, :len(matches)] = matches
    return torch.from_numpy(output)


def ball_query(new_xyz, xyz, radius, nsample):
    return _query(new_xyz, xyz, radius, nsample)


def cylinder_query(new_xyz, xyz, rot, radius, hmin, hmax, nsample):
    rotations = rot.detach().numpy().reshape(*new_xyz.shape[:2], 3, 3)
    return _query(new_xyz, xyz, radius, nsample, rotations, hmin, hmax)
