"""Separate nominal padding from observed material at a known target boundary.

Subtract only the carried target's convex volume from an obstacle's nominal
envelope, and only when every measured return lies outside that volume. Returns
remain world obstacles; this never reassigns them to the target or fills depth.
"""
from itertools import combinations, product

import numpy as np
from scipy.spatial import ConvexHull, QhullError


def _mesh(vertices):
    vertices = np.unique(np.round(vertices, 12), axis=0)
    if len(vertices) < 4:
        return None
    try:
        hull = ConvexHull(vertices)
    except QhullError:
        return None
    if hull.volume < 1e-18:
        return None
    faces = hull.simplices.copy()
    for face, equation in zip(faces, hull.equations):
        a, b, c = vertices[face]
        if np.dot(np.cross(b-a, c-a), equation[:3]) < 0:
            face[1], face[2] = face[2], face[1]
    return dict(vertices=vertices.tolist(), faces=faces.tolist())


def _clip(vertices, equation):
    """Keep the closed negative half-space of one plane."""
    vertices = np.asarray(vertices)
    distance = vertices @ equation[:3] + equation[3]
    keep = vertices[distance <= 1e-12].tolist()
    if not keep or np.all(distance <= 1e-12):
        return vertices if keep else np.empty((0, 3))
    # Every pair is bounded (these are small convex polyhedra). Redundant
    # interior intersections are removed by the final convex hull.
    for a, b in combinations(range(len(vertices)), 2):
        if distance[a] * distance[b] < 0:
            keep.append((vertices[a] + distance[a] / (distance[a]-distance[b])
                         * (vertices[b]-vertices[a])).tolist())
    mesh = _mesh(keep)
    if mesh is None:
        return np.empty((0, 3))
    v = np.asarray(mesh['vertices'])
    return v[np.unique(mesh['faces'])]


def separate_target_padding(bounds, returns, target_vertices):
    """Return convex mesh pieces, or None when the original box must remain.

    The one-micrometre separation is numerical only. No measured return may
    fall in it. Real target/world overlap and exact touching remain unresolved.
    """
    bounds, returns, target_vertices = map(np.asarray, (bounds, returns, target_vertices))
    if (len(returns) == 0 or np.any(bounds[1] < target_vertices.min(axis=0))
            or np.any(bounds[0] > target_vertices.max(axis=0))):
        return None
    equations = ConvexHull(target_vertices).equations.copy()
    equations[:, 3] -= 1e-6
    if np.any(np.max(returns @ equations[:, :3].T + equations[:, 3], axis=1) <= 1e-10):
        return None
    vertices = np.array(list(product(*zip(bounds[0], bounds[1]))))
    # First establish a genuine volumetric envelope overlap.
    intersection = vertices
    equations = np.unique(np.round(equations, 12), axis=0)
    for equation in equations:
        intersection = _clip(intersection, equation)
        if len(intersection) == 0:
            return None
    if _mesh(intersection) is None:
        return None
    pieces = []
    remaining = vertices
    for equation in equations:
        outside = _mesh(_clip(remaining, -equation))
        if outside is not None:
            pieces.append(outside)
        remaining = _clip(remaining, equation)
        if not len(remaining):
            break
    # Conservation is checked on the actual measurements, not box corners.
    covered = np.zeros(len(returns), dtype=bool)
    for mesh in pieces:
        e = ConvexHull(mesh['vertices']).equations
        covered |= np.all(returns @ e[:, :3].T + e[:, 3] <= 1e-10, axis=1)
    if not covered.all():
        return None
    return pieces
