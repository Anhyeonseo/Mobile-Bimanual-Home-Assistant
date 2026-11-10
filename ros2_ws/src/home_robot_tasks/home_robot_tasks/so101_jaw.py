"""Nominal stock SO-101 opening along the closing line through its official TCP.

Both printed jaws must intersect that line. This is a section of the CAD, not
a nameplate maximum, a servo calibration, or proof of contact on a real object.
"""
from dataclasses import dataclass
from functools import lru_cache
import hashlib
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

REVISION = 'a758567c3978dfeefe282ede0500085a48fe8f78'
PARTS = {
    'so101_new_calib.urdf': '3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c',
    'wrist_roll_follower_so101_v1.stl': '4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4',
    'moving_jaw_so101_v1.stl': '785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d',
}


def verified(path):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != PARTS[path.name]:
        raise ValueError(f'wrong SO-101 geometry revision: {path.name}')
    return data


def origin(element):
    rotation = Rotation.from_euler('xyz', np.fromstring(element.get('rpy'), sep=' ')).as_matrix()
    return rotation, np.fromstring(element.get('xyz'), sep=' ')


def mesh(link, name, directory):
    collision = next(c for c in link.findall('collision') if c.find('geometry/mesh').get('filename') == 'assets/' + name)
    record = np.dtype([('normal','<3f4'), ('vertices','<9f4'), ('attribute','<u2')])
    triangles = np.frombuffer(verified(directory / name), record, offset=84)['vertices'].reshape(-1,3,3)
    rotation, xyz = origin(collision.find('origin'))
    return triangles @ rotation.T + xyz


def section(triangles):
    """Segments where mesh triangles intersect the plane y=0 in the hinge basis."""
    end = np.roll(triangles, -1, axis=1)
    keep = (triangles[:,:,1] > 0) != (end[:,:,1] > 0)
    a, b = triangles[keep], end[keep]
    fraction = -a[:,1] / (b[:,1] - a[:,1])
    return (a + fraction[:,None]*(b-a))[:,[0,2]].reshape(-1,2,2)


def intersections(outline):
    a, b = outline[:,0], outline[:,1]
    keep = (a[:,1] > 0) != (b[:,1] > 0)
    a, b = a[keep], b[keep]
    return a[:,0] - a[:,1]*(b[:,0]-a[:,0])/(b[:,1]-a[:,1])


def occupied_slices(triangles, thickness=.004):
    """Enclose the entire solid in TCP-local Z slabs. Include triangles crossing either end."""
    z = triangles[:,:,2]
    edges = np.linspace(z.min(), z.max(), int(np.ceil(np.ptp(z)/thickness)) + 1)
    boxes = []
    for low, high in zip(edges[:-1], edges[1:]):
        tris = triangles[(z.max(axis=1) >= low) & (z.min(axis=1) <= high)]
        if not len(tris):
            continue
        starts = tris.reshape(-1,3)
        ends = np.roll(tris, -1, axis=1).reshape(-1,3)
        points = [starts[(starts[:,2] >= low) & (starts[:,2] <= high)]]
        for cut in (low, high):
            cross = (starts[:,2] < cut) != (ends[:,2] < cut)
            a, b = starts[cross], ends[cross]
            points.append(a + (b-a)*((cut-a[:,2])/(b[:,2]-a[:,2]))[:,None])
        vertices = np.concatenate(points)
        if len(vertices):
            boxes.append([vertices.min(axis=0), vertices.max(axis=0)])
    return np.asarray(boxes)


def touches_slices(slices, pose, centers, half_sizes, margin):
    # The table can be one long axis-aligned row. Its bounding box in the finger
    # frame alone overstates its height. Test the separating axes of both boxes.
    rotation = pose[:3,:3]
    slab_centers = slices.mean(axis=1) @ rotation.T + pose[:3,3]
    slab_half = (slices[:,1] - slices[:,0]) / 2 + margin
    span = slab_half @ abs(rotation).T
    delta = slab_centers[:,None] - centers
    slab, obstacle = np.nonzero(np.all(abs(delta) <= span[:,None] + half_sizes, axis=2))
    if not len(slab):
        return False
    world = np.eye(3)
    axes = np.vstack((world, rotation.T, np.cross(world[:,None], rotation.T[None,:]).reshape(-1,3)))
    distances = abs(delta[slab,obstacle] @ axes.T)
    limits = slab_half[slab] @ abs(axes @ rotation).T + half_sizes[obstacle] @ abs(axes).T
    return bool(np.any(np.all(distances <= limits + 1e-12, axis=1)))


@dataclass(frozen=True)
class StockJaw:
    angles: np.ndarray
    gaps: np.ndarray
    fixed_face_m: float
    tip_to_section_m: float
    fixed_slices: np.ndarray
    moving_slices: np.ndarray
    hinge_axis: np.ndarray
    hinge_point: np.ndarray
    hinge_radius_m: float
    hand_radius_m: float
    fixed_vertices: np.ndarray
    moving_vertices: np.ndarray

    def plane_clearance(self, base_from_stock_tcp, angle, base_from_support):
        """Exact STL surface minima against a plane, not a TCP/skeleton margin.

        Signed distance is linear over each triangle, so checking its vertices
        finds the full surface minimum at this pose. The support plane is
        extended infinitely; callers must scope this to the local pick region.
        This is nominal geometry, not measured real-robot accuracy.
        """
        from .s2_geometry import rigid
        if not np.isfinite(angle) or not self.angles[0] <= angle <= self.angles[-1]:
            raise ValueError('jaw angle outside verified stock section')
        pose=rigid(base_from_stock_tcp);support=rigid(base_from_support)
        rotation=Rotation.from_rotvec(self.hinge_axis*angle).as_matrix()
        moving=(self.moving_vertices-self.hinge_point)@rotation.T+self.hinge_point
        normal=support[:3,2]
        result={}
        for name,vertices in (('fixed',self.fixed_vertices),('moving',moving)):
            world=vertices@pose[:3,:3].T+pose[:3,3]
            distance=(world-support[:3,3])@normal
            index=int(np.argmin(distance))
            result[name]=dict(clearance_m=float(distance[index]),base_point_m=world[index].tolist())
        return result

    def may_touch(self, width, pose, scene_boxes, *, closing=False, margin=.003):
        """Conservative CAD refinement of a coarse jaw overlap, including the closing sweep if requested."""
        if not self.accepts(width):
            raise ValueError('SO101_JAW_SECTION_LIMIT')
        hinge = pose[:3,:3] @ self.hinge_point + pose[:3,3]
        radius = self.hand_radius_m + margin*np.sqrt(3)
        near = np.all((scene_boxes[:,1] >= hinge-radius) & (scene_boxes[:,0] <= hinge+radius), axis=1)
        scene_boxes = scene_boxes[near]
        if not len(scene_boxes):
            return False
        centers = scene_boxes.mean(axis=1)
        half_sizes = (scene_boxes[:,1] - scene_boxes[:,0]) / 2
        if touches_slices(self.fixed_slices, pose, centers, half_sizes, margin):
            return True
        open_angle = float(np.interp(width, self.gaps, self.angles))

        def moving_pose(angle):
            twist = Rotation.from_rotvec(self.hinge_axis * angle).as_matrix()
            transform = np.eye(4)
            transform[:3,:3] = twist
            transform[:3,3] = self.hinge_point - twist @ self.hinge_point
            return pose @ transform

        if not closing:
            return touches_slices(self.moving_slices, moving_pose(open_angle), centers, half_sizes, margin)

        # The physical jaw moves at most radius * half_span from an interval's midpoint.
        # Subdivide ambiguous intervals rather than treating that displacement as an obstacle.
        intervals = [(self.angles[0], open_angle)]
        while intervals:
            start, end = intervals.pop()
            middle = (start + end) / 2
            sample_pose = moving_pose(middle)
            sweep_bound = self.hinge_radius_m * (end - start) / 2
            if not touches_slices(self.moving_slices, sample_pose, centers, half_sizes, margin + sweep_bound):
                continue
            if end - start <= np.radians(.1) or touches_slices(self.moving_slices, sample_pose, centers, half_sizes, margin):
                return True
            intervals.extend(((start, middle), (middle, end)))
        return False

    def accepts(self, width):
        return self.gaps[0] <= width <= self.gaps[-1]

    def describe(self):
        return dict(model='stock_SO-ARM101', cad_revision=REVISION, section='official_gripper_frame_joint',
                    fixed_tip_insertion_mm=1000*self.tip_to_section_m,
                    minimum_mm=1000*float(self.gaps[0]), maximum_mm=1000*float(self.gaps[-1]),
                    maximum_at_deg=float(np.degrees(self.angles[-1])), angular_sampling_deg=.1,
                    installed_pad_or_encoder_verified=False)

    def center_from_fixed_m(self, width):
        if not self.accepts(width):
            raise ValueError('SO101_JAW_SECTION_LIMIT')
        return self.fixed_face_m + width/2


@lru_cache(maxsize=4)
def load_stock_jaw(urdf: Path, meshes: Path):
    root = ElementTree.fromstring(verified(urdf))
    tcp_rot, tcp = origin(root.find("./joint[@name='gripper_frame_joint']/origin"))
    joint = root.find("./joint[@name='gripper']")
    joint_rot, pivot = origin(joint.find('origin'))
    axis = joint_rot @ np.fromstring(joint.find('axis').get('xyz'), sep=' ')
    axis /= np.linalg.norm(axis)
    closing = -tcp_rot[:,0]
    closing -= axis * (closing @ axis)
    closing /= np.linalg.norm(closing)
    basis = np.column_stack((closing, axis, np.cross(closing, axis)))
    fixed = mesh(root.find("./link[@name='gripper_link']"), 'wrist_roll_follower_so101_v1.stl', meshes)
    moving = mesh(root.find("./link[@name='moving_jaw_so101_v1_link']"), 'moving_jaw_so101_v1.stl', meshes)
    fixed_outline = section((fixed - tcp) @ basis)
    moving_outline = section((moving @ joint_rot.T + pivot - tcp) @ basis)
    pivot_xz = ((pivot - tcp) @ basis)[[0,2]]
    face = float(intersections(fixed_outline).max())
    tip_to_section = float(fixed_outline[:,:,1].max())
    limits = joint.find('limit')
    lower, upper = (float(limits.get(key)) for key in ('lower','upper'))
    steps = int(np.ceil((upper-lower)/np.radians(.1)))
    samples, gaps = [], []
    for angle in np.linspace(lower, upper, steps+1):
        cosine, sine = np.cos(angle), np.sin(angle)
        outline = (moving_outline - pivot_xz) @ [[cosine,-sine],[sine,cosine]] + pivot_xz
        inner = intersections(outline)
        if len(inner):
            samples.append(angle)
            gaps.append(float(inner.min()-face))
        elif gaps:
            break  # Moving finger has left the contact line. Empty space is not a wider grip.
    if len(gaps) < 2 or np.any(np.diff(gaps) <= 0):
        raise ValueError('unexpected SO-101 jaw opening curve')
    fixed_tcp = (fixed - tcp) @ tcp_rot
    moving_tcp = (moving @ joint_rot.T + pivot - tcp) @ tcp_rot
    hinge_tcp = (pivot - tcp) @ tcp_rot
    radius = float(np.max(np.linalg.norm(moving_tcp - hinge_tcp, axis=2)))
    hand_radius = max(radius, float(np.max(np.linalg.norm(fixed_tcp - hinge_tcp, axis=2))))
    return StockJaw(np.asarray(samples), np.asarray(gaps), face, tip_to_section, occupied_slices(fixed_tcp),
                    occupied_slices(moving_tcp), axis @ tcp_rot, hinge_tcp, radius, hand_radius,
                    np.unique(fixed_tcp.reshape(-1,3),axis=0),np.unique(moving_tcp.reshape(-1,3),axis=0))
