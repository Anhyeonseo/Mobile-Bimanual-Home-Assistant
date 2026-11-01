"""Planning-map snapshot from planar LiDAR and full-height native depth.

The SLAM map is immutable. Laser free space never clears elevated depth hits.
This pure adapter produces an offline NavigationMap, not a commissioned Nav2
global publisher. Live motion still needs the independent fresh TravelGuard.
"""
from copy import deepcopy
from dataclasses import dataclass

from .fetch import InvalidTask
from .navigation import NavigationMap, identifier, number
from .ros_navigation_depth import projected_grid


@dataclass(frozen=True)
class TravelEnvelope:
    """Measured travel posture including chassis, stowed arms, lift and payload.

No ALOHA Mini 1 dimensions are guessed here. Height includes the vertical
clearance margin; the navigation map's clearance adds the horizontal margin.
"""
    geometry_id: str
    radius_m: float
    height_m: float

    def __post_init__(self):
        identifier(self.geometry_id, 'travel geometry revision')
        if not 0 < number(self.radius_m, 'full body radius') <= 2 or not 0 < number(self.height_m, 'full body height') <= 3:
            raise InvalidTask('invalid full body travel envelope')


def fuse_navigation_map(lidar_document, volume, envelope, *, calibration_id,
                        expected_epoch, revision, now_s, purpose='global_route_proposal'):
    """Same-frame/lattice input only; never silently resize or guess a TF.

    Free requires LiDAR free AND every configured height voxel observed free.
    Global proposals may use scan history; local clearance requires fresh free.
    Any obstacle wins, including old depth hits not yet cleared by valid 3D rays.
    Unknown is blocked by NavigationMap. Partial furniture geometry is NOT input.
    """
    nav = NavigationMap(lidar_document)
    if purpose not in ('global_route_proposal', 'local_traversability'):
        raise InvalidTask('invalid planning map purpose')
    identifier(revision, 'fused map revision')
    if not isinstance(envelope, TravelEnvelope):
        raise InvalidTask('explicit full body envelope required')
    if revision == nav.revision:
        raise InvalidTask('new planning map revision required')
    if (volume.p.world_frame != nav.frame_id or volume.p.calibration != calibration_id or
            volume.epoch != expected_epoch or not expected_epoch or
            abs(volume.p.resolution_m-nav.resolution) > 1e-9):
        raise InvalidTask('depth_lidar_frame_calibration_epoch_or_lattice_mismatch')
    if volume.p.minimum_height_m > nav.floor_z or volume.p.maximum_height_m < nav.floor_z+envelope.height_m:
        raise InvalidTask('depth_volume_does_not_cover_robot_height')
    if not volume.fresh(number(now_s, 'projection time')):
        raise InvalidTask('fresh_depth_required_for_traversability')
    depth = projected_grid(volume, origin_xy=(nav.ox, nav.oy), width=nav.width,
                           height=nav.height, now=now_s,
                           historical_free=purpose == 'global_route_proposal')
    rows = []
    for y, row in enumerate(nav.rows):
        merged = []
        for x, laser in enumerate(row):
            cell = depth[y*nav.width+x]
            merged.append('#' if laser == '#' or cell == 100 else
                          ('.' if laser == '.' and cell == 0 else '?'))
        rows.append(''.join(merged))
    fused = deepcopy(lidar_document)
    fused.update(revision=revision, rows=rows, robot_radius_m=envelope.radius_m)
    NavigationMap(fused)
    return dict(map_document=fused, source_map_revision=nav.revision,
                calibration_id=calibration_id, depth_epoch=expected_epoch,
                depth_observed_s=volume.last_stamp, geometry_id=envelope.geometry_id,
                purpose=purpose, live_travel_guard_required=True,
                policy='occupied_union_free_intersection_full_height',
                semantic_label_required=False, physical_path_verified=False)
