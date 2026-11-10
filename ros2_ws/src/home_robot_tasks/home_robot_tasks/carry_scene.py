"""Capture-bound observed scene -> oriented MoveIt boxes and payload geometry.

No free space is inferred behind the depth image. The returned geometry is
observed-only and does not authorize execution outside its coverage.
"""
from dataclasses import dataclass

import numpy as np

from .s2_geometry import rigid, JOINTS
from .carry_binding import BoundPayload


# The supported near-field capture is at most 640 x 480. A measured envelope
# may contain one primitive per return; a nearby floor can legitimately exceed
# the old 20,000-box budget. Bound allocation by a full exposure, never truncate
# geometry or change collision padding to make a dense observation fit.
MAX_OBSERVED_BOXES_PER_ROLE = 640 * 480


@dataclass(frozen=True)
class SceneBox:
    name: str
    base_pose: np.ndarray
    size_m: tuple
    role: str
    mesh: object = None


def box_from_bounds(name, bounds, base_from_support, role):
    values=np.asarray(bounds,dtype=float)
    if values.shape!=(2,3) or not np.isfinite(values).all() or np.any(values[1]<=values[0]):
        raise ValueError('finite nonempty observed box required')
    pose=rigid(base_from_support).copy()
    pose[:3,3]+=pose[:3,:3]@values.mean(axis=0)
    return SceneBox(name,pose,tuple(values[1]-values[0]),role)


def from_prepared_capture(prepared,profile, *, nominal_obstacles=None, obstacle_padding_m=.001):
    if (prepared.get('mode')!='offline_grasp_replay' or prepared.get('status')!='CAPTURE_CANDIDATES_READY'
            or prepared.get('motion_authorized') is not False or prepared.get('hardware_commands')!=0):
        raise ValueError('completed offline carry preparation required')
    review,scene,group=prepared['review'],prepared['observed_scene'],prepared['group']
    if nominal_obstacles is None:
        nominal_obstacles=scene.get('preferred_obstacle_envelope')=='point_bounds'
    if (review.get('mode')!='offline_capture_review' or review.get('status')!='TARGET_DISPLAY_ONLY'
            or review.get('executable') is not False or review.get('motion_authorized') is not False
            or review.get('frame')!='base_link' or review.get('calibration_id')!=profile['calibration_id']):
        raise ValueError('capture coordinate profile mismatch')
    for key in ('capture_id','source_sha256','calibration_id','session_id','observation_index'):
        if scene.get(key)!=review.get(key) or group.get(key)!=review.get(key):
            raise ValueError('scene/candidate capture binding differs: '+key)
    if (prepared['capture_id']!=review['capture_id'] or prepared['source_sha256']!=review['source_sha256']
            or scene.get('frame')!='capture_support' or scene.get('mask_sha256')!=group.get('mask_sha256')
            or scene.get('motion_authorized') is not False or scene.get('visible_payload_extent_only') is not True):
        raise ValueError('observed scene provenance mismatch')
    joints=review['model_joint_positions_si']
    required={'s2_lift_joint'}|{f'{side}_model_{j}' for side in ('left','right') for j in JOINTS}
    if (set(joints)!=required or not all(np.isfinite(v) for v in joints.values())
            or joints['s2_lift_joint']!=review['lift_height_m']
            or not 0<=joints['s2_lift_joint']<=profile['lift']['travel_m']):
        raise ValueError('complete captured arm/jaw/lift state required')
    for side in ('left','right'):
        if any(joints[f'{side}_model_{j}']!=review[side+'_anchor_rad'][j] for j in JOINTS[:5]):
            raise ValueError('arm anchor disagrees with captured model state')
    transform=rigid(profile['base_from_optical'])@np.linalg.inv(rigid(scene['support_from_optical']))
    boxes=[]
    obstacle_field='nominal_obstacle_boxes_m' if nominal_obstacles else 'obstacle_boxes_m'
    if obstacle_padding_m not in (0.,.001) or (not nominal_obstacles and obstacle_padding_m!=.001):
        raise ValueError('choose measured point bounds or their 1 mm nominal envelope')
    if nominal_obstacles and scene.get('nominal_obstacle_padding_m')!=.001:
        raise ValueError('nominal point envelope requires explicit 1 mm padding evidence')
    refinements={}
    if nominal_obstacles and obstacle_padding_m==.001:
        for item in scene.get('nominal_obstacle_padding_refinements',[]):
            index=item['box_index']
            if (type(index) is not int or index<0 or index>=len(scene[obstacle_field]) or index in refinements
                    or item.get('all_returns_preserved') is not True or item.get('measured_returns',0)<1
                    or not item.get('meshes')):
                raise ValueError('invalid observed obstacle padding refinement')
            refinements[index]=item['meshes']
    for role,field in [('support','support_boxes_m'),('obstacle',obstacle_field)]:
        values=scene[field]
        if (not isinstance(values,list) or len(values)>MAX_OBSERVED_BOXES_PER_ROLE
                or (role=='support' and not values)):
            raise ValueError(f'bounded observed {role} boxes required (maximum {MAX_OBSERVED_BOXES_PER_ROLE})')
        for i,bounds in enumerate(values):
            if role=='obstacle' and i in refinements:
                for j,mesh in enumerate(refinements[i]):
                    vertices=np.asarray(mesh['vertices']);faces=np.asarray(mesh['faces'])
                    if (vertices.ndim!=2 or vertices.shape[1:]!=(3,) or len(vertices)<4 or not np.isfinite(vertices).all()
                            or faces.ndim!=2 or faces.shape[1:]!=(3,) or len(faces)<4 or faces.dtype.kind not in 'iu'
                            or np.any(faces<0) or np.any(faces>=len(vertices))
                            or np.any(vertices<np.asarray(bounds)[0]-1e-9) or np.any(vertices>np.asarray(bounds)[1]+1e-9)):
                        raise ValueError('invalid bounded obstacle refinement mesh')
                    boxes.append(SceneBox(f'observed_obstacle_{i}_part_{j}',transform,tuple(np.ptp(vertices,axis=0)),role,mesh))
                continue
            # The source keeps point bounds + 1 mm. Removing that explicit
            # padding for offline comparison must never remove any return.
            if role=='obstacle' and nominal_obstacles and obstacle_padding_m==0.:
                bounds=np.asarray(bounds,float).copy()
                if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or np.any(bounds[1,:2]-bounds[0,:2]<.002-1e-12):
                    raise ValueError('invalid padded point envelope')
                bounds[0,:2]+=.001;bounds[1]-=.001
                if bounds[0,2]!=0.:bounds[0,2]+=.001
                middle=bounds.mean(axis=0);half=np.maximum((bounds[1]-bounds[0])/2,.0000005)
                bounds=np.array([middle-half,middle+half])
            boxes.append(box_from_bounds(f'observed_{role}_{i}',bounds,transform,role))
    target=box_from_bounds('target',scene['target_bounds_m'],transform,'target')
    if scene.get('support_from_payload') is not None:
        pose=transform@rigid(scene['support_from_payload']);size=np.asarray(scene['payload_size_m'],float)
        if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('invalid oriented payload envelope')
        target=SceneBox('target',pose,tuple(size),'target')
    payload=BoundPayload(review['capture_id'],review['source_sha256'],profile['calibration_id'],target.base_pose,target.size_m,scene.get('payload_mesh'))
    return boxes,payload,dict(joints)


def apply_boxes(call, boxes):
    """Use oriented primitives, preserving support-plane rotation exactly."""
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
    from geometry_msgs.msg import Point
    from .carry_moveit import MoveItCarryPlanner
    # One object per role reduces scene-message overhead without merging gaps.
    objects={}
    for box in boxes:
        obj=objects.setdefault(box.role,CollisionObject(id='capture_'+box.role,operation=CollisionObject.ADD))
        obj.header.frame_id='base_link'
        if box.mesh is None:
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,box.size_m))))
            obj.primitive_poses.append(MoveItCarryPlanner.pose(box.base_pose))
        else:
            obj.meshes.append(Mesh(vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in box.mesh['vertices']],
                                   triangles=[MeshTriangle(vertex_indices=list(map(int,f))) for f in box.mesh['faces']]))
            obj.mesh_poses.append(MoveItCarryPlanner.pose(box.base_pose))
    request=ApplyPlanningScene.Request();request.scene.is_diff=True
    request.scene.world.collision_objects=list(objects.values())
    if not call('apply_planning_scene',request).success:
        raise RuntimeError('observed scene update rejected')
