"""Calibrated stage poses and shared gripper/scene assembly.

Observations come from perception/TF or recorded fixtures. Acquisition times
are never replaced by command time. Offsets are in the named observation frame.
"""
from copy import deepcopy
from .arm_routes import SharedGripperPort
from .execution import Feedback
from .fetch import InvalidTask
from .navigation import number, identifier
from .pick_place import PickPlacePort
from .planning_scene_port import validate_scene_change

MOTION_STAGES = {'pick_pregrasp','pick_approach','pick_retreat',
                 'place_preplace','place_approach','place_retreat'}


class ManipulationBindings:
    def __init__(self, *, calibrations, scene_port, stage_observation, scene_evidence,
                 offsets_m, maximum_age_s):
        if set(offsets_m) != MOTION_STAGES: raise InvalidTask("all six explicit stage offsets required")
        self.offsets = {}
        for stage, xyz in offsets_m.items():
            if not isinstance(xyz,(tuple,list)) or len(xyz)!=3: raise InvalidTask("three stage offsets required")
            self.offsets[stage]=tuple(number(v,'stage offset') for v in xyz)
            if any(abs(v)>.3 for v in self.offsets[stage]): raise InvalidTask("bounded approach/retreat offset required")
        self.age=number(maximum_age_s,'stage observation age')
        if not 0<self.age<=2 or not callable(stage_observation) or not callable(scene_evidence):
            raise InvalidTask("fresh stage/scene observation providers required")
        self.calibrations,self.scene=calibrations,scene_port
        self.observe,self.scene_evidence=stage_observation,scene_evidence

    def factory(self, *, motion, arm, monitor, clock, catalog):
        gripper=SharedGripperPort(arm,self.calibrations)
        def resolve(stage, parameters, now):
            selected=parameters['arm'];obj=parameters['object_id']
            if stage.endswith(('_open','_close')):
                return {'arm':selected,'action':stage.rsplit('_',1)[1]}
            observation=deepcopy(self.observe(stage,deepcopy(parameters),now))
            if not isinstance(observation,dict) or observation.get('object_id')!=obj or observation.get('arm')!=selected:
                raise InvalidTask("stage observation object/arm mismatch")
            stamp=number(observation.get('observed_s'),'stage exposure')
            if not 0<=stamp<=now or now-stamp>self.age: raise InvalidTask("fresh stage observation required")
            if stage.endswith(('_attach','_detach')):
                change={k:v for k,v in observation.items() if k!='stage'}
                if observation.get('stage')!=stage or change.get('operation')!=stage.rsplit('_',1)[1]:
                    raise InvalidTask("scene stage identity mismatch")
                return validate_scene_change(change,now,gripper_links=self.scene.links,
                    world_frame=self.scene.world,maximum_age_s=self.scene.age)
            if stage not in MOTION_STAGES or observation.get('stage')!=stage: raise InvalidTask("motion stage identity mismatch")
            expected={'stage','object_id','arm','observed_s','capture_id','pose_revision','scene_revision',
                      'calibration_id','frame_id','position_m','quaternion_xyzw'}
            if set(observation)!=expected:raise InvalidTask("complete observed stage pose required")
            for k in ('capture_id','pose_revision','scene_revision','calibration_id','frame_id'): identifier(observation[k],k)
            if observation['pose_revision']!=catalog.motion().pose_revision or observation['scene_revision']!=catalog.scene():
                raise InvalidTask("stage pose/scene changed")
            if (observation['calibration_id']!=parameters['target']['calibration_id']
                    or observation['frame_id']!=parameters['target']['frame_id']):
                raise InvalidTask("stage frame/calibration mismatch")
            xyz,q=observation['position_m'],observation['quaternion_xyzw']
            if len(xyz)!=3 or len(q)!=4:raise InvalidTask("stage pose dimensions")
            xyz=tuple(number(v,'position') for v in xyz);q=tuple(number(v,'rotation') for v in q)
            if abs(sum(v*v for v in q)-1)>1e-6:raise InvalidTask("unit stage quaternion required")
            catalog.manipulation_target=deepcopy(observation)
            return {**parameters,'target':deepcopy(observation),
                    'position_m':[v+d for v,d in zip(xyz,self.offsets[stage])], 'quaternion_xyzw':q}

        def evidence(goal,phase,now):
            f=monitor(goal,phase,now);scene=self.scene_evidence(catalog.object_id,now)
            if (not isinstance(scene,dict) or set(scene)!={'object_id','observed_s','scene_revision','attached','detached'}
                    or scene['object_id']!=catalog.object_id or scene['scene_revision']!=catalog.scene()
                    or not 0<=now-number(scene['observed_s'],'scene stamp')<=self.age
                    or type(scene['attached']) is not bool or type(scene['detached']) is not bool
                    or scene['attached'] and scene['detached']):
                raise InvalidTask("fresh object-specific scene evidence required")
            return Feedback(f.goal_id,min(f.observed_s,scene['observed_s']),f.status,
                {**f.conditions,'object_attached':scene['attached'],'object_detached':scene['detached']},f.reason)
        return PickPlacePort(motion,gripper,self.scene,resolve,evidence,clock)
