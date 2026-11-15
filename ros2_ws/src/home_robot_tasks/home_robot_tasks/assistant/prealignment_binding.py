"""Shape alignment -> actual arrival -> new capture -> complete route planning.

This policy plugs into MissionCatalog/compose_fetch_adapter; it is not another
robot executor. Workers receive frozen snapshots, never a live state object.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import math
from .alignment_contracts import TaskScene, AlignmentSelection
from ..async_resolution import AsyncResolvedPort, PlanningWorker
from ..fetch_composition import ResolvedPort
from ..fetch import InvalidTask


@dataclass(frozen=True)
class PlanningRequest:
    skill: str
    parameters: dict
    scene: TaskScene
    configuration: tuple
    epoch: object
    prepared: object
    generation: int


class PrealignmentPolicy:
    HEAVY=frozenset(('align_for_pick','align_for_place','pick','place'))

    def __init__(self, *, scene_for, configuration, epoch, prepare, alignment_planner,
                 route_planner, navigation_plan, loaded_transport, maximum_alignment_s=180, program_sink=None, reset_transport=None):
        self.scene_for,self.configuration,self.epoch=scene_for,configuration,epoch
        self.prepare,self.alignment_planner,self.route_planner=prepare,alignment_planner,route_planner
        self.navigation_plan,self.loaded_transport=navigation_plan,loaded_transport
        self.program_sink,self.reset_transport=program_sink,reset_transport
        self.budget=maximum_alignment_s
        if not 1<=self.budget<=300:raise InvalidTask('bounded alignment time required')
        self.selected={};self.generation=0;self.ports=[]
        self.worker=PlanningWorker()

    def reset(self):
        self.generation+=1;self.selected.clear()

    def resolver_factory(self,catalog):
        def factory(skill,port,resolve,clock,*,before=None,result_sink=None):
            if skill in self.HEAVY:
                result=AsyncResolvedPort(skill,port,self,catalog,clock,before=before,worker=self.worker)
                self.ports.append(result);return result
            return ResolvedPort(skill,port,resolve,clock,before=before,result_sink=result_sink)
        return factory

    @staticmethod
    def operation(skill):return 'place' if skill=='place' or skill.endswith('_place') or skill=='reobserve_destination' else 'pick'

    def _arrived(self,selection,now,*,lift=True):
        pose,height,observed_s,stopped=self.configuration()
        if stopped is not True or not 0<=now-observed_s<=.5:raise InvalidTask('fresh stopped configuration required')
        x,y,yaw=pose;tx,ty,tyaw=selection.candidate['base_xy_yaw']
        if math.hypot(x-tx,y-ty)>.03 or abs(math.atan2(math.sin(yaw-tyaw),math.cos(yaw-tyaw)))>.1:
            raise InvalidTask('selected base pose not reached')
        if lift and abs(height-selection.candidate['lift_travel_m'])>.003:raise InvalidTask('selected lift not reached')
        if now>selection.expires_s:raise InvalidTask('alignment selection expired')

    def before_capture(self,skill,parameters,catalog,now):
        key=(self.operation(skill),parameters['object_id'])
        selection=self.selected.get(key)
        if selection is None:raise InvalidTask('alignment selection required before recapture')
        self._arrived(selection,now)
        self.selected[key]=replace(selection,aligned_s=now)

    def snapshot(self,skill,parameters,catalog,now):
        operation=self.operation(skill)
        target=catalog.checked_target(operation=='place')
        scene=self.scene_for(deepcopy(target),operation,parameters['object_id'])
        if not isinstance(scene,TaskScene) or scene.object_id!=parameters['object_id']:
            raise InvalidTask('bound scene object required')
        scene.fresh(scene.stamp.context,now)
        if scene.stamp.capture_id!=target['capture_id'] or scene.stamp.observed_s!=target['observed_s']:
            raise InvalidTask('scene and target exposure mismatch')
        if (scene.stamp.context.map_id,scene.stamp.context.map_revision)!=(catalog.map.map_id,catalog.map.revision):
            raise InvalidTask('alignment map mismatch')
        if skill in ('pick','place'):
            selection=self.selected.get((operation,scene.object_id))
            if selection is None:raise InvalidTask('alignment selection required')
            self._arrived(selection,now)
            selection.check_capture(scene,now=now,context=scene.stamp.context)
        cfg=deepcopy(self.configuration())
        if cfg[3] is not True or not 0<=now-cfg[2]<=.5:raise InvalidTask('fresh stationary state required')
        # prepare copies observation arrays, actual joint state, model/context,
        # support grid and held evidence. It performs no inference or planning.
        prepared=self.prepare(scene,operation,skill,now)
        return PlanningRequest(skill,deepcopy(parameters),deepcopy(scene),cfg,deepcopy(self.epoch()),prepared,self.generation)

    def check_pending(self,request,catalog,now):
        if request.generation!=self.generation or self.epoch()!=request.epoch:
            raise InvalidTask('planning epoch changed')
        if catalog.invalid_places:raise InvalidTask('registered surface changed')
        target=catalog.surface if self.operation(request.skill)=='place' else catalog.target
        if target and target.get('capture_id')!=request.scene.stamp.capture_id:
            raise InvalidTask('active target replaced while planning')
        pose,height,observed_s,stopped=self.configuration()
        oldpose,oldheight,_,_=request.configuration
        if stopped is not True or not 0<=now-observed_s<=.5:raise InvalidTask('planning state lost')
        if (math.hypot(pose[0]-oldpose[0],pose[1]-oldpose[1])>.005 or abs(height-oldheight)>.003
                or abs(math.atan2(math.sin(pose[2]-oldpose[2]),math.cos(pose[2]-oldpose[2])))>.02):
            raise InvalidTask('platform moved during planning')

    def compute(self,request):
        if request.skill.startswith('align_'):return self.alignment_planner(request)
        return self.route_planner(request)

    def commit(self,request,result,catalog,now):
        self.check_pending(request,catalog,now)
        operation=self.operation(request.skill)
        if request.skill.startswith('align_'):
            if result.get('status')!='SCREENED_PROPOSALS' or not result.get('candidates'):
                raise InvalidTask('no checked alignment route')
            selected=deepcopy(result['candidates'][0])
            # navigation_plan returns the already screened local path; this
            # does not invoke another unbounded planner on the runtime thread.
            nav=self.navigation_plan(selected,request)
            transport=request.prepared.get('transport')
            if not isinstance(transport,dict) or set(transport)!={'arm','height_um'}:
                raise InvalidTask('checked empty or loaded transport transition required')
            nav['_alignment_transport']=transport
            self.selected[(operation,request.scene.object_id)]=AlignmentSelection(request.scene,selected,now,now+self.budget)
            catalog.navigation_goal=deepcopy(nav['goal'])
            return nav
        from ..task_route_program import TaskRouteProgram
        if (not isinstance(result,TaskRouteProgram) or result.operation!=operation
                or result.object_id!=request.scene.object_id or result.capture_id!=request.scene.stamp.capture_id
                or result.model_revision!=request.scene.model_revision
                or result.calibration_id!=request.scene.stamp.context.calibration_id or result.lift_m!=request.scene.lift_m):
            raise InvalidTask('complete route binding mismatch')
        catalog.manipulation_target=deepcopy(catalog.surface if operation=='place' else catalog.target)
        if self.program_sink is not None:self.program_sink(result)
        return {'program':result}

    @staticmethod
    def transport_for_alignment(plan,carrying):
        return plan['_alignment_transport']

    def resolve(self,skill,parameters,catalog,now):
        if skill=='prepare_transport':return self.loaded_transport(parameters,now)
        if skill in self.HEAVY:raise InvalidTask('background planning port required')
        if skill.startswith('adjust_lift_'):
            entry=self.selected.get((self.operation(skill),parameters['object_id']))
            if entry is None:raise InvalidTask('alignment selection required')
            self._arrived(entry,now,lift=False)
            return {'height_um':round(entry.candidate['lift_travel_m']*1e6)}
        raise InvalidTask('unsupported prealignment skill')

    def close(self):
        self.reset()
        for p in self.ports:p.close()
        self.worker.close()
