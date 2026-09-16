"""One shared task/stop owner and complete fetch assembly from explicit devices."""
from dataclasses import dataclass
from .application import RobotApplication
from .fetch import InvalidTask
from .capture_port import CapturePort
from .execution import TaskLease
from .fetch_composition import compose_fetch_adapter, ResolvedPort
from .manipulation_port import ManipulationPort, PlanningContext
from .mission_catalog import MissionCatalog
from .payload_monitor import PayloadTaskMonitor
from .runtime import RobotRuntime
from .search import SearchSession
from .system_stop_backend import SystemStopBackend
from .view_motion import ViewMotionBuilder
from .workflow_ports import SequencePort, SearchSkillPort


@dataclass
class RuntimeAssembly:
    runtime: object
    catalog: object
    capture: object
    search: object
    manipulation: object
    stop: object


def build_runtime(*, world, navigation_map, catalog_document, state, payload, camera,
                  source, detector, navigation, arm, lift, planner, planning_context=None,
                  surface_observer, stop_client, clock, scene_revision,
                  maintenance=(), observations=(), maximum_tick_gap_s=.1,
                  manipulation_factory=None, surface_sampler=None):
    lease = TaskLease()
    stop = SystemStopBackend(stop_client, lease)
    # Device factories bind to the SAME stop owner as every nested task.
    arm = arm(stop) if callable(arm) else arm
    lift = lift(stop) if callable(lift) else lift
    catalog = MissionCatalog(catalog_document, world, navigation_map,
        lambda: state.latest.xy_yaw[:2], state.motion, scene_revision, clock)
    state.catalog = catalog
    monitor = PayloadTaskMonitor(state, payload)

    def navigation_parameters(skill, parameters, now):
        catalog.navigation_goal = dict(parameters['goal'])
        return parameters

    def lift_parameters(skill,parameters,now):
        state.lift_target_um = parameters['height_um']
        return parameters
    lift = ResolvedPort('lift',lift,lift_parameters,clock)
    nav = ResolvedPort('navigation', navigation, navigation_parameters, clock)
    capture = CapturePort(source, detector, camera, state.motion, clock, surface_sampler=surface_sampler)
    move = SequencePort(ViewMotionBuilder(navigation_map, lambda: state.latest.xy_yaw[:2],
        arm, lift, nav, transport_height_um=catalog.doc['transport']['height_um']), monitor, clock)
    search = SearchSkillPort(lambda g,p,t: catalog.make_search(g,p,t,camera), catalog.views,
        move, capture, state.motion, stop, clock, catalog.accept_target)
    def context():
        target = catalog.manipulation_target
        return PlanningContext(state.latest.observed_s,
            target['capture_id'],state.motion().pose_revision,scene_revision(),
            target['calibration_id'],target['frame_id'],state.latest.joints_rad,
            state.motion().base_stopped,state.motion().lift_stopped, target_observed_s=target['observed_s'])
    manipulation = ManipulationPort(planner, arm, planning_context or context, clock,
        maximum_age_s=catalog.doc['grasp']['maximum_target_age_s'])
    if manipulation_factory is not None:
        # Explicit stage resolver, jaw port and planning-scene port are supplied
        # by the deployment. An approach-only adapter is never silently promoted.
        manipulation = manipulation_factory(motion=manipulation, arm=arm,
            monitor=monitor, clock=clock, catalog=catalog)
    adapter = compose_fetch_adapter(backends=dict(arm=arm,lift=lift,navigation=nav,
        alignment=nav,search=search,capture=capture,manipulation=manipulation),
        resolve=catalog.resolve,monitor=monitor,phase_monitor=monitor,stop_backend=stop,
        clock=clock,before_start=monitor.before_start,alignment_transport=catalog.doc["transport"],
        capture_sink=lambda g,s,p,o: catalog.capture_sink(g,s,p,o,camera,surface_observer=surface_observer))
    app = RobotApplication(world,navigation_map,adapter,lease=lease,
        initial_xy=state.latest.xy_yaw[:2],exact_goal_simulation=False)
    def registered_geometry():
        if lease.owner is not None and catalog.invalid_places:
            raise InvalidTask('registered geometry changed; stop and re-register the affected place')
    runtime = RobotRuntime(app,clock,maintenance,observations=(registered_geometry,)+tuple(observations),
        maximum_tick_gap_s=maximum_tick_gap_s,fault_stop=stop_client.request)
    runtime.boot_id = state.boot
    app.admission = lambda: (runtime.fault is None and state.fresh() and not stop.requested
        and all(state.conditions().get(k) is True for k in
                ('hardware_ready','localized','base_stopped','lift_stopped','lift_homed','lift_hold_verified'))
        and payload.proof(clock()).conditions['payload_safe'] and not catalog.invalid_places
        and (app.map.map_id,app.map.revision)==(catalog.map.map_id,catalog.map.revision))
    return RuntimeAssembly(runtime,catalog,capture,search,manipulation,stop)
