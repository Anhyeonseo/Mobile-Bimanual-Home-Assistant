"""Isolated local MoveIt service process for offline diagnostics, no execution."""
from contextlib import contextmanager
from copy import deepcopy
from types import SimpleNamespace
import os
from pathlib import Path
import subprocess
import tempfile
import time
import yaml


@contextmanager
def session(parameters, state, output, *, acceleration='baseline', verify_batch=False, request_window=8):
    if acceleration not in ('baseline', 'pipelined', 'batch', 'parallel'):
        raise ValueError('unknown offline acceleration mode')
    native = acceleration in ('batch', 'parallel')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from moveit_msgs.srv import GetPositionFK, GetStateValidity, GetMotionPlan, ApplyPlanningScene, GetCartesianPath, GetPlanningScene
    from sensor_msgs.msg import JointState
    if native:
        from so101_planning_accel.srv import PlanSnapshot, CheckStates
        parameters = deepcopy(parameters)
        for name in ('fast_a', 'fast_b'):
            parameters[name] = deepcopy(parameters['ompl'])
            parameters[name]['planner_configs']['RRTConnectWide'] = dict(type='geometric::RRTConnect', range=.3)
            for side in ('left', 'right'):
                parameters[name][side+'_arm']['planner_configs'].append('RRTConnectWide')
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    processes = []; node = None
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='carry-moveit-') as temp, (output/'move_group.log').open('w') as log:
            param = Path(temp)/'parameters.yaml'
            param.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            namespace = '/offline_carry_'+str(os.getpid())
            programs = [('moveit_ros_move_group', 'move_group'), ('robot_state_publisher', 'robot_state_publisher')]
            if native:
                programs.append(('so101_planning_accel', 'planning_server'))
            for package, executable in programs:
                prefix = Path(get_package_prefix(package))
                env = os.environ.copy()
                if executable == 'move_group':
                    # Same local Jazzy 2.12.4 shutdown workaround as check_moveit_planning.py.
                    plugin = prefix/'lib/libmoveit_move_group_default_capabilities.so'
                    if not plugin.is_file():
                        raise RuntimeError('MoveIt capability library missing')
                    env['LD_PRELOAD'] = str(plugin)
                processes.append(subprocess.Popen([str(prefix/'lib'/package/executable), '--ros-args',
                    '-r', '__ns:='+namespace, '--params-file', str(param)], stdout=log, stderr=subprocess.STDOUT, env=env))
            node = rclpy.create_node('carry_check', namespace=namespace)
            publisher = node.create_publisher(JointState, 'joint_states', 10)
            def publish():
                state.joint_state.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(state.joint_state)
            node.create_timer(.02, publish)
            clients = {name: node.create_client(kind, name) for name, kind in [
                ('compute_fk', GetPositionFK), ('check_state_validity', GetStateValidity),
                ('plan_kinematic_path', GetMotionPlan), ('apply_planning_scene', ApplyPlanningScene),
                ('compute_cartesian_path', GetCartesianPath), ('get_planning_scene', GetPlanningScene)]}
            if native:
                clients.update({name: node.create_client(kind, name) for name, kind in (
                    ('plan_snapshot', PlanSnapshot), ('check_states', CheckStates))})
            deadline = time.monotonic()+30
            for client in clients.values():
                while not client.wait_for_service(timeout_sec=.2):
                    if processes[0].poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('MoveIt services unavailable; inspect move_group.log')
            def raw_call(name, request):
                future = clients[name].call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=12)
                if not future.done():
                    raise RuntimeError('MoveIt service timed out: '+name)
                return future.result()
            def snapshot():
                from moveit_msgs.msg import PlanningSceneComponents
                return raw_call('get_planning_scene', GetPlanningScene.Request(
                    components=PlanningSceneComponents(components=1023))).scene
            def call(name, request):
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('resident MoveIt process exited')
                if acceleration == 'parallel' and name == 'plan_kinematic_path':
                    result = raw_call('plan_snapshot', PlanSnapshot.Request(
                        scene=snapshot(), request=request.motion_plan_request))
                    if result.error:
                        raise RuntimeError('native planning error: '+result.error)
                    call.selected_planner = result.selected_planner
                    return GetMotionPlan.Response(motion_plan_response=result.response)
                return raw_call(name, request)
            if acceleration != 'baseline':
                def check_states(states):
                    if acceleration == 'pipelined':
                        from tools.diagnostics.windowed_requests import windowed_requests
                        responses = windowed_requests(states,
                            lambda s: clients['check_state_validity'].call_async(
                                GetStateValidity.Request(robot_state=s)),
                            lambda: rclpy.spin_once(node, timeout_sec=.01), window=request_window)
                        failed = next((i for i, r in enumerate(responses) if not r.valid), -1)
                        result = SimpleNamespace(valid=failed == -1, failed_index=failed,
                            checked_count=len(responses), reason='')
                        if failed >= 0:
                            result.reason = str([(c.contact_body_1, c.contact_body_2)
                                                 for c in responses[failed].contacts])
                    else:
                        result = raw_call('check_states', CheckStates.Request(scene=snapshot(), states=states))
                    if not result.valid and result.failed_index < 0:
                        raise RuntimeError('native state check error: '+result.reason)
                    if verify_batch:
                        # Diagnostic only: intentionally retains slow reference calls.
                        expected = next((i for i, s in enumerate(states) if not raw_call(
                            'check_state_validity', GetStateValidity.Request(robot_state=s)).valid), -1)
                        if result.valid != (expected == -1) or result.failed_index != expected:
                            raise RuntimeError(f'batch/reference mismatch: native={result.failed_index}, reference={expected}; {result.reason}')
                    return result
                call.check_states = check_states
            call.parallel = acceleration == 'parallel'
            # Preserve the pristine ACM, padding and empty world. Each request
            # restores a FULL scene; a diff would retain old payloads/obstacles.
            pristine = deepcopy(snapshot())
            def reset_scene(new_state):
                nonlocal state
                state = deepcopy(new_state)
                state.is_diff = False
                state.attached_collision_objects = []
                clean = deepcopy(pristine)
                clean.is_diff = False
                clean.robot_state = deepcopy(state)
                clean.world.collision_objects = []
                if not call('apply_planning_scene', ApplyPlanningScene.Request(scene=clean)).success:
                    raise RuntimeError('resident scene reset failed')
                publish()
            call.reset_scene = reset_scene
            yield call
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            process.terminate()
        for process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()


class PersistentSession:
    """One process-owned MoveIt session; reinitialize only for changed models.

    The same worker serializes all use. Exceptions retire the entire session,
    so pending responses or half-applied scenes cannot enter the next request.
    """
    def __init__(self, factory=session):
        self.factory = factory
        self.context = self.call = self.key = None
        self.busy = False
        self.starts = 0

    @contextmanager
    def __call__(self, parameters, state, output):
        import json
        key = json.dumps(parameters, sort_keys=True, allow_nan=False)
        if self.busy:
            raise RuntimeError('concurrent resident planner use')
        self.busy = True
        try:
            if self.key != key:
                self.close()
                context = self.factory(parameters, state, output)
                call = context.__enter__()
                self.context, self.call, self.key = context, call, key
                self.starts += 1
            self.call.reset_scene(state)
            yield self.call
        except BaseException:
            self.close()
            raise
        finally:
            self.busy = False

    def close(self):
        context = self.context
        self.context = self.call = self.key = None
        if context is not None:
            context.__exit__(None, None, None)
