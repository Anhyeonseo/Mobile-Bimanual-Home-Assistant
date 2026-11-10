"""Isolated local MoveIt service process for offline diagnostics, no execution."""
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import tempfile
import time
import yaml


@contextmanager
def session(parameters, state, output):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from moveit_msgs.srv import GetPositionFK, GetStateValidity, GetMotionPlan, ApplyPlanningScene, GetCartesianPath, GetPlanningScene
    from sensor_msgs.msg import JointState
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    processes = []; node = None
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='carry-moveit-') as temp, (output/'move_group.log').open('w') as log:
            param = Path(temp)/'parameters.yaml'
            param.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            namespace = '/offline_carry_'+str(os.getpid())
            for package, executable in [('moveit_ros_move_group', 'move_group'), ('robot_state_publisher', 'robot_state_publisher')]:
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
            deadline = time.monotonic()+30
            for client in clients.values():
                while not client.wait_for_service(timeout_sec=.2):
                    if processes[0].poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('MoveIt services unavailable; inspect move_group.log')
            def call(name, request):
                future = clients[name].call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=12)
                if not future.done():
                    raise RuntimeError('MoveIt service timed out: '+name)
                return future.result()
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
