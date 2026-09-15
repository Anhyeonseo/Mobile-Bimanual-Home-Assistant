#!/usr/bin/env python3
"""Actual Nav2 -> v2 host transport -> native C plant -> odom/scan -> Nav2.

The map, wheel scale, dynamics, feedback and scan are synthetic; AMCL is real.
No D415 navigation input, device connection, physical timing or traction proof.
Source ROS Jazzy plus Nav2 packages. Uses an isolated /offline_nav2 namespace.
"""
import argparse
import hashlib
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import secrets
import threading
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import yaml
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--library",
        type=Path,
        default=ROOT / "build/offline-core/libactuator_host_simulator.so",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "output/nav2-closed-loop")
    parser.add_argument(
        "--fault", choices=("none", "scan_loss", "tf_loss"), default="none"
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
    from action_msgs.msg import GoalStatusArray
    from tf2_ros import (
        TransformBroadcaster,
        StaticTransformBroadcaster,
        Buffer,
        TransformListener,
    )
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )
    from home_robot_tasks.base_motion import OmniKinematics, Wheel, WheelOdometry
    from home_robot_tasks.nav2_port import Nav2Port
    from home_robot_tasks.application import RobotApplication
    from home_robot_tasks.navigation import NavigationMap
    from home_robot_tasks.execution import Feedback
    from home_robot_tasks.gateway import Gateway, GatewayStore, handler_for
    from home_robot_tasks.skill_ports import RoutedSkillAdapter
    from so101_arm_bridge.host_simulator import NativeHostSerial
    from so101_arm_bridge.mobile_client import MobileClient
    from so101_arm_bridge.stream_transport_v2 import (
        StreamValidationTransportV2,
        MobileV2Exchange,
    )

    rclpy.init()
    node = rclpy.create_node("synthetic_mobile_plant", namespace="/offline_nav2")
    started = time.monotonic()
    bus_lock = threading.RLock()
    http_gateway = [None]
    http_server = None
    http_worker = None
    clock = lambda: int(time.monotonic() * 1000)
    serial = NativeHostSerial(args.library, clock, boot_id=42)
    transport = StreamValidationTransportV2(serial, response_timeout_s=0.05)
    client = MobileClient(MobileV2Exchange(transport), clock)
    geometry = OmniKinematics(
        tuple(
            Wheel(0.2 * math.cos(a), 0.2 * math.sin(a), a + math.pi / 2, 0.05)
            for a in (0, 2 * math.pi / 3, 4 * math.pi / 3)
        )
    )
    odometry = WheelOdometry(geometry, max_gap_s=0.25)
    odometry.pose = (-1.0, 0.0, 0.0)
    command = [0.0, 0.0, 0.0]
    scan_enabled = [True]
    tf_enabled = [True]
    scan_stamp_ns = [0]
    command_at = [time.monotonic()]
    moving_commands = [0]
    enabled = [False]
    last_feedback = [None]
    feedback_at = [-1.0]
    trace = []
    control_trace = []
    follow_status = {}
    localization = [None]

    def follow_update(message):
        follow_status.clear()
        follow_status.update(
            {
                bytes(item.goal_info.goal_id.uuid): item.status
                for item in message.status_list
            }
        )

    node.create_subscription(
        GoalStatusArray,
        "follow_path/_action/status",
        follow_update,
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "amcl_pose",
        lambda msg: localization.__setitem__(0, msg),
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    tf = TransformBroadcaster(node)
    static_tf = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child in [("base_footprint", "laser")]:
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transform.header.stamp = node.get_clock().now().to_msg()
        transforms.append(transform)
    static_tf.sendTransform(transforms)
    odom_pub = node.create_publisher(Odometry, "odom", 10)
    scan_pub = node.create_publisher(LaserScan, "scan", 10)
    map_pub = node.create_publisher(
        OccupancyGrid,
        "map",
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )

    def received_scan(message):
        scan_stamp_ns[0] = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )

    node.create_subscription(LaserScan, "scan", received_scan, 10)

    def receive(msg):
        command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
        command_at[0] = time.monotonic()
        if any(abs(v) > 0.001 for v in command):
            moving_commands[0] += 1

    node.create_subscription(Twist, "cmd_vel", receive, 10)

    def occupied(x, y):
        return abs(x) >= 2.9 or abs(y) >= 2.9 or (0.3 <= x <= 0.7 and -0.2 <= y <= 0.4)

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 120
    grid.info.origin.position.x = grid.info.origin.position.y = -3.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [
        100 if occupied(-3 + (x + 0.5) * 0.05, -3 + (y + 0.5) * 0.05) else 0
        for y in range(120)
        for x in range(120)
    ]

    def publish_map():
        grid.header.stamp = node.get_clock().now().to_msg()
        map_pub.publish(grid)

    node.create_timer(0.5, publish_map)

    def step():
        # HTTP cancellation and the ROS timer share the entire command cycle.
        # Serial frame locking alone cannot protect session/enable transitions.
        with bus_lock:
            step_locked()

    def step_locked():
        now = time.monotonic()
        measured = (0.0, 0.0, 0.0)
        if enabled[0]:
            last_feedback[0] = client.status()
            feedback_at[0] = now
            measured = tuple(v / 100.0 for v in last_feedback[0].velocity_raw[:3])
            odometry.observe(measured, now - started)
            velocity = command if now - command_at[0] <= 0.2 else (0.0, 0.0, 0.0)
            control_trace.append(
                {
                    "t": now - started,
                    "command": list(velocity),
                    "measured": list(measured),
                    "state": last_feedback[0].state,
                }
            )
            rates = geometry.wheel_rates(velocity, (10.0, 10.0, 10.0))
            client.velocity(tuple(round(v * 100) for v in rates) + (0,))
        x, y, yaw = odometry.pose
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        if tf_enabled[0]:
            tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = transform.transform.rotation
        vx, vy, wz = geometry.body_twist(measured)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom_pub.publish(odom)
        trace.append([round(now - started, 3), x, y, yaw])

    node.create_timer(0.05, step)

    def scan():
        if not scan_enabled[0]:
            return
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - math.tau / 360
        msg.angle_increment = math.tau / 360
        msg.range_min = 0.12
        msg.range_max = 3.5
        msg.scan_time = 0.1
        x, y, yaw = odometry.pose
        ranges = []
        for i in range(360):
            angle = yaw + msg.angle_min + i * msg.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            distance = float("inf")
            for n in range(3, 71):
                r = n * 0.05
                if occupied(x + r * dx, y + r * dy):
                    distance = r
                    break
            ranges.append(distance)
        msg.ranges = ranges
        scan_pub.publish(msg)

    node.create_timer(0.1, scan)
    processes = []
    logs = []
    port = Nav2Port(node)
    report = {
        "mode": "simulation",
        "hardware_commands": 0,
        "physical_task_completed": False,
        "real_nav2_planner": True,
        "real_nav2_controller": True,
        "native_c_mobile_plant": True,
        "localization": "real AMCL with synthetic scan/odometry",
        "injected_fault": args.fault,
        "passed": False,
    }
    report["ros_packages"] = {
        package: ET.parse(
            Path(get_package_share_directory(package)) / "package.xml"
        ).findtext("version")
        for package in (
            "nav2_amcl",
            "nav2_planner",
            "nav2_controller",
            "nav2_bt_navigator",
            "nav2_navfn_planner",
            "dwb_core",
        )
    }
    report["native_library_sha256"] = hashlib.sha256(
        args.library.read_bytes()
    ).hexdigest()
    application = [None]

    def until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate():
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("Nav2 process exited; inspect node logs")
            if time.monotonic() > deadline:
                raise TimeoutError("Nav2 closed-loop timeout")
            rclpy.spin_once(node, timeout_sec=0.02)
            if http_gateway[0] is not None:
                http_gateway[0].tick()
            elif application[0] is not None:
                application[0].tick(time.monotonic() - started)

    try:
        config = yaml.safe_load((ROOT / "config/nav2.simulation.yaml").read_text())
        config["/offline_nav2/bt_navigator"]["ros__parameters"][
            "default_nav_to_pose_bt_xml"
        ] = str(ROOT / "config/navigate.simulation.xml")
        with tempfile.TemporaryDirectory(prefix="alohamini-nav2-") as tmp:
            params = Path(tmp) / "params.yaml"
            params.write_text(yaml.safe_dump(config))
            for package, executable in [
                ("nav2_amcl", "amcl"),
                ("nav2_planner", "planner_server"),
                ("nav2_controller", "controller_server"),
                ("nav2_bt_navigator", "bt_navigator"),
            ]:
                path = Path(get_package_prefix(package)) / "lib" / package / executable
                log = (args.output / (executable + ".log")).open("w")
                logs.append(log)
                processes.append(
                    subprocess.Popen(
                        [
                            str(path),
                            "--ros-args",
                            "-r",
                            "__ns:=/offline_nav2",
                            "--params-file",
                            str(params),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                )
            for name in ("amcl", "planner_server", "controller_server", "bt_navigator"):
                lifecycle = node.create_client(ChangeState, name + "/change_state")
                until(lifecycle.service_is_ready, 20)
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = lifecycle.call_async(request)
                    until(future.done, 20)
                    if not future.result().success:
                        raise RuntimeError("Nav2 lifecycle transition failed: " + name)
            # A newly activated navigator has its own TF cache. Allow the AMCL
            # future-dated transform to overlap current odometry in that cache.
            navigator_activated = time.monotonic()
            until(
                lambda: time.monotonic() - navigator_activated >= 0.7
                and port.ready()
                and localization[0] is not None
                and tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time()),
                10,
            )
            client.synchronize()
            client.arm(1)
            enabled[0] = True

            # Same application envelope used by the future phone gateway.
            # Arms/lift are fixed fixtures here; only wheel feedback is simulated
            # by C. No arm hold or loaded travel has been physically verified.
            def conditions(goal_id, step, now_s):
                status = last_feedback[0]
                fresh = (
                    time.monotonic() - feedback_at[0] <= 0.2
                    and status is not None
                    and status.ready_for_motion(client.session)
                )
                stopped = (
                    fresh
                    and status is not None
                    and all(v == 0 for v in status.velocity_raw)
                )
                located = bool(
                    tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time())
                )
                scan_age = (node.get_clock().now().nanoseconds - scan_stamp_ns[0]) / 1e9
                scan_fresh = 0 <= scan_age <= 0.3
                arrived = False
                if located:
                    pose = tf_buffer.lookup_transform(
                        "map", "base_footprint", rclpy.time.Time()
                    )
                    pose_stamp = (
                        pose.header.stamp.sec * 1_000_000_000
                        + pose.header.stamp.nanosec
                    )
                    pose_age = (node.get_clock().now().nanoseconds - pose_stamp) / 1e9
                    located = 0 <= pose_age <= 0.3
                    goal = step["parameters"]["goal"]
                    arrived = (
                        math.hypot(
                            pose.transform.translation.x - goal["x"],
                            pose.transform.translation.y - goal["y"],
                        )
                        <= 0.12
                    )
                return Feedback(
                    goal_id,
                    now_s,
                    "READY",
                    {
                        "hardware_ready": fresh and scan_fresh,
                        "localized": located,
                        "localized_at_destination": arrived,
                        "base_stopped": stopped,
                        "lift_stopped": True,
                        "arm_in_transport_pose": True,
                        "lift_in_transport_position": True,
                    },
                )

            class StopBackend:
                def request(self, run_id, preserve_load, now_s):
                    enabled[0] = False
                    self.before = client.stop()
                    self.sent_ms = clock()
                    self.follow = {
                        goal
                        for goal, status in follow_status.items()
                        if status in (1, 2, 3)
                    }

                def poll(self, run_id, now_s):
                    status = client.status()
                    fresh = (
                        clock() > self.sent_ms + 20
                        and status.feedback_tick_ms != self.before.feedback_tick_ms
                        and status.feedback_is_fresh()
                    )
                    terminal = all(
                        follow_status.get(goal) in (4, 5, 6) for goal in self.follow
                    )
                    stopped = (
                        fresh
                        and bool(status.flags & 8)
                        and all(v == 0 for v in status.velocity_raw)
                    )
                    return Feedback(
                        run_id,
                        now_s,
                        "STOPPED" if stopped and terminal else "RUNNING",
                        {
                            "base_stopped": stopped,
                            "lift_stopped": True,
                            "arms_holding": True,
                        },
                    )

            navigation_map = NavigationMap(
                {
                    "schema_version": 1,
                    "map_id": "nav2-room",
                    "revision": "synthetic-1",
                    "floor_id": "ground",
                    "frame_id": "map",
                    "floor_z_m": 0.0,
                    "resolution_m": 0.05,
                    "origin": {"x": -3.0, "y": -3.0},
                    "robot_radius_m": 0.23,
                    "clearance_m": 0.0,
                    "surface_tolerance_m": 0.03,
                    "rows": [
                        "".join(
                            "#" if cell else "."
                            for cell in grid.data[y * 120 : (y + 1) * 120]
                        )
                        for y in range(120)
                    ],
                }
            )
            adapter = RoutedSkillAdapter(
                {"navigate_to": port}, conditions, StopBackend(), mode="simulation"
            )
            app = RobotApplication(
                {},
                navigation_map,
                adapter,
                initial_xy=(-1.0, 0.0),
                exact_goal_simulation=False,
            )
            application[0] = app
            gateway = Gateway(app, secrets.token_urlsafe(32), GatewayStore(":memory:"))
            gateway.started = started
            gateway.lock = bus_lock
            http_gateway[0] = gateway
            http_server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(gateway))
            http_server.daemon_threads = True
            http_worker = threading.Thread(
                target=lambda: http_server.serve_forever(poll_interval=0.02),
                daemon=True,
            )
            http_worker.start()

            def http(method, path, body=None, permit=None, authenticated=True):
                connection = HTTPConnection(
                    "127.0.0.1", http_server.server_port, timeout=1
                )
                headers = {"Content-Type": "application/json"}
                if authenticated:
                    headers["Authorization"] = "Bearer " + gateway.token
                if permit is not None:
                    headers["X-Request-Permit"] = permit
                try:
                    connection.request(
                        method,
                        path,
                        body=None if body is None else json.dumps(body),
                        headers=headers,
                    )
                    response = connection.getresponse()
                    return response.status, json.loads(response.read())
                finally:
                    connection.close()

            def submit_http(body):
                code, permission = http("GET", "/v1/permit")
                assert code == 200
                return http("POST", "/v1/requests", body, permission["permit"])

            assert http("GET", "/v1/health", authenticated=False)[0] == 401
            assert http("GET", "/v1/map")[1]["map_id"] == "nav2-room"

            def request(rid, x, y):
                return {
                    "schema_version": 1,
                    "operation": "navigate_to",
                    "request_id": rid,
                    "map_id": "nav2-room",
                    "map_revision": "synthetic-1",
                    "floor_id": "ground",
                    "frame_id": "map",
                    "point": {"x": x, "y": y, "z": 0.0},
                    "yaw_rad": 0.0,
                }

            first = request("arrive", 1.2, 0.0)
            start_goal = time.monotonic()
            assert http("POST", "/v1/requests", first)[0] == 409
            assert submit_http(first)[0] == 202
            assert http("POST", "/v1/requests", first)[0] == 200
            code, result = submit_http(request("busy", -1.0, 1.0))
            assert code == 409 and result["error"] == "robot_busy"
            until(lambda: app.status("arrive")["status"] in ("SUCCEEDED", "FAILED"), 45)
            assert app.status("arrive")["status"] == "SUCCEEDED", app.status("arrive")
            assert app.lease.owner is None and not app.pose_known
            assert http("GET", "/v1/requests/arrive")[1]["status"] == "SUCCEEDED"
            report["authenticated_http_navigation"] = True
            # Replace the ideal goal coordinate with the current AMCL/TF estimate
            # before planning another request.
            localized = tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            app.localize_simulated(
                (localized.transform.translation.x, localized.transform.translation.y),
                time.monotonic() - started,
            )
            report["application_idempotency_and_lease"] = True
            assert math.hypot(odometry.pose[0] - 1.2, odometry.pose[1]) < 0.12
            assert moving_commands[0] > 0
            report["navigate_wall_s"] = round(time.monotonic() - start_goal, 3)
            report["arrival_pose"] = odometry.pose
            estimate = localization[0].pose.pose.position
            report["amcl_pose"] = [estimate.x, estimate.y]
            assert (
                math.hypot(estimate.x - odometry.pose[0], estimate.y - odometry.pose[1])
                < 0.2
            )
            # Independently check the synthetic circular body against world geometry.
            clearance = min(
                math.hypot(max(0.3 - x, 0, x - 0.7), max(-0.2 - y, 0, y - 0.4))
                for _, x, y, _ in trace
            )
            assert clearance >= 0.22
            report["minimum_obstacle_clearance_m"] = clearance
            before = moving_commands[0]
            assert submit_http(request("cancel", -1.0, 1.0))[0] == 202
            until(
                lambda: moving_commands[0] > before
                and any(status in (1, 2, 3) for status in follow_status.values()),
                10,
            )
            active_follow = {
                goal for goal, status in follow_status.items() if status in (1, 2, 3)
            }
            if args.fault == "none":
                code, result = http("POST", "/v1/requests/cancel/cancel", {})
                assert code == 202
                assert result["control_owned"] and result["status"] == "STOPPING"
                final_status = "CANCELLED"
            else:
                injected_at = time.monotonic()
                if args.fault == "scan_loss":
                    scan_enabled[0] = False
                else:
                    tf_enabled[0] = False
                until(lambda: not enabled[0], 1.0)
                report["fault_stop_request_latency_s"] = time.monotonic() - injected_at
                assert app.status("cancel")["control_owned"]
                final_status = "FAILED"
            until(lambda: app.status("cancel")["status"] == final_status, 10)
            assert all(follow_status.get(goal) in (4, 5, 6) for goal in active_follow)
            assert app.lease.owner is None
            assert app.status("cancel")["simulated_stop_confirmed"]
            report["follow_path_terminal_confirmed"] = True
            report["cancel_confirmed"] = args.fault == "none"
            report["fault_stop_confirmed"] = args.fault != "none"
            report["application_stop_lease_release"] = True
            assert http("GET", "/v1/requests/cancel")[1]["status"] == final_status
            assert http("GET", "/v1/health")[1]["active_request"] is None
            if args.fault != "none":
                assert app.status("cancel")["reason"] == "continuous_condition_lost"
            report["http_status_and_stop"] = True
            report["moving_cmd_vel_count"] = moving_commands[0]
            report["host_frames"] = len(serial.writes)
            report["passed"] = True
    finally:
        if http_server is not None:
            http_server.shutdown()
            http_server.server_close()
        if http_worker is not None:
            http_worker.join(timeout=1)
        if http_gateway[0] is not None:
            http_gateway[0].store.db.close()
        enabled[0] = False
        if client.session is not None:
            try:
                client.stop()
            except Exception:
                report["cleanup_stop_failed"] = True
        for process in reversed(processes):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        report["process_exit_codes"] = [p.returncode for p in processes]
        report["passed"] = report["passed"] and all(
            p.returncode == 0 for p in processes
        )
        (args.output / "report.json").write_text(json.dumps(report, indent=2))
        (args.output / "path.json").write_text(json.dumps(trace))
        (args.output / "control.json").write_text(json.dumps(control_trace))
        port.client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report))
    if not report["passed"]:
        raise RuntimeError("Nav2 run or shutdown failed")


if __name__ == "__main__":
    main()
