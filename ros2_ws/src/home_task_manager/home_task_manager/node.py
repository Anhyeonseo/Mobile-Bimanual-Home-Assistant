"""Single-threaded ROS transport. Watchdog time is monotonic, not /clock."""
import json
import time
import threading

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from home_task_interfaces.srv import SubmitTask, CancelTask, GetTaskStatus
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from .core import TaskManager, STAGES


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.origin = time.monotonic()
        self.bridge = None
        self.transport = None
        self.state_lock = threading.RLock()
        backend = self.declare_parameter("backend", "mock").value
        gateway_enabled = self.declare_parameter("gateway_enabled", False).value
        token_file = self.declare_parameter("gateway_token_file", "").value
        database = self.declare_parameter("gateway_database", "").value
        port = self.declare_parameter("gateway_port", 8765).value
        duration = self.declare_parameter("mock_step_duration_s", 0.5).value
        stop_delay = self.declare_parameter("mock_stop_delay_s", 0.2).value
        fail = self.declare_parameter("mock_fail_state", "").value
        never_stop = self.declare_parameter("mock_never_stop", False).value
        timeout = self.declare_parameter("step_timeout_s", 10.0).value
        if fail and fail not in dict((s, k) for s, k, _ in STAGES):
            raise ValueError("mock_fail_state must name a Pick stage")
        # Validate numeric parameters at startup, before opening services.
        FakeRobot(duration_s=duration, stop_delay_s=stop_delay)

        def factory():
            robot = FakeRobot(duration_s=duration, stop_delay_s=stop_delay)
            robot.fail_skill = next((k for s, k, _ in STAGES if s == fail), None)
            robot.never_stop = never_stop
            return robot

        if backend == "ros_action":
            from .ros_adapter import RosSkillTransport
            self.transport = RosSkillTransport(self, self.now)
            factory = self.transport.adapter
        elif backend != "mock":
            raise ValueError("backend must be mock or ros_action")
        self.manager = TaskManager(factory, step_timeout_s=timeout)
        if gateway_enabled:
            if not token_file or not database:
                raise ValueError("Gateway requires gateway_token_file and gateway_database")
            from .gateway_bridge import GatewayBridge
            self.bridge = GatewayBridge(self.manager, token_file, database, port, self.origin)
            self.state_lock = self.bridge.gateway.lock
            self.get_logger().info(f"Authenticated Pick Gateway: http://127.0.0.1:{self.bridge.server.server_port}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "task/status", qos)
        if not gateway_enabled:
            self.create_service(SubmitTask, "task/submit", self.submit)
            self.create_service(CancelTask, "task/cancel", self.cancel)
        self.create_service(GetTaskStatus, "task/get_status", self.get_status)
        self.timer = self.create_timer(0.1, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()
        self.get_logger().info("Pick Task Manager ready (simulation only)")

    def now(self):
        return time.monotonic() - self.origin

    def publish(self):
        with self.state_lock:
            self.publisher.publish(String(data=json.dumps(self.manager.status(), ensure_ascii=False)))

    def tick(self):
        with self.state_lock:
            if self.bridge:
                try:
                    self.bridge.gateway.tick()
                except Exception as error:
                    self.bridge.gateway.latch_fault()
                    self.get_logger().error(f"Gateway fault latched: {type(error).__name__}")
            else:
                self.manager.tick(self.now())
            self.publish()

    def submit(self, request, response):
        try:
            status = self.manager.submit(request.json_request, self.now())
            response.accepted, response.task_id = True, status["task_id"]
            response.message = "accepted (simulation)"
            response.status_json = json.dumps(status, ensure_ascii=False)
            self.publish()
        except InvalidTask as error:
            response.accepted, response.message = False, str(error)
            response.status_json = json.dumps(self.manager.status())
        return response

    def cancel(self, request, response):
        try:
            status = self.manager.cancel(request.task_id, self.now())
            response.accepted, response.message = True, "inspect state for stop confirmation"
            response.status_json = json.dumps(status)
            self.publish()
        except InvalidTask as error:
            response.accepted, response.message = False, str(error)
            response.status_json = json.dumps(self.manager.status())
        return response

    def get_status(self, request, response):
        with self.state_lock:
            try:
                response.status_json = json.dumps(self.manager.status(request.task_id), ensure_ascii=False)
                response.found = True
            except InvalidTask as error:
                response.found, response.message = False, str(error)
        return response

    def stop_and_close(self):
        if self.bridge:
            self.bridge.close_http()
        with self.state_lock:
            if self.manager.lease.owner:
                self.manager.cancel(self.manager.lease.owner, self.now())
        deadline = time.monotonic() + 3
        while rclpy.ok() and self.manager.lease.owner and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.manager.lease.owner:
            self.get_logger().error("Shutdown without confirmed stop; task ownership remains uncertain")
        if self.bridge:
            self.bridge.close_store()
        if self.transport:
            self.transport.destroy()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = TaskManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
