#!/usr/bin/env python3
"""PC DDS test of Nav2Port using a synthetic NavigateToPose action server.

Requires ROS Jazzy and nav2_msgs. Uses only /offline_nav2/navigate_to_pose;
no cmd_vel, sensor, motor or device connection. This tests action transport,
not the Nav2 planner/controller or localization.
"""
import json
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose, FollowPath
from home_robot_tasks.nav2_port import Nav2Port, Nav2FollowPathPort


def main():
    rclpy.init()
    server_node = rclpy.create_node("offline_nav2_contract_server")
    client_node = rclpy.create_node("offline_nav2_contract_client")

    def execute(handle):
        for _ in range(30):
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(
        server_node,
        NavigateToPose,
        "/offline_nav2/navigate_to_pose",
        execute_callback=execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    def follow(handle):
        if len(handle.request.path.poses)!=3:
            handle.abort();return FollowPath.Result()
        for _ in range(30):
            if handle.is_cancel_requested:
                handle.canceled();return FollowPath.Result()
            time.sleep(.01)
        handle.succeed();return FollowPath.Result()
    path_server=ActionServer(server_node,FollowPath,"/offline_nav2/follow_path",execute_callback=follow,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    path_port=Nav2FollowPathPort(client_node)
    port = Nav2Port(client_node)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(client_node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    def until(predicate):
        deadline = time.monotonic() + 10
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError("offline action contract timeout")
            time.sleep(0.01)

    try:
        until(port.ready)
        params = {"frame_id": "map", "goal": {"x": 1.0, "y": 2.0, "yaw_rad": 0.5}}
        port.start("success", params)
        until(lambda: port.poll("success").state != "RUNNING")
        assert port.poll("success").state == "SUCCEEDED"
        port.start("cancel_before_accept", params)
        port.cancel("cancel_before_accept")
        until(lambda: port.poll("cancel_before_accept").state != "RUNNING")
        assert port.poll("cancel_before_accept").state == "CANCELLED"
        until(path_port.ready)
        path=dict(frame_id='map',poses_xy_yaw=[[0.,0.,0.],[.1,.05,.2],[.2,.1,.4]])
        path_port.start('path-success',path)
        until(lambda:path_port.poll('path-success').state!='RUNNING')
        assert path_port.poll('path-success').state=='SUCCEEDED'
        path_port.start('path-cancel',path);path_port.cancel('path-cancel')
        until(lambda:path_port.poll('path-cancel').state!='RUNNING')
        assert path_port.poll('path-cancel').state=='CANCELLED'
        print(
            json.dumps(
                {
                    "mode": "simulation",
                    "transport": "ROS2 DDS NavigateToPose",
                    "cases": ["success", "cancel_before_accept", "follow_checked_path", "cancel_follow_path"],
                    "hardware_commands": 0,
                    "nav2_planner_tested": False,
                }
            )
        )
    finally:
        executor.shutdown(timeout_sec=5)
        worker.join(timeout=5)
        path_server.destroy()
        path_port.client.destroy()
        server.destroy()
        port.client.destroy()
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
