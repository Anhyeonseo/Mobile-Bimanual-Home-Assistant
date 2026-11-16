"""Sensor-input-only SLAM. No driver, serial owner, Nav2 controller or motors."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,EmitEvent,RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    node=LifecycleNode(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
        namespace=LaunchConfiguration('namespace'),output='screen',
        parameters=[LaunchConfiguration('parameters'),{'use_sim_time':LaunchConfiguration('use_sim_time')}],
        remappings=[('/map','map'),('/map_metadata','map_metadata')])
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='offline_s4'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('parameters',default_value=PathJoinSubstitution([FindPackageShare('home_robot_tasks'),'config','slam.replay.yaml'])),
        node,
        RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,goal_state='inactive',entities=[
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),transition_id=Transition.TRANSITION_ACTIVATE))])),
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),transition_id=Transition.TRANSITION_CONFIGURE)),
    ])
