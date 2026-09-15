"""Publish the explicitly unmeasured mobile model; no hardware drivers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = PathJoinSubstitution(
        [
            FindPackageShare("so101_description"),
            "urdf",
            "alohamini.simulation.urdf.xacro",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", model]), value_type=str
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
