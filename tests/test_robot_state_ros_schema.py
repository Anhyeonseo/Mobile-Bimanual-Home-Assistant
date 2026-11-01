"""Catch Python observation / ROS wire drift even on hosts without ROS installed."""
from dataclasses import fields
from pathlib import Path

from home_robot_tasks.robot_state import RobotObservation


def test_robot_state_ros_schema_carries_every_observation_field():
    path=Path(__file__).resolve().parents[1]/'ros2_ws/src/so101_interfaces/msg/RobotStateEvidence.msg'
    declarations={}
    for line in path.read_text().splitlines():
        line=line.split('#',1)[0].strip()
        if line:
            kind,name=line.split()
            declarations[name]=kind
    expected={f.name for f in fields(RobotObservation)}-{'observed_s'}
    assert set(declarations)==expected|{'header','profile_sha256'}
    assert declarations['joint_names']=='string[<=12]'
    assert declarations['joints_rad']=='float64[<=12]'
    assert declarations['header']=='std_msgs/Header'
