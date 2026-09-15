import json
import math
from pathlib import Path
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.kinematic_robot import KinematicRobot
from home_robot_tasks.navigation import NavigationMap

ROOT = Path(__file__).resolve().parents[1]


def read(name):
    return json.loads((ROOT / "config" / name).read_text())


def app():
    return RobotApplication(
        read("home.example.json"),
        NavigationMap(read("navigation_map.simulation.json")),
        KinematicRobot(),
    )


def test_plant_follows_path_and_rotation_without_colliding_in_static_grid():
    a = app()
    request = read("navigate_to.simulation.json")
    request["yaw_rad"] = 1
    a.submit(request, 0)
    for i in range(1800):
        a.tick(i / 10)
        a.map._cell(*a.adapter.xy)
        if a.status(request["request_id"])["status"] == "SUCCEEDED":
            break
    assert i > 10 and i < 1800
    assert math.dist(a.adapter.xy, (7.25, 4.25)) < 1e-8
    assert abs(a.adapter.yaw - 1) < 1e-6
    assert a.adapter.velocity == 0


def test_cancel_waits_for_kinematic_deceleration():
    a = app()
    rid = read("navigate_to.simulation.json")["request_id"]
    a.submit(read("navigate_to.simulation.json"), 0)
    for i in range(11):
        a.tick(i / 10)
    speed = a.adapter.velocity
    assert speed > 0
    result = a.cancel(rid, 1)
    assert result["status"] == "STOPPING" and result["control_owned"]
    for i in range(11, 31):
        a.tick(i / 10)
    assert a.status(rid)["status"] == "CANCELLED" and a.adapter.velocity == 0
