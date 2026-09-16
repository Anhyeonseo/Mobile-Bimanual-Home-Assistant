#!/usr/bin/env python3
"""Expand candidate geometry and verify mobile mount FK without a robot."""
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from home_robot_tasks.grasp_yaw import GraspYawKinematics

ROOT = Path(__file__).resolve().parents[2]


def main():
    xml = xacro.process_file(
        str(ROOT / "ros2_ws/src/so101_description/urdf/alohamini.simulation.urdf.xacro")
    ).toxml()
    robot = ET.fromstring(xml)
    links = [node.attrib["name"] for node in robot.findall("link")]
    joints = robot.findall("joint")
    children = [node.find("child").attrib["link"] for node in joints]
    assert len(links) == len(set(links)) and len(children) == len(set(children))
    assert set(links) - set(children) == {"base_footprint"}
    assert len(joints) == len(links) - 1
    mounts={j.attrib['name']:j for j in joints}
    assert mounts['camera_mount'].find('parent').attrib['link']=='lift_link'
    assert mounts['depth_optical'].find('parent').attrib['link']=='camera_link'
    for name in ('front','rear'):
        assert mounts[name+'_rgb_mount'].find('parent').attrib['link']=='base_link'
        assert name+'_rgb_optical_frame' in links
    tilted=ET.fromstring(xacro.process_file(str(ROOT/'ros2_ws/src/so101_description/urdf/alohamini.simulation.urdf.xacro'),
                                           mappings={'camera_rpy':'0 0.25 0'}).toxml())
    assert tilted.find("joint[@name='camera_mount']/origin").attrib['rpy']=='0 0.25 0'
    assert all(node.find("parent").attrib["link"] in links for node in joints)
    with tempfile.TemporaryDirectory(prefix="alohamini-model-") as directory:
        model = Path(directory) / "robot.urdf"
        model.write_text(xml)
        for arm in ("left_", "right_"):
            fk = GraspYawKinematics(model, arm)
            try:
                fk.point_in_base_frame(np.array([1.0, 0.0, 1.0]))
            except ValueError:
                pass
            else:
                raise AssertionError("moving mount accepted without lift position")
            low = fk.point_in_base_frame(
                np.array([1.0, 0.0, 1.0]), joint_positions={"lift_joint": 0.0}
            )
            high = fk.point_in_base_frame(
                np.array([1.0, 0.0, 1.0]), joint_positions={"lift_joint": 0.2}
            )
            assert np.allclose(high - low, [0, 0, -0.2])
    print(
        json.dumps(
            {
                "mode": "simulation",
                "links": len(links),
                "joints": len(joints),
                "root": "base_footprint",
                "lift_fk_checked_both_arms": True,
                "dimensions_measured": False,
            }
        )
    )


if __name__ == "__main__":
    main()
