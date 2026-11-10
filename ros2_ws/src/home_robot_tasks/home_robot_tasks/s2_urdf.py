"""Render an observation model after the caller has validated its profile."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from .s2_geometry import ARMS, JOINTS, UPSTREAM, origin, rigid


def _rpy(m):
    r = rigid(m)[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        return [0., pitch, math.atan2(-r[0, 1], r[1, 1])]
    return [math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])]


def render_observation_urdf(profile, geometry, *, lift_axis="0 0 1", lift_travel=.64, ground_z=None):
    """Internal renderer: callers must validate their own profile contract first."""
    root = ET.Element("robot", name="alohamini1_s2_coordinates")
    root.append(ET.Comment("Kinematic observation model ONLY; no collision model or motion authorization."))
    root.append(ET.Comment("Calibration " + profile["calibration_id"]))
    ET.SubElement(root, "link", name="base_link")
    ET.SubElement(root, "link", name="s2_lift_link")

    def joint(name, parent, child, matrix, kind="fixed", axis=None, limits=None):
        j = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=" ".join(map(str, matrix[:3, 3])), rpy=" ".join(map(str, _rpy(matrix))))
        if axis:
            ET.SubElement(j, "axis", xyz=axis)
        if limits:
            ET.SubElement(j, "limit", **limits)

    if ground_z is not None:
        ET.SubElement(root, "link", name="base_footprint")
        footprint = np.eye(4)
        footprint[2, 3] = ground_z
        joint("s2_floor_projection", "base_link", "base_footprint", footprint)
    joint("s2_lift_joint", "base_link", "s2_lift_link", np.eye(4), "prismatic", lift_axis,
          {"lower": "0", "upper": str(lift_travel), "effort": "0", "velocity": "0"})
    ET.SubElement(root, "link", name=profile["source_frame"])
    joint("s2_fixed_camera", "base_link", profile["source_frame"], rigid(profile["base_from_optical"]))
    names = dict(zip(UPSTREAM, JOINTS))
    for side in ARMS:
        prefix = side + "_s2_"
        links = {j[tag]["link"] for j in geometry["arm_joints"] for tag in ("parent", "child")}
        for link in sorted(links):
            ET.SubElement(root, "link", name=prefix + link)
        arm = profile["arms"][side]
        joint(side + "_s2_mount", "s2_lift_link", prefix + "base_link", rigid(arm["base_from_arm_at_lift_zero"]))
        for j in geometry["arm_joints"]:
            name = f"{side}_model_{names[j['name']]}" if j["name"] in names else side + "_s2_" + j["name"]
            joint(name, prefix + j["parent"]["link"], prefix + j["child"]["link"], origin(j), j["type"],
                  j.get("axis", {}).get("xyz") if j["type"] != "fixed" else None, j.get("limit"))
        ET.SubElement(root, "link", name=side + "_tcp")
        joint(side + "_s2_tcp", prefix + "gripper_frame_link", side + "_tcp", rigid(arm["reference_from_tcp"]))
    ET.indent(root)
    return ET.tostring(root, encoding="unicode") + "\n"
