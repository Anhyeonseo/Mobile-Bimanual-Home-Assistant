"""Shared ALOHA Mini 1 geometry and kinematics. No hardware access or acceptance policy."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper")
UPSTREAM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARMS = ("left", "right")
def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def nominal_base_from_cad(geometry):
    """Mini 1 axle-centroid reference, not a measured installation transform."""
    if geometry['robot_model'] != 'alohamini1':
        raise ValueError('Mini 1 geometry required for nominal base reference')
    joints={j['name']:j for j in geometry['mobile_reference_joints']}
    center=np.mean([origin(joints[name])[:3,3] for name in ('wheel1','wheel2','wheel3')],axis=0)
    result=np.eye(4)
    result[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
    result[:3,3]=-result[:3,:3]@center
    return result


def number(value, name, maximum=100):
    if type(value) not in (int, float) or abs(value) > maximum or not math.isfinite(value):
        raise ValueError(f"{name}: finite value in ±{maximum} required")
    return float(value)


def vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name}: {length} values required")
    return np.array([number(v, name) for v in value])


def text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 200:
        raise ValueError(f"{name}: nonempty text required")
    return value


def rotation(axis, angle):
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross
    return result


def origin(joint):
    o = joint.get("origin", {})
    xyz = [float(x) for x in o.get("xyz", "0 0 0").split()]
    r, p, y = [float(x) for x in o.get("rpy", "0 0 0").split()]
    result = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    result[:3, 3] = xyz
    return result


def rigid(value):
    m = np.asarray(value, dtype=float)
    if (m.shape != (4, 4) or not np.isfinite(m).all() or
            not np.allclose(m[3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-7) or
            not math.isclose(np.linalg.det(m[:3, :3]), 1, abs_tol=1e-7)):
        raise ValueError("proper rigid transform required")
    return m


def load_geometry(path):
    path = Path(path)
    data = json.loads(path.read_text())
    if data["robot_model"] != "alohamini1" or data["arm_model"] != "SO-ARM101":
        raise ValueError("ALOHA Mini 1 + SO-ARM101 required")
    expected = {"alohamini1.urdf", "so101_new_calib.urdf", "alohamini_config.py.txt"}
    if len(data["sources"]) != 3 or {s["file"] for s in data["sources"]} != expected:
        raise ValueError("all three pinned official sources required")
    for source in data["sources"]:
        name = source["file"]
        if Path(name).name != name:
            raise ValueError("source must be a local basename")
        raw = (path.parent / "official_sources" / name).read_bytes()
        if hashlib.sha256(raw).hexdigest() != source["sha256"]:
            raise ValueError("official source hash changed")
        key = {"so101_new_calib.urdf": "arm_joints", "alohamini1.urdf": "mobile_reference_joints"}.get(name)
        if key:
            extracted = []
            for j in ET.fromstring(raw).findall("joint"):
                row = dict(j.attrib)
                for tag in ("parent", "child", "origin", "axis", "limit"):
                    e = j.find(tag)
                    if e is not None:
                        row[tag] = dict(e.attrib)
                extracted.append(row)
            if data[key] != extracted:
                raise ValueError("derived geometry differs from pinned official source")
    if data["lift_nominal_m_per_turn"] != .084 or data["encoder_counts_per_turn"] != 4096:
        raise ValueError("unexpected Mini 1 nominal transmission")
    return data


def model_from_control(control_rad, joint):
    return joint['home_model_rad'] + joint['control_to_model_sign'] * number(control_rad, 'control angle')


def control_from_model(model_rad, joint):
    return joint['control_to_model_sign'] * (number(model_rad, 'model angle') - joint['home_model_rad'])


def model_angle_from_raw(raw, joint, name):
    delta = (number(raw, name, 32768) - 2048) * math.tau / 4096
    if 'control_to_model_sign' in joint:
        return model_from_control(joint['raw_to_control_sign'] * delta, joint)
    # Historical development and independently measured commissioning maps store the raw slope.
    return joint['home_model_rad'] + joint['sign'] * delta


def model_angles(raw, mapping):
    if set(raw) != set(JOINTS) or set(mapping) != set(JOINTS):
        raise ValueError("exactly six named joints required; positional arrays are not accepted")
    return {j: model_angle_from_raw(raw[j], mapping[j], j) for j in JOINTS}


def arm_transforms(geometry, angles):
    joints = {j["name"]: j for j in geometry["arm_joints"]}
    transforms = {"base_link": np.eye(4)}
    for key, name in zip(JOINTS, UPSTREAM):
        j = joints[name]
        axis = [float(x) for x in j["axis"]["xyz"].split()]
        transforms[j["child"]["link"]] = transforms[j["parent"]["link"]] @ origin(j) @ rotation(axis, angles[key])
    tcp = joints["gripper_frame_joint"]
    transforms["gripper_frame_link"] = transforms[tcp["parent"]["link"]] @ origin(tcp)
    return transforms


def tool_pose(geometry, arm, raw):
    transforms = arm_transforms(geometry, model_angles(raw, arm["alignment"]))
    return transforms["gripper_frame_link"] @ rigid(arm["reference_from_tcp"])

def optical_to_base(base_from_optical, point):
    """Transform a point, retaining its caller-defined surface/center semantics."""
    xyz = vector(point, 3, 'camera point')
    if xyz[2] <= 0:
        raise ValueError('positive optical depth required')
    return (rigid(base_from_optical) @ np.r_[xyz, 1])[:3].tolist()
