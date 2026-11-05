"""S2 geometry and acceptance, with explicit installation evidence and no device I/O.

Official geometry, encoder calibration and installed mounts are separate inputs.
An accepted coordinate profile never grants motion permission.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .camera_registration import fit_fixed_camera, _spread

JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper")
UPSTREAM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARMS = ("left", "right")
POLICY = {"camera_max_m": .010, "tcp_max_m": .010, "combined_max_m": .015,
          "lift_max_m": .005, "depth_max_m": .010, "maximum_skew_s": .050,
          "maximum_age_s": .200, "reference_angle_max_rad": math.radians(2),
          "tcp_orientation_max_rad": math.radians(3), "report_maximum_age_s": 5.0}


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


def alignment(checks):
    """Two independently known model angles per joint; signed unwrapped raw readings.

    The usual first angle is upstream new_calib zero. No EEPROM is modified.
    The second angle checks direction AND the 4096-count scale.
    """
    if set(checks) != set(JOINTS):
        raise ValueError("six named joint alignment checks required")
    result = {}
    for joint in JOINTS:
        samples = checks[joint]
        if not isinstance(samples, list) or len(samples) != 2:
            raise ValueError(f"{joint}: two reference angles required")
        raw = [number(s["unwrapped_raw"], joint, 32768) for s in samples]
        q = [number(s["model_rad"], joint, 2 * math.pi) for s in samples]
        delta = (raw[1] - raw[0]) * 2 * math.pi / 4096
        if abs(delta) < math.radians(10) or abs(q[1] - q[0]) < math.radians(10):
            raise ValueError(f"{joint}: use reference angles at least 10 degrees apart")
        sign = 1 if delta * (q[1] - q[0]) > 0 else -1
        if abs(sign * delta - (q[1] - q[0])) > POLICY["reference_angle_max_rad"]:
            raise ValueError(f"{joint}: direction/encoder scale/reference angles disagree")
        result[joint] = {"sign": sign, "home_model_rad": q[0] - sign * (raw[0] - 2048) * 2 * math.pi / 4096}
    return result


def model_angles(raw, mapping):
    if set(raw) != set(JOINTS) or set(mapping) != set(JOINTS):
        raise ValueError("exactly six named joints required; positional arrays are not accepted")
    return {j: mapping[j]["home_model_rad"] + mapping[j]["sign"] *
            (number(raw[j], j, 32768) - 2048) * 2 * math.pi / 4096 for j in JOINTS}


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


def fit_mount(local_points, base_points):
    local, base = np.asarray(local_points), np.asarray(base_points)
    _spread(local, "mount local points")
    _spread(base, "mount base points")
    for points in (local, base):
        if len(np.unique(np.round(points, 7), axis=0)) != len(points):
            raise ValueError("distinct mount fit points required")
    x, y = local.mean(axis=0), base.mean(axis=0)
    u, _, vt = np.linalg.svd((local - x).T @ (base - y))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    m = np.eye(4)
    m[:3, :3] = vt.T @ correction @ u.T
    m[:3, 3] = y - m[:3, :3] @ x
    return m


def lift_height(lift, raw, origin_raw=None):
    current = number(raw, "lift raw", 1e12)
    zero = number(lift["origin_raw_unwrapped"] if origin_raw is None else origin_raw, "lift origin", 1e12)
    h = -(current - zero) * lift["m_per_turn"] / 4096
    if not -.005 <= h <= .645:
        raise ValueError("lift coordinate outside surveyed 0..0.64m travel")
    return h


def timing(row):
    text(row["clock_id"], "common clock_id")
    values = [number(row[k], k, 1e12) for k in ("camera_time_s", "joints_time_s", "lift_time_s")]
    if min(values) < 0 or max(values) - min(values) > POLICY["maximum_skew_s"]:
        raise ValueError("camera/joint/lift capture time mismatch")


def template(geometry):
    def pose(identifier, check=False):
        row = {"id": identifier, "raw": dict.fromkeys(JOINTS), "lift_raw_unwrapped": None,
               "base_tcp_xyz_m": [None] * 3, "clock_id": None,
               "camera_time_s": None, "joints_time_s": None, "lift_time_s": None}
        if check:
            row["camera_tcp_xyz_m"] = [None] * 3
            row["base_tcp_rotation"] = [[None] * 3 for _ in range(3)]
        return row
    def camera_pair(identifier):
        return {"id": identifier, "camera_xyz_m": [None] * 3, "base_xyz_m": [None] * 3}
    return {"schema_version": 1, "robot_model": "alohamini1", "arm_model": "SO-ARM101",
            "geometry_sha256": digest(geometry), "provenance": "measured", "operator": None,
            "assembly_revision": None, "base_reference": None, "units": "m",
            "stock_parts_confirmed": None,
            "camera": {"schema_version": 1, "units": "m", "source_frame": "color_camera_optical_frame",
                       "target_frame": "base_link", "mount_revision": None, "base_reference": None,
                       "provenance": "measured", "maximum_error_m": POLICY["camera_max_m"],
                       "fit": [camera_pair(f"camera-fit-{i}") for i in range(6)],
                       "check": [camera_pair(f"camera-check-{i}") for i in range(3)]},
            "camera_device": {"device_id_hash": None, "stream_profile_id": None,
                              "factory_calibration_sha256": None, "fixed_to_base_confirmed": None,
                              "depth_checks": [{"reference_z_m": None, "observed_z_m": None} for _ in range(3)]},
            "live_checks": [{"capture_id": None, "assembly_revision": None,
                             "camera_device": {"device_id_hash": None, "stream_profile_id": None,
                                               "factory_calibration_sha256": None},
                             "source_frame": "color_camera_optical_frame", "session_id": None,
                             "origin_id": None, "clock_id": None, "received_s": None,
                             "camera_time_s": None, "joints_time_s": None, "lift_time_s": None,
                             "camera_xyz_m": [None] * 3, "lift_raw_unwrapped": None,
                             "raw": {side: dict.fromkeys(JOINTS) for side in ARMS}} for _ in range(3)],
            "lift": {"m_per_turn": geometry["lift_nominal_m_per_turn"], "scale_source": "official_nominal",
                     "origin_raw_unwrapped": None, "origin_id": None, "session_id": None,
                     "bottom_platform_height_m": None,
                     "checks": [{"raw_unwrapped": None, "platform_height_m": None} for _ in range(4)],
                     "rehome_check": {"previous_session_id": None, "new_session_id": None,
                                      "reference_height_m": None, "observed_height_m": None}},
            "arms": {side: {"alignment_checks": {j: [{"unwrapped_raw": None, "model_rad": 0.0},
                                                     {"unwrapped_raw": None, "model_rad": None}] for j in JOINTS},
                            "tcp_definition": "official gripper_frame_link; fixed reference, not opening-dependent contact center",
                            "reference_from_tcp": np.eye(4).tolist(),
                            "fit": [pose(f"{side}-fit-{i}") for i in range(6)],
                            "check": [pose(f"{side}-check-{i}", True) for i in range(3)]} for side in ARMS}}


def missing_fields(value, path=""):
    if value is None:
        return [path]
    if isinstance(value, dict):
        return [p for k, v in value.items() for p in missing_fields(v, f"{path}.{k}".strip("."))]
    if isinstance(value, list):
        return [p for i, v in enumerate(value) for p in missing_fields(v, f"{path}[{i}] ".strip())]
    return []


def commission(data, geometry):
    """Recompute all fits and held-out errors; never trust prefilled 'passed' flags."""
    # Require the full schema even when a caller removes a whole unfinished section.
    required = set(template(geometry))
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("complete S2 commissioning record required")
    pending = missing_fields(data)
    if pending:
        return {"status": "needs_physical_checks", "missing": pending, "s2_complete": False,
                "coordinate_use_allowed": False, "motion_authorized": False}
    if (type(data["schema_version"]) is not int or data["schema_version"] != 1 or data["robot_model"] != "alohamini1" or
            data["arm_model"] != "SO-ARM101" or data["units"] != "m" or
            data["geometry_sha256"] != digest(geometry)):
        raise ValueError("model/units/official geometry revision mismatch")
    if data["provenance"] not in ("measured", "simulation") or data["stock_parts_confirmed"] is not True:
        raise ValueError("explicit provenance and stock geometry confirmation required")
    for key in ("operator", "assembly_revision", "base_reference"):
        text(data[key], key)
    camera = data["camera"]
    if (camera["mount_revision"] != data["assembly_revision"] or camera["base_reference"] != data["base_reference"] or
            camera["provenance"] != data["provenance"] or camera["maximum_error_m"] > POLICY["camera_max_m"]):
        raise ValueError("camera survey revision/reference/provenance/threshold mismatch")
    fitted = fit_fixed_camera(camera)
    failures = []
    if not fitted["within_threshold"]:
        failures.append("camera independent position error")
    device = data["camera_device"]
    for k in ("device_id_hash", "stream_profile_id", "factory_calibration_sha256"):
        text(device[k], k)
    for k in ("device_id_hash", "factory_calibration_sha256"):
        if len(device[k]) != 64 or any(c not in "0123456789abcdef" for c in device[k]):
            raise ValueError(f"{k}: lowercase SHA256 required")
    if device["fixed_to_base_confirmed"] is not True:
        raise ValueError("fixed-to-base camera required")
    depth = device["depth_checks"]
    if len(depth) < 3:
        raise ValueError("three independent optical-Z depth checks required")
    zs = [number(d["reference_z_m"], "reference Z") for d in depth]
    if min(zs) <= 0 or max(zs) - min(zs) < .2:
        raise ValueError("depth checks must cover at least 0.2m and positive optical Z")
    if any(abs(number(d["observed_z_m"], "depth Z") - z) > POLICY["depth_max_m"] for d, z in zip(depth, zs)):
        failures.append("depth scale/reference error")
    lift = data["lift"]
    if lift["scale_source"] != "official_nominal" or lift["m_per_turn"] != geometry["lift_nominal_m_per_turn"]:
        raise ValueError("Mini 1 nominal lift scale required; revise model explicitly if independent measurements reject it")
    for key in ("origin_id", "session_id"):
        text(lift[key], key)
    bottom = number(lift["bottom_platform_height_m"], "platform height")
    if bottom <= 0:
        raise ValueError("positive bottom platform height required")
    checks = lift["checks"]
    if len(checks) < 4:
        raise ValueError("lift bottom/middle/upper/return samples required")
    heights = [lift_height(lift, c["raw_unwrapped"]) for c in checks]
    if len(set(round(h, 3) for h in heights)) < 3 or max(heights) - min(heights) < .4:
        raise ValueError("lift checks need three heights across at least 0.4m")
    if abs(heights[0]) > .005 or abs(heights[-1]) > .005:
        raise ValueError("lift check must begin and return to its bottom reference")
    lift_errors = [abs(bottom + h - number(c["platform_height_m"], "measured platform")) for c, h in zip(checks, heights)]
    rehome = lift["rehome_check"]
    if (not text(rehome["previous_session_id"], "previous session") or
            rehome["previous_session_id"] == rehome["new_session_id"] or rehome["new_session_id"] != lift["session_id"]):
        raise ValueError("independent restart/re-reference evidence required")
    lift_errors.append(abs(number(rehome["observed_height_m"], "rehome") - number(rehome["reference_height_m"], "reference")))
    lift_errors.append(abs(rehome["reference_height_m"] - bottom))
    if max(lift_errors) > POLICY["lift_max_m"]:
        failures.append("lift scale/return/reference error")
    arms, arm_reports = {}, {}
    ids = {r["id"] for r in camera["fit"] + camera["check"]}
    base_from_camera = rigid(fitted["base_from_optical"])
    for side in ARMS:
        record = data["arms"][side]
        text(record["tcp_definition"], "TCP definition")
        arm = {"alignment": alignment(record["alignment_checks"]),
               "reference_from_tcp": rigid(record["reference_from_tcp"]).tolist(),
               "tcp_definition": record["tcp_definition"]}
        if len(record["fit"]) < 6 or len(record["check"]) < 3:
            raise ValueError("each arm needs six fit and three held-out poses")
        local, base, heights = [], [], []
        for row in record["fit"] + record["check"]:
            identifier = text(row["id"], "pose ID")
            if identifier in ids:
                raise ValueError("camera/arm fit/check IDs must be unique")
            ids.add(identifier)
            timing(row)
            heights.append(lift_height(lift, row["lift_raw_unwrapped"]))
            local.append(tool_pose(geometry, arm, row["raw"])[:3, 3])
            base.append(vector(row["base_tcp_xyz_m"], 3, "measured TCP") - [0, 0, heights[-1]])
        count = len(record["fit"])
        if max(heights) - min(heights) < .2:
            raise ValueError("arm observations must include at least two lift heights 0.2m apart")
        if len(np.unique(np.round(local, 6), axis=0)) != len(local):
            raise ValueError("held-out arm poses must differ from fit poses")
        _spread(np.asarray(local[count:]), "held-out arm points")
        mount = fit_mount(local[:count], base[:count])
        predicted = np.asarray(local) @ mount[:3, :3].T + mount[:3, 3]
        errors = np.linalg.norm(predicted - base, axis=1)
        combined, camera_tcp, orientation_errors = [], [], []
        for i, row in enumerate(record["check"], count):
            p = vector(row["camera_tcp_xyz_m"], 3, "camera TCP")
            if p[2] <= 0:
                raise ValueError("positive camera TCP depth required")
            observed = (base_from_camera @ np.r_[p, 1])[:3]
            camera_tcp.append(float(np.linalg.norm(observed - row["base_tcp_xyz_m"])))
            combined.append(float(np.linalg.norm(observed - (predicted[i] + [0, 0, heights[i]]))))
            observed_pose = np.eye(4)
            measured_rotation = np.asarray(row["base_tcp_rotation"], dtype=float)
            if measured_rotation.shape != (3, 3):
                raise ValueError("independently measured 3x3 TCP rotation required")
            observed_pose[:3, :3] = measured_rotation
            measured_rotation = rigid(observed_pose)[:3, :3]
            expected_rotation = (mount @ tool_pose(geometry, arm, row["raw"]))[:3, :3]
            orientation_errors.append(math.acos(float(np.clip((np.trace(measured_rotation.T @ expected_rotation) - 1) / 2, -1, 1))))
        if max(errors) > POLICY["tcp_max_m"]:
            failures.append(f"{side} independent FK/TCP error")
        if max(camera_tcp) > POLICY["camera_max_m"] or max(combined) > POLICY["combined_max_m"]:
            failures.append(f"{side} camera/TCP end-to-end error")
        if max(orientation_errors) > POLICY["tcp_orientation_max_rad"]:
            failures.append(f"{side} independent TCP orientation error")
        arm["base_from_arm_at_lift_zero"] = mount.tolist()
        arms[side] = arm
        arm_reports[side] = {"fit_max_m": float(max(errors[:count])), "check_max_m": float(max(errors[count:])),
                             "combined_max_m": max(combined), "camera_tcp_max_m": max(camera_tcp),
                             "orientation_max_rad": max(orientation_errors),
                             "survey_local_bounds_m": [np.min(local, axis=0).tolist(), np.max(local, axis=0).tolist()]}
    measured = data["provenance"] == "measured"
    profile = {"schema_version": 1, "robot_model": "alohamini1", "arm_model": "SO-ARM101",
               "geometry_sha256": digest(geometry), "evidence_sha256": digest(data),
               "assembly_revision": data["assembly_revision"], "base_reference": data["base_reference"],
               "camera_device": {k: device[k] for k in ("device_id_hash", "stream_profile_id", "factory_calibration_sha256")},
               "source_frame": camera["source_frame"], "base_from_optical": fitted["base_from_optical"],
               "arms": arms, "lift": {k: lift[k] for k in ("m_per_turn", "origin_raw_unwrapped", "origin_id", "session_id", "bottom_platform_height_m")},
               "policy": dict(POLICY), "provenance": data["provenance"],
               "validation_passed": not failures,
               "coordinate_use_allowed": measured and not failures, "motion_authorized": False}
    profile["calibration_id"] = "s2-" + digest(profile)[:24]
    live = data["live_checks"]
    if not isinstance(live, list) or len(live) < 3:
        raise ValueError("three live synchronized captures required")
    live_ids, times = set(), []
    for row in live:
        identifier = text(row["capture_id"], "live capture ID")
        if identifier in live_ids:
            raise ValueError("distinct live captures required")
        live_ids.add(identifier)
        times.append(number(row["camera_time_s"], "capture time", 1e12))
        if row["clock_id"] != live[0]["clock_id"]:
            raise ValueError("live checks must share one clock")
        # Apply the same runtime validation to recorded acquisition evidence.
        if not failures:
            observe(profile, geometry, {**row, "calibration_id": profile["calibration_id"]},
                    now_s=row["received_s"], allow_simulation=not measured)
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("live capture times must increase")
    return {"status": "rejected" if failures else ("accepted_coordinates" if measured else "simulation_pass"),
            "s2_complete": measured and not failures, "coordinate_use_allowed": profile["coordinate_use_allowed"],
            "motion_authorized": False, "failures": failures, "camera": fitted,
            "lift_max_error_m": max(lift_errors), "arms": arm_reports, "profile": profile}


def observe(profile, geometry, snapshot, *, now_s, allow_simulation=False):
    """Synchronized read-only coordinates for the existing RGB-D report boundary."""
    unsigned = {k: v for k, v in profile.items() if k != "calibration_id"}
    if profile["calibration_id"] != "s2-" + digest(unsigned)[:24] or profile["geometry_sha256"] != digest(geometry):
        raise ValueError("profile/geometry content changed; recommission required")
    if not profile["validation_passed"] or (not profile["coordinate_use_allowed"] and not (allow_simulation and profile["provenance"] == "simulation")):
        raise ValueError("coordinate profile not accepted")
    for key in ("calibration_id", "assembly_revision", "camera_device", "source_frame"):
        if snapshot[key] != profile[key]:
            raise ValueError(f"{key} mismatch")
    if snapshot["origin_id"] != profile["lift"]["origin_id"] or snapshot["session_id"] != profile["lift"]["session_id"]:
        raise ValueError("lift session/origin changed; establish a new origin and rebind explicitly")
    timing(snapshot)
    now = number(now_s, "now", 1e12)
    for key in ("camera_time_s", "joints_time_s", "lift_time_s"):
        if not 0 <= now - snapshot[key] <= profile["policy"]["maximum_age_s"]:
            raise ValueError("stale/future observation")
    point = vector(snapshot["camera_xyz_m"], 3, "camera point")
    if point[2] <= 0:
        raise ValueError("positive optical depth required")
    h = lift_height(profile["lift"], snapshot["lift_raw_unwrapped"])
    tools, positions = {}, {"s2_lift_joint": h}
    for side in ARMS:
        arm = profile["arms"][side]
        mount = rigid(arm["base_from_arm_at_lift_zero"]).copy()
        mount[2, 3] += h
        tools[side] = (mount @ tool_pose(geometry, arm, snapshot["raw"][side])).tolist()
        positions.update({f"{side}_model_{j}": q for j, q in model_angles(snapshot["raw"][side], arm["alignment"]).items()})
    return {"calibration_id": profile["calibration_id"], "frame": "base_link",
            "observed_s": snapshot["camera_time_s"],
            "object_surface_xyz_m": (rigid(profile["base_from_optical"]) @ np.r_[point, 1])[:3].tolist(),
            "base_from_tcp": tools, "model_joint_positions_si": positions,
            "grasp_pose": None, "motion_authorized": False}


def observe_report(profile, geometry, snapshot, report, *, stamp, context, now_s,
                   allow_simulation=False):
    """Bind a DINO/SAM report to validated coordinates and the existing L3A API.

    The capture producer supplies the acquisition stamp, not inference completion
    time. It must convert device clocks before assembling the snapshot.
    """
    from .assistant.perception import TransformSample, report_entities
    if (context.calibration_id != profile["calibration_id"] or context.frame_id != "base_link" or
            context.clock_id != snapshot["clock_id"] or stamp.context != context or
            stamp.observed_s != snapshot["camera_time_s"] or stamp.capture_id != snapshot["capture_id"]):
        raise ValueError("report capture/context does not match S2 snapshot")
    # Inference may finish later: validate freshness when the synchronized state
    # was received, while preserving the acquisition stamp for the delayed report.
    received = number(snapshot["received_s"], "capture receipt", 1e12)
    now = number(now_s, "now", 1e12)
    if now < received:
        raise ValueError("report received before its capture state")
    coordinates = observe(profile, geometry, {**snapshot, "camera_xyz_m": [0., 0., 1.]},
                          now_s=received, allow_simulation=allow_simulation)
    del coordinates["object_surface_xyz_m"]
    transform = TransformSample(stamp, profile["source_frame"], "base_link",
                                profile["base_from_optical"], profile["provenance"])
    coordinates["entities"] = report_entities(report, stamp=stamp, context=context, now_s=now_s,
                                               transform=transform, maximum_age_s=POLICY["report_maximum_age_s"])
    coordinates["coordinate_mode"] = "capture_time"
    coordinates["inference_age_s"] = now - snapshot["camera_time_s"]
    return coordinates


def rebind_origin(profile, evidence):
    """Refresh a manually re-established bottom reference without re-fitting the arms."""
    unsigned = {k: v for k, v in profile.items() if k != "calibration_id"}
    if profile["calibration_id"] != "s2-" + digest(unsigned)[:24] or not profile["validation_passed"]:
        raise ValueError("valid commissioned profile required")
    if evidence["previous_calibration_id"] != profile["calibration_id"] or evidence["assembly_revision"] != profile["assembly_revision"]:
        raise ValueError("origin rebind revision mismatch")
    if evidence["provenance"] != profile["provenance"]:
        raise ValueError("origin rebind provenance mismatch")
    for key in ("operator", "session_id", "origin_id"):
        text(evidence[key], key)
    if evidence["origin_id"] == profile["lift"]["origin_id"]:
        raise ValueError("new origin ID required")
    if abs(number(evidence["platform_height_m"], "bottom check") - profile["lift"]["bottom_platform_height_m"]) > POLICY["lift_max_m"]:
        raise ValueError("bottom reference not re-established")
    number(evidence["origin_raw_unwrapped"], "origin raw", 1e12)
    result = deepcopy(profile)
    for key in ("origin_raw_unwrapped", "session_id", "origin_id"):
        result["lift"][key] = evidence[key]
    result["origin_rebind_evidence_sha256"] = digest(evidence)
    del result["calibration_id"]
    result["calibration_id"] = "s2-" + digest(result)[:24]
    return result


def _rpy(m):
    r = rigid(m)[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        return [0., pitch, math.atan2(-r[0, 1], r[1, 1])]
    return [math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])]


def export_urdf(profile, geometry, *, allow_simulation=False):
    """A kinematic TF model, using model-angle JointStates, not actuator commands.

    Joint names are deliberately separate from historical Home-relative ROS names.
    This model has no collision geometry and must not be used for motion planning.
    """
    unsigned = {k: v for k, v in profile.items() if k != "calibration_id"}
    if profile["calibration_id"] != "s2-" + digest(unsigned)[:24] or profile["geometry_sha256"] != digest(geometry):
        raise ValueError("profile/geometry changed")
    if not profile["validation_passed"] or (not profile["coordinate_use_allowed"] and not (allow_simulation and profile["provenance"] == "simulation")):
        raise ValueError("validated coordinate profile required")
    return _render_urdf(profile, geometry)


def _render_urdf(profile, geometry, *, lift_axis="0 0 1", lift_travel=.64, ground_z=None):
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
