"""Validate the calibrated joint envelope shared by host and firmware.

Schema 1 fixes zero=2048 and signs below; a new calibration needs a schema
migration, not an unreviewed change to these constants. This is a historical
measured envelope, not a certificate of mobile mechanical/collision clearance.
"""
from __future__ import annotations

import json
from pathlib import Path

JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper")
DIRECTIONS = (1, 1, -1, -1, 1, -1)
JOINT_NAMES = tuple(f"{arm}_{joint}_joint" for arm in ("left", "right") for joint in JOINTS)


def _object(pairs: list[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate manifest key: {key}")
        result[key] = value
    return result


def _integer(value: object) -> int:
    if type(value) is not int or not -(1 << 31) <= value < (1 << 31):
        raise ValueError("joint limits must be int32 integers (no coercion)")
    return value


def read_limit_manifest(path: Path) -> tuple[dict, tuple[dict, ...]]:
    document = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_object)
    return document, validate_limit_manifest(document)


def validate_limit_manifest(document: object) -> tuple[dict, ...]:
    if not isinstance(document, dict):
        raise ValueError("operational-limit document must be an object")
    if (type(document.get("schema_version")) is not int or document["schema_version"] != 1
        or document.get("record_kind") != "bimanual_operational_limits"
        or document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True):
        raise ValueError("operational-limit document is not operator-authorized schema 1")
    if document.get("joint_order") != list(JOINT_NAMES):
        raise ValueError("operational-limit joint order does not match the 12-axis contract")
    if type(document.get("raw_units_per_turn")) is not int or document["raw_units_per_turn"] != 4096:
        raise ValueError("unsupported raw units per turn")
    entries = []
    try:
        arms = document["arms"]
        if not isinstance(arms, dict) or set(arms) != {"left", "right"}:
            raise ValueError("expected exactly two arms")
        for arm in ("left", "right"):
            if not isinstance(arms[arm], dict) or set(arms[arm]) != set(JOINTS):
                raise ValueError("expected exactly six joints per arm")
            for joint, direction in zip(JOINTS, DIRECTIONS, strict=True):
                entry = arms[arm][joint]
                coordinate = "unwrapped_raw" if joint == "shoulder" else "semantic_raw" if joint == "gripper" else "raw"
                if entry["coordinate"] != coordinate:
                    raise ValueError(f"unsupported coordinate for {arm}.{joint}")
                lo, hi, u_lo, u_hi = (_integer(entry[key]) for key in (
                    "minimum_unwrapped_raw", "maximum_unwrapped_raw", "minimum_urad", "maximum_urad"))
                # Runtime branch search supports turns -1, 0, 1 and requires
                # a unique modulo->unwrapped match; a full turn is ambiguous.
                if not -4096 <= lo < hi <= 8191 or hi - lo >= 4096 or u_lo >= u_hi:
                    raise ValueError(f"invalid or ambiguous range for {arm}.{joint}")
                if coordinate != "unwrapped_raw" and not 0 <= lo < hi < 4096:
                    raise ValueError(f"modulo/semantic range outside one turn for {arm}.{joint}")
                expected = sorted(direction * (raw - 2048) * 6283185 / 4096 for raw in (lo, hi))
                if any(abs(actual - target) > 1 for actual, target in zip((u_lo, u_hi), expected, strict=True)):
                    raise ValueError(f"raw/urad calibration mismatch for {arm}.{joint}")
                entries.append(entry)
    except (KeyError, TypeError) as error:
        raise ValueError("operational-limit document is incomplete") from error
    return tuple(entries)
