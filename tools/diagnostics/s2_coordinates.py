#!/usr/bin/env python3
"""Offline S2 survey and fixed-camera fit; no robot or camera is connected."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/home_robot_tasks"))


GEOMETRY = ROOT / "config/home_assistant/official_geometry.json"


def audit():
    from home_robot_tasks.s2_commissioning import load_geometry, digest, POLICY
    geometry = load_geometry(GEOMETRY)
    installation = json.loads((ROOT / "config/home_assistant/installation.development.json").read_text())
    lift = json.loads((ROOT / installation["lift"]["geometry"]).read_text())
    camera_observation = installation["d415"].get("device_observation", {})
    camera_recorded = camera_observation.get("status") == "recorded_metadata_validated"
    nominal_retained = installation["d415"].get("development_transform_choice", {}).get("status") == "nominal_model_retained_by_operator"
    from home_robot_tasks import s2_development as dev
    profile = dev.build(ROOT)
    integration_path=ROOT/'config/home_assistant/s2_integration.observed.json'
    integration=json.loads(integration_path.read_text()) if integration_path.exists() else {}
    integrated=(integration.get('status')=='development_integration_verified'
                and integration.get('profile_id')==profile['calibration_id']
                and integration.get('development_s2_complete') is True
                and integration.get('motor_write_commands')==0
                and integration.get('synchronized_coordinate_count',0)>0)
    raw_verified=False
    if integrated:
        raw=ROOT/integration['source_report']
        if raw.exists():
            import hashlib
            raw_verified=hashlib.sha256(raw.read_bytes()).hexdigest()==integration['source_report_sha256']
            if not raw_verified:raise ValueError('integrated hardware evidence changed')
    return {
        "stage": "S2", "status": "development_integration_verified" if integrated else "software_ready_pending_integrated_hardware_check",
        "s2_complete": integrated, "development_s2_complete": integrated, "software_connection_complete": True,
        "acceptance_scope": "operator-agreed nominal development coordinates; strict precision commissioning remains separate",
        "historical_integration_evidence": integration,
        "raw_integration_report_hash_verified_locally": raw_verified,
        "strict_commissioning_passed": False, "hardware_commands": 0, "motion_authorized": False,
        "robot_model": geometry["robot_model"], "arm_model": geometry["arm_model"],
        "geometry_sha256": digest(geometry), "profile_id": profile["calibration_id"],
        "official_sources": geometry["sources"],
        "camera_moves_with_lift": installation["d415"]["moves_with_lift"],
        "lift_historical_mm_per_turn": lift["measured_um_per_turn"] / 1000,
        "lift_nominal_mm_per_turn": geometry["lift_nominal_m_per_turn"] * 1000,
        "next_step": "S3_task_accuracy_and_grasp_development" if integrated else "power_off_then_switch_to_read_only_s2_observer",
        "remaining_s2": [] if integrated else ["stationary integrated RGB-D + 13-axis acquisition on the robot",
                         "fresh lift bottom / arm turn anchors for a live coordinate session"],
        "additional_precision_measurements_required_now": False,
        "deferred_to_s3": ["camera extrinsic task accuracy", "multi-pose TCP accuracy and historical direction verification"],
        "current_sequence": [
            {"id": "floor", "state": "operator_confirmed_approximate", "value_m": installation['base_reference']['floor_measurement']['clearance_m']},
            {"id": "camera_device", "state": "recorded_metadata_validated" if camera_recorded else "pending_capture", "live_connection_checked_by_audit": False},
            {"id": "camera_extrinsic", "state": "nominal_retained_precision_validation_deferred" if nominal_retained else "pending_independent_reference"},
            {"id": "arm_home", "state": "both_arms_reference_captured_software_offsets_derived"},
            {"id": "arm_mounts", "state": "official_shared_mesh_adapter_derived"},
            {"id": "lift_model", "state": "CAD_contacts_and_approximate_up_down_observations_retained"},
            {"id": "coordinate_software", "state": "FK_URDF_viewer_and_capture_time_contracts_connected"},
            {"id": "live_session", "state": "stationary_integrated_capture_verified_session_closed" if integrated else "new_read_only_firmware_prepared_not_flashed"},
        ],
        "strict_acceptance_policy_reference": POLICY,
        "confirmed_installation": installation,
    }


def read_profile(path):
    value = json.loads(path.read_text())
    return value.get("profile", value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    survey = sub.add_parser("audit", help="official geometry and remaining physical evidence")
    survey.add_argument("--output", type=Path)
    evidence = sub.add_parser("camera-evidence", help="validate saved D415 SDK calibration; no device access")
    evidence.add_argument("--snapshot", type=Path, required=True)
    evidence.add_argument("--output", type=Path, required=True)
    fit = sub.add_parser("fit-camera", help="standalone camera fit, no device access")
    fit.add_argument("--points", type=Path, required=True)
    fit.add_argument("--output", type=Path, required=True)
    prepare = sub.add_parser("prepare", help="create an unfilled physical evidence record")
    prepare.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("commission", help="recompute calibration and independent acceptance")
    check.add_argument("--record", type=Path, required=True)
    check.add_argument("--output", type=Path, required=True)
    for name in ("observe", "export-urdf", "rebind-origin"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--profile", type=Path, required=True)
        cmd.add_argument("--output", type=Path, required=True)
        if name != "rebind-origin":
            cmd.add_argument("--allow-simulation", action="store_true")
        if name == "observe":
            cmd.add_argument("--snapshot", type=Path, required=True)
            cmd.add_argument("--now-s", type=float, required=True, help="same clock as snapshot")
        elif name == "rebind-origin":
            cmd.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output and args.output.exists():
            raise ValueError("output already exists; use a new filename")
        from home_robot_tasks import s2_commissioning as s2
        geometry = s2.load_geometry(GEOMETRY)
        if args.command == "audit":
            result = audit()
        elif args.command == "camera-evidence":
            import hashlib
            from home_robot_tasks.camera_evidence import from_metadata
            metadata_path=args.snapshot/'metadata.json'
            result=from_metadata(json.loads(metadata_path.read_text()))
            result['source_files_sha256']={name:hashlib.sha256((args.snapshot/name).read_bytes()).hexdigest()
                for name in ('metadata.json','color.png','depth_aligned.png','depth_native.png')}
        elif args.command == "fit-camera":
            from home_robot_tasks.camera_registration import fit_fixed_camera
            result = fit_fixed_camera(json.loads(args.points.read_text()))
        elif args.command == "prepare":
            result = s2.template(geometry)
        elif args.command == "commission":
            result = s2.commission(json.loads(args.record.read_text()), geometry)
        elif args.command == "observe":
            result = s2.observe(read_profile(args.profile), geometry, json.loads(args.snapshot.read_text()),
                                now_s=args.now_s, allow_simulation=args.allow_simulation)
        elif args.command == "export-urdf":
            result = s2.export_urdf(read_profile(args.profile), geometry, allow_simulation=args.allow_simulation)
        else:
            result = s2.rebind_origin(read_profile(args.profile), json.loads(args.evidence.read_text()))
        encoded = result if isinstance(result, str) else json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x") as stream:
                stream.write(encoded)
        print(encoded, end="")
        if isinstance(result, dict):
            if result.get("within_threshold") is False or result.get("status") == "rejected":
                return 2
            if args.command == "commission" and result.get("status") == "needs_physical_checks":
                return 2
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"S2 확인 실패: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
