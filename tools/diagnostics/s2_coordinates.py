#!/usr/bin/env python3
"""Offline S2 survey and fixed-camera fit; no robot or camera is connected."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/home_robot_tasks"))


def audit():
    installation = json.loads((ROOT / "config/home_assistant/installation.development.json").read_text())
    lift = json.loads((ROOT / installation["lift"]["geometry"]).read_text())
    return {
        "stage": "S2", "hardware_commands": 0,
        "camera_moves_with_lift": installation["d415"]["moves_with_lift"],
        "lift_measured_travel_m": lift["maximum_height_um"] / 1e6,
        "lift_measured_mm_per_turn": lift["measured_um_per_turn"] / 1000,
        "confirmed_model": installation,
        "measurement_groups": [
            {"id": "base_reference", "needed": "베이스 기준점·정면 방향 정의"},
            {"id": "mounts", "needed": "기준점에서 최저 상판과 양팔 장착 원점까지의 위치·방향"},
            {"id": "home_tcp", "needed": "현재 양팔 Home과 기존 모델 영점 비교, 집게 끝 기준"},
            {"id": "camera", "needed": "고정 카메라와 베이스에서 같은 점을 잰 보정·독립 확인 자료"},
        ],
        "reusable": ["SO101 link meshes and nominal link geometry",
                     "Existing observation frame/time/revision validation",
                     "Measured lift descent scale"],
        "not_measured": ["Mobile simulation dimensions and mounts",
                         "Legacy arm Home offsets on the current assembly",
                         "Lift origin after restart and camera extrinsics"],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    survey = sub.add_parser("audit", help="show existing measurements and remaining survey groups")
    survey.add_argument("--output", type=Path)
    fit = sub.add_parser("fit-camera", help="fit explicit metre correspondences without device access")
    fit.add_argument("--points", type=Path, required=True)
    fit.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output and args.output.exists():
            raise ValueError("output already exists; use a new filename")
        if args.command == "audit":
            result = audit()
        else:
            from home_robot_tasks.camera_registration import fit_fixed_camera
            result = fit_fixed_camera(json.loads(args.points.read_text()))
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x") as stream:
                stream.write(encoded)
        print(encoded, end="")
        return 2 if result.get("within_threshold") is False else 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"S2 확인 실패: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
