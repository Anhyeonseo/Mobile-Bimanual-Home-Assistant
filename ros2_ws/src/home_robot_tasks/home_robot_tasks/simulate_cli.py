"""Run fetch or future-app map navigation against a deterministic fake robot."""

import argparse
import json
from pathlib import Path

from .application import RobotApplication
from .kinematic_robot import KinematicRobot
from .navigation import NavigationMap
from .replay import _seconds


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--cancel-at-s", type=float)
    parser.add_argument("--fail-skill")
    args = parser.parse_args(argv)
    try:
        world, map_data, request = (
            json.loads(p.read_text()) for p in (args.world, args.map, args.request)
        )
        if request.get("task") == "fetch_object":
            request = {
                "schema_version": 1,
                "operation": "fetch_object",
                "request_id": "simulated-fetch-001",
                "request": request,
            }
        if args.cancel_at_s is not None:
            _seconds(args.cancel_at_s, "cancel_at_s")
        robot = KinematicRobot()
        robot.fail_skill = args.fail_skill
        app = RobotApplication(world, NavigationMap(map_data), robot)
        result = app.submit(request, 0)
        rid = result["request_id"]
        cancelled = False
        for i in range(10001):
            now = i / 10
            if (
                args.cancel_at_s is not None
                and now >= args.cancel_at_s
                and not cancelled
            ):
                app.cancel(rid, now)
                cancelled = True
            app.tick(now)
            result = app.status(rid)
            if result["status"] in {
                "SUCCEEDED",
                "FAILED",
                "CANCELLED",
                "STOP_UNCONFIRMED",
            }:
                break
        print(
            json.dumps(
                {
                    **result,
                    "evidence_source": "synthetic_adapter",
                    "adapter_log": robot.log,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0 if result["status"] == "SUCCEEDED" else 1
    except (OSError, ValueError) as error:
        parser.exit(2, f"Invalid simulation: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
