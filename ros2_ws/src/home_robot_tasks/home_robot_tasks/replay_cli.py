"""Replay local task evidence without a robot, ROS, network or inference model."""
import argparse
import json
from pathlib import Path

from .fetch import FetchRequest
from .replay import replay_fetch


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--request", required=True, type=Path)
    parser.add_argument("--recording", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        request = FetchRequest.from_dict(json.loads(args.request.read_text(encoding="utf-8")))
        result = replay_fetch(request,
                              json.loads(args.world.read_text(encoding="utf-8")),
                              json.loads(args.recording.read_text(encoding="utf-8")))
    except (OSError, ValueError) as error:
        parser.exit(2, f"Invalid replay: {error}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["replay_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
