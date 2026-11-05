import json
from .core import TaskManager

EXAMPLE = {"schema_version": 1, "task_id": "pick-001", "operation": "pick",
           "target": {"object": "remote", "color": "black", "location": "desk"}}


def main():
    manager = TaskManager()
    print(manager.status()["state"])
    previous = manager.submit(json.dumps(EXAMPLE), 0)["state"]
    print(previous)
    for i in range(1, 101):
        state = manager.tick(i / 10)["state"]
        if state != previous:
            print(state)
            previous = state
        if state == "IDLE":
            break
    result = manager.status("pick-001")
    assert result["state"] == "COMPLETED", result
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
