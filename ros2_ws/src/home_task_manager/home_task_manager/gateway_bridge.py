"""Adapt Pick tasks to the existing Gateway API, auth, permits and SQLite store."""
from copy import deepcopy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading

from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.gateway import Gateway, GatewayStore, handler_for


class PickApplication:
    """Gateway application protocol backed by exactly one manager and lease."""
    def __init__(self, manager):
        self.manager = manager
        self.lease = manager.lease
        self.tasks = manager.tasks
        self.requests = {}

    @property
    def map(self):
        raise InvalidTask("navigation_not_supported_by_pick_gateway")

    @property
    def pose_known(self):
        raise InvalidTask("navigation_not_supported_by_pick_gateway")

    def submit(self, request, now_s):
        if not isinstance(request, dict) or set(request) != {
            "schema_version", "request_id", "operation", "target"
        }:
            raise InvalidTask("expected schema_version, request_id, operation, target")
        raw = {**request, "task_id": request["request_id"]}
        del raw["request_id"]
        result = self.manager.submit(json.dumps(raw, allow_nan=False), now_s)
        self.requests[result["task_id"]] = deepcopy(request)
        return self.status(result["task_id"])

    def status(self, request_id):
        return {**self.manager.status(request_id), "request_id": request_id, "operation": "pick"}

    def tick(self, now_s):
        active = self.manager.active
        self.manager.tick(now_s)
        return self.status(active) if active is not None else None

    def cancel(self, request_id, now_s):
        self.manager.cancel(request_id, now_s)
        return self.status(request_id)


class GatewayBridge:
    def __init__(self, manager, token_file, database, port, origin):
        token = Path(token_file).read_text().strip()
        self.store = GatewayStore(database)
        try:
            for rid, raw in self.store.db.execute("SELECT id,result FROM requests").fetchall():
                result = json.loads(raw)
                if result["status"] == "INTERRUPTED":
                    result.update(state="INTERRUPTED", stop_confirmation_required=True)
                    self.store.db.execute("UPDATE requests SET result=? WHERE id=?", (json.dumps(result), rid))
            self.store.db.commit()
            self.gateway = Gateway(PickApplication(manager), token, self.store)
            # Both HTTP and ROS callbacks must use the same monotonic origin.
            self.gateway.started = origin
            self.server = ThreadingHTTPServer(("127.0.0.1", port), handler_for(self.gateway))
            # Close waits for bounded HTTP handlers before closing SQLite.
            self.server.daemon_threads = False
            self.gateway.public_origin = f"http://127.0.0.1:{self.server.server_port}"
            # A previous process cannot prove a remote action stopped. Existing
            # GatewayStore marks interrupted requests; block admission on restart.
            interrupted = any(json.loads(row[0])["status"] == "INTERRUPTED"
                              for row in self.store.db.execute("SELECT result FROM requests"))
            self.gateway.faulted = interrupted
            self.worker = threading.Thread(
                target=lambda: self.server.serve_forever(poll_interval=0.05), daemon=True)
            self.worker.start()
        except Exception:
            self.store.db.close()
            raise

    def close_http(self):
        with self.gateway.lock:
            self.gateway.latch_fault()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(timeout=2)

    def close_store(self):
        with self.gateway.lock:
            self.store.db.close()
