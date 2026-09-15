"""Authenticated local HTTP gateway for the simulation application.

A lock serializes application state; HTTP parsing never blocks executor ticks. Default loopback;
LAN binding requires TLS. No hardware/serial adapter is enabled. SQLite persists
request identity across restart; unfinished tasks become INTERRUPTED and never
resume automatically. Clients poll status; no background third-party service.
"""

from __future__ import annotations
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sqlite3
import ssl
import time
import threading
import secrets
from urllib.parse import urlsplit

from .application import RobotApplication
from .execution import DEFAULT_TASK_TIMEOUT_S
from .kinematic_robot import KinematicRobot
from .navigation import NavigationMap, NavigateRequest, identifier
from .fetch import InvalidTask


class GatewayStore:
    def __init__(self, path):
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS requests (id TEXT PRIMARY KEY, body TEXT NOT NULL, result TEXT NOT NULL)"
        )
        for rid, result in self.db.execute("SELECT id,result FROM requests").fetchall():
            data = json.loads(result)
            if data["status"] not in {
                "SUCCEEDED",
                "FAILED",
                "CANCELLED",
                "INTERRUPTED",
            }:
                data.update(
                    status="INTERRUPTED",
                    reason="server_restarted",
                    control_owned=False,
                    physical_task_completed=False,
                    hardware_commands=0,
                )
                self.db.execute(
                    "UPDATE requests SET result=? WHERE id=?", (json.dumps(data), rid)
                )
        self.db.commit()

    def get(self, rid):
        row = self.db.execute(
            "SELECT body,result FROM requests WHERE id=?", (rid,)
        ).fetchone()
        return None if row is None else (json.loads(row[0]), json.loads(row[1]))

    def save(self, body, result):
        self.db.execute(
            "INSERT INTO requests VALUES (?,?,?) ON CONFLICT(id) DO UPDATE SET result=excluded.result",
            (body["request_id"], json.dumps(body, sort_keys=True), json.dumps(result)),
        )
        self.db.commit()


class Gateway:
    def __init__(self, application, token, store, *, public_origin=None):
        if (
            not isinstance(token, str)
            or len(token) < 32
            or not token.isascii()
            or any(c.isspace() for c in token)
        ):
            raise ValueError(
                "gateway token must be at least 32 ASCII non-whitespace characters"
            )
        if public_origin is not None:
            parsed = urlsplit(public_origin)
            if (
                parsed.scheme not in {"http", "https"}
                or not parsed.hostname
                or parsed.username
                or parsed.password
                or parsed.path
                or parsed.query
                or parsed.fragment
                or (
                    parsed.scheme == "http"
                    and parsed.hostname not in {"localhost", "127.0.0.1", "::1"}
                )
            ):
                raise ValueError("exact HTTPS origin required outside loopback")
        self.public_origin = public_origin
        self.app, self.token, self.store = application, token, store
        self.started = time.monotonic()
        self.lock = threading.RLock()
        self.permits = {}
        self.faulted = False

    def latch_fault(self):
        with self.lock:
            self.faulted = True
            self.permits.clear()
            if self.app.lease.owner:
                self.app.cancel(self.app.lease.owner, self.now())

    def now(self):
        return time.monotonic() - self.started

    def tick(self):
        with self.lock:
            self._tick()

    def _tick(self):
        result = self.app.tick(self.now())
        if result:
            self.store.save(self.app.requests[result["request_id"]], result)

    def dispatch(self, method, path, authorization, body, permit=None):
        with self.lock:
            return self._dispatch(method, path, authorization, body, permit)

    def _dispatch(self, method, path, authorization, body, permit):
        if not isinstance(authorization, str) or not hmac.compare_digest(
            authorization.encode(), ("Bearer " + self.token).encode()
        ):
            return 401, {"error": "unauthorized"}
        if self.faulted and (
            path == "/v1/permit" or (path == "/v1/requests" and method == "POST")
        ):
            return 503, {"error": "gateway_fault_latched"}
        if path == "/v1/permit" and method == "GET":
            now = self.now()
            self.permits = {k: v for k, v in self.permits.items() if v > now}
            if len(self.permits) >= 128:
                return 429, {"error": "too_many_pending_permits"}
            nonce = secrets.token_urlsafe(24)
            self.permits[nonce] = now + 3
            return 200, {"permit": nonce, "valid_for_ms": 3000}
        if path == "/v1/map" and method == "GET":
            return 200, self.app.map.descriptor()
        if path == "/v1/navigation/preview" and method == "POST":
            if not self.app.pose_known:
                raise InvalidTask("localization_required")
            return 200, self.app.map.plan(NavigateRequest.from_dict(body), self.app.xy)
        if path == "/v1/health" and method == "GET":
            return (503 if self.faulted else 200), {
                "mode": "simulation",
                "hardware_enabled": False,
                "faulted": self.faulted,
                "active_request": self.app.lease.owner,
            }
        if path == "/v1/requests" and method == "POST":
            if not isinstance(body, dict):
                raise InvalidTask("JSON object required")
            rid = identifier(body.get("request_id"), "request_id")
            previous = self.store.get(rid)
            if previous:
                if previous[0] != body:
                    return 409, {"error": "request_id_conflict"}
                return 200, (
                    self.app.status(rid) if rid in self.app.tasks else previous[1]
                )
            if not isinstance(permit, str) or self.permits.get(permit, 0) <= self.now():
                return 409, {"error": "fresh_request_permit_required"}
            # Persist identity BEFORE starting: crash after admission can never
            # re-execute the same request automatically. Validation is repeated
            # by the application. Invalid requests are not stored.
            result = self.app.submit(body, self.now())
            self.permits.pop(permit)
            try:
                self.store.save(body, result)
            except Exception:
                self.app.cancel(rid, self.now())
                raise
            return 202, result
        parts = path.strip("/").split("/")
        if len(parts) in (3, 4) and parts[:2] == ["v1", "requests"]:
            rid = identifier(parts[2], "request_id")
            previous = self.store.get(rid)
            if not previous:
                return 404, {"error": "unknown_request"}
            if len(parts) == 3 and method == "GET":
                return 200, (
                    self.app.status(rid) if rid in self.app.tasks else previous[1]
                )
            if len(parts) == 4 and parts[3] == "cancel" and method == "POST":
                if body != {}:
                    raise InvalidTask("cancel body must be {}")
                if rid not in self.app.tasks:
                    return 200, previous[1]
                result = self.app.cancel(rid, self.now())
                self.store.save(previous[0], result)
                return 202, result
        return 404, {"error": "unknown_endpoint"}


def handler_for(gateway):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *args):
            pass  # Do not log authorization headers or request bodies.

        def setup(self):
            super().setup()
            self.connection.settimeout(0.2)

        def do_GET(self):
            self.handle_api()

        def do_POST(self):
            self.handle_api()

        def handle_api(self):
            try:
                origins = self.headers.get_all("Origin", [])
                if (
                    self.headers.get("Transfer-Encoding")
                    or len(origins) > 1
                    or (origins and origins[0] != gateway.public_origin)
                ):
                    self.respond(400, {"error": "unsupported_transfer_or_origin"})
                    return
                lengths = self.headers.get_all("Content-Length", [])
                if len(lengths) > 1:
                    raise InvalidTask("ambiguous content length")
                length = int(lengths[0]) if lengths else 0
                if length < 0 or length > 16384:
                    self.respond(413, {"error": "body_too_large"})
                    return
                if (
                    self.command == "POST"
                    and self.headers.get("Content-Type", "").split(";")[0]
                    != "application/json"
                ):
                    self.respond(415, {"error": "application_json_required"})
                    return
                if self.command == "GET" and length:
                    raise InvalidTask("GET body not allowed")
                content = self.rfile.read(length) if length else b""
                if len(content) != length:
                    raise InvalidTask("incomplete body")

                def unique(pairs):
                    result = {}
                    for key, value in pairs:
                        if key in result:
                            raise InvalidTask("duplicate JSON field")
                        result[key] = value
                    return result

                body = (
                    json.loads(content, object_pairs_hook=unique) if content else None
                )
                url = urlsplit(self.path)
                if url.query or url.fragment:
                    raise InvalidTask("unexpected URL parameters")
                assets = {
                    "/": ("index.html", "text/html; charset=utf-8"),
                    "/app.css": ("app.css", "text/css"),
                    "/map.js": ("map.js", "text/javascript"),
                    "/app.js": ("app.js", "text/javascript"),
                }
                if self.command == "GET" and url.path in assets:
                    name, mime = assets[url.path]
                    self.respond_bytes(
                        200, (Path(__file__).parent / "web" / name).read_bytes(), mime
                    )
                    return
                code, result = gateway.dispatch(
                    self.command,
                    url.path,
                    self.headers.get("Authorization"),
                    body,
                    self.headers.get("X-Request-Permit"),
                )
                self.respond(code, result)
            except InvalidTask as error:
                self.respond(
                    409 if str(error) in {"robot_busy", "request_id_conflict"} else 400,
                    {"error": str(error)},
                )
            except (ValueError, UnicodeError, socket.timeout):
                self.respond(400, {"error": "invalid_request"})
            except Exception:
                self.respond(500, {"error": "internal_error"})

        def respond(self, code, result):
            data = json.dumps(result, allow_nan=False).encode()
            self.respond_bytes(code, data, "application/json")

        def respond_bytes(self, code, data, mime):
            try:
                self.send_response(code)
                self.send_header("Content-Type", mime)
                self.send_header(
                    "Content-Security-Policy",
                    "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'",
                )
                self.send_header("Referrer-Policy", "no-referrer")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                pass

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "map", "token-file", "database"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--public-origin", help="Exact browser origin; HTTPS required for LAN"
    )
    parser.add_argument("--task-timeout", type=float, default=DEFAULT_TASK_TIMEOUT_S)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    args = parser.parse_args(argv)
    if args.bind not in {"127.0.0.1", "localhost"} and not (args.cert and args.key):
        parser.error("LAN binding requires --cert and --key for TLS")
    if bool(args.cert) != bool(args.key):
        parser.error("both TLS certificate and key required")
    app = RobotApplication(
        json.loads(args.world.read_text()),
        NavigationMap(json.loads(args.map.read_text())),
        KinematicRobot(),
        task_timeout_s=args.task_timeout,
    )
    gateway = Gateway(
        app,
        args.token_file.read_text().strip(),
        GatewayStore(args.database),
        public_origin=args.public_origin
        or (
            f"{'https' if args.cert else 'http'}://{args.bind}:{args.port}"
            if args.bind in {"localhost", "127.0.0.1"}
            else None
        ),
    )
    server = ThreadingHTTPServer((args.bind, args.port), handler_for(gateway))
    server.daemon_threads = True
    if args.cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(str(args.cert), str(args.key))
        server.socket = context.wrap_socket(server.socket, server_side=True)
    stopping = threading.Event()

    def ticks():
        while not stopping.wait(0.02):
            try:
                gateway.tick()
            except Exception:
                gateway.latch_fault()
                # Keep polling the stop state even if persistence has failed.
                # Admission stays disabled until an explicit process restart.

    worker = threading.Thread(target=ticks, daemon=True)
    worker.start()
    try:
        server.serve_forever(poll_interval=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        worker.join(timeout=1)
        server.server_close()
        with gateway.lock:
            if app.lease.owner:
                app.cancel(app.lease.owner, gateway.now())
            gateway.store.db.close()


if __name__ == "__main__":
    main()
