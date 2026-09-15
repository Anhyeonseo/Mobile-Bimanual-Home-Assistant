import json
from pathlib import Path
import threading
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
import pytest
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.navigation import NavigationMap
from home_robot_tasks.gateway import Gateway, GatewayStore, handler_for

ROOT = Path(__file__).resolve().parents[1]
TOKEN = "t" * 48
AUTH = "Bearer " + TOKEN


def read(name):
    return json.loads((ROOT / "config" / name).read_text())


def gateway(path=":memory:"):
    app = RobotApplication(
        read("home.example.json"),
        NavigationMap(read("navigation_map.simulation.json")),
        FakeRobot(),
    )
    g = Gateway(app, TOKEN, GatewayStore(path))
    g.now = lambda: 0.0
    return g


def permit(g):
    return g.dispatch("GET", "/v1/permit", AUTH, None)[1]["permit"]


def submit(g, request=None):
    return g.dispatch(
        "POST",
        "/v1/requests",
        AUTH,
        request or read("navigate_to.simulation.json"),
        permit(g),
    )


def test_auth_permit_idempotency_and_conflict():
    g = gateway()
    req = read("navigate_to.simulation.json")
    assert g.dispatch("GET", "/v1/map", "wrong", None)[0] == 401
    assert g.dispatch("POST", "/v1/requests", AUTH, req)[0] == 409
    code, result = submit(g)
    assert code == 202
    assert g.dispatch("POST", "/v1/requests", AUTH, req)[0] == 200
    req["point"]["x"] = 7.5
    assert g.dispatch("POST", "/v1/requests", AUTH, req)[0] == 409
    assert len(g.app.tasks) == 1


def test_delayed_new_command_rejected_without_trusting_client_clock():
    g = gateway()
    p = permit(g)
    g.now = lambda: 3.0
    assert (
        g.dispatch(
            "POST", "/v1/requests", AUTH, read("navigate_to.simulation.json"), p
        )[0]
        == 409
    )
    assert not g.app.tasks


def test_restart_does_not_resume_or_duplicate_unfinished_task(tmp_path):
    path = tmp_path / "requests.db"
    g = gateway(path)
    submit(g)
    g.store.db.close()
    restarted = gateway(path)
    code, result = restarted.dispatch(
        "POST", "/v1/requests", AUTH, read("navigate_to.simulation.json")
    )
    assert code == 200 and result["status"] == "INTERRUPTED"
    assert not restarted.app.tasks and restarted.app.lease.owner is None


def test_cancel_status_persists_and_clock_runs():
    g = gateway()
    _, result = submit(g)
    rid = result["request_id"]
    g.tick()
    g.now = lambda: 0.05
    code, result = g.dispatch("POST", f"/v1/requests/{rid}/cancel", AUTH, {})
    assert code == 202 and result["status"] == "STOPPING"
    g.now = lambda: 0.3
    g.tick()
    assert g.store.get(rid)[1]["status"] == "CANCELLED"


def test_gateway_fault_blocks_admission_but_keeps_stop_polling():
    g = gateway()
    _, result = submit(g)
    g.tick()
    g.latch_fault()
    assert g.dispatch("GET", "/v1/health", AUTH, None)[0] == 503
    assert g.dispatch("GET", "/v1/permit", AUTH, None)[0] == 503
    assert (
        g.dispatch("POST", "/v1/requests", AUTH, read("navigate_to.simulation.json"))[0]
        == 503
    )
    g.now = lambda: 0.3
    g.tick()
    assert g.app.status(result["request_id"])["status"] == "CANCELLED"


def test_actual_http_auth_json_limits_and_submit(tmp_path):
    g = gateway(tmp_path / "http.db")
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(g))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"

    def call(path, body=None, auth=AUTH):
        req = urllib.request.Request(
            base + path,
            data=body,
            headers={"Authorization": auth, "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=2) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    try:
        assert call("/v1/health", auth="bad")[0] == 401
        assert call("/v1/map")[1]["navigation_dimensions"] == 2
        assert call("/v1/requests", b"{broken")[0] == 400
        assert call("/v1/requests", b"a" * 16385)[0] == 413
        assert call("/v1/requests", b'{"a":1,"a":2}')[0] == 400
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        g.store.db.close()


def test_navigation_preview_has_no_motion_or_request_side_effects():
    g = gateway()
    code, plan = g.dispatch(
        "POST", "/v1/navigation/preview", AUTH, read("navigate_to.simulation.json")
    )
    assert code == 200 and len(plan["path_xy"]) > 1
    assert g.app.lease.owner is None and not g.app.tasks and not g.permits
    request = read("navigate_to.simulation.json")
    request["map_revision"] = "stale"
    with pytest.raises(ValueError):
        g.dispatch("POST", "/v1/navigation/preview", AUTH, request)


def test_browser_assets_and_exact_origin(tmp_path):
    g = gateway(tmp_path / "browser.db")
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(g))
    base = f"http://127.0.0.1:{server.server_port}"
    g.public_origin = base
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def call(path, origin=None, body=None):
        headers = {"Authorization": AUTH, "Content-Type": "application/json"}
        if origin:
            headers["Origin"] = origin
        request = urllib.request.Request(base + path, headers=headers, data=body)
        try:
            return urllib.request.urlopen(request, timeout=2)
        except urllib.error.HTTPError as e:
            return e

    try:
        for path, mime in [
            ("/", "text/html"),
            ("/app.js", "text/javascript"),
            ("/map.js", "text/javascript"),
            ("/app.css", "text/css"),
        ]:
            with call(path) as response:
                assert response.status == 200 and response.headers[
                    "Content-Type"
                ].startswith(mime)
                assert (
                    "frame-ancestors 'none'"
                    in response.headers["Content-Security-Policy"]
                )
        with call(
            "/v1/navigation/preview",
            base,
            json.dumps(read("navigate_to.simulation.json")).encode(),
        ) as response:
            assert response.status == 200
        with call(
            "/v1/navigation/preview", "https://untrusted.invalid", b"{}"
        ) as response:
            assert response.status == 400
        with call("/../gateway.py") as response:
            assert response.status == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        g.store.db.close()
