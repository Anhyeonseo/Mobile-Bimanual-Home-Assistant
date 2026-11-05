"""Small authenticated CLI for the existing Gateway HTTP contract."""
import argparse
import json
from pathlib import Path
import urllib.error
import urllib.request
from urllib.parse import urlsplit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--token-file", type=Path, required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("submit").add_argument("--request-file", type=Path, required=True)
    for name in ("status", "cancel"):
        commands.add_parser(name).add_argument("request_id")
    args = parser.parse_args(argv)
    url = urlsplit(args.url)
    if url.scheme != "http" or url.hostname not in {"127.0.0.1", "localhost", "::1"} or url.username or url.password or url.path or url.query or url.fragment:
        parser.error("this prototype client requires a loopback HTTP origin")
    token = args.token_file.read_text().strip()

    def call(method, path, body=None, permit=None):
        headers = {"Authorization": "Bearer " + token, "Content-Type": "application/json"}
        if permit:
            headers["X-Request-Permit"] = permit
        request = urllib.request.Request(args.url + path, method=method, headers=headers,
                                         data=None if body is None else json.dumps(body).encode())
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.load(response)

    try:
        if args.command == "submit":
            body = json.loads(args.request_file.read_text())
            permit = call("GET", "/v1/permit")["permit"]
            result = call("POST", "/v1/requests", body, permit)
        else:
            from urllib.parse import quote
            path = "/v1/requests/" + quote(args.request_id, safe="")
            result = call("GET", path) if args.command == "status" else call("POST", path + "/cancel", {})
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except urllib.error.HTTPError as error:
        print(f"HTTP {error.code}: {error.read().decode()}")
        raise SystemExit(1)
    except urllib.error.URLError as error:
        print(f"Gateway connection failed: {error.reason}")
        raise SystemExit(1)
