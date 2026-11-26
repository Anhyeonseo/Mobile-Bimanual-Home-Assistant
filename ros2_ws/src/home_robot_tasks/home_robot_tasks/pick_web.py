"""Dependency-free loopback monitor. No device access from request threads."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
from urllib.parse import urlsplit

from .pick_service import Rejected


def handler_for(station):
    token = secrets.token_urlsafe(32)
    template = Path(__file__).with_name('web')/'pick_monitor.html'

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, status, body, content_type='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def allowed_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.allowed_host():
                return self.respond(403, {'error':'loopback host required'})
            path = urlsplit(self.path).path
            if path == '/':
                return self.respond(200, template.read_text().replace('__TOKEN__', token).encode(), 'text/html; charset=utf-8')
            if path == '/api/cabinet-map' and station.backend.mode == 'station':
                from tools.run.cabinet_navigation import map_preview
                try:
                    return self.respond(200, map_preview())
                except (ValueError, OSError) as exc:
                    return self.respond(409, {'error':str(exc)})
            if path == '/api/status':
                return self.respond(200, station.snapshot())
            if path.startswith('/api/artifacts/'):
                key = path.removeprefix('/api/artifacts/')
                with station.lock:
                    file = station.assets.get(key)
                if file is not None and file.resolve().is_relative_to(station.output) and file.is_file():
                    content_type='application/x-ndjson' if file.suffix.lower()=='.jsonl' else mimetypes.guess_type(file.name)[0] or 'application/octet-stream'
                    return self.respond(200, file.read_bytes(), content_type)
            return self.respond(404, {'error':'not found'})

        def do_POST(self):
            origin = self.headers.get('Origin')
            origins = (f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}')
            if (not self.allowed_host() or (origin is not None and origin not in origins)
                    or not secrets.compare_digest(self.headers.get('X-Pick-Token', '').encode(), token.encode())):
                return self.respond(403, {'error':'invalid local request'})
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.respond(415, {'error':'JSON required'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 1 <= length <= 2048:
                    raise ValueError('bounded JSON request required')
                self.connection.settimeout(2)
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError('object required')
                if self.path == '/api/requests':
                    options = {k:data[k] for k in ('operation','supported_empty_bottom','supported_empty_top','supported_startup_release','scene_unchanged','initial_pose') if k in data}
                    return self.respond(202, {'id':station.submit(data.get('text'), **options)})
                if self.path == '/api/reset':
                    return self.respond(202, {'requested':station.request_reset(supported_empty=data.get('supported_empty',False))})
                if self.path == '/api/cancel':
                    return self.respond(202, {'requested':station.cancel()})
                return self.respond(404, {'error':'not found'})
            except Rejected as exc:
                return self.respond(409, {'error':str(exc)})
            except (ValueError, UnicodeError, TimeoutError) as exc:
                return self.respond(400, {'error':str(exc)})

    return Handler


def create_server(station, port=18768):
    server = ThreadingHTTPServer(('127.0.0.1', port), handler_for(station))
    server.daemon_threads = True
    return server
