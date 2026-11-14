"""Live D415 preview in the browser + named shots typed in the terminal. Read-only.

Run on the Jetson from inside the installed release folder:  python live_cam.py
Laptop:   ssh -N -L 18765:127.0.0.1:8765 hyper@<jetson>   then open http://127.0.0.1:18765
Terminal: NAME          -> camera-only shot   (floor, table, checkerboard)
          NAME LIFT_M   -> shot + raw joint encoders read before/after (arm marker)
          q             -> quit
Never enables torque or moves anything. Only one program can use the camera.
"""
import importlib.util, io, json, re, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import numpy as np
import serial
from PIL import Image
from so101_arm_bridge.limit_manifest import JOINTS
try:
    import cv2
    DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
except Exception:
    cv2 = None

CAL = Path.home()/'cal_20261113'
MAX_MOVE_RAW = 3
latest = dict(frame=None, t=0.0, jpeg=b'', info='camera starting...')
lock = threading.Lock(); stop = threading.Event(); errors = []


def detect(gray):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(DICTIONARY, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, DICTIONARY)
    return corners, ([] if ids is None else ids.flatten().tolist())


def camera_loop():
    try:
        with sp.Capture(sp.camera_profile(), sp.read('camera_device.near.observed.json'), retain_frame=True) as camera:
            last_jpeg = 0.0
            while not stop.is_set():
                record, images = camera.next_frame()
                now = time.monotonic()
                with lock:
                    latest.update(frame=(record, images), t=now)
                if now - last_jpeg < .12:
                    continue
                last_jpeg = now
                view, info = images[0], 'marker check unavailable (no cv2)'
                if cv2 is not None:
                    corners, ids = detect(cv2.cvtColor(view, cv2.COLOR_BGR2GRAY))
                    if ids:
                        view = view.copy()
                        cv2.aruco.drawDetectedMarkers(view, corners, np.asarray(ids).reshape(-1, 1))
                        side = min(float(np.linalg.norm(q.reshape(4, 2) - np.roll(q.reshape(4, 2), -1, 0), axis=1).min()) for q in corners)
                        info = f"marker {ids}  smallest side {side:.0f}px  {'OK' if side >= 40 else 'TOO SMALL'}"
                    else:
                        info = 'no marker'
                buffer = io.BytesIO()
                Image.fromarray(view[:, :, ::-1]).save(buffer, 'JPEG', quality=70)
                with lock:
                    latest.update(jpeg=buffer.getvalue(), info=info)
    except Exception as exc:
        errors.append(exc); stop.set()


PAGE = b"""<!doctype html><meta charset=utf-8><title>D415 live</title>
<body style="margin:0;background:#111;color:#eee;font:16px sans-serif">
<img src="/stream" style="width:100%;max-width:960px;display:block">
<div id=i style="padding:8px"></div>
<script>setInterval(()=>fetch('/info').then(r=>r.text()).then(t=>i.textContent=t),400)</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == '/info':
            with lock:
                body = latest['info'].encode()
            self.send_response(200); self.send_header('Content-Type', 'text/plain; charset=utf-8'); self.end_headers(); self.wfile.write(body)
        elif self.path == '/stream':
            self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f'); self.end_headers()
            try:
                while not stop.is_set():
                    with lock:
                        jpeg = latest['jpeg']
                    if jpeg:
                        self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg) + jpeg + b'\r\n')
                    time.sleep(.12)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(200); self.send_header('Content-Type', 'text/html'); self.end_headers(); self.wfile.write(PAGE)


def wait_frame(after):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        with lock:
            if latest['frame'] is not None and latest['t'] > after:
                return latest['frame']
        time.sleep(.01)
    raise RuntimeError('no fresh camera frame')


def save_frame(folder, record, images):
    folder.mkdir(parents=True)
    Image.fromarray(images[0][:, :, ::-1]).save(folder/'color.png')
    Image.fromarray(images[1]).save(folder/'depth_aligned.png')
    (folder/'metadata.json').write_text(json.dumps(images[2], indent=2))
    (folder/'record.json').write_text(json.dumps(record, default=str))


def read_all(transport):
    rows = {}
    for bus, side in enumerate(('left', 'right')):
        for i, name in enumerate(JOINTS):
            row = transport.read_device_inventory(bus, i + 1)
            if row['command']['torque_enabled_raw'] != 0:
                raise RuntimeError(f'torque is on at {side}/{name}; only torque-off arms are read')
            rows[f'{side}/{name}'] = row
    rows['lift/id10'] = transport.read_device_inventory(0, 10)
    return rows


def shot(name, lift_m):
    raw = lambda rows: {k: r['feedback']['position_raw'] for k, r in rows.items()}
    if lift_m is None:
        out = CAL/name/'cam'
        if out.exists():
            raise RuntimeError(f'{name} already exists; use another name')
        record, images = wait_frame(time.monotonic())
        save_frame(out, record, images)
        return f'saved {out}'
    out = CAL/name
    if out.exists():
        raise RuntimeError(f'{name} already exists; use another name')
    with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
        transport = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
        hello = sp.identity(transport)
        start = time.time(); before = read_all(transport); mark = time.monotonic()
        record, images = wait_frame(mark)
        after = read_all(transport); end = time.time()
    b, a = raw(before), raw(after)
    moved = {k: (b[k], a[k]) for k in b if not k.startswith('lift') and min(abs(b[k] - a[k]), 4096 - abs(b[k] - a[k])) > MAX_MOVE_RAW}
    if moved:
        raise RuntimeError(f'arm moved during shot {moved}; hold still and retry')
    save_frame(out, record, images)
    (out/'joints.json').write_text(json.dumps(dict(
        kind='raw_calibration_shot', lift_m_operator=lift_m, raw_before=b, raw_after=a,
        read_window_utc_s=[start, end], camera_record=record, controller=hello,
        inventory_before=before, inventory_after=after, motor_write_commands=0), indent=2, default=str))
    return f"saved {out}  left raw {[a['left/' + j] for j in JOINTS]}  lift raw {a['lift/id10']}"


def main():
    camera = threading.Thread(target=camera_loop, daemon=True); camera.start()
    server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('live view: on the laptop run  ssh -N -L 18765:127.0.0.1:8765 hyper@<jetson>  and open http://127.0.0.1:18765')
    print('type: NAME (camera only) | NAME LIFT_M (with arm joints) | q (quit)')
    try:
        while not stop.is_set():
            try:
                words = input('shot> ').split()
            except EOFError:
                break
            if not words:
                continue
            if words[0] == 'q':
                break
            if not re.fullmatch(r'[A-Za-z0-9_-]+', words[0]) or len(words) > 2:
                print('usage: NAME  or  NAME LIFT_M   e.g. CB-1   or   L1-01 0.20'); continue
            try:
                print(shot(words[0], float(words[1]) if len(words) == 2 else None))
            except Exception as exc:
                print('REJECTED:', exc)
    finally:
        stop.set(); server.shutdown(); camera.join(timeout=3)
        if errors:
            print('camera error:', errors[0])


if __name__ == '__main__':
    main()
