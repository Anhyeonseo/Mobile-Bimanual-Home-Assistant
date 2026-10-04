"""Local browser key-down/key-up control for the STM32 manual base session."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import threading
import time
from teleop_mobile_base import inspect, send, receive, frame, validate, stop, PERIOD

BROWSER_LEASE = .25


class Control:
    def __init__(self):
        self.lock = threading.Lock()
        self.owner = None
        self.sequence = 0
        self.last = float("-inf")
        self.motion = "Z"
        self.status = "준비 중"
        self.active = False
        self.expired = False
        self.retired = set()

    def claim(self, owner, now):
        if not isinstance(owner, str) or not 16 <= len(owner) <= 64: return False
        with self.lock:
            # Reclaim must use a new browser identity. Never reset sequence for
            # an old identity: delayed commands/claim retries must stay invalid.
            if owner == self.owner or owner in self.retired: return False
            if self.owner is not None and now - self.last < 1: return False
            if len(self.retired) >= 256: return False  # bounded session history
            if self.owner is not None: self.retired.add(self.owner)
            self.owner, self.sequence, self.motion, self.last = owner, 0, "Z", now
            self.expired = False
            return True

    def update(self, owner, sequence, motion, now):
        if type(sequence) is not int or not 0 < sequence <= 0xffffffff or motion not in tuple("FBLRADZ"):
            return False
        with self.lock:
            if owner != self.owner or sequence <= self.sequence or not self.active: return False
            self._expire(now)
            if self.expired: return False
            self.motion, self.sequence, self.last = motion, sequence, now
            return True

    def _expire(self, now):
        # Called with lock held, including in update before accepting late input.
        if self.owner is not None and now - self.last >= BROWSER_LEASE:
            self.expired, self.motion = True, "Z"

    def desired(self, now):
        with self.lock:
            self._expire(now)
            return self.motion if self.active and not self.expired else "Z"

    def snapshot(self):
        with self.lock: return dict(active=self.active, status=self.status)

    def set_status(self, active, status):
        with self.lock:
            self.active, self.status = active, status
            if not active: self.motion = "Z"


PAGE = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AlohaMini 베이스 조종</title><style>
body{font:18px system-ui;background:#101923;color:#eef4f8;margin:0;padding:30px;max-width:700px;margin:auto}
h1{font-size:28px}#status{padding:16px;background:#233447;border-radius:12px;margin:20px 0}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}button{font:inherit;border:0;border-radius:12px;background:#d9e9f3;color:#132535;min-height:90px;touch-action:none;user-select:none}
button.on{background:#58d9bd}#stop{background:#efae9c}small{display:block;color:#aebdca;line-height:1.6;margin-top:24px}
</style><h1>AlohaMini 베이스 조종</h1><p>키나 버튼을 누르고 있으면 이동하고, 놓으면 멈춥니다.</p>
<div id="status">연결 준비 중…</div><div class="grid">
<button data-key="q">Q ↶<br>왼쪽 회전</button><button data-key="w">W ↑<br>전진</button><button data-key="e">E ↷<br>오른쪽 회전</button>
<button data-key="a">A ←<br>왼쪽 이동</button><button id="stop">Space<br>정지</button><button data-key="d">D →<br>오른쪽 이동</button>
<div></div><button data-key="s">S ↓<br>후진</button></div>
<small>한 방향씩 조작합니다. 창을 벗어나면 정지 요청을 보냅니다. 통신이 끊기면 정지하고, 재연결하려면 페이지를 새로고침하세요.<br>종료·토크 해제: Pi 터미널에서 Ctrl+C. 정지하지 않으면 모터 12V를 차단하세요.</small>
<script>
const token=location.hash.slice(1), newClient=()=>crypto.randomUUID?crypto.randomUUID():Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
const mapping={w:'F',s:'B',a:'L',d:'R',q:'A',e:'D'},pressed=new Map();
let client,seq=0,claimed=false,active=false,inFlight=false,pending=false;
const status=document.querySelector('#status'), buttons=[...document.querySelectorAll('[data-key]')];
function motion(){const keys=[...pressed.values()];return mapping[keys[keys.length-1]]||'Z';}
function paint(){for(const b of buttons)b.classList.toggle('on',[...pressed.values()].includes(b.dataset.key));}
async function post(path,data){
 const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),200);
 try{
  const r=await fetch(path,{method:'POST',headers:{'Authorization':'Bearer '+token,'Content-Type':'application/json'},body:JSON.stringify(data),cache:'no-store',signal:abort.signal});
  if(!r.ok)throw Error('HTTP '+r.status);
  return await r.json();
 }finally{clearTimeout(timer);}
}
async function transmit(){
 if(!claimed)return;
 if(inFlight){pending=true;return;}
 inFlight=true;pending=false;
 const m=motion(),n=++seq;
 try{
  const r=await post('/control',{client,sequence:n,motion:m});
  if(!claimed)return;
  active=r.active;status.textContent=r.status;
  if(!active){pressed.clear();paint();}
 }catch(e){
  claimed=false;active=false;pressed.clear();paint();
  status.textContent='연결 만료 · 키를 놓고 페이지를 새로고침하세요';
 }finally{
  inFlight=false;
  if(pending&&claimed){pending=false;transmit();}
 }
}
function release(){pressed.clear();paint();transmit();}
function keyName(e){return /^Key[WASDQE]$/.test(e.code||'')?e.code.slice(3).toLowerCase():(e.key||'').toLowerCase();}
window.addEventListener('keydown',e=>{const k=keyName(e);if(e.repeat)return;if(k===' '||k==='escape'){e.preventDefault();release();return;}if(mapping[k]){e.preventDefault();if(active){pressed.set('key:'+k,k);paint();transmit();}}});
window.addEventListener('keyup',e=>{const k=keyName(e);if(mapping[k]){e.preventDefault();pressed.delete('key:'+k);paint();transmit();}});
for(const b of buttons){b.addEventListener('pointerdown',e=>{e.preventDefault();if(active){b.setPointerCapture(e.pointerId);pressed.set('ptr:'+e.pointerId,b.dataset.key);paint();transmit();}});for(const kind of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(kind,e=>{pressed.delete('ptr:'+e.pointerId);paint();transmit();});}
document.querySelector('#stop').addEventListener('pointerdown',release);
window.addEventListener('blur',release);window.addEventListener('pagehide',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
async function claim(){client=newClient();seq=0;if(!token){status.textContent='Pi 터미널에 표시된 전체 조종 주소로 접속하세요.';return;}try{const r=await post('/claim',{client});claimed=true;active=r.active;status.textContent=r.status;setInterval(transmit,100);}catch(e){status.textContent='연결 대기 · 다른 조종창은 닫아주세요.';setTimeout(claim,1000);}}
claim();
</script></html>'''


def handler_for(control, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass  # do not log control tokens or key events
        def respond(self, code, value, content_type="application/json"):
            data = value.encode() if isinstance(value, str) else json.dumps(value).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Length", str(len(data))); self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass
        def do_GET(self):
            if self.path == "/": self.respond(200, PAGE, "text/html")
            else: self.respond(404, {})
        def do_POST(self):
            auth = self.headers.get("Authorization", "")
            if not hmac.compare_digest(auth.encode("utf-8"), ("Bearer " + token).encode("utf-8")): self.respond(403, {}); return
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers.get("Host", ""):
                self.respond(403, {}); return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                if not 0 < n <= 512: raise ValueError()
                self.connection.settimeout(.3)
                data = json.loads(self.rfile.read(n))
                if not isinstance(data, dict): raise ValueError()
                now = time.monotonic()
                if self.path == "/claim": ok = control.claim(data.get("client"), now)
                elif self.path == "/control": ok = control.update(data.get("client"), data.get("sequence"), data.get("motion"), now)
                else: self.respond(404, {}); return
                self.respond(200 if ok else 409, control.snapshot())
            except (ValueError, OSError): self.respond(400, {})
    return Handler


def run_web(port, bind="127.0.0.1", web_port=8765):
    info = inspect(port)
    control, token = Control(), secrets.token_urlsafe(24)
    server = ThreadingHTTPServer((bind, web_port), handler_for(control, token))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    arm_sent, stopped, error = False, None, None
    try:
        nonce = secrets.randbelow(0xffffffff) + 1
        arm_sent = True; send(port, frame(f"ARM {nonce:08X}"))
        validate(receive(port, 6), "armed", nonce, 0)
        control.set_status(True, "연결됨 · 키를 누르고 조종하세요")
        print(json.dumps(info), flush=True)
        print(f"조종 주소: http://{bind}:{server.server_port}/#{token}", flush=True)
        print("PC 브라우저에서 위 주소를 여세요. 종료·토크 해제는 이 터미널에서 Ctrl+C.", flush=True)
        seq = 0
        while True:
            tick = time.monotonic(); motion = control.desired(tick); seq += 1
            if seq > 0xffffffff: raise RuntimeError("sequence exhausted")
            send(port, frame(f"D {nonce:08X} {seq:08X} {motion}"))
            validate(receive(port, .3), "ack", nonce, seq)
            remaining = PERIOD - (time.monotonic() - tick)
            if remaining > 0: time.sleep(remaining)
    except BaseException as exc: error = exc
    finally:
        control.set_status(False, "조종 종료 · 정지 확인 중")
        try:
            if arm_sent: stopped = stop(port)
        except Exception as exc: error = RuntimeError(f"{error or 'exit'}; STOP failed: {exc}. Cut motor 12 V power.")
        finally: server.shutdown(); server.server_close(); thread.join(timeout=1)
    if stopped: print(json.dumps(stopped, indent=2))
    if error is not None and not isinstance(error, KeyboardInterrupt): raise error
    return stopped
