"""Local browser key-down/key-up control for the STM32 manual base session."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import os
from pathlib import Path
from datetime import datetime, timezone
import threading
import time
from teleop_mobile_base import inspect, send, receive, frame, validate, stop, PERIOD, response_timeout, prepare_hold, check_hold, stop_failure_message

BROWSER_LEASE = .25


def failure_status(stopped):
    if not stopped or stopped.get('armed') is not False or stopped.get('stop_confirmed') is not True:
        return '바퀴 정지 확인 실패 · 움직이면 모터 전원을 차단하세요'
    if stopped.get('hold_confirmed') is not True:
        slot = stopped.get('hold_failed_slot', 0)
        axis = f'왼팔 {slot}번' if 1 <= slot <= 6 else f'오른팔 {slot-6}번' if 7 <= slot <= 12 else '리프트' if slot == 13 else '팔·리프트'
        issue = '상태 조회 실패' if stopped.get('hold_reason') in ('HOLD_READ_FAILED', 'LIFT_POSITION_READ_FAILED') else '자세 유지 확인 실패'
        return f'{axis} {issue} · 바퀴 정지 확인 · 받침 유지'
    return '바퀴 정지·자세 유지 확인 · 오류로 조종 종료 · 기록 확인 필요'


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
        self.action = None
        self.busy = False
        self.ready = False
        self.ended = False
        self.recording = ""
        self.feedback = False

    def claim(self, owner, now):
        if not isinstance(owner, str) or not 16 <= len(owner) <= 64: return False
        with self.lock:
            # Reclaim must use a new browser identity. Never reset sequence for
            # an old identity: delayed commands/claim retries must stay invalid.
            if self.active or self.busy or self.ended: return False
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
            if owner != self.owner or sequence <= self.sequence: return False
            if self.ended: return motion == "Z"
            if not self.active and motion != "Z": return False
            self._expire(now)
            if self.expired: return False
            self.motion, self.sequence, self.last = motion, sequence, now
            return True

    def _expire(self, now):
        # Called with lock held, including in update before accepting late input.
        if (self.active or self.busy) and self.owner is not None and now - self.last >= BROWSER_LEASE:
            self.expired, self.motion = True, "Z"

    def desired(self, now):
        with self.lock:
            self._expire(now)
            return self.motion if self.active and not self.expired else "Z"

    def snapshot(self):
        with self.lock:
            return dict(active=self.active, status=self.status, busy=self.busy,
                        ready=self.ready, ended=self.ended, recording=self.recording,
                        feedback=self.feedback)

    def set_status(self, active, status):
        with self.lock:
            self.active, self.status = active, status
            if not active: self.motion = "Z"

    def request(self, owner, sequence, action, now):
        if action not in ("hold", "start", "finish"): return False
        if type(sequence) is not int or not 0 < sequence <= 0xffffffff: return False
        with self.lock:
            self._expire(now)
            if owner != self.owner or sequence <= self.sequence or self.expired or self.ended:
                return False
            if self.busy or self.action is not None: return False
            if action == "hold" and self.active: return False
            if action == "start" and (self.active or not self.ready): return False
            self.sequence, self.last, self.motion = sequence, now, "Z"
            self.action, self.busy = action, True
            return True

    def take_action(self, now):
        with self.lock:
            self._expire(now)
            action, self.action = self.action, None
            if self.expired and action != "finish":
                self.busy = False
                return None
            return action

    def configure(self, **values):
        with self.lock:
            for key, value in values.items(): setattr(self, key, value)


def validate_feedback(reply):
    wheels = reply.get("wheel_feedback")
    if not isinstance(wheels, list) or len(wheels) != 3:
        raise RuntimeError("missing live wheel feedback")
    for expected, sample in zip((7, 8, 9), wheels):
        if not isinstance(sample, dict) or sample.get("id") != expected:
            raise RuntimeError("invalid wheel feedback identity")
        for key, maximum in (("position_raw", 65535), ("speed_raw", 65535),
                             ("tick_begin_ms", 0xffffffff), ("tick_end_ms", 0xffffffff)):
            value = sample.get(key)
            if type(value) is not int or not 0 <= value <= maximum:
                raise RuntimeError("invalid wheel feedback value")
        if (sample["tick_end_ms"] - sample["tick_begin_ms"]) & 0xffffffff >= 400:
            raise RuntimeError("wheel feedback exceeded command lease")


class SessionRecord:
    """Append-only raw evidence. Never an odometry or SLAM completion claim."""
    def __init__(self, directory):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") +
                                 "-" + secrets.token_hex(4) + ".jsonl")
        self.file = self.path.open("x", encoding="utf-8")
        os.chmod(self.path, 0o600)
        self.count = 0

    def write(self, event, **data):
        self.file.write(json.dumps(dict(event=event, host_monotonic_ns=time.monotonic_ns(),
                                       host_utc=datetime.now(timezone.utc).isoformat(), **data)) + "\n")
        self.file.flush()
        self.count += 1

    def close(self):
        try:
            self.file.flush()
            os.fsync(self.file.fileno())
        finally:
            self.file.close()


PAGE = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AlohaMini 베이스 조종</title><style>
body{font:18px system-ui;background:#101923;color:#eef4f8;margin:0;padding:20px;max-width:700px;margin:auto}
h1{font-size:26px}#status{padding:16px;background:#233447;border-radius:12px;margin:20px 0}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}button{font:inherit;font-size:16px;word-break:keep-all;border:0;border-radius:12px;background:#d9e9f3;color:#132535;min-height:90px;touch-action:none;user-select:none}
.actions{display:flex;gap:8px;margin:20px 0}.actions button{flex:1;min-height:64px;font-size:15px}button:disabled{opacity:.4}#record{font-size:14px;color:#aebdca;margin-top:16px}button.on{background:#58d9bd}#stop{background:#efae9c}small{display:block;color:#aebdca;line-height:1.6;margin-top:24px}
</style><h1>AlohaMini 베이스 조종</h1><p>키나 버튼을 누르고 있으면 이동하고, 놓으면 멈춥니다.</p>
<div class="actions"><button id="hold" disabled>자세 유지</button><button id="start" disabled>조종 시작</button><button id="finish" disabled>기록·종료</button></div>
<label id="support-row"><input type="checkbox" id="supported"> 양팔과 리프트를 받쳤어요</label>
<div id="record"></div><div id="status">연결 준비 중…</div><div class="grid">
<button data-key="q">Q ↶<br>왼쪽 회전</button><button data-key="w">W ↑<br>전진</button><button data-key="e">E ↷<br>오른쪽 회전</button>
<button data-key="a">A ←<br>왼쪽 이동</button><button id="stop">Space<br>정지</button><button data-key="d">D →<br>오른쪽 이동</button>
<div></div><button data-key="s">S ↓<br>후진</button></div>
<small>한 방향씩 조작합니다. 창을 벗어나면 정지 요청을 보냅니다. 통신이 끊기면 잠시 정지하고 연결을 다시 확인합니다. 연결 복구 후 조종 시작을 눌러 이어가세요.<br>저장하고 종료는 바퀴를 끄고 조종 기록을 저장합니다. 지도 저장은 별도 매핑 기능에서 합니다. 정지하지 않으면 모터 12V를 차단하세요.</small>
<script>
const token=location.hash.slice(1), newClient=()=>crypto.randomUUID?crypto.randomUUID():Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
const mapping={w:'F',s:'B',a:'L',d:'R',q:'A',e:'D'},pressed=new Map();
let client,seq=0,claimed=false,active=false,inFlight=false,pending=false,action=null,heartbeat=null;
const status=document.querySelector('#status'), buttons=[...document.querySelectorAll('[data-key]')];
const holdButton=document.querySelector('#hold'),startButton=document.querySelector('#start'),finishButton=document.querySelector('#finish'),supported=document.querySelector('#supported'),recordLabel=document.querySelector('#record');
function render(r){active=r.active;status.textContent=r.status;
 holdButton.disabled=active||r.busy||r.ended||!supported.checked;
 for(const b of buttons)b.disabled=!active;
 startButton.disabled=active||r.busy||r.ended||!r.ready;
 finishButton.disabled=r.busy||r.ended;
 recordLabel.textContent='조종 기록: '+(r.recording||'준비 중')+' · '+(r.feedback?'바퀴 피드백 포함':'명령 기록 · 바퀴 피드백 없음');
 if(!active){pressed.clear();paint();}
 if(r.ended){claimed=false;active=false;}
}
let latest={};
supported.addEventListener('change',()=>render(latest));
function requestAction(value){if(!claimed||action)return;pressed.clear();paint();action=value;transmit();}
holdButton.addEventListener('click',()=>{if(supported.checked)requestAction('hold');});
startButton.addEventListener('click',()=>requestAction('start'));
finishButton.addEventListener('click',()=>requestAction('finish'));
function motion(){const keys=[...pressed.values()];return mapping[keys[keys.length-1]]||'Z';}
function paint(){for(const b of buttons)b.classList.toggle('on',[...pressed.values()].includes(b.dataset.key));}
async function post(path,data){
 const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1500);
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
 const m=motion(),n=++seq,a=action;action=null;
 try{
  const r=await post(a?'/action':'/control',a?{client,sequence:n,action:a}:{client,sequence:n,motion:m});
  if(!claimed)return;
  latest=r;render(r);
 }catch(e){
  claimed=false;active=false;pressed.clear();paint();
  action=null;clearInterval(heartbeat);heartbeat=null;
  render({...latest,active:false,busy:true,ended:false,status:'일시정지 · 연결을 다시 확인하고 있어요'});
  setTimeout(claim,1000);
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
async function claim(){client=newClient();seq=0;if(!token){status.textContent='실행 시 표시된 전체 조종 주소로 접속하세요.';return;}try{const r=await post('/claim',{client});claimed=true;latest=r;render(r);clearInterval(heartbeat);heartbeat=setInterval(transmit,100);}catch(e){status.textContent='연결 대기 · 다른 조종창은 닫아주세요.';setTimeout(claim,1000);}}
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
                elif self.path == "/action": ok = control.request(data.get("client"), data.get("sequence"), data.get("action"), now)
                elif self.path == "/control": ok = control.update(data.get("client"), data.get("sequence"), data.get("motion"), now)
                else: self.respond(404, {}); return
                self.respond(200 if ok else 409, control.snapshot())
            except (ValueError, OSError): self.respond(400, {})
    return Handler


def run_web(port, bind="127.0.0.1", web_port=8765, record_dir="teleop-sessions"):
    info = inspect(port)
    record = SessionRecord(record_dir)
    control, token = Control(), secrets.token_urlsafe(24)
    held_mode = info.get("hold_setup") is True
    control.configure(ready=not held_mode, recording=record.path.name,
                      feedback=info.get("wheel_feedback") is True)
    control.set_status(False, "현재 자세를 유지한 뒤 조종을 시작하세요" if held_mode else "조종 시작을 눌러 주세요")
    server = None
    thread = None
    arm_sent, ever_armed, stopped, error = False, False, None, None
    def pause_drive(reason):
        nonlocal arm_sent, stopped
        control.configure(active=False, busy=True, motion="Z",
                          status="입력 연결 확인 중 · 바퀴 정지 중")
        stopped = stop(port)  # also consumes any already queued controller reply
        if stopped.get("fault") is not False:
            raise RuntimeError("controller fault requires inspection: " + json.dumps(stopped))
        arm_sent = False
        record.write("paused", reason=reason, stop=stopped)
        control.configure(busy=False, ready=True,
                          status="일시정지 · 연결 복구 후 조종 시작을 눌러 주세요")

    try:
        record.write("session", firmware=info, calibrated_odometry=False,
                     clock_note="Host timestamps bracket replies; MCU ticks wrap at 2^32 ms. Not hardware synchronized.")
        server = ThreadingHTTPServer((bind, web_port), handler_for(control, token))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(json.dumps(info), flush=True)
        print(f"조종 주소: http://{bind}:{server.server_port}/#{token}", flush=True)
        print(f"기록: {record.path} · 화면에서 자세 유지 / 조종 시작 / 저장하고 종료", flush=True)
        seq, nonce = 0, 0
        while True:
            tick = time.monotonic()
            action = control.take_action(tick)
            if action == "finish": break
            if action == "hold":
                control.set_status(False, "현재 자세 유지 설정 중 · 받침을 유지하세요")
                record.write("hold_requested")
                result = prepare_hold(port, execute=True) if held_mode else None
                record.write("hold_ready", reply=result)
                control.configure(ready=True, busy=False)
                control.set_status(False, "자세 유지 확인 · 조종 시작을 눌러 주세요")
            elif action == "start":
                if held_mode:
                    record.write("hold_checked", reply=check_hold(port))
                # A slow preparation cannot turn an expired click into motion.
                control.desired(time.monotonic())
                if control.expired:
                    control.configure(busy=False)
                    continue
                nonce = secrets.randbelow(0xffffffff) + 1
                record.write("arm_requested", session=nonce)
                arm_sent = ever_armed = True
                stopped = None
                seq = 0
                send(port, frame(f"ARM {nonce:08X}"))
                reply = receive(port, 6)
                validate(reply, "armed", nonce, 0)
                record.write("armed", reply=reply)
                control.configure(busy=False, active=True, status="조종 중 · 키나 버튼을 놓으면 멈춥니다")
            if arm_sent:
                motion = control.desired(time.monotonic())
                if control.expired:
                    record.write("browser_expired")
                    pause_drive("browser_input_gap")
                    continue
                seq += 1
                if seq > 0xffffffff: raise RuntimeError("sequence exhausted")
                # Write intent before issuing a command. Disk errors never lead to
                # another movement command; cleanup still attempts STOP.
                record.write("command", sequence=seq, session=nonce, motion=motion)
                send(port, frame(f"D {nonce:08X} {seq:08X} {motion}"))
                reply = receive(port, response_timeout())
                record.write("reply", reply=reply)
                if (reply.get("type") == "paused" and reply.get("armed") is False
                        and reply.get("fault") is False and reply.get("stop_confirmed") is True
                        and (not held_mode or reply.get("hold_confirmed") is True)):
                    pause_drive("controller_input_gap")
                    continue
                validate(reply, "ack", nonce, seq)
                if info.get("wheel_feedback") is True: validate_feedback(reply)
            remaining = PERIOD - (time.monotonic() - tick)
            if remaining > 0: time.sleep(remaining)
    except BaseException as exc:
        error = exc
    finally:
        control.set_status(False, "조종 종료 · 정지 확인 중")
        try:
            if arm_sent: stopped = stop(port)
        except Exception as exc:
            stopped = getattr(exc, 'reply', None)
            error = RuntimeError(stop_failure_message(f"{error or 'exit'}; stop verification: {exc}", stopped))
        try:
            record.write("finished", stop=stopped, error=str(error) if error else None,
                         wheel_session_started=ever_armed, map_saved=False)
            record.close()
        except Exception as exc:
            error = RuntimeError(f"{error or 'exit'}; recording save failed: {exc}")
            if not record.file.closed: record.file.close()
        control.configure(ended=True, busy=False, active=False, motion="Z",
                          status="정지·기록 저장 완료 · 팔과 리프트의 힘은 유지됩니다" if not error and held_mode and ever_armed
                          else "기록 저장 완료" if not error else failure_status(stopped))
        # Let the page receive the final persisted result before closing HTTP.
        if server:
            try:
                time.sleep(.5)
            finally:
                server.shutdown(); server.server_close()
                if thread: thread.join(timeout=1)
    if stopped: print(json.dumps(stopped, indent=2))
    print(json.dumps(dict(recording=str(record.path), error=str(error) if error else None)), flush=True)
    if error is not None and not isinstance(error, KeyboardInterrupt): raise error
    return stopped
