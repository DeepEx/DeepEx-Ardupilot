#!/usr/bin/env python3
"""DeepEx VESC maintenance mode: BlueOS extension service.

Web page (BlueOS sidebar) with two actions:
  * Maintenance ON  - refuses if the vehicle is armed; stops the autopilot through the BlueOS
                      autopilot manager; starts the VESC Tool TCP <-> can0 bridge.
  * Maintenance OFF - stops the bridge; starts the autopilot again.
Status shows armed state, autopilot heartbeat, bridge state, motor-command activity on can0
and the VESC Tool client.

Environment:
  BLUEOS_HOST   (default http://127.0.0.1)   nginx front end of BlueOS (host network)
  WEB_PORT      (default 9135)               port of this page / API
  VESC_IFACE    (default can0), VESC_TCP_PORT (65102), VESC_TARGET (1), VESC_BRIDGE_ID (120)

Python 3 standard library only.
"""
import json, os, threading, time, logging, types, urllib.request
from datetime import datetime, timezone
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import vesc_can_tcp_bridge as vb

BLUEOS = os.environ.get('BLUEOS_HOST', 'http://127.0.0.1').rstrip('/')
WEB_PORT = int(os.environ.get('WEB_PORT', '9135'))
CFG = types.SimpleNamespace(iface=os.environ.get('VESC_IFACE', 'can0'),
                            port=int(os.environ.get('VESC_TCP_PORT', '65102')),
                            target=int(os.environ.get('VESC_TARGET', '1')),
                            bridge_id=int(os.environ.get('VESC_BRIDGE_ID', '120')),
                            quiet_s=float(os.environ.get('VESC_QUIET_S', '2.0')))
VERSION = '0.1.0'
log = logging.getLogger('vesc-maint')


def http(method, path, timeout=5.0):
    req = urllib.request.Request(BLUEOS + path, method=method, data=b'' if method == 'POST' else None)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read()
        return json.loads(body) if body and body[:1] in b'{[' else body.decode(errors='replace')


def vehicle_state():
    """(armed, heartbeat_age_s) from mavlink2rest; (None, None) when no fresh heartbeat (autopilot stopped)."""
    try:
        m = http('GET', '/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT', 2.0)
        bits = m['message']['base_mode']['bits']
        last = m['status']['time']['last_update'].replace('Z', '+00:00')
        age = (datetime.now(timezone.utc) - datetime.fromisoformat(last)).total_seconds()
    except Exception as e:                      # autopilot stopped, mavlink2rest down or format change
        log.debug('heartbeat unavailable: %s', e)
        return None, None
    if age > 3.0:
        return None, age                        # stale: autopilot not running
    return bool(bits & 128), age


class Maintenance:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = None
        self.thread = None
        self.stop_event = None
        self.message = 'idle'

    @property
    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def enable(self):
        with self.lock:
            if self.active:
                return True, 'already in maintenance'
            armed, _ = vehicle_state()
            if armed:
                return False, 'vehicle ARMED: disarm first'
            try:
                http('POST', '/ardupilot-manager/v1.0/stop', 20.0)
            except Exception as e:
                return False, f'could not stop the autopilot: {e}'
            try:
                can = vb.open_can(CFG.iface)
            except OSError as e:
                return False, f'cannot open {CFG.iface}: {e} (autopilot stopped: press OFF to restart it)'
            self.bridge = vb.Bridge(can, CFG.iface, CFG)
            self.stop_event = threading.Event()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            self.message = 'maintenance ON: autopilot stopped, bridge waiting for a quiet bus'
            return True, self.message

    def _run(self):
        try:
            self.bridge.run(self.stop_event)
        except Exception as e:
            log.exception('bridge failed')
            self.message = f'bridge error: {e}'
        finally:
            try:
                self.bridge.can.close()
            except Exception:
                pass

    def disable(self):
        with self.lock:
            if self.active:
                self.stop_event.set(); self.thread.join(3.0)
            self.thread = None; self.bridge = None
            try:
                http('POST', '/ardupilot-manager/v1.0/start', 30.0)
            except Exception as e:
                self.message = f'bridge stopped, autopilot start FAILED: {e}'
                return False, self.message
            self.message = 'maintenance OFF: bridge stopped, autopilot started'
            return True, self.message

    def status(self):
        armed, hb = vehicle_state()
        s = {'version': VERSION, 'maintenance': self.active, 'message': self.message,
             'armed': armed, 'autopilot_heartbeat': armed is not None, 'heartbeat_age_s': None if hb is None else round(hb, 1),
             'iface': CFG.iface, 'tcp_port': CFG.port, 'default_target': CFG.target}
        if self.active and self.bridge:
            s['bridge'] = self.bridge.status()
        return s


M = Maintenance()

PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VESC maintenance</title><style>
:root{--bg:#fff;--fg:#1b1f24;--mut:#5b6470;--ok:#1a7f37;--warn:#9a6700;--bad:#cf222e;--card:#f6f8fa;--bd:#d0d7de}
@media (prefers-color-scheme:dark){:root{--bg:#0d1117;--fg:#e6edf3;--mut:#9da7b3;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--card:#161b22;--bd:#30363d}}
body{background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif;margin:0;padding:16px;max-width:720px}
h1{font-size:20px;margin:0 0 4px}p{color:var(--mut);margin:4px 0 14px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:12px 14px;margin:10px 0}
table{border-collapse:collapse;width:100%}td{padding:4px 0;border-bottom:1px solid var(--bd)}td:first-child{color:var(--mut);width:46%}
button{font:inherit;padding:10px 16px;border-radius:6px;border:1px solid var(--bd);cursor:pointer;margin-right:8px;background:var(--card);color:var(--fg)}
#on{border-color:var(--warn)}#off{border-color:var(--ok)}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
code{font-size:13px}</style></head><body>
<h1>VESC maintenance mode</h1>
<p>ON stops ArduSub and opens VESC Tool access to can0 (TCP <code id=port></code>). OFF closes it and restarts ArduSub.
Motor detection from VESC Tool spins the motor: vehicle secured, propeller clear.</p>
<div class=card><button id=on>Maintenance ON</button><button id=off>Maintenance OFF</button><div id=msg style="margin-top:10px"></div></div>
<div class=card><table id=t></table></div>
<script>
const $=s=>document.querySelector(s);
function row(k,v,c){return `<tr><td>${k}</td><td class="${c||''}">${v}</td></tr>`}
async function refresh(){try{const s=await (await fetch('api/status')).json();$('#port').textContent=s.tcp_port;
 let h=row('Mode',s.maintenance?'MAINTENANCE':'normal (ArduSub)',s.maintenance?'warn':'ok');
 h+=row('Autopilot heartbeat',s.autopilot_heartbeat?'present':'absent',s.autopilot_heartbeat?'ok':(s.maintenance?'':'bad'));
 h+=row('Armed',s.armed===null?'unknown':(s.armed?'ARMED':'disarmed'),s.armed?'bad':'');
 if(s.bridge){const b=s.bridge;h+=row('Motor commands on '+s.iface,b.bus_quiet?'none (last '+b.last_motor_cmd_age_s+' s ago)':'PRESENT',b.bus_quiet?'ok':'bad');
  h+=row('VESC Tool client',b.client||'not connected');h+=row('Last CAN scan',b.last_ping_ids.length?b.last_ping_ids.join(', '):'-');}
 h+=row('Default target id',s.default_target);$('#t').innerHTML=h;$('#msg').textContent=s.message;}catch(e){$('#msg').textContent='status error: '+e}}
async function act(on){$('#msg').textContent='working...';try{const r=await (await fetch('api/maintenance',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enable:on})})).json();$('#msg').textContent=r.message;}catch(e){$('#msg').textContent='error: '+e}refresh()}
$('#on').onclick=()=>{if(confirm('Stop ArduSub and enter VESC maintenance?'))act(true)};$('#off').onclick=()=>act(false);
refresh();setInterval(refresh,1500);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='application/json'):
        b = body if isinstance(body, bytes) else (json.dumps(body) if ctype == 'application/json' else body).encode()
        self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(b)))
        self.send_header('Access-Control-Allow-Origin', '*'); self.end_headers(); self.wfile.write(b)

    def log_message(self, fmt, *args):
        log.debug(fmt, *args)

    def do_GET(self):
        p = self.path.split('?')[0]
        if p in ('/', '/index.html'):
            self._send(200, PAGE, 'text/html; charset=utf-8')
        elif p == '/register_service':
            self._send(200, {'name': 'VESC Maintenance', 'description': 'Stop ArduSub and open VESC Tool access to the CAN bus',
                             'icon': 'mdi-wrench-cog', 'company': 'DeepEx', 'version': VERSION, 'webpage': '',
                             'api': '', 'works_in_relative_paths': True})
        elif p == '/api/status':
            self._send(200, M.status())
        else:
            self._send(404, {'error': 'not found'})

    def do_POST(self):
        p = self.path.split('?')[0]
        if p != '/api/maintenance':
            return self._send(404, {'error': 'not found'})
        try:
            n = int(self.headers.get('Content-Length', '0'))
            req = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            return self._send(400, {'ok': False, 'message': 'bad JSON'})
        ok, msg = M.enable() if req.get('enable') else M.disable()
        self._send(200 if ok else 409, {'ok': ok, 'message': msg})


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    srv = ThreadingHTTPServer(('0.0.0.0', WEB_PORT), Handler)
    log.info('VESC maintenance %s on http port %d (BlueOS %s)', VERSION, WEB_PORT, BLUEOS)
    srv.serve_forever()


if __name__ == '__main__':
    main()
