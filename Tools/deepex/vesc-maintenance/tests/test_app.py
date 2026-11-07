import os, json, threading, time, socket, urllib.request, random, sys
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from datetime import datetime, timezone
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
# --- fake BlueOS
state = {'running': True, 'armed': False, 'calls': []}
class B(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if not state['running']:
            t = '2026-01-01T00:00:00Z'
        else:
            t = datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')
        b = json.dumps({'message': {'base_mode': {'bits': 209 if state['armed'] else 81}}, 'status': {'time': {'last_update': t}}}).encode()
        self.send_response(200); self.end_headers(); self.wfile.write(b)
    def do_POST(self):
        state['calls'].append(self.path); state['running'] = self.path.endswith('/start')
        self.send_response(200); self.end_headers(); self.wfile.write(b'null')
bport = random.randint(30000, 39999); threading.Thread(target=ThreadingHTTPServer(('127.0.0.1', bport), B).serve_forever, daemon=True).start()
os.environ['BLUEOS_HOST'] = f'http://127.0.0.1:{bport}'; wport = random.randint(40000, 49999); os.environ['WEB_PORT'] = str(wport)
tport = random.randint(50000, 59999); os.environ['VESC_TCP_PORT'] = str(tport); os.environ['VESC_QUIET_S'] = '0.3'
import vesc_can_tcp_bridge as vb, app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_bridge import FakeVesc  # noqa  (test_bridge starts its own bridge too; harmless)
a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
fv = FakeVesc(b); threading.Thread(target=fv.run, daemon=True).start()
vb.open_can = lambda iface: a
threading.Thread(target=app.main, daemon=True).start(); time.sleep(0.5)
def api(path, body=None):
    req = urllib.request.Request(f'http://127.0.0.1:{wport}{path}', data=None if body is None else json.dumps(body).encode(), method='GET' if body is None else 'POST')
    try:
        with urllib.request.urlopen(req) as r: return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e: return e.code, json.loads(e.read())
print('register', api('/register_service')[1]['name'])
s = api('/api/status')[1]; assert s['autopilot_heartbeat'] and s['armed'] is False and not s['maintenance']; print('status normal OK')
state['armed'] = True; c, r = api('/api/maintenance', {'enable': True}); assert c == 409 and 'ARMED' in r['message']; print('refused while armed OK')
state['armed'] = False; c, r = api('/api/maintenance', {'enable': True}); assert c == 200, r; assert state['calls'][-1].endswith('/v1.0/stop'); print('ON:', r['message'])
time.sleep(0.6); s = api('/api/status')[1]; assert s['maintenance'] and not s['autopilot_heartbeat'] and s['bridge']['bus_quiet']; print('status maintenance OK', s['bridge'])
cl = socket.create_connection(('127.0.0.1', tport)); cl.settimeout(3); cl.sendall(vb.frame_packet(bytes([62]))); d = vb.PacketDecoder(); out = []
while not out: out = d.feed(cl.recv(4096))
assert out[0] == bytes([62, 1, 2, 3, 4, 5, 6]); print('VESC Tool scan through extension OK')
time.sleep(0.2); assert api('/api/status')[1]['bridge']['client'] == '127.0.0.1'
c, r = api('/api/maintenance', {'enable': False}); assert c == 200 and state['calls'][-1].endswith('/v1.0/start'); print('OFF:', r['message'])
assert not api('/api/status')[1]['maintenance']
try: cl.sendall(vb.frame_packet(bytes([0]))); assert cl.recv(10) == b''
except (ConnectionResetError, BrokenPipeError): pass
print('client closed on OFF OK')
