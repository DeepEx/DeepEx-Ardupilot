import os
import socket, threading, time, struct, random, sys, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import vesc_can_tcp_bridge as vb

a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)   # a: bridge side, b: fake bus

class FakeVesc:
    """VESC controller ids 1..6 on the fake bus: answers PING, FW_VERSION, echoes big packets."""
    def __init__(self, sock): self.s = sock; self.re = {i: vb.CanReassembler(i) for i in range(1, 7)}; self.inject_rpm = False
    def send(self, eid, data): self.s.send(vb.CAN_FRAME.pack(eid | vb.CAN_EFF_FLAG, len(data), data.ljust(8, b'\0')))
    def run(self):
        while True:
            if self.inject_rpm: self.send(2 | (vb.CAN_SET_RPM << 8), struct.pack('>i', 0)); time.sleep(0.01)
            try: fr = self.s.recv(16, socket.MSG_DONTWAIT)
            except BlockingIOError: time.sleep(0.0005); continue
            except OSError: return
            if len(fr) < 16: return
            eid, dlc, data = vb.CAN_FRAME.unpack(fr); eid &= vb.CAN_EFF_MASK; data = data[:dlc]
            cid, cmd = eid & 0xFF, (eid >> 8) & 0xFF
            if cid not in self.re: continue
            if cmd == vb.CAN_PING: self.send(data[0] | (vb.CAN_PONG << 8), bytes([cid, 1])); continue
            r = self.re[cid].handle(eid, data)
            if r:
                sender, mode, p = r; assert mode == 0
                rep = bytes([0, 6, 5, cid]) if p[0] == 0 else bytes([p[0], cid]) + p[1:]    # FW_VERSION or echo
                for e, d in vb.can_frames_for(sender, cid, rep, 1): self.send(e, d)

def main():
    fv = FakeVesc(b); threading.Thread(target=fv.run, daemon=True).start()
    args = types.SimpleNamespace(iface='fake', port=0, target=1, bridge_id=120, quiet_s=0.3)
    port = random.randint(40000, 50000); args.port = port
    br = vb.Bridge(a, 'fake', args); threading.Thread(target=br.run, daemon=True).start()
    time.sleep(0.6)

    def conn():
        c = socket.create_connection(('127.0.0.1', port)); c.settimeout(3); return c
    def req(c, p):
        c.sendall(vb.frame_packet(p)); d = vb.PacketDecoder(); t = time.time()
        while time.time() - t < 3:
            out = d.feed(c.recv(65536))
            if out: return out[0]
    c = conn()
    assert req(c, bytes([0])) == bytes([0, 6, 5, 1]); print('FW_VERSION default target 1: OK')
    assert req(c, bytes([34, 4, 0])) == bytes([0, 6, 5, 4]); print('FORWARD_CAN to id 4: OK')
    ping = req(c, bytes([62])); assert ping == bytes([62, 1, 2, 3, 4, 5, 6]), ping; print('PING_CAN ids', list(ping[1:]), 'OK')
    for n in (5, 6, 7, 8, 100, 255, 256, 262, 263, 700, 1500):
        p = bytes([50]) + bytes(random.getrandbits(8) for _ in range(n - 1))
        r = req(c, bytes([34, 3]) + p); assert r == bytes([50, 3]) + p[1:], n
    print('buffer segmentation/reassembly 5..1500 bytes: OK')
    fv.inject_rpm = True; time.sleep(0.2)
    try:
        r = c.recv(10); assert r == b'', r; print('motor commands on bus -> client dropped: OK')
    except (ConnectionResetError, socket.timeout) as e: print('dropped:', type(e).__name__)
    try:
        c2 = conn(); r = c2.recv(10); assert r == b''; print('new connection refused while unsafe: OK')
    except ConnectionResetError: print('new connection refused while unsafe: OK')
    fv.inject_rpm = False; time.sleep(0.6)
    c3 = conn(); assert req(c3, bytes([0])) == bytes([0, 6, 5, 1]); print('after bus quiet, reconnect: OK')


if __name__ == "__main__":
    main()
