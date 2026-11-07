#!/usr/bin/env python3
"""VESC maintenance bridge: VESC Tool (TCP) <-> SocketCAN, for the Calypso ROV.

VESC Tool on the surface laptop connects with "TCP" to <pi-ip>:65102 and sees one VESC
(the default target); "CAN scan" lists every controller on can0 and selecting one
forwards all traffic to it, exactly as when a VESC is connected by USB.

How it works
  * TCP side: VESC packet framing (start 2/3/4, length, payload, CRC16-XMODEM, end 3).
  * CAN side: VESC buffer protocol (comm_can.c): PROCESS_SHORT_BUFFER for payloads
    <= 6 bytes, otherwise FILL_RX_BUFFER(_LONG) + PROCESS_RX_BUFFER with CRC; requests
    are sent with send mode 0, replies come back to BRIDGE_ID with mode 1.
  * COMM_FORWARD_CAN [34, id, ...] is unwrapped and sent to that id.
  * COMM_PING_CAN [62] is answered by the bridge itself (CAN PING/PONG to ids 1..253).

Safety interlock (independent of process names or containers)
  * The bridge listens to can0 and refuses to forward anything while CAN_PACKET_SET_RPM
    (or any other motor command frame) is seen on the bus: ArduSub must be stopped
    (or disarmed with CAN mode off) first. If motor commands appear while the bridge is
    running it drops the TCP client and waits for silence again.
  * It never sends motor commands itself. Motor detection started from VESC Tool DOES
    spin the motor: do it only with the vehicle secured and the propeller clear.

No third-party packages: Python 3 standard library only (socket.AF_CAN).

usage: vesc_can_tcp_bridge.py [--iface can0] [--port 65102] [--target 1]
                              [--bridge-id 120] [--quiet-s 2.0]
"""
import argparse, socket, struct, select, time, sys, logging

# ---- VESC constants (bldc/datatypes.h) ----
CAN_SET_DUTY, CAN_SET_CURRENT, CAN_SET_CURRENT_BRAKE, CAN_SET_RPM, CAN_SET_POS = 0, 1, 2, 3, 4
CAN_FILL_RX_BUFFER, CAN_FILL_RX_BUFFER_LONG, CAN_PROCESS_RX_BUFFER, CAN_PROCESS_SHORT_BUFFER = 5, 6, 7, 8
CAN_SET_CURRENT_REL, CAN_SET_CURRENT_BRAKE_REL, CAN_SET_CURRENT_HANDBRAKE, CAN_SET_CURRENT_HANDBRAKE_REL = 10, 11, 12, 13
CAN_PING, CAN_PONG = 17, 18
MOTOR_CMDS = {CAN_SET_DUTY, CAN_SET_CURRENT, CAN_SET_CURRENT_BRAKE, CAN_SET_RPM, CAN_SET_POS,
              CAN_SET_CURRENT_REL, CAN_SET_CURRENT_BRAKE_REL, CAN_SET_CURRENT_HANDBRAKE, CAN_SET_CURRENT_HANDBRAKE_REL}
COMM_FORWARD_CAN, COMM_PING_CAN = 34, 62
CAN_EFF_FLAG, CAN_EFF_MASK = 0x80000000, 0x1FFFFFFF
CAN_FRAME = struct.Struct('=IB3x8s')

log = logging.getLogger('vesc-bridge')


def crc16(data: bytes) -> int:
    """CRC16-CCITT/XMODEM as used by the VESC firmware (poly 0x1021, init 0)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------- TCP framing
def frame_packet(payload: bytes) -> bytes:
    n = len(payload)
    if n <= 255:
        head = bytes([2, n])
    elif n <= 65535:
        head = bytes([3, n >> 8, n & 0xFF])
    else:
        head = bytes([4, (n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF])
    c = crc16(payload)
    return head + payload + bytes([c >> 8, c & 0xFF, 3])


class PacketDecoder:
    """Incremental decoder of VESC-framed packets from a byte stream."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while self.buf:
            s = self.buf[0]
            if s not in (2, 3, 4):
                del self.buf[0]; continue
            hl = {2: 2, 3: 3, 4: 4}[s]
            if len(self.buf) < hl:
                break
            n = int.from_bytes(self.buf[1:hl], 'big')
            if n == 0 or n > 512 * 1024:
                del self.buf[0]; continue
            total = hl + n + 3
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[hl:hl + n])
            c = int.from_bytes(self.buf[hl + n:hl + n + 2], 'big')
            if self.buf[total - 1] == 3 and c == crc16(payload):
                out.append(payload); del self.buf[:total]
            else:
                del self.buf[0]          # resync
        return out


# ---------------------------------------------------------------- CAN buffer protocol
def can_frames_for(target: int, sender: int, payload: bytes, send_mode: int = 0):
    """List of (eid, data) frames that deliver `payload` to `target` (comm_can_send_buffer)."""
    frames = []
    n = len(payload)
    if n <= 6:
        frames.append((target | (CAN_PROCESS_SHORT_BUFFER << 8), bytes([sender, send_mode]) + payload))
        return frames
    end_a = 0
    i = 0
    while i < n:
        if i > 255:
            break
        end_a = i + 7
        frames.append((target | (CAN_FILL_RX_BUFFER << 8), bytes([i]) + payload[i:i + 7]))
        i += 7
    i = end_a
    while i < n:
        frames.append((target | (CAN_FILL_RX_BUFFER_LONG << 8), bytes([i >> 8, i & 0xFF]) + payload[i:i + 6]))
        i += 6
    c = crc16(payload)
    frames.append((target | (CAN_PROCESS_RX_BUFFER << 8), bytes([sender, send_mode, n >> 8, n & 0xFF, c >> 8, c & 0xFF])))
    return frames


class CanReassembler:
    """Collects buffer frames addressed to `my_id`; returns (sender, mode, payload) when complete."""
    def __init__(self, my_id: int):
        self.my_id = my_id
        self.rx = bytearray(4096)

    def handle(self, eid: int, data: bytes):
        if (eid & 0xFF) != self.my_id:
            return None
        cmd = (eid >> 8) & 0xFF
        if cmd == CAN_FILL_RX_BUFFER and len(data) >= 1:
            o = data[0]; self.rx[o:o + len(data) - 1] = data[1:]
        elif cmd == CAN_FILL_RX_BUFFER_LONG and len(data) >= 2:
            o = (data[0] << 8) | data[1]
            if o + len(data) - 2 > len(self.rx):
                self.rx.extend(bytes(o + len(data) - len(self.rx)))
            self.rx[o:o + len(data) - 2] = data[2:]
        elif cmd == CAN_PROCESS_RX_BUFFER and len(data) >= 6:
            sender, mode = data[0], data[1]
            n = (data[2] << 8) | data[3]; c = (data[4] << 8) | data[5]
            payload = bytes(self.rx[:n])
            if crc16(payload) != c:
                log.warning('CAN RX buffer CRC mismatch from id %d (%d bytes)', sender, n)
                return None
            return sender, mode, payload
        elif cmd == CAN_PROCESS_SHORT_BUFFER and len(data) >= 2:
            return data[0], data[1], bytes(data[2:])
        return None


# ---------------------------------------------------------------- bridge
class Bridge:
    def __init__(self, can_sock, iface, args):
        self.can = can_sock
        self.args = args
        self.reasm = CanReassembler(args.bridge_id)
        self.last_motor_cmd = time.monotonic()        # assume unsafe until proven quiet
        self.pongs = set()
        self.client = None
        self.client_addr = None
        self.dec = PacketDecoder()
        self.current_target = args.target

    # -- CAN I/O
    def can_send(self, eid, data):
        fr = CAN_FRAME.pack(eid | CAN_EFF_FLAG, len(data), data.ljust(8, b'\0'))
        for _ in range(200):                        # ENOBUFS: TX queue of the adapter full, retry
            try:
                self.can.send(fr); break
            except OSError as e:
                if e.errno != 105:
                    raise
                time.sleep(0.001)
        else:
            raise OSError('CAN TX queue stuck (ENOBUFS)')
        time.sleep(0.0002)                          # do not flood the adapter queue

    def can_poll(self, timeout=0.0):
        r, _, _ = select.select([self.can], [], [], timeout)
        if not r:
            return []
        out = []
        while True:
            try:
                fr = self.can.recv(16, socket.MSG_DONTWAIT)
            except BlockingIOError:
                break
            eid, dlc, data = CAN_FRAME.unpack(fr)
            if not eid & CAN_EFF_FLAG:
                continue
            out.append((eid & CAN_EFF_MASK, data[:dlc]))
        return out

    def handle_can(self, frames):
        for eid, data in frames:
            cmd = (eid >> 8) & 0xFF
            if cmd in MOTOR_CMDS:
                self.last_motor_cmd = time.monotonic()
                continue
            if cmd == CAN_PONG and (eid & 0xFF) == self.args.bridge_id and data:
                self.pongs.add(data[0]); continue
            res = self.reasm.handle(eid, data)
            if res and self.client:
                sender, mode, payload = res
                self.client.sendall(frame_packet(payload))

    def bus_quiet(self):
        return time.monotonic() - self.last_motor_cmd > self.args.quiet_s

    # -- requests from VESC Tool
    def send_to(self, target, payload):
        for eid, data in can_frames_for(target, self.args.bridge_id, payload, 0):
            self.can_send(eid, data)

    def ping_can(self):
        self.pongs.clear()
        for cid in range(0, 254):
            if cid == self.args.bridge_id:
                continue
            self.can_send(cid | (CAN_PING << 8), bytes([self.args.bridge_id]))
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            self.handle_can(self.can_poll(0.02))
        ids = sorted(self.pongs)
        log.info('CAN ping: controllers %s', ids)
        return bytes([COMM_PING_CAN] + ids)

    def handle_tcp_packet(self, p):
        if not p:
            return
        if p[0] == COMM_PING_CAN:
            self.client.sendall(frame_packet(self.ping_can()))
        elif p[0] == COMM_FORWARD_CAN and len(p) >= 2:
            self.send_to(p[1], p[2:])
        else:
            self.send_to(self.current_target, p)

    def status(self):
        return {'bus_quiet': self.bus_quiet(),
                'last_motor_cmd_age_s': round(time.monotonic() - self.last_motor_cmd, 1),
                'client': self.client_addr, 'last_ping_ids': sorted(self.pongs)}

    # -- main loop
    def run(self, stop_event=None):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.args.port)); srv.listen(1)
        try:
            self._loop(srv, stop_event)
        finally:
            if self.client:
                self.client.close(); self.client = None; self.client_addr = None
            srv.close()
            log.info('bridge stopped')

    def _loop(self, srv, stop_event):
        log.info('listening on TCP %d, CAN %s, bridge id %d, default target %d',
                 self.args.port, self.args.iface, self.args.bridge_id, self.args.target)
        warned = 0
        while not (stop_event and stop_event.is_set()):
            self.handle_can(self.can_poll(0.0))
            rl = [srv, self.can] + ([self.client] if self.client else [])
            r, _, _ = select.select(rl, [], [], 0.05)
            if self.can in r:
                self.handle_can(self.can_poll(0.0))
            if not self.bus_quiet():
                if self.client:
                    log.error('motor command frames on the bus: ArduSub is commanding the VESCs. '
                              'Dropping the VESC Tool connection.')
                    self.client.close(); self.client = None; self.client_addr = None; self.dec = PacketDecoder()
                if time.monotonic() - warned > 5:
                    log.warning('waiting: motor commands seen on %s in the last %.1f s; stop ArduSub first',
                                self.args.iface, self.args.quiet_s)
                    warned = time.monotonic()
                if srv in r:                          # refuse connections while unsafe
                    c, a = srv.accept(); log.warning('refused %s (bus not quiet)', a[0]); c.close()
                continue
            if srv in r:
                c, a = srv.accept()
                if self.client:
                    c.close(); log.warning('second client %s refused', a[0])
                else:
                    self.client = c; self.client_addr = a[0]; self.dec = PacketDecoder(); log.info('VESC Tool connected from %s', a[0])
            if self.client and self.client in r:
                try:
                    data = self.client.recv(4096)
                except OSError:
                    data = b''
                if not data:
                    log.info('VESC Tool disconnected'); self.client.close(); self.client = None; self.client_addr = None; continue
                for p in self.dec.feed(data):
                    self.handle_tcp_packet(p)


def open_can(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--port', type=int, default=65102)
    ap.add_argument('--target', type=int, default=1, help='controller id VESC Tool talks to by default')
    ap.add_argument('--bridge-id', type=int, default=120, help='CAN id used by the bridge (must be unused)')
    ap.add_argument('--quiet-s', type=float, default=2.0, help='seconds without motor commands before forwarding')
    a = ap.parse_args()
    if not 1 <= a.bridge_id <= 253 or a.bridge_id == a.target:
        sys.exit('bad --bridge-id')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    Bridge(open_can(a.iface), a.iface, a).run()


if __name__ == '__main__':
    main()
