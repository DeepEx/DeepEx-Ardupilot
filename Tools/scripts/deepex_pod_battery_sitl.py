#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
ArduSub SITL test of libraries/AP_Scripting/applets/pod_battery.lua (ADR-0024).

Starts build/sitl/bin/ardusub in a scratch directory with the applet and the MAVLink Lua modules in ./scripts,
emulates the two DeepEx pods (MAVLink 2, sysid 1, compid 180/181, truncated payloads as a real MAVLink 2 sender
produces them) on SERIAL1 (tcp:5762), and checks the vehicle side on SERIAL0 (tcp:5760).

    ./waf sub                                   (board sitl, once)
    python3 Tools/scripts/deepex_pod_battery_sitl.py

Needs pymavlink. Exit status 0 only if every check passed.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

os.environ["MAVLINK20"] = "1"  # must be set before pymavlink is imported

from pymavlink import mavutil  # noqa: E402
from pymavlink import DFReader  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
ARDUSUB = os.path.join(ROOT, "build", "sitl", "bin", "ardusub")
APPLETS = os.path.join(ROOT, "libraries", "AP_Scripting", "applets")
MODULES = os.path.join(ROOT, "libraries", "AP_Scripting", "modules", "MAVLink")
SUB_PARM = os.path.join(ROOT, "Tools", "autotest", "default_params", "sub.parm")

MAV_SYS_STATUS_SENSOR_BATTERY = 0x2000000
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511

PWR_NAMES = {0: "UNKNOWN", 1: "DUAL", 2: "SINGLE_BAT1", 3: "SINGLE_BAT2", 4: "NONE"}

# AP_Arming::Check bits (ARMING_SKIPCHK is a bitmask of checks to SKIP)
ARM_CHECKS = {1: "BARO", 2: "COMPASS", 3: "GPS", 4: "INS", 5: "PARAMETERS", 6: "RC", 7: "VOLTAGE", 8: "BATTERY",
              9: "AIRSPEED", 10: "LOGGING", 11: "SWITCH", 12: "GPS_CONFIG", 13: "SYSTEM", 14: "MISSION",
              15: "RANGEFINDER", 16: "CAMERA", 17: "AUX_AUTH", 18: "VISION", 19: "FFT", 20: "OSD"}
SKIPCHK_BATTERY_BIT = 1 << 8
SKIPCHK_EXPECTED = 16777023   # ArduSub default 16776767 | 256
SUB_MODE_SURFACE = 9


def describe_mask(mask):
    checked = [n for b, n in ARM_CHECKS.items() if not mask & (1 << b)]
    return "0x%06X (%d): checked = %s" % (mask, mask, ", ".join(checked) if checked else "none")


class Pod:
    """One emulated pod: HEARTBEAT 1 Hz, BATTERY_STATUS 10 Hz, NAMED_VALUE_INT PWR_MODE / PWR_RSN 2 Hz."""

    def __init__(self, sock, master, sysid=1, compid=None):
        self.master = master
        self.compid = compid if compid is not None else (180 if master else 181)
        self.mav = mavutil.mavlink.MAVLink(sock, srcSystem=sysid, srcComponent=self.compid)
        self.battery_id = 0 if master else 1
        self.alive = True
        self.voltage_mv = 50123 if master else 49870
        self.current_ca = 1234 if master else 550
        self.consumed_mah = 777 if master else 321
        self.temp_cdeg = 3150 if master else 2800
        self.charge_state = 1
        self.mode = 1
        self.reason = 5
        self.last_frame_payload_len = None
        self.sent_battery = 0
        self.sent_named = 0
        self.t0 = time.time()
        self._lock = threading.Lock()

    def _boot_ms(self):
        return int((time.time() - self.t0) * 1000)

    def send_battery(self):
        voltages = [self.voltage_mv] + [65535] * 9
        self.mav.battery_status_send(
            self.battery_id, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL, mavutil.mavlink.MAV_BATTERY_TYPE_LION,
            self.temp_cdeg, voltages, self.current_ca, self.consumed_mah, -1, -1,
            0, self.charge_state, [0, 0, 0, 0], 0, 0)
        self.sent_battery += 1

    def frame_payload_len(self):
        """Payload length of a BATTERY_STATUS frame as sent on the wire (to prove MAVLink 2 truncation)."""
        msg = mavutil.mavlink.MAVLink_battery_status_message(
            self.battery_id, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL, mavutil.mavlink.MAV_BATTERY_TYPE_LION,
            self.temp_cdeg, [self.voltage_mv] + [65535] * 9, self.current_ca, self.consumed_mah, -1, -1,
            0, self.charge_state, [0, 0, 0, 0], 0, 0)
        frame = msg.pack(self.mav)
        return len(frame) - 12  # 10 header bytes + 2 checksum bytes

    def send_power(self):
        self.mav.named_value_int_send(self._boot_ms(), b"PWR_MODE", self.mode)
        self.mav.named_value_int_send(self._boot_ms(), b"PWR_RSN", self.reason)
        self.sent_named += 2

    def send_heartbeat(self):
        self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                mavutil.mavlink.MAV_STATE_ACTIVE)


class PodRunner(threading.Thread):
    def __init__(self, pods, rate_mult=1):
        super().__init__(daemon=True)
        self.pods = pods
        self.rate_mult = rate_mult
        self.stop_flag = False

    def run(self):
        tick = 0
        while not self.stop_flag:
            for p in self.pods:
                if not p.alive:
                    continue
                try:
                    p.send_battery()
                    if tick % 5 == 0:
                        p.send_power()
                    if tick % 10 == 0:
                        p.send_heartbeat()
                except OSError:
                    return
            tick += 1
            time.sleep(0.1 / self.rate_mult)


class Flood(threading.Thread):
    """Other components sending NAMED_VALUE_INT (and one foreign system claiming a pod compid)."""

    def __init__(self, sock, per_second_each=25):
        super().__init__(daemon=True)
        self.senders = [mavutil.mavlink.MAVLink(sock, srcSystem=1, srcComponent=c) for c in (100, 101, 102, 103)]
        self.senders.append(mavutil.mavlink.MAVLink(sock, srcSystem=2, srcComponent=180))
        self.period = 1.0 / per_second_each
        self.stop_flag = False
        self.sent = 0

    def run(self):
        n = 0
        while not self.stop_flag:
            for snd in self.senders:
                try:
                    snd.named_value_int_send(n & 0xFFFFFF, b"OTHER", n)
                    self.sent += 1
                except OSError:
                    return
            n += 1
            time.sleep(self.period)


class ScenarioDone(Exception):
    pass


def run_load(args, res, vehicle, emu, pod1, pod2, ctx):
    """Pods at 3x their real rate (BATTERY_STATUS 30 Hz, PWR_MODE/PWR_RSN 6 Hz each) plus 125 NAMED_VALUE_INT/s from
    other components, for --load-seconds. No false timeout, no queue loss."""
    runner = PodRunner([pod1, pod2], rate_mult=3)
    flood = Flood(emu, 25)
    runner.start()
    flood.start()
    ready = wait_for(lambda: all(vehicle.get_battery(i)[0] is not None and vehicle.get_battery(i)[0].voltages[0] > 0
                                 for i in (0, 1)), 20)
    res.check("load: both batteries reported", ready)
    time.sleep(2.0)
    t_load = time.time()
    low0 = vehicle.sys_health_low_samples
    zero_samples = 0
    samples = 0
    while time.time() < t_load + args.load_seconds:
        for i, pod in ((0, pod1), (1, pod2)):
            b, _ = vehicle.get_battery(i)
            samples += 1
            if b is None or b.voltages[0] != pod.voltage_mv:
                zero_samples += 1
        time.sleep(0.05)
    # judge BEFORE stopping the pods (the silence after the stop is expected)
    texts = vehicle.texts_since(t_load)
    health_low = vehicle.sys_health_low_samples - low0
    runner.stop_flag = True
    flood.stop_flag = True
    time.sleep(2.5)
    for line in vehicle.texts_since(0)[-12:]:
        print("INFO vehicle text: %s" % line, flush=True)
    print("INFO load: %d s, pods 3x (BATTERY_STATUS 30 Hz, PWR 6 Hz), foreign NAMED_VALUE_INT %d sent (~125/s)" % (
        args.load_seconds, flood.sent), flush=True)
    res.check("load: no false timeout (no unhealthy text, voltage never 0)",
              not any("pod silent" in x or "voltage not valid" in x for x in texts) and zero_samples == 0,
              "%d/%d samples off; %s" % (zero_samples, samples, [x for x in texts if "PodBatt" in x][:3]))
    res.check("load: battery health bit never cleared", health_low == 0)
    ctx.update(pod1=pod1, pod2=pod2, skip_rows=int(2 + 3))
    return runner, flood


class Vehicle(threading.Thread):
    """Reads everything ArduSub sends on SERIAL0 and keeps the interesting state."""

    def __init__(self, conn):
        super().__init__(daemon=True)
        self.conn = conn
        self.stop_flag = False
        self.battery = {}        # BATTERY_STATUS by id (last)
        self.battery_t = {}
        self.texts = []          # (time, severity, text)
        self.armed = False
        self.custom_mode = None
        self.sys_health = 0
        self.sys_present = 0
        self.acks = {}
        self.params = {}
        self.sys_health_low_samples = 0
        self.lock = threading.Lock()

    def run(self):
        while not self.stop_flag:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            t = m.get_type()
            if t in ("BATTERY_STATUS", "SYS_STATUS", "STATUSTEXT", "COMMAND_ACK", "PARAM_VALUE") and \
                    (m.get_srcSystem(), m.get_srcComponent()) != (1, 1):
                # ArduSub forwards the frames it receives from the pods (and from foreign senders) to this
                # link: only what the autopilot itself (sysid 1, compid 1) reports counts here
                continue
            with self.lock:
                if t == "BATTERY_STATUS":
                    self.battery[m.id] = m
                    self.battery_t[m.id] = time.time()
                elif t == "STATUSTEXT":
                    self.texts.append((time.time(), m.severity, m.text))
                elif t == "HEARTBEAT" and m.get_srcComponent() == 1:
                    self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.custom_mode = m.custom_mode
                elif t == "PARAM_VALUE":
                    self.params[m.param_id.rstrip("\x00")] = m.param_value
                elif t == "SYS_STATUS":
                    self.sys_health = m.onboard_control_sensors_health
                    if not (m.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_BATTERY):
                        self.sys_health_low_samples += 1
                    self.sys_present = m.onboard_control_sensors_present
                elif t == "COMMAND_ACK":
                    self.acks[m.command] = m.result

    def texts_since(self, t0):
        with self.lock:
            return [x[2] for x in self.texts if x[0] >= t0]

    def get_battery(self, idx):
        with self.lock:
            return self.battery.get(idx), self.battery_t.get(idx)


class Results:
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append((name, bool(ok), detail))
        print("%-4s %s %s" % ("PASS" if ok else "FAIL", name, ("- " + detail) if detail else ""), flush=True)
        return ok


def arm(conn, vehicle, on, timeout, force=False):
    """Normal (or forced) arm/disarm through MAV_CMD_COMPONENT_ARM_DISARM; True when the state was reached."""
    conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if on else 0, 21196 if force else 0,
                               0, 0, 0, 0, 0)
    return wait_for(lambda: vehicle.armed == on, timeout)


def set_param(conn, name, value):
    conn.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def wait_for(cond, timeout, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def read_skipchk(ardusub, defaults, instance=1):
    """Start a throw-away SITL with the given defaults files and read ARMING_SKIPCHK from it."""
    work = tempfile.mkdtemp(prefix="pod_battery_skipchk_")
    proc = subprocess.Popen([ardusub, "-w", "--model", "vectored", "--speedup", "1", "--defaults", defaults,
                             "-I%d" % instance, "--sysid", "1"], cwd=work,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    value = None
    try:
        conn = None
        for _ in range(40):
            try:
                conn = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % (5760 + 10 * instance))
                break
            except OSError:
                time.sleep(0.5)
        if conn is not None:
            conn.wait_heartbeat(timeout=20)
            for _ in range(10):
                conn.mav.param_request_read_send(1, 1, b"ARMING_SKIPCHK", -1)
                m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                if m is not None and m.param_id.rstrip("\x00") == "ARMING_SKIPCHK":
                    value = int(m.param_value)
                    break
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(work, ignore_errors=True)
    return value


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardusub", default=ARDUSUB)
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    ap.add_argument("--timeout-ms", type=int, default=1000, help="PBAT_TIMEOUT_MS under test")
    ap.add_argument("--scenario", choices=("functional", "load"), default="functional",
                    help="functional: all checks; load: pods at 3x rate + foreign NAMED_VALUE_INT traffic")
    ap.add_argument("--load-seconds", type=int, default=30)
    return ap.parse_args()


def main():
    args = parse_args()
    if not os.path.exists(args.ardusub):
        print("ardusub SITL binary not found: %s (run ./waf configure --board sitl && ./waf sub)" % args.ardusub)
        return 2

    res = Results()
    if args.scenario == "functional":
        # ARMING_SKIPCHK before (ArduSub default) and after (pod_battery.parm): only the battery bit may change
        before = read_skipchk(args.ardusub, SUB_PARM)
        after = read_skipchk(args.ardusub, ",".join([SUB_PARM, os.path.join(APPLETS, "pod_battery.parm")]))
        print("INFO ARMING_SKIPCHK before: %s" % (describe_mask(before) if before is not None else "n/a"), flush=True)
        print("INFO ARMING_SKIPCHK after : %s" % (describe_mask(after) if after is not None else "n/a"), flush=True)
        res.check("ARMING_SKIPCHK: only the battery bit (256) is added to the ArduSub default",
                  before is not None and after is not None and after == before | SKIPCHK_BATTERY_BIT and
                  not before & SKIPCHK_BATTERY_BIT and after == SKIPCHK_EXPECTED,
                  "before %s after %s" % (before, after))
        res.check("ARMING_SKIPCHK: RC and board voltage checks are still active",
                  after is not None and not after & (1 << 6) and not after & (1 << 7))

    work = tempfile.mkdtemp(prefix="pod_battery_sitl_")
    os.makedirs(os.path.join(work, "scripts", "modules", "MAVLink"))
    shutil.copy(os.path.join(APPLETS, "pod_battery.lua"), os.path.join(work, "scripts"))
    for f in os.listdir(MODULES):
        if f.endswith(".lua"):
            shutil.copy(os.path.join(MODULES, f), os.path.join(work, "scripts", "modules", "MAVLink"))
    extra = os.path.join(work, "test.parm")
    with open(extra, "w") as f:
        f.write("LOG_DISARMED 1\nRC9_OPTION 153\nSCR_DEBUG_OPTS 8\nPBAT_TIMEOUT_MS %d\n" % args.timeout_ms)
    defaults = ",".join([SUB_PARM, os.path.join(APPLETS, "pod_battery.parm"), extra])

    log = open(os.path.join(work, "ardusub.out"), "w")
    proc = subprocess.Popen([args.ardusub, "-w", "--model", "vectored", "--speedup", "1", "--defaults", defaults,
                             "-I0", "--sysid", "1", "--serial1", "tcp:5762"],
                            cwd=work, stdout=log, stderr=subprocess.STDOUT)
    vehicle = runner = flood = None
    load_ctx = {}
    try:
        conn = None
        for _ in range(60):
            try:
                conn = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, source_component=190)
                break
            except OSError:
                time.sleep(0.5)
        if conn is None:
            print("cannot connect to SITL on tcp:5760")
            return 2
        conn.wait_heartbeat(timeout=30)
        vehicle = Vehicle(conn)
        vehicle.start()
        for msgid in (147, 1):  # BATTERY_STATUS, SYS_STATUS at 10 Hz
            conn.mav.command_long_send(1, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, 100000, 0, 0, 0, 0, 0)

        emu = mavutil.mavlink_connection("tcp:127.0.0.1:5762", source_system=1, source_component=180)
        pod1 = Pod(emu, master=True)
        pod2 = Pod(emu, master=False)
        t_start = time.time()
        res.check("script started", wait_for(lambda: any("PodBatt: started" in x for x in vehicle.texts_since(0)), 40),
                  "; ".join(vehicle.texts_since(0)[-3:]))

        # truncation really happens on the wire (a real MAVLink 2 sender drops trailing zeros)
        wl = pod1.frame_payload_len()
        res.check("pod frames are truncated MAVLink 2 (payload < 54)", wl < 54, "payload %d of 54 bytes" % wl)

        if args.scenario == "load":
            runner, flood = run_load(args, res, vehicle, emu, pod1, pod2, load_ctx)
            raise ScenarioDone()

        # arming with both pods absent since boot (battery check skipped by ARMING_SKIPCHK)
        res.check("arming accepted with both pods absent from boot",
                  arm(conn, vehicle, True, 12), "; ".join(vehicle.texts_since(t_start)[-3:]))
        arm(conn, vehicle, False, 8)

        runner = PodRunner([pod1, pod2])
        runner.start()

        # a) batteries 0 and 1 reported by ArduSub with the right values
        def have_both():
            return all(vehicle.get_battery(i)[0] is not None and vehicle.get_battery(i)[0].voltages[0] != 65535 and
                       vehicle.get_battery(i)[0].voltages[0] > 0 for i in (0, 1))
        res.check("a) ArduSub reports BATTERY_STATUS 0 and 1 with a voltage", wait_for(have_both, 20))
        for idx, pod in ((0, pod1), (1, pod2)):
            b, _ = vehicle.get_battery(idx)
            if b is None:
                res.check("a) battery %d values" % idx, False, "no message")
                continue
            ok = (abs(b.voltages[0] - pod.voltage_mv) <= 2 and abs(b.current_battery - pod.current_ca) <= 2 and
                  b.current_consumed == pod.consumed_mah and abs(b.temperature - pod.temp_cdeg) <= 2 and
                  b.battery_remaining == -1)
            res.check("a) battery %d values (V/I/mAh/T, no SOC)" % idx, ok,
                      "mV %d (pod %d) cA %d (pod %d) mAh %d (pod %d) cdegC %d (pod %d) remaining %d" % (
                          b.voltages[0], pod.voltage_mv, b.current_battery, pod.current_ca, b.current_consumed,
                          pod.consumed_mah, b.temperature, pod.temp_cdeg, b.battery_remaining))
        res.check("a) battery monitors healthy (SYS_STATUS)", wait_for(
            lambda: vehicle.sys_health & MAV_SYS_STATUS_SENSOR_BATTERY, 5))

        # d) foreign senders are ignored
        v_before = vehicle.get_battery(0)[0].voltages[0]
        c_before = vehicle.get_battery(0)[0].current_battery
        t_d = time.time()
        for _ in range(30):
            for sysid, compid in ((1, 190), (2, 180), (1, 100), (255, 181)):
                foreign = mavutil.mavlink.MAVLink(emu, srcSystem=sysid, srcComponent=compid)
                foreign.battery_status_send(0, 1, 2, 9999, [12345] + [65535] * 9, 4321, 99999, -1, -1, 0, 1,
                                            [0, 0, 0, 0], 0, 0)
                foreign.named_value_int_send(1, b"PWR_MODE", 4)
                foreign.named_value_int_send(1, b"PWR_RSN", 99)
            time.sleep(0.1)
        b0, _ = vehicle.get_battery(0)
        b1, _ = vehicle.get_battery(1)
        res.check("d) messages with another sysid/compid are ignored (battery 0 unchanged)",
                  b0.voltages[0] == v_before and b0.current_battery == c_before and b0.current_consumed == pod1.consumed_mah,
                  "mV %d (expected %d) cA %d (expected %d)" % (b0.voltages[0], v_before, b0.current_battery, c_before))
        res.check("d) ... and battery 1 unchanged",
                  b1.voltages[0] == pod2.voltage_mv and b1.current_battery == pod2.current_ca)
        res.check("d) ... and no power-state text from foreign senders",
                  not any("power mode NONE reason 99" in x for x in vehicle.texts_since(t_d)))
        # a mismatched id/compid (pod 181 claiming battery id 0) must not swap the packs
        liar = mavutil.mavlink.MAVLink(emu, srcSystem=1, srcComponent=181)
        for _ in range(10):
            liar.battery_status_send(0, 1, 2, 1000, [11111] + [65535] * 9, 100, 5, -1, -1, 0, 1, [0, 0, 0, 0], 0, 0)
            time.sleep(0.1)
        b1b, _ = vehicle.get_battery(1)
        res.check("d) compid/id mismatch refused (no BAT1/BAT2 swap)", b1b.voltages[0] != 11111,
                  "battery 1 mV %d" % b1b.voltages[0])

        # c) power mode change -> text message and PWRM log line
        t_c = time.time()
        pod1.mode, pod1.reason = 2, 7
        res.check("c) PWR_MODE change gives a text message", wait_for(
            lambda: any("BAT1 power mode SINGLE_BAT1 reason 7" in x for x in vehicle.texts_since(t_c)), 6),
            "; ".join(vehicle.texts_since(t_c)[-3:]))
        t_c2 = time.time()
        pod2.mode, pod2.reason = 3, 11
        res.check("c) change on the second pod is reported for BAT2", wait_for(
            lambda: any("BAT2 power mode SINGLE_BAT2 reason 11" in x for x in vehicle.texts_since(t_c2)), 6))

        # b) stop a pod while armed: unhealthy within the timeout, no failsafe action
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        armed_ok = wait_for(lambda: vehicle.armed, 8)
        res.check("arming succeeds with both pod batteries healthy", armed_ok,
                  "acks %s; %s" % (vehicle.acks.get(MAV_CMD_COMPONENT_ARM_DISARM),
                                   "; ".join(vehicle.texts_since(t_start)[-4:])))
        mode_before = vehicle.custom_mode
        t_b = time.time()
        pod2.alive = False
        unhealthy_at = []

        def bat2_unhealthy():
            b, t = vehicle.get_battery(1)
            if b is not None and b.voltages[0] in (0, 65535) and time.time() - t < 1.0:
                unhealthy_at.append(time.time() - t_b)
                return True
            return False
        res.check("b) pod stopped -> battery 2 not healthy", wait_for(bat2_unhealthy, args.timeout_ms / 1000.0 + 3.0),
                  "after %.2f s (timeout %.2f s)" % (unhealthy_at[0] if unhealthy_at else -1, args.timeout_ms / 1000.0))
        res.check("b) ... SYS_STATUS battery health bit cleared", wait_for(
            lambda: not (vehicle.sys_health & MAV_SYS_STATUS_SENSOR_BATTERY), 4))
        # armed + unhealthy for more than 5 s: ArduPilot's own "missing" warning (action 0 = no action in Sub)
        res.check("b) ... ArduPilot warns 'Battery 2 is missing' after its 5 s health timeout (armed)", wait_for(
            lambda: any("Battery 2 is missing" in x for x in vehicle.texts_since(t_b)), 8),
            "; ".join(x for x in vehicle.texts_since(t_b) if "Battery" in x))
        time.sleep(1.0)
        texts_b = vehicle.texts_since(t_b)
        res.check("b) ... text message about the silent pod", any("BAT2 pod silent" in x for x in texts_b),
                  "; ".join(texts_b[-3:]))
        bad = [x for x in texts_b if "ailsafe" in x or "isarm" in x]
        res.check("b) ... no failsafe action: still armed, same mode, no failsafe text",
                  vehicle.armed and vehicle.custom_mode == mode_before and not bad,
                  "armed=%s mode %s->%s texts=%s" % (vehicle.armed, mode_before, vehicle.custom_mode, bad))
        b0_now, t0_now = vehicle.get_battery(0)
        res.check("b) ... battery 1 (other pod) unaffected", b0_now.voltages[0] == pod1.voltage_mv and
                  time.time() - t0_now < 1.0)

        # e) arming with an unhealthy battery: accepted, the battery check is skipped by ARMING_SKIPCHK
        res.check("disarm", arm(conn, vehicle, False, 8))
        t_e = time.time()
        res.check("e) normal arming ACCEPTED while a pod (battery 2) is silent", arm(conn, vehicle, True, 8),
                  "; ".join(vehicle.texts_since(t_e)[-3:]))
        res.check("e) ... no 'Battery N unhealthy' arming message",
                  not any("unhealthy" in x for x in vehicle.texts_since(t_e)))
        arm(conn, vehicle, False, 8)
        # the pod comes back
        pod2.alive = True
        res.check("pod back -> battery 2 reports the pod voltage again", wait_for(
            lambda: vehicle.get_battery(1)[0].voltages[0] == pod2.voltage_mv, 8))
        time.sleep(2.0)
        latched = not (vehicle.sys_health & MAV_SYS_STATUS_SENSOR_BATTERY)
        print("INFO SYS_STATUS battery health bit after the pod is back (failsafe 'missing' fired while armed): %s" % (
            "still cleared (latched)" if latched else "set"), flush=True)
        conn.mav.command_long_send(1, 1, 42651, 0, 2, 100, 0, 0, 0, 0, 0)  # MAV_CMD_BATTERY_RESET, battery 2
        res.check("pod back -> health bit set again after MAV_CMD_BATTERY_RESET (or already set)", wait_for(
            lambda: vehicle.sys_health & MAV_SYS_STATUS_SENSOR_BATTERY, 6),
            "sys_health 0x%x" % vehicle.sys_health)

        # 2) low-voltage failsafe vs a silent pod (published as 0 V, healthy=false)
        for name, value in (("BATT_LOW_VOLT", 40), ("BATT_CRT_VOLT", 35), ("BATT_LOW_TIMER", 2),
                            ("BATT2_LOW_VOLT", 40), ("BATT2_CRT_VOLT", 35), ("BATT2_LOW_TIMER", 2)):
            set_param(conn, name, value)
        time.sleep(1.0)

        def silent_pod_scenario(label):
            arm(conn, vehicle, True, 8)
            mode0 = vehicle.custom_mode
            t0 = time.time()
            pod2.alive = False
            wait_for(lambda: any("Battery 2 is missing" in x for x in vehicle.texts_since(t0)), 10)
            time.sleep(6.0)  # well past BATT_LOW_TIMER (2 s)
            texts = vehicle.texts_since(t0)
            low = [x for x in texts if "is low" in x or "is critical" in x]
            res.check("%s: silent pod (0 V, unhealthy) gives no low/critical voltage failsafe" % label,
                      not low and vehicle.armed and vehicle.custom_mode == mode0,
                      "armed=%s mode %s->%s texts=%s" % (vehicle.armed, mode0, vehicle.custom_mode,
                                                         [x for x in texts if "Battery" in x]))
            pod2.alive = True
            wait_for(lambda: vehicle.get_battery(1)[0].voltages[0] == pod2.voltage_mv, 8)
            conn.mav.command_long_send(1, 1, 42651, 0, 3, 100, 0, 0, 0, 0, 0)
            time.sleep(1.5)
            arm(conn, vehicle, False, 8)

        silent_pod_scenario("S1 BATT_LOW_VOLT=40, actions 0")
        for name in ("BATT_FS_LOW_ACT", "BATT2_FS_LOW_ACT"):
            set_param(conn, name, 3)   # Surface
        for name in ("BATT_FS_CRT_ACT", "BATT2_FS_CRT_ACT"):
            set_param(conn, name, 2)   # Disarm
        time.sleep(1.0)
        silent_pod_scenario("S2 actions Surface/Disarm")

        # S3: the pod is alive and reports a REAL low voltage, actions non-zero: the failsafe acts
        arm(conn, vehicle, True, 8)
        mode0 = vehicle.custom_mode
        t3 = time.time()
        pod2.voltage_mv = 39000
        res.check("S3 real low voltage (39 V < BATT_LOW_VOLT 40) with action Surface: warning text", wait_for(
            lambda: any("Battery 2 is low" in x for x in vehicle.texts_since(t3)), 10),
            "; ".join(x for x in vehicle.texts_since(t3) if "Battery" in x))
        time.sleep(1.5)
        print("INFO S3 real low voltage with BATT2_FS_LOW_ACT=3 (Surface): mode %s -> %s, armed=%s "
              "(the SITL vehicle is already at the surface, so Surface = %d completes at once into ALT_HOLD = 2, "
              "ArduSub/mode_surface.cpp)" % (mode0, vehicle.custom_mode, vehicle.armed, SUB_MODE_SURFACE), flush=True)
        res.check("S3 ... the action fires (mode leaves MANUAL: Surface requested)", vehicle.custom_mode != mode0)
        pod2.voltage_mv = 49870
        for name in ("BATT_FS_LOW_ACT", "BATT2_FS_LOW_ACT", "BATT_FS_CRT_ACT", "BATT2_FS_CRT_ACT"):
            set_param(conn, name, 0)
        time.sleep(1.0)
        conn.mav.command_long_send(1, 1, 42651, 0, 3, 100, 0, 0, 0, 0, 0)
        arm(conn, vehicle, False, 8)
        time.sleep(1.0)
    except ScenarioDone:
        pass
    finally:
        if runner:
            runner.stop_flag = True
        if flood:
            flood.stop_flag = True
        if vehicle:
            vehicle.stop_flag = True
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()

    # DataFlash log: BAT rows (with the health field H), PBAT statistics, PWRM lines, Lua statistics
    logs = []
    for d, _, files in os.walk(work):
        logs += [os.path.join(d, f) for f in files if f.endswith(".BIN")]
    bat, pwrm, scr, pbat = {0: [], 1: []}, [], [], {180: [], 181: []}
    if logs:
        reader = DFReader.DFReader_binary(sorted(logs)[-1])
        while True:
            m = reader.recv_match(type=["BAT", "PWRM", "SCR", "PBAT"])
            if m is None:
                break
            if m.get_type() == "SCR":
                scr.append((m.Name, m.Runtime, m.Total_mem, m.Run_mem))
            elif m.get_type() == "BAT":
                bat.setdefault(m.Inst, []).append((m.Volt, m.Curr, m.CurrTot, m.H))
            elif m.get_type() == "PBAT":
                pbat.setdefault(m.Comp, []).append((m.Healthy, m.AgeMs, m.RxB, m.RxN, m.Ign, m.Cap))
            else:
                pwrm.append((m.Comp, m.Mode, m.Rsn))

    if args.scenario == "load":
        for comp, pod in ((180, load_ctx.get("pod1")), (181, load_ctx.get("pod2"))):
            rows = pbat.get(comp, [])
            if not rows or pod is None:
                res.check("load: PBAT rows for compid %d" % comp, False)
                continue
            rx = rows[-1][2]
            sent = pod.sent_battery
            lost = sent - rx
            print("INFO load compid %d: BATTERY_STATUS sent %d, received by the script %d, lost %d (%.2f %%); "
                  "NAMED_VALUE_INT received %d of %d; decode-budget hits %d" % (
                      comp, sent, rx, lost, 100.0 * lost / max(sent, 1), rows[-1][3], pod.sent_named, rows[-1][5]),
                  flush=True)
            res.check("load: compid %d lost at most 1 %% of the BATTERY_STATUS" % comp, lost <= max(2, sent * 0.01))
            res.check("load: compid %d no NAMED_VALUE_INT lost beyond 1 %%" % comp,
                      pod.sent_named - rows[-1][3] <= max(2, pod.sent_named * 0.01))
            res.check("load: compid %d never reported unhealthy after start" % comp,
                      all(h == 1 for h, age, *_ in rows[load_ctx.get("skip_rows", 3):-4]),
                      "unhealthy rows %d" % sum(1 for h, *_ in rows[load_ctx.get("skip_rows", 3):-4] if h == 0))
    else:
        res.check("a) log has BAT rows for battery 0 and 1 with the pod voltage",
                  any(abs(v - 50.123) < 0.01 for v, _, _, _ in bat.get(0, [])) and
                  any(abs(v - 49.87) < 0.01 for v, _, _, _ in bat.get(1, [])),
                  "BAT0 rows %d, BAT1 rows %d" % (len(bat.get(0, [])), len(bat.get(1, []))))
        res.check("a) log BAT current of battery 0 is 12.34 A",
                  any(abs(c - 12.34) < 0.05 for _, c, _, _ in bat.get(0, [])))
        res.check("c) log has PWRM lines for the two changes", (180, 2, 7) in pwrm and (181, 3, 11) in pwrm, str(pwrm[:8]))
        unhealthy = [r for r in bat.get(1, []) if r[3] == 0]
        res.check("3) BAT field H records the health: unhealthy rows exist and all carry 0 V",
                  bool(unhealthy) and all(abs(r[0]) < 0.001 for r in unhealthy) and
                  any(r[3] == 1 and r[0] > 40 for r in bat.get(1, [])),
                  "BAT1 rows %d, H=0 rows %d" % (len(bat.get(1, [])), len(unhealthy)))
        res.check("3) PBAT rows (1 Hz) carry Healthy/AgeMs/counters for both pods",
                  len(pbat.get(180, [])) > 10 and len(pbat.get(181, [])) > 10 and
                  any(h == 0 for h, *_ in pbat.get(181, [])) and any(h == 1 for h, *_ in pbat.get(181, [])))

    mine = [r for r in scr if "pod_battery" in str(r[0])]
    if mine:
        print("INFO Lua stats for pod_battery.lua (%d samples): max runtime %d us, max run mem %d bytes, "
              "max total heap in use %d bytes (SCR_HEAP_SIZE default 204800)" % (
                  len(mine), max(r[1] for r in mine), max(r[3] for r in mine), max(r[2] for r in mine)), flush=True)
    res.check("Lua statistics logged for the applet", bool(mine))

    print("\nSummary: %d/%d checks passed (scratch dir %s)" % (
        sum(1 for r in res.rows if r[1]), len(res.rows), work))
    if not args.keep and all(r[1] for r in res.rows):
        shutil.rmtree(work, ignore_errors=True)
    return 0 if all(r[1] for r in res.rows) else 1


if __name__ == "__main__":
    sys.exit(main())
