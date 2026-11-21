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

    def send_heartbeat(self):
        self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                mavutil.mavlink.MAV_STATE_ACTIVE)


class PodRunner(threading.Thread):
    def __init__(self, pods):
        super().__init__(daemon=True)
        self.pods = pods
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
            time.sleep(0.1)


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
        self.lock = threading.Lock()

    def run(self):
        while not self.stop_flag:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            t = m.get_type()
            if t in ("BATTERY_STATUS", "SYS_STATUS", "STATUSTEXT", "COMMAND_ACK") and \
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
                elif t == "SYS_STATUS":
                    self.sys_health = m.onboard_control_sensors_health
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


def wait_for(cond, timeout, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardusub", default=ARDUSUB)
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    ap.add_argument("--timeout-ms", type=int, default=1000, help="PBAT_TIMEOUT_MS under test")
    return ap.parse_args()


def main():
    args = parse_args()
    if not os.path.exists(args.ardusub):
        print("ardusub SITL binary not found: %s (run ./waf configure --board sitl && ./waf sub)" % args.ardusub)
        return 2

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

    res = Results()
    log = open(os.path.join(work, "ardusub.out"), "w")
    proc = subprocess.Popen([args.ardusub, "-w", "--model", "vectored", "--speedup", "1", "--defaults", defaults,
                             "-I0", "--sysid", "1", "--serial1", "tcp:5762"],
                            cwd=work, stdout=log, stderr=subprocess.STDOUT)
    vehicle = runner = None
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

        # e) arming with an unhealthy battery
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        res.check("disarm", wait_for(lambda: not vehicle.armed, 8))
        t_e = time.time()
        vehicle.acks.pop(MAV_CMD_COMPONENT_ARM_DISARM, None)
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        wait_for(lambda: MAV_CMD_COMPONENT_ARM_DISARM in vehicle.acks, 5)
        time.sleep(1.0)
        texts_e = vehicle.texts_since(t_e)
        res.check("e) normal arming refused while a battery is unhealthy",
                  not vehicle.armed and vehicle.acks.get(MAV_CMD_COMPONENT_ARM_DISARM) != 0,
                  "ack result %s; texts: %s" % (vehicle.acks.get(MAV_CMD_COMPONENT_ARM_DISARM), "; ".join(texts_e[-3:])))
        res.check("e) ... with a pre-arm message naming the battery",
                  any("Battery 2" in x and "unhealthy" in x for x in texts_e), "; ".join(texts_e[-3:]))
        vehicle.acks.pop(MAV_CMD_COMPONENT_ARM_DISARM, None)
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)  # forced
        forced = wait_for(lambda: vehicle.armed, 6)
        print("INFO forced arming (param2=21196) with an unhealthy battery: %s" % (
            "armed" if forced else "refused"), flush=True)
        if forced:
            conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
            wait_for(lambda: not vehicle.armed, 6)
        # the pod comes back: arming is possible again
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
        vehicle.acks.pop(MAV_CMD_COMPONENT_ARM_DISARM, None)
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        res.check("e) arming possible again once the battery is healthy", wait_for(lambda: vehicle.armed, 8))
        conn.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        wait_for(lambda: not vehicle.armed, 6)
        time.sleep(1.0)
    finally:
        if runner:
            runner.stop_flag = True
        if vehicle:
            vehicle.stop_flag = True
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()

    # DataFlash log: BAT rows for both batteries and the PWRM lines
    logs = []
    for d, _, files in os.walk(work):
        logs += [os.path.join(d, f) for f in files if f.endswith(".BIN")]
    bat, pwrm, scr = {0: [], 1: []}, [], []
    if logs:
        reader = DFReader.DFReader_binary(sorted(logs)[-1])
        while True:
            m = reader.recv_match(type=["BAT", "PWRM", "SCR"])
            if m is None:
                break
            if m.get_type() == "SCR":
                scr.append((m.Name, m.Runtime, m.Total_mem, m.Run_mem))
            elif m.get_type() == "BAT":
                bat.setdefault(m.Inst, []).append((m.Volt, m.Curr, m.CurrTot))
            else:
                pwrm.append((m.Comp, m.Mode, m.Rsn))
    res.check("a) log has BAT rows for battery 0 and 1 with the pod voltage",
              any(abs(v - 50.123) < 0.01 for v, _, _ in bat.get(0, [])) and
              any(abs(v - 49.87) < 0.01 for v, _, _ in bat.get(1, [])),
              "BAT0 rows %d, BAT1 rows %d" % (len(bat.get(0, [])), len(bat.get(1, []))))
    res.check("a) log BAT current of battery 0 is 12.34 A",
              any(abs(c - 12.34) < 0.05 for _, c, _ in bat.get(0, [])))
    res.check("c) log has PWRM lines for the two changes", (180, 2, 7) in pwrm and (181, 3, 11) in pwrm, str(pwrm[:8]))

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
