# Pod Battery

`pod_battery.lua` feeds the two ArduSub battery monitors from the two DeepEx pods (ESP32 distribution board firmware
0.8.8, `docs/MAVLINK_BATTERY.md` in `DeepEx/distribution_board`) and reports the pod power mode. Decision: ADR-0024
(`DeepEx/DeepEx-System`, accepted 21/11/2026). It never acts on the motors: the single-battery limit of ADR-0016 is **not** part of
this script.

## What the pods send

UDP to the Raspberry Pi (port 14560, BlueOS MAVLink router endpoint), MAVLink 2, sysid 1:

| compid | pod | `BATTERY_STATUS.id` | battery monitor |
| --- | --- | --- | --- |
| 180 | master / BAT1 | 0 | `BATT_` (index 0) |
| 181 | slave / BAT2 | 1 | `BATT2_` (index 1) |

`HEARTBEAT` 1 Hz, `BATTERY_STATUS` 10 Hz, `NAMED_VALUE_INT` `PWR_MODE` / `PWR_RSN` 2 Hz. The script accepts only sysid
`PBAT_SYSID` (1) and compid 180/181; everything else is dropped before decoding. A message whose `id` does not match its
compid is refused (a mapping error would swap BAT1 and BAT2) and reported once.

**Security: there is no authentication.** The only filter is sysid and compid, which anybody on the network can forge (no MAVLink
signing). A forged `BATTERY_STATUS` shows false battery data, and a forged low voltage would trigger a failsafe action if one were
configured. Acceptable only on a closed vehicle LAN: do not expose the BlueOS MAVLink endpoint that receives the pods (UDP 14560) to
any other network, and keep the battery failsafe actions at 0 (see below).

## What the script does

- **Per pack** (`battery:handle_scripting`), 40 Hz:
  - `voltage` = `voltages[0]` in V, **invalid if 65535**; `current_amps` = `current_battery` / 100, **invalid if -1**,
    positive on discharge; `consumed_mah` = `current_consumed`, **invalid if -1** (then ArduPilot integrates the current
    itself); `temperature` = `temperature` / 100, **invalid if 32767**. Invalid values are sent as NaN ("not known").
  - `healthy = false` if the voltage is not valid, or no `BATTERY_STATUS` arrived for more than `PBAT_TIMEOUT_MS`. An
    unhealthy pack is published with 0 V (never a stale number as if it were live).
  - `capacity_remaining_pct` is **not** set (the pods have no SOC; `battery_remaining` is -1).
- **Power mode:** the last `PWR_MODE` (0 UNKNOWN, 1 DUAL, 2 SINGLE_BAT1, 3 SINGLE_BAT2, 4 NONE) and `PWR_RSN` per pod. At
  every change: a DataFlash line `PWRM` (`Comp`, `Mode`, `Rsn`) and an INFO text `PodBatt: BATn power mode <name> reason <n>`
  (unchanged states are not repeated). The pod's current budget is not sent and not used.
- **Texts:** `PodBatt: started`, `BATn no data from pod yet`, `BATn pod silent for >N ms`, `BATn voltage not valid`,
  `BATn pod battery online`, and a warning if `BATT_MONITOR`/`BATT2_MONITOR` is not 29.

### Log

- The `BAT` line of ArduPilot (instance 0 and 1) **does record the health**: field `H` (`health`, 1 = healthy, 0 = not), next to
  `Volt`, `Curr`, `CurrTot`, `Temp`. Analysis must discard the rows with `H = 0`: an unhealthy pod is published as 0 V, and
  those 0 V are not a measurement (SITL: every `H = 0` row carries 0 V).
- `PWRM` (`Comp`, `Mode`, `Rsn`): at every power-mode change.
- `PBAT` (1 Hz, one row per pod): `Comp`, `Healthy`, `AgeMs` (ms since the last `BATTERY_STATUS`, -1 = none yet), `RxB`
  (BATTERY_STATUS accepted), `RxN` (NAMED_VALUE_INT accepted), `Ign` (id/compid mismatches), `Cap` (calls that used up the
  decode budget). The queue API has no drop counter: a loss shows as `RxB` growing slower than the pod's 10 Hz.

## Parameters of the script (table key 118, prefix `PBAT_`)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `PBAT_ENABLE` | 1 | 0 = the script does nothing (the scripting batteries then become unhealthy after 5 s) |
| `PBAT_TIMEOUT_MS` | 1000 | pack unhealthy if no `BATTERY_STATUS` for longer than this |
| `PBAT_SYSID` | 1 | MAVLink system id accepted from the pods |

## Vehicle parameters

Reference file: `pod_battery.parm` (not applied automatically; do not apply them to the ROV without review).

| Parameter | Value | Why |
| --- | --- | --- |
| `SCR_ENABLE` | 1 | scripting on (reboot) |
| `BATT_MONITOR`, `BATT2_MONITOR` | 29 | scripting backend |
| `BATT_CAPACITY`, `BATT2_CAPACITY` | 0 | the pod mAh restart at every pod boot: a capacity would give a false 100 % |
| `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`, `BATT2_FS_LOW_ACT`, `BATT2_FS_CRT_ACT` | **0** | warning only, see "Failsafe" |
| `ARMING_SKIPCHK` | 16777023 | the ROV must be armable with a pod off, see "Arming" |

### Load and memory

| | |
| --- | --- |
| Messages from the pods (real rate) | `BATTERY_STATUS` 10 Hz x 2 pods = 20/s; `NAMED_VALUE_INT` 2 names x 2 Hz x 2 pods = 8/s; **28 messages/s** (HEARTBEAT is not received by the script) |
| Script period | **25 ms** (40 Hz) |
| `mavlink:init` queue | **25** messages, the maximum the API allows (a second argument of 2 message ids) |
| Work per call | at most 3 CRC checks/decodes and 60 queue pops (VM budget), 120 decodes/s of capacity against 28/s needed |

`SCR_HEAP_SIZE` defaults to 200 KiB on Linux boards and SITL (`AP_Scripting.cpp`), which is enough: the whole Lua heap in use
(interpreter included) peaked at about 60 KB, 12 KB used per run (`SCR` log rows, `SCR_DEBUG_OPTS` bit 3). No change needed.

One run of a script is limited to `SCR_VM_I_COUNT` (10000 by default) VM instructions and ArduSub **stops the script** that goes over
(`Lua: ./scripts/pod_battery.lua exceeded time limit`). The first version did exactly that under load; the script now drops
messages of other senders after a few instructions, decodes at most 4 per call, and bounds work per call. The earlier claim that the C parser only hands over verified frames was incorrect: this fork explicitly passes BAD_CRC packets to scripting. The applet now verifies CRC before decoding, with a three-message budget.

SITL load test (`Tools/scripts/deepex_pod_battery_sitl.py --scenario load`, 60 s): pods at **3x** the real rate (30 Hz per pod,
84 msg/s) plus 125 `NAMED_VALUE_INT`/s from other components (4 compids with sysid 1 and one foreign system claiming compid 180):
**0 of 1809 `BATTERY_STATUS` and 0 of 724 `NAMED_VALUE_INT` lost per pod** (counted by `PBAT` against what the emulator sent), no
false timeout, battery health never dropped, longest run 4 ms.

## Install

Copy to the scripts directory of the vehicle (the directory is `./scripts` relative to the working directory of `ardusub` on Linux
boards, `SCRIPTING_DIRECTORY` in `lua_common_defs.h`):

```
scripts/pod_battery.lua
scripts/modules/MAVLink/mavlink_msgs.lua
scripts/modules/MAVLink/mavlink_msg_BATTERY_STATUS.lua
scripts/modules/MAVLink/mavlink_msg_NAMED_VALUE_INT.lua
```

The MAVLink modules are **not** embedded in the firmware (no ROMFS entry for them); they must be present in `scripts/modules/MAVLink/`.

### BlueOS 1.4.3 on the vehicle (read-only inspection, 2026-11-21)

Seen on the Pi (BlueOS `bluerobotics/blueos-core:1.4.3`, Raspberry Pi, Debian 12): the autopilot is the file
`ardupilot_navigator64` (not named `ardusub`), started by the `blueos-core` container as
`/root/.config/ardupilot-manager/firmware/ardupilot_navigator64 -A udp:127.0.0.1:8852 --log-directory .../firmware/logs/
--storage-directory .../firmware/storage/ -C /dev/ttyAMA0 -B /dev/ttyAMA2 -E /dev/ttyAMA3 -F /dev/ttyAMA4`, with
`cwd = /root/.config/ardupilot-manager/firmware`. The scripts directory is therefore
`/root/.config/ardupilot-manager/firmware/scripts/` in the container; it **already exists and is empty**. `/root/.config` of the container is a
bind mount of the host directory **`/root/.config/blueos`** (not `/home/pi/...`), so on the host (as root, `sudo`) the scripts go in

```
/root/.config/blueos/ardupilot-manager/firmware/scripts/            (pod_battery.lua)
/root/.config/blueos/ardupilot-manager/firmware/scripts/modules/MAVLink/
```

Installed firmware: `ArduSub V4.8.0-dev (4bffa1cb)` (`AUTOPILOT_VERSION` custom version `4bffa1cb`; binary SHA-256
`4f16b33d5ba04cbc0cbaafdf6762924cea82bd42919978b0a4c76a1419cef08f`), one commit before `e8f381590e`; the two differ only in `AP_VESC` and
`ArduSub/VESC_CAN_V1.md`, so everything this script depends on (scripting, battery monitor, arming, MAVLink definitions) is identical.
Installing a firmware (`install_firmware_from_file`, also `waf --upload-blueos`) only copies the ELF into that folder and never touches `scripts/`, so the
scripts survive firmware updates (this is from the BlueOS 1.4.3 source: `FirmwareInstall.install_firmware` is a `shutil.copy` of the ELF); it was **not** tested by updating.

**The router has no endpoint for the pods.** `mavlink-server` listens on `127.0.0.1:8852` (autopilot), `0.0.0.0:14550` (UDP server), `127.0.0.1:14001`,
`0.0.0.0:14660` and TCP `127.0.0.1:5777`; **nothing listens on UDP 14560**. The pods' frames do reach the Pi (a passive `tcpdump` on the Pi
showed them on 14560: HEARTBEAT 21 bytes, BATTERY_STATUS 53, NAMED_VALUE_INT 27-28) but nothing consumes them. In BlueOS, add an endpoint of type
**UDP Server**, address `0.0.0.0`, port **14560** (the pods send to it) before installing the script.

Read-only checks, from the BlueOS "red-pill" terminal (host shell) or SSH as `pi` (`sudo` works without a password):

```
docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
docker inspect blueos-core --format '{{range .Mounts}}{{.Type}} {{.Source}} -> {{.Destination}}{{"\n"}}{{end}}'
curl -s http://localhost/version-chooser/v1.0/version/current
for p in $(pgrep -f '^/root/.config.*ardupilot_navigator64'); do sudo tr '\0' ' ' < /proc/$p/cmdline; echo; sudo readlink /proc/$p/cwd; done
sudo ls -la /root/.config/blueos/ardupilot-manager/firmware /root/.config/blueos/ardupilot-manager/firmware/scripts
sudo sha256sum /root/.config/blueos/ardupilot-manager/firmware/ardupilot_navigator64
sudo ss -lunp | grep -E '14560|14550'
```

### Procedure: install (nothing is installed by this repo)

1. Save the current parameters (`PARAM_REQUEST_LIST`, see the evidence capture of 2026-11-21) so that they can be restored. On the vehicle
   `SCR_ENABLE = 0` and `BATT_MONITOR = BATT2_MONITOR = 0`; `ARMING_SKIPCHK` is the default 16776767.
2. In BlueOS add the UDP Server endpoint on 14560 (above) and check that the pods' frames arrive (`tcpdump`, or MAVLink Inspector).
3. On the Pi host:
   ```
   D=/root/.config/blueos/ardupilot-manager/firmware/scripts
   sudo mkdir -p $D/modules/MAVLink
   sudo cp pod_battery.lua $D/
   sudo cp mavlink_msgs.lua mavlink_msg_BATTERY_STATUS.lua mavlink_msg_NAMED_VALUE_INT.lua $D/modules/MAVLink/
   ```
4. **Two steps for the parameters**, because most of them do not exist until the first restart:
   - first set `SCR_ENABLE = 1`, `BATT_MONITOR = 29`, `BATT2_MONITOR = 29` and restart **the autopilot** (BlueOS; the Pi and the containers are not restarted);
   - after the restart `SCR_HEAP_SIZE`, `SCR_VM_I_COUNT` and the `BATT_*` / `BATT2_*` groups exist: set `BATT_CAPACITY = 0`, `BATT2_CAPACITY = 0`
     (**the default is 3300 mAh**, which would give a false remaining percentage) and confirm the four `*_FS_*_ACT` are 0 (they are by default;
     `BATT_LOW_VOLT` and `BATT_CRT_VOLT` default to 0 on ArduSub).
   `ARMING_SKIPCHK` is a separate decision (see "Arming"): until the battery bit is skipped, a battery that is not healthy blocks arming, which includes
   the time between the first restart and the arrival of the pods' data.
5. Check: the text `PodBatt: started`, `BATTERY_STATUS` for id 0 and 1 with the pod voltage (compare with the pod ENV and a multimeter), no
   `Lua: ... exceeded time limit`.

### Procedure: remove

1. Disarmed. Set `BATT_MONITOR = 0`, `BATT2_MONITOR = 0`, restore `ARMING_SKIPCHK` to its previous value (16776767 on this vehicle) if it was changed, and
   `SCR_ENABLE = 0` only if no other script is installed (none is, at the time of writing).
2. Delete `pod_battery.lua` from the host scripts directory (the three modules can stay: nothing loads them without the script).
3. Restart the autopilot from BlueOS.
Removing only the script while `BATT_MONITOR` stays 29 leaves both batteries unhealthy after 5 s: it blocks arming unless the battery check is skipped, and
produces the "missing" warning when armed (see below).

## Arming

This tree (4.7+) names the parameter `ARMING_SKIPCHK` (bitmask of checks to **skip**); `ARMING_CHECK` is the old name, converted at boot.
ArduSub's default is `~(RC | VOLTAGE | BATTERY) & 0xFFFFFF` (`ArduSub/Parameters.h`), which checks only RC, board voltage and **battery**.
The ROV must be armable with a pod off or unreachable (single-pack mode: bringing it home is the priority), so `pod_battery.parm` adds
**only the battery bit** (1 << 8 = 256):

| | value | checked |
| --- | --- | --- |
| ArduSub default | 16776767 (0xFFFE3F) | RC, board voltage, battery |
| `pod_battery.parm` | 16777023 (0xFFFF3F) | RC, board voltage |

(read from the running SITL before and after loading the file; the other bits are identical). If the vehicle's value is not the default, OR 256 into
the value it has. In SITL a normal arming (`MAV_CMD_COMPONENT_ARM_DISARM`) is **accepted** with a silent pod, and with both pods absent since boot.
This also means that a battery which is really low does not block arming by itself.

## Failsafe

`BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT` (and the `BATT2_` ones) **must stay 0** with `BATT_MONITOR = 29`.

- **A silent pod is not a failsafe action in this source.** A battery unhealthy for more than 5 s while armed makes ArduPilot print
  `Battery N is missing, last: 0.00V used X mAh`, set its battery-failsafe flag (LED/buzzer) and call the handler with action **0**
  hard-wired for "unhealthy" (`AP_BattMonitor::check_failsafes`): no action whatever `BATT_FS_*_ACT` is. SITL, `BATT_LOW_VOLT = 40`,
  `BATT_CRT_VOLT = 35`, `BATT_LOW_TIMER = 2`, pod stopped while armed: the published 0 V with `healthy = false` does **not** raise a
  low-voltage failsafe (`check_failsafe_types` ignores a voltage that is not above 0), with the actions at 0 and also with
  `BATT_FS_LOW_ACT = 3` (Surface) and `BATT_FS_CRT_ACT = 2` (Disarm): only the "missing" warning, the vehicle stays armed in the same mode.
- **A pod that reports a real low voltage does act once an action is non-zero.** SITL, pod alive at 39 V against `BATT_LOW_VOLT = 40`
  and `BATT2_FS_LOW_ACT = 3`: `Battery 2 is low 39.00V used 321 mAh` and the mode leaves MANUAL for Surface (the SITL vehicle is already
  at the surface, so Surface completes at once into ALT_HOLD). With `BATT_FS_CRT_ACT = 2` a critical voltage would disarm.
- **Why they stay 0:** the voltage seen here is one pack, reported over the LAN; in single-pack mode it sags under load, and a Surface or
  Disarm action on it would take the ROV off the job it must finish; and, with no authentication on the LAN (see above), a forged low voltage
  would trigger it. Until the thresholds are verified at the bench the batteries only warn.
- **The flag is latched.** After the "missing" warning fired, the `SYS_STATUS` battery health bit stays cleared (and
  `AP_BattMonitor::has_failsafed()` stays true) after the pod returns, until the failsafe state is reset. Reset it with
  `MAV_CMD_BATTERY_RESET` (command 42651; `param1` = bitmask of the batteries, 1 = BATT, 2 = BATT2, 3 = both; `param2` = percentage, e.g. 100)
  or by rebooting the autopilot. The battery data themselves resume at once and arming is not affected. SITL: the bit returns right after
  `MAV_CMD_BATTERY_RESET` with `param1 = 2`.

## Test

`Tools/scripts/deepex_pod_battery_sitl.py` starts the SITL `ardusub` (`./waf sub`) with this script and emulates both pods (truncated MAVLink 2
payloads, 41 of 54 bytes for `BATTERY_STATUS`). `--scenario functional` (default, 43 checks): `ARMING_SKIPCHK` before/after, arming, the batteries,
the timeout, power-mode reporting (including rapid changes with exactly one log row and INFO text per observed change),
foreign senders, the failsafe scenarios, the DataFlash log (`BAT` with `H`, `PWRM`, `PBAT`, `SCR`).
`--scenario load`: the 3x test above, including an assertion that the emulator actually reaches its requested rate. Needs pymavlink.

MAVLink 2 truncation: the sender drops trailing zero bytes of the payload. In the `mavlink_message_t` that reaches Lua those bytes are
already zeros (the C parser zero-fills the packet to the full message length, `mavlink_helpers.h`), and the script also zero-fills the
payload before decoding, so it does not depend on that. `mavlink_msgs.decode()` is not used and is unchanged.

## CRC diagnosis follow-up (21/11/2026)

The fork passes `MAVLINK_FRAMING_BAD_CRC` packets to scripting so that scripts can handle dialects unknown to C++.
Our message types are known: their CRC must be checked in Lua. CRC is checked against the transmitted header and
payload before zero-filling. Rejected packets do not update measurements, freshness or power state.
`PBAT.Crc` counts rejected packets per pod; `PBAD` records metadata for the first three rejects per pod per script start.
CRC validates integrity, not sender authentication. Functional SITL includes corrupt-CRC messages with a matching
sysid/compid/id: they cannot replace voltage or prevent the normal timeout.
See [POD_BATTERY_CRC_DIAGNOSIS_20261121.md](POD_BATTERY_CRC_DIAGNOSIS_20261121.md) for evidence and limits.
