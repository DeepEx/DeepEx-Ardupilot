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
  (text rate-limited to one per second per pod). The pod's current budget is not sent and not used.
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
| Work per call | at most 4 decodes and 60 queue pops (VM budget), 160 decodes/s of capacity against 28/s needed |

`SCR_HEAP_SIZE` defaults to 200 KiB on Linux boards and SITL (`AP_Scripting.cpp`), which is enough: the whole Lua heap in use
(interpreter included) peaked at about 60 KB, 12 KB used per run (`SCR` log rows, `SCR_DEBUG_OPTS` bit 3). No change needed.

One run of a script is limited to `SCR_VM_I_COUNT` (10000 by default) VM instructions and ArduSub **stops the script** that goes over
(`Lua: ./scripts/pod_battery.lua exceeded time limit`). The first version did exactly that under load; the script now drops
messages of other senders after a few instructions, decodes at most 4 per call, and skips the CRC re-check in Lua (the C parser
only hands over frames whose CRC it has already verified; the Lua CRC loop was about 40 % of a decode).

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

### BlueOS 1.4.3 (navigator64): where, and what to check on the Pi

From the BlueOS **1.4.3** source (tag `1.4.3`, `core/services/ardupilot_manager` and `install/install.sh`): `ardusub` is started by
the `blueos-core` container with `cwd = firmware_folder`, default `appdirs.user_config_dir("ardupilot-manager")/firmware`, i.e.
`/root/.config/ardupilot-manager/firmware` inside the container; so the scripts go in
`/root/.config/ardupilot-manager/firmware/scripts/` (and `.../scripts/modules/MAVLink/`). Logs and storage of that process are in
`.../firmware/logs/` and `.../firmware/storage/`. `/root/.config` of the core container is a bind mount of the host directory
`$HOME/.config/blueos` (the installer starts the bootstrap container with `-e BLUEOS_CONFIG_PATH=$HOME/.config/blueos`), normally
`/home/pi/.config/blueos`. Installing a firmware (`install_firmware_from_file`, also `waf --upload-blueos`) only copies the ELF into
that folder and never touches `scripts/`, so the scripts survive firmware updates and container restarts.
**This is from the source, not yet seen on the vehicle**: check it with the read-only commands below.

Read-only checks, from the BlueOS "red-pill" terminal (host shell):

```
docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
docker inspect blueos-core --format '{{range .Mounts}}{{.Type}} {{.Source}} -> {{.Destination}}{{"\n"}}{{end}}'
docker exec blueos-core sh -c 'for p in /proc/[0-9]*; do if tr "\0" " " < $p/cmdline | grep -q ardusub; then echo "pid ${p#/proc/}"; tr "\0" " " < $p/cmdline; echo; readlink $p/cwd; fi; done'
docker exec blueos-core ls -la /root/.config/ardupilot-manager/firmware /root/.config/ardupilot-manager/firmware/scripts
ls -la "$(docker inspect blueos-core --format '{{range .Mounts}}{{if eq .Destination "/root/.config"}}{{.Source}}{{end}}{{end}}')/ardupilot-manager/firmware"
curl -s http://localhost/version-chooser/v1.0/version/current
```

The `.Source` of the `/root/.config` mount is the host directory to use below (`HOSTCFG`); the `cwd` printed by `readlink` must be
`/root/.config/ardupilot-manager/firmware`. If it is not, the scripts directory is `<that cwd>/scripts`.

### Procedure: install (nothing is installed by this repo)

1. Save the current parameters (ground station or `pymavlink` `PARAM_REQUEST_LIST`) so that they can be restored.
2. On the Pi host, with `HOSTCFG` = the mount source found above:
   ```
   sudo mkdir -p "$HOSTCFG/ardupilot-manager/firmware/scripts/modules/MAVLink"
   sudo cp pod_battery.lua "$HOSTCFG/ardupilot-manager/firmware/scripts/"
   sudo cp mavlink_msgs.lua mavlink_msg_BATTERY_STATUS.lua mavlink_msg_NAMED_VALUE_INT.lua \
       "$HOSTCFG/ardupilot-manager/firmware/scripts/modules/MAVLink/"
   ```
   (files only need to be readable by root in the container).
3. Set the vehicle parameters of `pod_battery.parm` (BlueOS Autopilot parameters, Cockpit or a ground station). `SCR_ENABLE = 1` only
   takes effect after a reboot **of the autopilot** (restart the autopilot from BlueOS; the Pi and the containers are not restarted).
4. After the restart check: the text `PodBatt: started`, `BATTERY_STATUS` for id 0 and 1 with the pod voltage (compare with the pod ENV
   and a multimeter), no `Lua: ... exceeded time limit`.

### Procedure: remove

1. Disarmed. Set `BATT_MONITOR = 0`, `BATT2_MONITOR = 0`, restore `ARMING_SKIPCHK` to the value saved in step 1 of the install (the ArduSub
   default is 16776767), and `SCR_ENABLE = 0` only if no other script is installed.
2. Delete `scripts/pod_battery.lua` (the three modules can stay: nothing loads them without the script) from the host directory.
3. Restart the autopilot from BlueOS.
Removing only the script while `BATT_MONITOR` stays 29 leaves both batteries unhealthy after 5 s; with the `ARMING_SKIPCHK` of this
file that does not block arming, but it does produce the "missing" warning when armed (see below).

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
payloads, 41 of 54 bytes for `BATTERY_STATUS`). `--scenario functional` (default, 37 checks): `ARMING_SKIPCHK` before/after, arming, the batteries,
the timeout, power-mode reporting, foreign senders, the failsafe scenarios, the DataFlash log (`BAT` with `H`, `PWRM`, `PBAT`, `SCR`).
`--scenario load`: the 3x test above. Needs pymavlink.

MAVLink 2 truncation: the sender drops trailing zero bytes of the payload. In the `mavlink_message_t` that reaches Lua those bytes are
already zeros (the C parser zero-fills the packet to the full message length, `mavlink_helpers.h`), and the script also zero-fills the
payload before decoding, so it does not depend on that. `mavlink_msgs.decode()` is not used and is unchanged.
