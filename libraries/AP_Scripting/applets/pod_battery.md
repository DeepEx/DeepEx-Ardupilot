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

## What the script does

- **Per pack** (`battery:handle_scripting`), 20 Hz:
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

The `BAT` log lines of both batteries are written by ArduPilot itself (`BAT` instance 0 and 1).

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
| `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`, `BATT2_FS_LOW_ACT`, `BATT2_FS_CRT_ACT` | 0 | warning only, no action |

**Scripting memory.** `SCR_HEAP_SIZE` defaults to 200 KiB on Linux boards and SITL (`AP_Scripting.cpp`, `SCRIPTING_HEAP_SIZE`), which
is enough: in the SITL run the whole Lua heap in use (interpreter included) peaked at about 60 KB with this script alone, with 15.7 KB
used per run (`SCR` log rows, `SCR_DEBUG_OPTS` bit 3). The default is fine; no change needed. One run of the script is limited to
`SCR_VM_I_COUNT` (10000) VM instructions: each decode costs about 1000, so the script decodes at most 6 messages per 50 ms call
(longest run measured: 5 ms) and drops foreign messages before decoding.

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

**BlueOS (navigator64 build).** From the BlueOS source (`core/services/ardupilot_manager`, upstream `master` of 20/11/2026), ardusub is started
with `cwd = firmware_folder`, whose default is `appdirs.user_config_dir("ardupilot-manager") / "firmware"`. Inside the BlueOS core
container this is `/root/.config/ardupilot-manager/firmware`, so the scripts go in
`/root/.config/ardupilot-manager/firmware/scripts/` (create the directory if it is missing) and the modules in
`.../scripts/modules/MAVLink/`. Logs and storage of the same process are in `.../firmware/logs/` and `.../firmware/storage/`.
Persistence: installing a firmware (`install_firmware_from_file`, also used by `waf --upload-blueos`) only copies the ardusub ELF into
that folder and does not touch `scripts/`; `/root/.config` of the core container is a bind mount of a host directory
(`BLUEOS_CONFIG_PATH` of the BlueOS bootstrap), so the scripts survive firmware updates and container restarts.
**Not verified on the vehicle:** the host directory behind that bind mount and the BlueOS version installed on the Pi were not
checked. On the Pi, confirm with `readlink /proc/$(pidof ardusub)/cwd` inside the core container, and that `scripts/` is under it.

## Effect on arming and failsafe (read from this tree, confirmed in SITL)

This tree (4.7+) names the parameter `ARMING_SKIPCHK` (bitmask of checks to **skip**); `ARMING_CHECK` is the old name, converted at boot.
ArduSub's default is `ARMING_SKIPCHK = ~(RC | VOLTAGE | BATTERY) & 0xFFFFFF` (`ArduSub/Parameters.h`): everything is skipped
except the RC, board voltage and **battery** checks, so the battery check is **active by default**. Not changed here.

- **Arming** (`AP_Arming::battery_checks` -> `AP_BattMonitor::arming_checks`, for every instance with `BATTx_MONITOR` not 0): a battery
  that is not healthy refuses the arming with the text `Battery N unhealthy` (N = 1 for BATT, 2 for BATT2). The same function
  also refuses below `BATT_ARM_VOLT` / `BATT_ARM_MAH` (0 = off) and while a low/critical voltage or capacity failsafe is active.
  In SITL, `MAV_CMD_COMPONENT_ARM_DISARM` with a silent pod 2 is refused (ACK result 4, text `Arm: Battery 2 unhealthy`); it is
  accepted as soon as the pod is back. A forced arming (param2 = 21196) skips the checks and succeeds. A pod that never sent anything
  also leaves the battery unhealthy, so **with BATT_MONITOR = 29 and a pod that is off or unreachable the ROV cannot be armed** (and
  if the script itself stops, the scripting backend goes unhealthy after 5 s: `AP_BATT_MONITOR_SCRIPTING_TIMEOUT_US`).
- **While armed**, a battery unhealthy for more than 5 s makes ArduPilot send the warning `Battery N is missing, last: 0.00V used X mAh`,
  set the battery-failsafe flag (notify LED/buzzer) and call the battery failsafe handler with action 0 (`Sub::handle_battery_failsafe`:
  no action): in SITL the vehicle stayed armed, in the same mode, with no failsafe action. That failsafe state is **latched**: the
  `SYS_STATUS` battery health bit stays cleared after the pod returns until `MAV_CMD_BATTERY_RESET` (or a reboot); the battery data
  themselves resume at once and arming is possible again.

## Test

`Tools/scripts/deepex_pod_battery_sitl.py` starts the SITL `ardusub` (`./waf sub`) with this script, emulates both pods (truncated MAVLink 2
payloads, 41 of 54 bytes for `BATTERY_STATUS`) and checks the batteries, the timeout, the power-mode reporting, foreign senders, the arming
behaviour and the DataFlash log (`BAT`, `PWRM`, `SCR`). It needs pymavlink.

MAVLink 2 truncation: the sender drops trailing zero bytes of the payload. In the `mavlink_message_t` that reaches Lua those bytes are
already zeros (the C parser zero-fills the packet to the full message length, `mavlink_helpers.h`), and the script also zero-fills the
payload before decoding, so it does not depend on that. `mavlink_msgs.decode()` itself is unchanged.
