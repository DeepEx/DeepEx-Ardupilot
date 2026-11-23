# Pod battery Lua installation — 2026-11-21

Owner-authorized installation on the Pi at `192.168.2.2`, following the
[implementation review](pod_battery_review.md). AI-assisted work.
This records software installation and observed telemetry, not physical measurement qualification.

## Initial inspection and backup

- ROV disarmed, mode 19. No arm/disarm, motor, power-switch, or pod commands were issued.
- BlueOS core `bluerobotics/blueos-core:1.4.3`; MAVLink server listening on UDP `0.0.0.0:14560`.
- Existing autopilot version `ArduSub 4.8.0-dev`, custom version `4bffa1cb`; firmware unchanged.
- Script directory existed but was empty.
- `SCR_ENABLE=1`, `SCR_HEAP_SIZE=204800`, `SCR_VM_I_COUNT=10000`.
- `BATT_MONITOR=BATT2_MONITOR=29`; all four battery failsafe actions were already zero.
- `BATT_CAPACITY=0`, but `BATT2_CAPACITY=3300`; `ARMING_SKIPCHK=16776767` still checked battery health.
- All **1093/1093** parameters captured locally before changes:
  `/Users/deepex/Documents/GitHub/ardupilot_pod_backup/20261121-194416-ardusub-params.{json,parm}`.
- Original scripts directory archived on the Pi host at
  `/root/.config/blueos/ardupilot-manager/pod-battery-backups/scripts-before-20261121-codex.tar.gz`.

## Installed files and activation

Host directory: `/root/.config/blueos/ardupilot-manager/firmware/scripts/`.
Container directory: `/root/.config/ardupilot-manager/firmware/scripts/`.

| Relative file | SHA-256, matched locally and on Pi |
| --- | --- |
| `pod_battery.lua` | `b7b8877428dfb1c4397f5c02c5007b5346c3b44c7ad5b4f760c9417fe16dbf63` |
| `modules/MAVLink/mavlink_msgs.lua` | `53c21f156e85347e356657c0e75cf9f0726bddd34f2953b21ac3589f65197548` |
| `modules/MAVLink/mavlink_msg_BATTERY_STATUS.lua` | `b702d8315c619b47a0b7d147b99d41780a24d7d43dd7e6701e2b4e35e0a7ba49` |
| `modules/MAVLink/mavlink_msg_NAMED_VALUE_INT.lua` | `9171dda76096eb54e8c66373026c790561999acace896ad91f56110a5739b19a` |

Changed and read back only:

- `BATT2_CAPACITY`: **3300 → 0**, preventing a false remaining percentage.
- `ARMING_SKIPCHK`: **16776767 → 16777023**, OR-ing only the battery bit (256), as authorized by the owner.

Restarted **scripting only**, via `MAV_CMD_SCRIPTING / SCRIPTING_CMD_STOP_AND_RESTART`;
autopilot ACK accepted. Did not restart the autopilot, containers, or Pi.
Observed `Scripting: stopped`, `Scripting: restarted`, `PodBatt: started (timeout 1000 ms)`.
Script parameters read back: `PBAT_ENABLE=1`, `PBAT_TIMEOUT_MS=1000`, `PBAT_SYSID=1`.

## Reporting fix and local verification

Before installation, removed the one-second text gate that left a change pending across multiple updates.
Each new observed mode/reason pair now writes one `PWRM` row and emits its INFO text, then marks the pair reported.
The 25 ms scheduler and four-decode budget are unchanged.
Added functional SITL checks for three changes at 200 ms intervals: INFO text for each and exactly one log row
for each distinct test reason (101, 102, 103). The functional scenario passes **39/39**, including these regressions.
The earlier functional run without the extra assertions also passed 37/37 after the fix.
The 30-second 3x load scenario passes **12/12**: each pod delivered 934/934 battery messages
and 374/374 named-value messages to the applet, with no loss measured.
Python flake8 and `git diff --check` pass.
No ArduSub binary was rebuilt; SITL uses the reviewed `e8f38159` executable and loads current Lua files.

## Observed vehicle telemetry and DataFlash

Over the BlueOS MAVLink router, both pod-origin and autopilot-origin `BATTERY_STATUS` were observed.
In a separate 20-second steady observation, the two pod streams delivered 200/201 frames;
the autopilot emitted 30 frames for each battery. Both monitors were healthy in the final health samples.

| Autopilot battery | Voltage snapshot | Current snapshot | Temperature | Consumed | Remaining |
| --- | --- | --- | --- | --- | --- |
| 0 / BAT1 | 45.943 V | 0.12 A | 24 °C | 0 mAh | -1 |
| 1 / BAT2 | 45.953 V | 0.31 A | 25 °C | 673 mAh | -1 |

Pod voltage snapshots matched the autopilot snapshots; current readings varied slightly between asynchronous samples.
These are telemetry values, not measurements verified against a multimeter.
The data are now available on the autopilot telemetry link for Cockpit; Cockpit's widgets were not inspected.

Copied a read-only snapshot of the active vehicle log `00000165.BIN` and decoded it locally:

- `BAT` instances 0/1 show the reported voltage/current/temperature with `H=1`.
- `PBAT` counters increase for both pods, with recent sample ages; no timeout observed after startup.
- `PWRM`: pod 180 reports mode 0 / reason 25; pod 181 reports mode 3 / reason 81.
  The script reports those values without changing switches or motor commands.
- No Lua crash or instruction-limit error observed in the captured activation/steady observation.

### Open observation

The script warned about inconsistent incoming battery IDs (180 reported id 41; 181 reported id 79).
Those samples were rejected by the existing compid/id guard. In the first copied log,
`PBAT.Ign` reached 2 for BAT1 and 1 for BAT2; both continued healthy.
The cause is **not established**. This remains a diagnostic follow-up, rather than claiming all packets are valid.
No pod/network-disconnection or live arming test was performed; timeout and arming behavior are SITL evidence.

## Evidence

Local bundle: `/Users/deepex/Documents/Codex/2026-11-21/pod-battery-installation/`.
Contains pre/post/steady telemetry captures, the parameter backup references, installed-file hashes,
SITL output, the live DataFlash snapshot and decoded summary, and SHA-256 manifest.
No access credentials are included in the bundle or repository.
