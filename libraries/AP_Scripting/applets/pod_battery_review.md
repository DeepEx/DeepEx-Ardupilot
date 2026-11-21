# Pod battery implementation review — 2026-11-21

AI-assisted review of the work inherited on `feat/pod-battery-lua`.
This report records observations and local SITL results; it does not qualify hardware or install anything.
The inherited implementation is unchanged by this review.

## Source and scope

- Base: `e8f381590e456e283d17282b2ef3d47d3989d239`, `feat/vesc-v1-5hz-on-installed`.
- Reviewed head: `a3b80d16166a2a92f597403bb5e76dabcb309b6f`, already on `origin/feat/pod-battery-lua`.
- Inherited commits: `183fb012d2`, `31df4a1b99`, `a3b80d1616`.
- Inherited diff: seven added/extended files, 1354 insertions; no C++ changes or submodule changes.
- Engineering authority: [ADR-0024][adr], accepted; sender contract: [MAVLINK_BATTERY.md][sender] at pod commit `3f41ae9`.
- No ROV, Pi, pod, vehicle parameters, base branch, or release branch was modified. No PR was opened.

[adr]: https://github.com/DeepEx/DeepEx-System/blob/specifications/pod-power-sm-revision/adr/ADR-0024-publish-pod-battery-and-power-state-to-ardusub-over-mavlink.md
[sender]: https://github.com/DeepEx/distribution_board/blob/3f41ae9ff5b2494139674e34ec7f5413544f233b/docs/MAVLINK_BATTERY.md

## Findings requiring follow-up

1. **Arming policy differs from the supplied task.** `pod_battery.parm` adds
   `ARMING_SKIPCHK=16777023`, skipping the battery check. The task reserved that decision to the owner.
   This is a reference parameter file, not a C++ change; it was not applied to a vehicle in this review.
   Whether the owner subsequently authorized it is not established by the supplied conversation.
   The inherited SITL test expects permissive arming, so its green result does not demonstrate compliance
   with the original request to retain the battery check.
2. **Power-state reporting is not one event per change.** In `pod_battery.lua`, function
   `report_power_state` (reviewed lines 318–342), `logger:write("PWRM", ...)` precedes the text
   rate limiter, but `pwr_pub_mode`/`pwr_pub_rsn` are updated only when a text is sent.
   A pending change therefore writes the same row every 25 ms until the text is permitted.
   A local SITL reproduction sent BAT1 states `(mode,reason)=(2,7),(3,11),(4,12)` at 200 ms intervals.
   Their DataFlash row counts were **1, 8, 25**, respectively. The intermediate `(3,11)` state had no INFO text.
   The functional suite only checks that expected rows exist; it cannot detect this defect.
3. **The requested 20 Hz loop became 40 Hz.** The implementation uses 25 ms, queue depth 25,
   at most four decodes per invocation. The previous agent documented this as a response to the Lua
   instruction limit under load. The reviewed implementation passes the load test, but this remains
   a deviation from the requested loop period rather than evidence that 20 Hz is impossible.

No motor limiting is implemented. ADR-0016 control remains outside this task.

## Independently repeated checks

The existing SITL executable identifies itself as `ArduSub V4.8.0-dev (e8f38159)`.
SHA-256: `b6a08b43cba3bd5e42b8093d02e14188349466995df577152e445e9dc0f15fbe`.
It was reused, not rebuilt. The reviewed branch has no C++ changes relative to that base;
the Lua applet and modules are copied from the reviewed working tree by the test.
The test transport is local SITL TCP serial links, not the BlueOS UDP router.

| Check | Result |
| --- | --- |
| Existing functional SITL scenario | **37/37 passed** |
| Existing load scenario, 30 seconds, pods at 3x rate plus foreign traffic | **12/12 passed** |
| Additional SITL checks with battery arming check retained | **6/6 passed** |
| Regenerate both Lua message definitions from fork `all.xml`, MAVLink 2 | Byte-identical to committed modules |
| Python flake8 on the inherited SITL test | Passed |
| `git diff --check e8f381590e` | Passed |
| Branch conventions against `e8f381590e` | Failed on inherited commit organization/prefixes |

The convention failures are the mixed `AP_Scripting`/`Tools` first commit and the unrecognized
`pod_battery:` prefix of the next two commits. Published history was not rewritten.
Markdown lint was skipped by that checker because `markdownlint-cli2` is not installed.

### Requested SITL outcomes

- **a — measurements and BAT log:** BAT1 50.123 V / 12.34 A / 777 mAh / 31.5 °C;
  BAT2 49.870 V / 5.50 A / 321 mAh / 28 °C. Autopilot-origin `BATTERY_STATUS` ids 0/1
  and DataFlash `BAT` rows verified; remaining percentage is -1. Input payloads were truncated
  MAVLink 2, 41 of 54 bytes. Invalid-field sentinel combinations were inspected in source,
  but the inherited functional test does not exercise every such combination.
- **b — pod stopped:** battery unhealthy, BAT voltage zero and `H=0`, other battery unaffected;
  no disarm or mode change. The MAVLink observation was approximately **1.11 s** after stopping
  the sender with a 1000 ms script timeout. This includes the backend and telemetry update cadence;
  it is not proof of an externally observable transition within exactly 1000 ms.
  The autopilot's separate five-second unhealthy warning and latched battery-failsafe flag
  occur without a control action. Data resume when the pod returns; the flag can remain latched.
- **c — power mode:** expected INFO texts and `PWRM` rows pass for separated changes.
  Rapid changes expose finding 2, so the general requirement is not fully met.
- **d — identity filtering:** other sysids/compids and battery id/compid mismatches ignored.
- **e — arming:** the inherited reference parameters allow normal arming with missing pods.
  An additional isolated SITL run overriding only `ARMING_SKIPCHK` to the ArduSub default
  **16776767** rejects absent BAT1 (`Battery 1 unhealthy`) and silent BAT2 (`Battery 2 unhealthy`),
  and accepts normal arming with both packs healthy. No forced-arm command was used.

`AP_Arming::battery_checks` calls `AP_BattMonitor::arming_checks` when `Check::BATTERY` is enabled.
The latter validates each configured driver and calls its backend's `arming_checks`.
`AP_BattMonitor_Backend::arming_checks` rejects `!_state.healthy`; it also checks configured
arming voltage/capacity and voltage/capacity failsafe conditions. Failsafe actions of zero
do not disable these arming checks. `ArduSub/Parameters.h` enables the battery check by default.

### Load and scripting memory

Per pod, the repeat load test sent and the applet accepted **925/925 BATTERY_STATUS** and
**370/370 NAMED_VALUE_INT**, with no false timeout. Peak Lua heap use was **58118 bytes**;
peak measured applet runtime **4165 µs**. Functional-test peak heap was **60461 bytes**.
Linux/SITL default `SCR_HEAP_SIZE` is **204800 bytes**, sufficient in these tests.
This is Mac SITL evidence, not a CPU timing or memory qualification of the installed Navigator build.

## BlueOS installation evidence and limits

The inherited `pod_battery.md` records a prior read-only inspection of BlueOS 1.4.3:

- Container: `/root/.config/ardupilot-manager/firmware/scripts/`.
- Host: `/root/.config/blueos/ardupilot-manager/firmware/scripts/`.
- Applet in that directory; modules under `modules/MAVLink/`.
- No UDP 14560 endpoint at the time of that inspection; a UDP Server endpoint still needed configuration.
- Installed version reported there was `4bffa1cb`, one commit before the requested source base.

These host/container deployment observations were **not rechecked on the Pi** in this review.
The locally retained BlueOS 1.4.3 source confirms that the manager launches the executable
with `cwd=self.settings.firmware_folder`; ELF installation uses `shutil.copy` into the
firmware destination and does not remove `scripts/`. The fork's Linux scripting directory
is `./scripts`. Persistence across an ordinary firmware replacement follows from those sources;
neither an update nor a broader BlueOS reset/reinstallation was tested here.

## Evidence and reproduction

Local evidence bundle:
`/Users/deepex/Documents/Codex/2026-11-21/pod-battery-review/`.
It retains stdout, DataFlash BIN logs, two supplemental reproduction harnesses,
the MAVLink generation log, `pwrm-counts.json`, and `sha256.json`.
The large logs are local artifacts and are not included in Git.

Commands run from the repository root with a Python environment containing pymavlink and flake8:

```sh
python Tools/scripts/deepex_pod_battery_sitl.py --keep
python Tools/scripts/deepex_pod_battery_sitl.py --scenario load --load-seconds 30 --keep
python modules/mavlink/pymavlink/tools/mavgen.py --lang Lua --wire-protocol 2.0 \
  --output /tmp/codex-pod-review/modules/MAVLink modules/mavlink/message_definitions/v1.0/all.xml
flake8 Tools/scripts/deepex_pod_battery_sitl.py
git diff --check e8f381590e
python Tools/scripts/check_branch_conventions.py --base-branch e8f381590e
```

The two generated message files were compared with `cmp`; `mavlink_msgs.lua` was not modified.
Follow-up is to resolve the owner's arming-policy decision and correct/test change reporting
before treating the entire supplied task as complete. Deployment and physical checks remain with the owner.
