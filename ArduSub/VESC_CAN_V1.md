# ArduSub VESC CAN integration v1

Status: implementation candidate requiring hardware bench validation

Target: ArduSub on Raspberry Pi 5 with Blue Robotics Navigator and a native
Linux SocketCAN USB-CAN adapter.

## Architecture and safety boundary

The propulsion path is:

```text
ArduSub mixer -> SRV motor functions -> AP_VESC -> AP_HAL CANIface
              -> Linux SocketCAN can0 -> USB-CAN -> VESC IDs 1..6
```

The Distribution Board is only a passive electrical fan-out. Its firmware does
not create, translate, or interpret VESC traffic.

`AP_VESC` supports two deliberate boot-time modes:

- `CAN_D1_VE_MODE=0` (`PPM`): normal PWM/PPM motor outputs remain active.
  `AP_VESC` may receive telemetry but never transmits a motor command. A missing
  or down `can0` does not block arming.
- `CAN_D1_VE_MODE=1` (`CAN`): `AP_VESC` is the only command source for selected
  motors. It snapshots final mixer PWM, converts it to signed eRPM, and replaces
  the corresponding physical PWM with `CAN_D1_VE_IN_MID` before the HAL output
  push. It sends `CAN_PACKET_SET_RPM` only when armed, command data is fresh,
  the interface is healthy, and every selected controller is command-ready.

The exact prefix is `CAN_Dn_VE_` when a VESC protocol is assigned to CAN driver
`n`. The examples below use driver 1.

There is no automatic CAN-to-PPM fallback. Mode changes require disarming,
changing the parameter, and rebooting. Disarm, emergency stop, stale mixer
commands, stale required telemetry, an active fault, interface-down state, and
recent TX failure all cause zero-eRPM commands in CAN mode.

On disarm, zero commands are sent for `CAN_D1_VE_STOPMS`. `VESC_SAFE=1` means
that this flush has completed; VMOT must not be removed before then. The
VESC-side command timeout remains an independent safety requirement.

## Parameters

Set `CAN_D1_PROTOCOL=15` for VESC and associate `CAN_P1_DRIVER=1`.

| Parameter | Initial value | Meaning |
|---|---:|---|
| `CAN_D1_VE_MODE` | `0` | `0=PPM`, `1=CAN`; reboot required |
| `CAN_D1_VE_PROTO` | `0` | `0=STANDARD`, `1=DEEPEX_V1`; reboot required |
| `CAN_D1_VE_ESC_BM` | `0x003F` | Motors controlled by CAN |
| `CAN_D1_VE_ESC_RT` | `100` | SET_RPM rate in Hz |
| `CAN_D1_VE_MAXRPM` | `17000` | Maximum signed electrical RPM |
| `CAN_D1_VE_PPAIRS` | `7` | Motor pole pairs |
| `CAN_D1_VE_IN_MIN` | `1100` | Full reverse mixer reference |
| `CAN_D1_VE_IN_MID` | `1500` | Neutral mixer/PWM value |
| `CAN_D1_VE_IN_MAX` | `1900` | Full forward mixer reference |
| `CAN_D1_VE_CMD_TO` | `100` | Mixer command timeout in ms |
| `CAN_D1_VE_TLM_TO` | `500` | STATUS_1 freshness limit in ms |
| `CAN_D1_VE_EXT_TO` | `1000` | STATUS_4 and STATUS_5 limit in ms |
| `CAN_D1_VE_ENG_TO` | `2000` | STATUS_2 and STATUS_3 limit in ms |
| `CAN_D1_VE_ID1..ID6` | `1..6` | Motor-to-controller mapping |
| `CAN_D1_VE_STOPMS` | `500` | Disarm zero-command flush |

Do not change mode while operating. Invalid or duplicate selected IDs, a
selected motor without an assigned SRV motor function, a missing interface,
interface-down state, recent TX failure, missing STATUS_1, stale STATUS_4/5, or
an active DeepEx fault blocks the VESC pre-arm check in CAN mode.

Energy telemetry is reported independently and does not make an otherwise
present controller offline.

## SocketCAN and physical bus

BlueOS/Linux owns interface creation and bitrate. A typical setup is:

```sh
ip link set can0 down
ip link set can0 type can bitrate 500000 restart-ms 100
ip link set can0 up
ip -details -statistics link show can0
```

Use one linear 500 kbit/s bus, twisted CAN-H/CAN-L, short stubs, a common
reference appropriate to the isolated adapter/VESC installation, and exactly
two 120-ohm termination resistors at the physical ends. Confirm approximately
60 ohms across CAN-H/CAN-L with power removed. Controller IDs must be unique.

The Linux receive implementation accepts ordinary external frames. `MSG_CONFIRM`
is used only to recognize this socket's local TX confirmation and release TX
bookkeeping. Local loopback is not presented as external VESC telemetry.
Malformed/truncated frames are rejected and VESC decoding rejects error, RTR,
non-extended, wrong-length, and unknown packet frames.

### First physical receive test and software correction

The first Calypso Beta receive test used the SH-C30A through the Linux `gs_usb`
driver. `can0` was configured at 500 kbit/s and remained `ERROR-ACTIVE`, with
zero RX errors, zero dropped frames, and no bus-off. The VESC used controller
ID 2 and produced STATUS_1 (`0x00000902`) at approximately 20 Hz and STATUS_2
through STATUS_5 (`0x00000E02`, `0x00000F02`, `0x00001002`, and `0x00001B02`)
at approximately 5 Hz. The kernel received more than 112,000 frames. No
SET_RPM frame (`0x00000302`) was observed in PPM mode.

The `VESC_0` thread was present, but the pre-fix MAVLink state was
`VESC_EXP=2`, `VESC_PRES=0`, `VESC_MISS=2`, and `VESC_DIAG=34`. Diagnostic
bits 1 and 5 reported no external VESC RX and the required controller missing.
This established that the interface and thread were initialized while frames
received by the Linux kernel did not reach `AP_VESC::handle_frame()`.

The defect was the AP_VESC zero-timeout select-first sequence. Linux
`CANIface::select()` checked only its internal RX queue once the deadline had
elapsed. With that queue empty, AP_VESC did not call `CANIface::receive()`, so
the SocketCAN socket was never polled and the external frame could not enter
the queue. AP_VESC now calls the generic non-blocking `CANIface::receive()`
directly and continues draining all available frames before the existing
one-millisecond loop delay.

The C++ vcan regression injects an external extended STATUS_1 frame for
controller 2, verifies that the former current-time select-first sequence does
not consume it, and verifies that the AP_VESC direct receive path returns and
decodes it with its identifier, DLC, payload, and external/non-loopback
classification preserved. Result: PASS. This is software regression evidence
only. The rebuilt binary has not been installed or physically retested on
BlueOS, and the integration remains NOT QUALIFIED.

## VESC application configuration

Configure each VESC locally; ArduSub does not rewrite or persist VESC settings.

```text
can_status_rate_1 = 20
can_status_msgs_r1 = 0x01
can_status_rate_2 = 5
can_status_msgs_r2 = 0x1E
```

The masks are STATUS_1 bit 0, STATUS_2 bit 1, STATUS_3 bit 2, STATUS_4 bit 3,
and STATUS_5 bit 4.

## Telemetry decoding

All VESC multibyte values are signed big-endian where the protocol defines a
signed value.

| Packet | Stored fields | Scaling |
|---|---|---|
| STATUS_1 | electrical eRPM, motor current, duty | raw `int32`; `int16/10` A; `int16/1000` |
| STATUS_2 | Ah consumed, Ah regenerated | each `int32/10000` Ah |
| STATUS_3 | Wh consumed, Wh regenerated | each `int32/10000` Wh |
| STATUS_4 | MOSFET temp, motor temp, input current, PID position | `int16/10` C, `int16/10` C, `int16/10` A, `int16/50` |
| STATUS_5 | tachometer, input voltage, optional status | `int32`, `int16/10` V, optional `uint16` |

Mechanical RPM is `electrical_eRPM / pole_pairs`. Both signed values are kept.

`STANDARD` accepts the normal six-byte STATUS_5 and an eight-byte frame but
always ignores bytes 6..7. `DEEPEX_V1` interprets an eight-byte STATUS_5 field
as a big-endian `uint16`: bits 0..7 are `mc_fault_code`, bits 8..15 are DeepEx
warning flags. A standard VESC is never treated as DeepEx unless the parameter
explicitly selects it.

Freshness groups are independent:

- fast: STATUS_1;
- extended thermal/power: both STATUS_4 and STATUS_5;
- energy: both STATUS_2 and STATUS_3.

## MAVLink producer contract

The existing `ESC_TELEMETRY_1_TO_4`, `_5_TO_8`, and `_9_TO_12` messages retain
their standard meanings. Motor index maps directly to ESC index:

- `temperature`: STATUS_4 MOSFET temperature, integer degC;
- `voltage`: STATUS_5 input voltage, centivolts;
- `current`: STATUS_4 input current, centiamps, constrained by the unsigned
  standard field;
- `totalcurrent`: STATUS_2 consumed energy, mAh;
- `rpm`: absolute STATUS_1 electrical eRPM because the standard field is
  unsigned;
- `count`: the existing AP_ESC_Telem update counter.

Signed values, motor current, motor temperature, regenerated energy, watt-hour
counters, duty, faults, and independent freshness must not overload those
standard fields.

The following global `NAMED_VALUE_INT` values are produced:

| Name | Meaning |
|---|---|
| `VESC_STATE` | `0=off,1=waiting,2=ready,3=armed,4=zero flush,5=fault` |
| `VESC_MODE` | `0=PPM,1=CAN` |
| `VESC_PROTO` | `0=STANDARD,1=DEEPEX_V1` |
| `VESC_EXP` | selected motor mask |
| `VESC_PRES` | fresh STATUS_1 motor mask |
| `VESC_MISS` | expected but not present mask |
| `VESC_SAFE` | disarm zero-flush complete |
| `VESC_DIAG` | diagnostic bitmask below |
| `VESC_FLT` | active fault motor mask |

`VESC_DIAG` bits are:

```text
0 driver/interface unavailable
1 no external VESC RX seen
2 interface down
3 recent CAN TX failure
4 unexpected controller received
5 required controller missing
6 active VESC fault
7 mixer command timeout
8 zero-command flush incomplete
```

Fields not representable by standard ESC telemetry use the existing MAVLink
`DEBUG_FLOAT_ARRAY` message without a dialect change. The name is `VESC_V1`,
`array_id` is the one-based ArduSub motor number, and one selected motor is
published per INFO cycle. The MAVLink name field is exactly 10 bytes: the seven
ASCII bytes `VESC_V1` followed by three zero bytes. Consumers must check index
0 before decoding.

| Index | Value |
|---:|---|
| 0 | contract version, exactly `1` |
| 1 | one-based motor number |
| 2 | CAN controller ID |
| 3 | state bits: fast, extended, energy, stale, present, command-ready, active-fault, configured, expected at bits 0..8 |
| 4 | signed electrical eRPM |
| 5 | signed mechanical RPM |
| 6 | motor current A |
| 7 | input current A |
| 8 | signed duty ratio |
| 9 | input voltage V |
| 10 | MOSFET temperature C |
| 11 | motor temperature C |
| 12 | Ah consumed |
| 13 | Ah regenerated |
| 14 | Wh consumed |
| 15 | Wh regenerated |
| 16 | PID position |
| 17 | tachometer |
| 18 | fault code |
| 19 | warning flags |
| 20 | last STATUS_1 boot time ms |
| 21 | last STATUS_4 boot time ms |
| 22 | last STATUS_5 boot time ms |
| 23 | last STATUS_2 boot time ms |
| 24 | last STATUS_3 boot time ms |
| 25..57 | reserved, zero in v1 |

This layout is append-only within version 1. A semantic or scaling change
requires a new name/version.

Measurement values at indices 4..17 are zero when their corresponding
freshness group has never become valid or has become stale. Fault and warning
codes at indices 18..19 retain the last STATUS_5 report for diagnosis;
timestamps at indices 20..24 retain the last update time. Consumers must use
the validity bits in index 3 rather than treating zero as a valid measurement.
Standard ESC telemetry omits a four-controller group when every member is
stale.

## Verified software checks

The Linux SocketCAN software path was verified on a Raspberry Pi 5 at commit
`b2844e34a3c0b3b5a20fd573b71e03521485493b`:

```sh
sudo bash libraries/AP_HAL_Linux/tests/run_vcan_rx_test.sh
```

Result: PASS. This covered external extended-frame reception without
`MSG_CONFIRM`, local TX confirmation with `MSG_CONFIRM`, external/local
loopback distinction, extended identifier and signed payload preservation, and
safe interface-down failure.

This result does not verify the physical USB-CAN adapter, 500 kbit/s bus,
Distribution Board wiring, termination, or real VESC controllers.

## Navigator deployment target and build validation

The `navigator` target produces a 32-bit ARM binary. The `navigator64` target
produces the AArch64 binary required by the active Calypso Beta BlueOS
platform:

```sh
/home/ardupilot/venv-ardupilot/bin/python3 waf configure --board navigator64
/home/ardupilot/venv-ardupilot/bin/python3 waf sub
```

The previous artifact had this identity:

```text
target: navigator
size: 3,022,380 bytes
SHA-256: cc76ae63543c8d7a7114b751ed8c3ab2713219a8a451cd56a13c659229bc1c93
type: ELF 32-bit ARM
internal version: ArduSub V4.8.0-dev
```

BlueOS installed that artifact into its Navigator64 firmware slot, but the
autopilot did not remain running and produced no MAVLink heartbeat. The
observed BlueOS notifications were:

```text
AUTOPILOT_VEHICLE_TYPE_FETCH_FAIL
AUTOPILOT_FIRMWARE_VEHICLE_TYPE_FETCH_FAIL
```

The 32-bit artifact in a Navigator64 slot is an identified incompatibility and
the leading cause of those symptoms. It is not established as the sole cause
until the new Navigator64 binary is installed and starts successfully on
BlueOS. This is a deployment-target mismatch, not physical CAN validation.

### BlueOS dynamic-loader compatibility

Calypso Beta BlueOS runs on Debian 12 with glibc 2.36. A structurally correct
ELF64 AArch64 `navigator64` artifact built with the Ubuntu 24.04 AArch64
cross-compilation sysroot required `GLIBC_2.38` and could not start. The
dynamic loader reported:

```text
libc.so.6: version `GLIBC_2.38' not found
libm.so.6: version `GLIBC_2.38' not found
```

Firmware upload and integrity verification succeeded, but ArduSub produced no
heartbeat. MAVLink-Server restart errors and vehicle-type notifications were
secondary effects of ArduSub failing in the dynamic loader.

The replacement artifact is built in an official Ubuntu 22.04 environment.
ELF version-information validation shows a maximum requirement of
`GLIBC_2.34`, with no required GLIBC symbol newer than the BlueOS glibc 2.36
runtime. This is software deployment compatibility evidence only. It does not
claim successful execution on BlueOS; the replacement artifact has not yet
been installed and observed running on the Raspberry Pi.

The integration remains **NOT QUALIFIED**. Physical validation of the SH-C30A
USB-CAN adapter, 500 kbit/s bus, termination, cabling, and real VESC controllers
is still outstanding.

## Bench procedure

1. Remove propellers or mechanically isolate every thruster. Provide a physical
   VMOT disconnect and keep the vehicle disarmed.
2. Back up parameters and the known-good Navigator binary.
3. Verify IDs 1..6, VESC command timeout, telemetry groups, 500 kbit/s bitrate,
   polarity, termination, and `candump can0`.
4. Boot in PPM mode with USB-CAN disconnected. Confirm arming behavior and
   ordinary PPM outputs.
5. Reconnect USB-CAN in PPM mode. Confirm STATUS_1..5 and verify that no
   SET_RPM frames appear.
6. Disarm, select CAN mode, and reboot. Confirm arming is rejected with each of:
   `can0` down, one controller absent, duplicate mapping, stale STATUS_1,
   stale STATUS_4/5, and an injected DeepEx fault.
7. With all six controllers fresh, arm at zero demand. Confirm all selected PWM
   pins stay neutral and CAN SET_RPM payloads are zero.
8. Exercise low positive and negative demand one motor at a time. Compare raw
   eRPM, mechanical RPM, direction, current, and tachometer.
9. Exercise emergency stop, command-input loss, CAN interface-down, and disarm.
   Confirm zero commands and wait for `VESC_SAFE=1` before removing VMOT.
10. Preserve CAN capture, parameter file, firmware identities, and test log.

Do not perform an in-water test until the bench evidence has been reviewed.

## PPM rollback

Disarm, wait for `VESC_SAFE=1`, remove VMOT, set `CAN_D1_VE_MODE=0`, restore the
known PPM parameters if needed, and reboot. Verify PPM with propulsion isolated
before restoring VMOT. Do not change modes or reconnect a second command source
while armed.

## Known limitations

- Hardware validation with the selected USB-CAN adapter and VESC firmware is
  still required.
- Linux/BlueOS, not ArduPilot, configures `can0` bitrate and restart behavior.
- v1 does not configure VESC firmware, isolate individual failed thrusters,
  reallocate the mixer, or automatically fall back to PPM.
- Standard MAVLink ESC fields cannot preserve signed RPM/current; consumers
  needing signed or extended values must use `VESC_V1`.
- `DEBUG_FLOAT_ARRAY` stores timestamps and tachometer in IEEE-754 float, so
  consumers must use freshness bits rather than expecting full-width integer
  precision for large values.

## Contribution note

This implementation and document were prepared with AI assistance. The human
submitter remains responsible for understanding, validating, and maintaining
the change.
