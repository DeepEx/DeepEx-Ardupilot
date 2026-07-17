# ArduSub VESC CAN Thruster Control and Fault-Tolerant Allocation

Status: Draft for engineering review
Target platform: Blue Robotics Navigator on Raspberry Pi 5 / BlueOS
Initial vehicle class: ArduSub
Source baseline reviewed: ArduPilot `e7805549c5da3a9089a9e681f2bc23b1c2a868f0`

## 1. Purpose

This document specifies two related features:

1. Native control and telemetry of VESC motor controllers over CAN from ArduSub.
2. Detection, isolation, and control reallocation after a thruster becomes unavailable.

The flight-control process remains the sole owner of propulsion commands. BlueOS is
responsible for operating-system setup and monitoring, while the ground-control UI is
responsible for presentation and operator requests. Neither BlueOS nor the UI may be a
required real-time link in the propulsion control loop.

This is a safety-critical change. Implementation must be split into reviewable phases,
validated in simulation and hardware-in-the-loop, and reviewed by an ArduPilot control
systems maintainer before in-water operation.

## 2. Goals

- Send final ArduSub motor outputs to individually addressed VESCs over CAN.
- Command the VESC internal speed PID with signed electrical-RPM setpoints, producing
  regulated propeller speed instead of open-loop duty or current control.
- Preserve existing ArduSub arming, disarming, emergency stop, spool-state, direction,
  output limiting, and motor-test behavior.
- Receive per-VESC RPM, current, duty cycle, voltage, temperature, and communication
  health where supported by the configured VESC firmware.
- Feed VESC telemetry into the existing `AP_ESC_Telem` frontend.
- Detect a thruster that does not produce the expected response, without triggering on
  low-command, transient, or stale-data conditions.
- Allow an operator to disable or restore a thruster explicitly.
- Prevent a failed or disabled thruster from receiving non-zero commands.
- Reallocate requested vehicle wrench to the remaining thrusters when the remaining
  actuator set permits it.
- Report the applied failure mask, fault reasons, degraded axes, and actuator telemetry
  to the ground-control UI.
- Retain PWM output for non-VESC devices and optionally for unmasked motor outputs.

## 3. Non-goals

- Running the primary propulsion loop in Calypso UI or a BlueOS extension.
- Using `SERVO_OUTPUT_RAW` or another MAVLink telemetry stream as the production motor
  command transport.
- Implementing VESC firmware configuration or firmware updates in ArduSub.
- Commanding propulsion through VESC duty, current, or relative-current control in the
  initial implementation.
- Guaranteeing full six-degree-of-freedom control after every actuator failure.
- Automatically re-enabling a thruster immediately after telemetry recovers.
- Supporting arbitrary vendor-specific CAN adapters that do not expose a Linux
  SocketCAN interface.
- Replacing ArduSub attitude, depth, or position controllers.

## 4. Existing architecture and verified integration points

### 4.1 ArduSub motor mixer

`AP_Motors6DOF` defines a contribution vector for every enabled motor:

```text
[roll, pitch, yaw, vertical, forward, lateral]
```

The standard frame matrices are created in
`libraries/AP_Motors/AP_Motors6DOF.cpp`. The current mixer calculates motor commands
from those fixed factors, applies output constraints and direction, converts the result
to an output value, and writes it through the ArduPilot RC output abstraction.

The existing `motor_enabled[]` state is populated while a frame is configured. It is
not currently a public runtime fault-management interface.

### 4.2 CAN on Linux

ArduPilot already contains a Linux SocketCAN implementation in:

```text
libraries/AP_HAL_Linux/CANSocketIface.cpp
libraries/AP_HAL_Linux/CANSocketIface.h
```

Linux CAN interface zero is bound to `can0`, interface one to `can1`, and so on. The
Navigator hardware definition does not currently declare a CAN interface. The custom
Navigator build therefore needs to enable the required number of CAN interfaces, with
the final mechanism reviewed against upstream build-size and configuration policy.

The VESC protocol should use `AP_CANManager` and `AP_HAL::CANIface`; it must not open a
platform-specific Linux socket from a shared protocol library.

### 4.3 Existing CAN ESC driver pattern

`AP_PiccoloCAN` is the closest existing architectural reference. It:

- implements `AP_CANDriver`;
- attaches to an `AP_HAL::CANIface` supplied by `AP_CANManager`;
- copies final motor functions from `SRV_Channels`;
- sends commands from a background thread;
- observes soft-armed state;
- publishes received ESC data through `AP_ESC_Telem_Backend`.

The proposed VESC backend should follow this pattern where practical instead of adding
an ArduSub-only socket or bypassing `SRV_Channels`.

## 5. System architecture

```text
Pilot / autonomous setpoints
             |
             v
ArduSub attitude, depth and position controllers
             |
             v
Fault-aware AP_Motors6DOF control allocator
             |
             v
Final SRV motor functions and safety state
             |
             v
AP_VESC CAN protocol driver <------ VESC status frames
             |                              |
             v                              v
AP_CANManager / AP_HAL CANIface       AP_ESC_Telem
             |
             v
Linux SocketCAN can0
             |
             v
USB-CAN adapter ---- CAN bus ---- VESC 1..N ---- thrusters
```

BlueOS configures and monitors `can0` before ArduSub starts. ArduSub owns all VESC
command generation, command timeout supervision, telemetry interpretation, and active
failure state. The UI communicates with ArduSub over MAVLink but is not required for
continued safe control.

## 6. VESC CAN transport requirements

### 6.1 Supported adapter class

The initial implementation shall support USB-CAN adapters that appear as a native
SocketCAN network device. A `gs_usb`-compatible adapter is a suitable reference device.
SLCAN may be investigated separately but is not an acceptance target because its serial
transport adds latency and throughput constraints.

### 6.2 Bus setup

- BlueOS shall create and bring up `can0` before ArduSub initializes its CAN driver.
- Bus bitrate shall match every VESC on the bus.
- Automatic bus restart should be configured where supported.
- Every VESC shall have a unique controller ID.
- The physical bus shall use a linear topology, appropriate twisted pair, and one
  termination resistor at each end.
- An isolated USB-CAN adapter is recommended for the production vehicle.
- Bus ground/reference and shielding shall follow the adapter and VESC manufacturers'
  requirements.

An indicative host command is:

```sh
ip link set can0 down
ip link set can0 type can bitrate <configured-bitrate> restart-ms 100
ip link set can0 up
```

The bitrate is a deployment setting, not a hard-coded assumption in the protocol
driver. The current Linux HAL records the requested bitrate but expects the OS-facing
interface to have already been configured.

### 6.3 VESC frame encoding

The implementation shall be validated against the pinned VESC firmware source used on
the vehicle. In the upstream VESC implementation reviewed for this specification,
direct CAN commands use an extended CAN identifier constructed as:

```text
extended_id = controller_id | (packet_type << 8)
```

The propulsion command used by this design is:

| VESC packet | Payload | Intended use |
| --- | --- | --- |
| `CAN_PACKET_SET_RPM` | signed int32 electrical RPM | VESC internal speed-PID setpoint |

Multi-byte fields use the byte order implemented by the VESC buffer helpers. Unit tests
shall use golden frames generated from the pinned VESC source rather than relying only
on a duplicated encoder implementation.

`CAN_PACKET_SET_RPM` calls the VESC speed-control path and resets the VESC command
timeout. Positive and negative setpoints request opposite directions. Duty and current
packets may be decoded for diagnostics but shall not be used for propulsion commands in
the initial implementation.

### 6.4 Speed-control units and propeller conversion

VESC firmware expresses the speed command and the standard CAN status speed in
electrical revolutions per minute (eRPM). For a direct-drive motor:

```text
motor mechanical RPM = eRPM / pole_pairs
propeller RPM = motor mechanical RPM
```

For a transmission where `gear_ratio` is motor revolutions per propeller revolution:

```text
eRPM = propeller_RPM * pole_pairs * gear_ratio
```

The motor pole-pair count and gear ratio shall therefore be explicit configuration or
deployment-manifest values. They must not be inferred from a generic motor type. The UI
may display mechanical propeller RPM, while CAN encoding and the VESC PID continue to
use eRPM.

### 6.4.1 Confirmed initial propulsion configuration

The initial Calypso propulsion configuration is:

| Property | Confirmed value |
| --- | --- |
| Motor magnetic poles | 14 |
| Motor pole pairs | 7 |
| Transmission | Direct drive |
| Gear ratio | 1.0 motor revolution per propeller revolution |
| Configured maximum electrical speed magnitude | 17000 eRPM |
| Corresponding maximum mechanical propeller speed | approximately 2429 RPM |

The conversion for this installation is:

```text
propeller_RPM = eRPM / 7
eRPM = propeller_RPM * 7
```

Until forward and reverse operation are characterized separately, the design uses
`-17000 eRPM` to `+17000 eRPM` as a provisional symmetric command range. This does not
assert that forward and reverse thrust, efficiency, current, or safe speed limits are
symmetric.

The VESC speed PID, minimum eRPM, speed ramp, current limits, and reverse behavior shall
be configured and validated in VESC Tool for the installed motor and load. ArduSub shall
not implement a competing low-level speed PID. It supplies the target and monitors the
signed tracking error.

The VESC `s_pid_min_erpm` behavior creates a low-speed region where the controller may
not enter or remain in speed mode. Zero, deadband, startup, and direction-reversal
behavior must be tested with the pinned firmware. The ArduSub mapping shall avoid
oscillating across that threshold.

### 6.5 Thrust command to speed setpoint

`AP_Motors6DOF` outputs a normalized actuator contribution that represents requested
thrust, not requested RPM. Propeller thrust is generally nonlinear with speed. A useful
first model is:

```text
thrust = K * sign(RPM) * abs(RPM)^p
target_RPM = sign(thrust) * RPM_max * abs(thrust)^(1/p)
```

where `p` is often near 2 but shall be determined from measured thruster data. Forward
and reverse may require different coefficients, maximum speeds, and exponents. A simple
linear mapping from normalized thrust to RPM is only acceptable if test data supports
it or if the resulting control error is explicitly accepted for an early bench test.

With the confirmed initial configuration and a provisional quadratic thrust model, the
first mapping evaluated on the bench is:

```text
target_eRPM = sign(thrust) * 17000 * sqrt(abs(thrust))
```

This formula is a test baseline, not a validated propulsion model. It shall be replaced
or parameterized using measured forward and reverse thrust-versus-RPM data before
fault-aware allocation is approved for in-water use.

The mapping shall be monotonic, continuous at zero, bounded by configured forward and
reverse eRPM limits, and independently testable. It shall preserve zero through disarm,
emergency stop, failed-motor masking, and spool-state transitions.

### 6.6 Command rate and bus load

- Command rate shall be configurable, with an initial target of 100 Hz per active VESC.
- The allowed range shall be bounded to prevent bus saturation.
- The implementation shall calculate or document worst-case bus utilization for the
  selected VESC count, command rate, telemetry rates, and CAN bitrate.
- Telemetry configuration shall not be allowed to starve command frames.
- Transmit deadlines and error counters supplied by `AP_HAL::CANIface` shall be used.

### 6.7 VESC-side timeout

Every propulsion VESC shall have its internal command timeout enabled. Loss of incoming
commands must produce zero torque or another explicitly reviewed safe state.

The VESC timeout shall:

- exceed normal command jitter and at least three nominal command periods;
- remain short enough to stop a thruster after ArduSub, USB, or CAN failure;
- be verified experimentally with the exact VESC firmware and configuration;
- not depend on a stop frame being successfully transmitted.

The deployment checklist shall record the configured timeout and timeout-brake-current
behavior for every controller.

## 7. ArduPilot VESC driver

### 7.1 Library structure

The proposed implementation is a new optional library, provisionally named `AP_VESC`:

```text
libraries/AP_VESC/AP_VESC.cpp
libraries/AP_VESC/AP_VESC.h
libraries/AP_VESC/AP_VESC_config.h
libraries/AP_VESC/tests/
```

The class should implement `AP_CANDriver` and `AP_ESC_Telem_Backend`. It shall be
compile-time guarded and omitted from targets that do not enable it.

`AP_CAN::Protocol` requires a new, non-conflicting protocol value. The actual value is
not assigned by this document and must be selected during implementation without
reusing a retired value.

### 7.2 Output ownership and mapping

The driver shall consume final motor outputs from `SRV_Channels`, following the existing
CAN ESC driver pattern. This ensures that VESC commands reflect the actual armed,
disarmed, motor-test, reversal, and spool-state output.

Each selected motor output maps to exactly one VESC controller ID. Configuration shall
reject or produce a pre-arm failure for:

- duplicate controller IDs;
- invalid IDs;
- a selected motor without an assigned VESC ID;
- a VESC output that conflicts with another active CAN ESC protocol;
- a required CAN interface that is unavailable;
- missing required VESC telemetry when strict pre-arm discovery is enabled.

The same propulsion function must not be actively driven by both PWM and VESC CAN unless
a separately reviewed redundancy design explicitly requires it.

### 7.3 Proposed parameters

Names are provisional and must comply with ArduPilot's 16-character full parameter-name
limit. Parameters should be a subgroup of the selected CAN driver, similar to other CAN
protocol drivers.

| Parameter concept | Purpose |
| --- | --- |
| enable/output mask | Select motor outputs sent to VESCs |
| output rate | Command transmission frequency |
| per-output VESC ID | Map ArduPilot motor number to VESC controller ID |
| direction | Optional CAN-side sign inversion, with a single authoritative direction setting |
| pole pairs | Convert between electrical and mechanical RPM |
| gear ratio | Convert motor RPM to propeller RPM; one for direct drive |
| forward/reverse maximum eRPM | Bound signed speed commands independently |
| forward/reverse thrust curve | Convert normalized requested thrust to speed setpoint |
| minimum usable eRPM/deadband | Coordinate low-speed behavior with the VESC speed PID |
| telemetry timeout | Maximum accepted age of required status data |
| discovery check | Select warning or pre-arm failure for absent controllers |

Runtime health and active failure state shall not be stored as ordinary persistent
parameters. A reboot must never silently turn an automatically failed actuator back on,
nor permanently disable it merely because a transient fault changed a saved parameter.
Boot policy shall be explicit and covered by an acceptance test.

### 7.4 Arming and stop behavior

- While disarmed, selected VESCs which have recently announced themselves shall receive
  zero commands at the configured rate. No frames are addressed to absent controllers,
  so an intentionally unpowered propulsion bus does not accumulate transmit failures.
- The transition to armed shall not send non-zero commands until normal ArduSub spool
  logic permits it.
- Disarm and motor emergency stop shall make the next available command to every selected
  VESC zero.
- The driver shall continue attempting zero commands for a bounded interval after a
  transition to the safe state.
- Process termination and USB disconnection safety shall rely on the VESC-side timeout.
- A failed or manually disabled motor shall always be forced to zero after the allocator,
  independent of controller output or integrator state.

The aggregate power/readiness states exposed by the initial driver are `POWERED_OFF`,
`WAITING_TELEMETRY`, `READY`, `ARMED`, `DISARM_FLUSH`, and `FAULT`. Presence is derived
from fresh periodic VESC `STATUS_1` frames. Consequently every installed VESC must be
configured to broadcast that frame at a reviewed rate before this arming sequence can
be enabled. `VE_STOP_MS` defines the bounded zero-command flush (default 500 ms), and
`VE_TLM_REQ` defaults to strict pre-arm discovery.

The Calypso arming sequence is:

1. The operator requests ARM in Calypso UI.
2. The UI commands both battery pods to enable all VMOT outputs and verifies all six
   physical power-feedback bits.
3. ArduSub remains disarmed, addresses only discovered VESCs with zero eRPM, and reports
   `READY` only when the expected and fresh-presence masks match.
4. The UI sends `MAV_CMD_COMPONENT_ARM_DISARM` only after `READY`; ArduSub remains the
   final arming and pre-arm-check authority.
5. On DISARM the UI sends the MAVLink disarm first, waits for ArduSub's zero-command
   flush-complete indication, and only then removes VMOT power. A timeout leaves VMOT
   powered and reports an error rather than cutting power before the flush.

### 7.5 Telemetry

The first implementation shall decode the standard periodic VESC status frames needed
to provide, where configured by the VESC:

- electrical RPM;
- motor current;
- input current;
- applied duty cycle;
- input voltage;
- MOSFET temperature;
- motor temperature;
- message age and receive count.

Values supported by `AP_ESC_Telem_Backend` shall be published there using the motor
index associated with the VESC controller ID. Unsupported or absent fields must not be
reported as valid zero values.

The basic periodic status frames do not necessarily contain the active VESC fault code.
Retrieving detailed fault state through forwarded VESC communication packets is a later
capability unless it is implemented and tested explicitly.

Unknown controller IDs, malformed payloads, duplicate IDs, stale telemetry, and CAN
receive errors shall be counted and made observable without flooding `STATUSTEXT`.

## 8. Thruster health management

### 8.1 State machine

Each configured thruster shall have one of these runtime states:

```text
UNSEEN -> HEALTHY -> SUSPECT -> FAILED
             ^          |          |
             |          v          v
             +------ RECOVERING   MANUAL_DISABLED
```

Exact transitions must be documented in code and unit tested. `FAILED` shall require an
explicit operator acknowledgement or a conservative recovery sequence before the motor
can contribute again. `MANUAL_DISABLED` shall never clear automatically.

### 8.2 Detection inputs

The detector may use:

- final command sent to the VESC;
- telemetry freshness;
- measured electrical RPM;
- motor and input current;
- measured duty cycle;
- temperature limits;
- detailed VESC fault code, if implemented;
- CAN transmit and receive health.

No-response detection shall only accumulate evidence when the absolute command exceeds
a configurable validation threshold. Zero or very small commands cannot prove that a
thruster has failed.

For speed control, response consistency shall primarily compare commanded signed eRPM
with measured signed eRPM after the configured acceleration grace period. Current and
duty remain supporting evidence: high duty/current with persistent speed error suggests
a blocked or overloaded thruster, while low duty/current and missing speed response may
indicate a controller, mapping, or communication problem.

Successful RPM tracking proves that the motor is rotating; it does not by itself prove
that the propeller is attached or producing the expected thrust. A lost propeller or
coupling may reach target RPM with abnormally low current. Mechanical-contribution
detection should therefore compare current/duty against a calibrated RPM-and-load model
and report a distinct confidence level. Without a thrust sensor or vehicle-response
observer, ArduSub shall not claim direct measurement of generated thrust.

Direction mismatch detection shall account for signed command and signed RPM where the
installed sensors and VESC firmware provide reliable sign information.

### 8.3 Debounce and confidence

- A single missing frame shall not fail a thruster.
- Startup, arming transitions, command reversal, and acceleration transients shall have
  explicit grace periods.
- Failure and recovery shall use different thresholds or durations to provide hysteresis.
- Telemetry-loss and mechanical-no-response faults shall remain distinguishable.
- Detection thresholds shall be validated using logs from healthy thrusters across the
  expected voltage, load, and operating envelope.
- Automatic isolation shall be independently enableable from telemetry and warning-only
  operation.

### 8.4 Manual control

The operator shall be able to request disable and recovery through a MAVLink-accessible
runtime interface. ArduSub shall acknowledge the applied state. The UI must display the
state reported by ArduSub, not merely the state it requested.

The production interface should have command semantics rather than using a saved
parameter as a transient mailbox. Standard MAVLink mechanisms should be preferred. A
custom dialect addition requires separate compatibility review and regeneration of all
affected MAVLink clients.

## 9. Fault-aware control allocation

### 9.1 Allocation model

Let `B` be the configured six-by-N thruster effectiveness matrix and `u` the N-element
thruster command vector. The requested vehicle wrench is:

```text
w_requested = [roll, pitch, yaw, vertical, forward, lateral]
```

The allocator shall solve for active thrusters only:

```text
B_active * u_active ~= w_requested
u_failed = 0
u_min <= u_active <= u_max
```

### 9.1.1 Confirmed initial Calypso layout

The initial vehicle follows the six-thruster BlueRobotics vectored numbering when
viewed from above with the bow at the top:

```text
                 Bow

          Motor 2     Motor 1
          Motor 6     Motor 5    vertical thrusters
          Motor 4     Motor 3

                 Stern
```

Motors 1 through 4 are horizontal vectored thrusters. Three mechanically selectable
installations shall be supported and tested: 40, 45, and 50 degrees measured from the Y
axis. These shall be validated geometry profiles generated by the same allocator
implementation, not independent allocation algorithms. The default profile shall be
45 degrees. Changing the profile shall require the vehicle to be disarmed and shall
rebuild and validate the effectiveness matrix before arming.

The initial implementation shall expose a discrete angle-profile parameter rather than
an unrestricted floating-point angle. Its allowed values and nominal behavior are:

| Angle from Y | Surge magnitude | Sway magnitude | Nominal preference |
| ---: | ---: | ---: | --- |
| 40 deg | 0.643 | 0.766 | lateral |
| 45 deg | 0.707 | 0.707 | balanced, default |
| 50 deg | 0.766 | 0.643 | forward |

Unsupported values shall produce a pre-arm configuration failure. The selected profile
shall be reported to the UI and recorded in logs.

Using the ArduPilot body-frame convention `+X` forward and `+Y` starboard, the confirmed
horizontal-thruster positions are:

| Motor | X (mm) | Y (mm) | Location |
| --- | ---: | ---: | --- |
| 1 | +200 | +174 | forward starboard |
| 2 | +200 | -174 | forward port |
| 3 | -200 | +174 | aft starboard |
| 4 | -200 | -174 | aft port |

The position measurement is taken at the motor support, which is centered on the
propeller shaft/thrust line. The common absolute dimensions are therefore
`|X| = 200 mm` and `|Y| = 174 mm`. The coordinate origin is the geometric center of the
vehicle and coincides with the confirmed center of mass used by the allocator.

The installation angle is measured from the vehicle lateral Y axis. The 40-degree
installation is therefore equivalent to 50 degrees from the longitudinal X axis. The
nominal horizontal force magnitudes are `sin(angle)` in surge and `cos(angle)` in sway.
For the confirmed symmetric layout, the magnitude of the yaw lever coefficient is:

```text
yaw_lever = 200 * cos(angle_from_Y) + 174 * sin(angle_from_Y)
```

This gives approximately `265.0 mm` at 40 degrees, `264.5 mm` at 45 degrees, and
`261.9 mm` at 50 degrees before normalization. At 40 degrees from Y, the geometry
provides about 19 percent more unsaturated lateral force than forward force for equal
per-thruster limits. At 45 degrees the two components are equal. At 50 degrees, forward
force is about 19 percent greater than lateral force. Positive command direction shall
be verified during commissioning.

Motors 5 and 6 thrust vertically and have `x = 0`. Their lateral coordinates are on
opposite sides of the vehicle, so together they provide heave and roll authority. They
do not provide pitch authority in the confirmed nominal geometry. The nominal vehicle
therefore has five actuated degrees of freedom: surge, sway, heave, roll, and yaw.

The confirmed vertical-thruster positions are:

| Motor | X (mm) | Y (mm) | Location |
| --- | ---: | ---: | --- |
| 5 | 0 | +247 | starboard vertical |
| 6 | 0 | -247 | port vertical |

Equal vertical thrust from motors 5 and 6 produces heave with cancelling roll moments.
Differential thrust produces roll with a `247 mm` lateral lever arm and no nominal pitch
moment. After loss of either vertical thruster, vertical force and roll moment are
unavoidably coupled according to `roll_moment = Y * vertical_force`; the ascent-priority
degraded policy must account for this coupling explicitly.

All horizontal thrusters are installed with the propeller toward the stern and the
electric motor toward the bow. Left- and right-handed propellers are fitted to obtain
counter-rotation. Propeller handedness is not itself an allocator coefficient; the
commissioned VESC command sign for each motor shall be verified independently so that
a positive allocator contribution produces the expected vehicle force.

Yaw effectiveness shall be calculated from both thrust direction and lever arm, rather
than being inferred from propeller handedness.

Removing a motor contribution from the current sum is not sufficient. The remaining
commands must be recomputed, and command saturation must be handled without assuming
that every requested axis remains independently achievable.

### 9.2 Rank and controllability

For every supported frame and failure mask, tests shall calculate the rank and the
achievable subspace of `B_active`. The allocator shall expose a degraded-axis or
capability result for telemetry and controller decisions.

Examples such as a standard vectored vehicle losing one of two vertical thrusters must
not be described as fully controllable without a matrix/rank analysis. Passive vehicle
stability may help operationally but does not restore actuator rank.

### 9.3 Allocation algorithm design gate

The final algorithm requires control-systems review. Candidate approaches are:

1. Precomputed degraded matrices for each supported single-thruster failure.
2. Weighted damped least-squares allocation with bounds.
3. A bounded active-set allocator with explicit axis priorities.

The first implementation should favor deterministic, testable behavior for known frame
types. A plain unconstrained pseudoinverse is not acceptable as the final implementation
because actuator saturation and underactuated cases require explicit handling.

### 9.4 Axis priority

Axis priority shall be configurable only within a reviewed, bounded policy. The initial
ArduSub policy should prioritize vehicle safety and recoverability, normally:

1. attitude axes needed to prevent loss of control;
2. vertical/depth authority;
3. yaw;
4. forward and lateral translation.

This ordering is not universal. The approved policy must be defined per frame and
operational mode, and it must define what happens when depth and attitude cannot both be
controlled.

For the confirmed Calypso six-thruster layout, loss of one vertical thruster leaves only
one actuator for the coupled heave/roll subspace. Independent heave and roll control is
then impossible. The reviewed degraded policy shall prioritize positive heave command
for ascent over roll regulation. The allocator shall expose loss of roll authority,
prevent roll-controller windup, and command a controlled ascent or surface failsafe as
appropriate to the active mode. It shall not report the vehicle as retaining all five
nominal degrees of freedom.

Loss of one horizontal thruster may retain rank for surge, sway, and yaw using the three
remaining horizontal thrusters. This shall be confirmed for the 40-degree, 45-degree,
and 50-degree profiles using the final measured coordinates and bounded allocation
tests; reduced authority and asymmetric saturation shall still be reported.

### 9.5 Controller interaction

- Limit flags shall reflect authority lost through the active allocation, so integrators
  do not continue winding up against unavailable actuators.
- Mode controllers shall be told when a required axis is unavailable.
- Modes that require an unavailable axis shall reject entry or transition to a reviewed
  degraded/failsafe behavior.
- Manual mode behavior after a failure shall be defined and tested separately from
  stabilized and autonomous modes.
- Allocation changes shall avoid discontinuous command steps where safety permits, but
  zeroing the failed actuator takes precedence over smooth transfer.

## 10. MAVLink, logging, and UI contract

### 10.1 Required information from ArduSub

The ground-control UI needs:

- configured motor-to-VESC-ID mapping;
- applied manual-disable mask;
- automatically detected failure mask;
- per-thruster runtime state and fault reason;
- command and available VESC telemetry;
- telemetry age;
- current degraded axes/capability;
- CAN interface health and aggregate error counts;
- acknowledgement of disable and recovery requests.

Existing standard MAVLink ESC telemetry/status messages and ArduPilot logging should be
used where they preserve the required meaning. `STATUSTEXT` is appropriate for state
transitions, not continuous telemetry. New MAVLink messages shall only be proposed after
documenting why existing messages cannot represent the required state.

For the first integration, aggregate sequencing state is carried at low rate in
`NAMED_VALUE_INT` fields `VESC_STATE`, `VESC_EXP`, `VESC_PRES`, `VESC_MISS`, and
`VESC_SAFE`. This is an explicitly temporary experimental bridge for Calypso UI, not a
stable public MAVLink API. Standard ESC telemetry continues through the existing
ArduPilot ESC telemetry backend. Before upstreaming or freezing the external contract,
the aggregate fields shall move to a reviewed dialect message or another standard
representation with identical semantics.

### 10.2 Logging

DataFlash logs shall contain enough data to reproduce a fault decision and allocation
transition:

- timestamped motor commands;
- VESC telemetry and age;
- detector state and accumulated evidence;
- failure/manual masks before and after transitions;
- requested wrench and allocated motor commands;
- saturation and degraded-axis flags;
- CAN errors and missed command deadlines.

Logging rate must remain bounded and must not interfere with control or CAN timing.

### 10.3 UI behavior

Calypso UI shall:

- show reported rather than assumed actuator state;
- distinguish warning, suspected failure, automatic isolation, and manual disable;
- require confirmation for manual recovery while armed;
- show which vehicle axes are degraded;
- preserve access to emergency stop and disarm independently of this feature;
- never claim a motor is stopped based only on a command acknowledgement.

## 11. BlueOS responsibilities

BlueOS does not perform mixing or continuously translate MAVLink servo telemetry into
VESC commands. Its responsibilities are limited to:

- USB device access and stable adapter identification;
- SocketCAN interface creation and bitrate configuration;
- bringing `can0` up before ArduSub;
- exposing interface state for diagnostics;
- installing and launching the custom Navigator `ardusub` binary;
- preserving a tested rollback path to the previous binary and parameter set.

A small BlueOS service or extension may configure and monitor `can0`. If it stops, the
already configured interface may continue operating, but propulsion safety must not
depend on that behavior.

## 12. Configuration and deployment safety

- Back up all ArduSub parameters before installing the custom binary.
- Pin the ArduPilot commit, VESC firmware version, and VESC configuration used for every
  test campaign.
- Record motor number, physical location, VESC ID, serial number, direction, and current
  limits in a deployment manifest.
- Record motor pole pairs, transmission ratio, forward/reverse eRPM limits, VESC speed
  PID gains, minimum eRPM, and eRPM ramp settings in the deployment manifest.
- Verify that PWM propulsion outputs are inactive before connecting powered CAN VESCs.
- Verify VESC timeout behavior with the propeller removed or mechanically isolated.
- Provide a physical method to remove propulsion power during bench testing.
- Never perform first-time direction or allocation tests with exposed propellers.
- Keep a known-good ArduSub Navigator binary and parameter backup available for rollback.

## 13. Implementation phases

### Phase 0: design and fixtures

- Pin the VESC firmware/protocol baseline.
- Select and validate the USB-CAN adapter on Raspberry Pi 5 and BlueOS.
- Add golden VESC frame encoder/decoder vectors.
- Create a `vcan`-based test fixture and a simulated VESC responder.
- Review the control-allocation proposal before implementing automatic isolation.

### Phase 1: CAN output only, bench use

- Enable one SocketCAN interface in the Navigator build.
- Add the optional `AP_VESC` CAN driver.
- Implement motor-to-ID mapping, thrust-to-eRPM conversion, signed RPM commands,
  arm/disarm, zero output, timeouts, pre-arm checks, and driver statistics.
- Keep automatic fault isolation disabled.
- Validate one motor, then multiple motors, without propellers.

### Phase 2: telemetry and warning-only diagnostics

- Decode periodic VESC status frames.
- Publish supported values through `AP_ESC_Telem`.
- Add telemetry-age and response-consistency logging.
- Run fault detection in warning-only mode and collect healthy/faulted datasets.

### Phase 3: manual isolation and degraded allocation

- Add the runtime manual disable/recovery interface.
- Force disabled outputs to zero.
- Implement reviewed degraded allocation for supported single-thruster failures.
- Add rank, saturation, limit-flag, and mode-behavior tests.

### Phase 4: automatic isolation

- Enable automatic transition from `SUSPECT` to `FAILED` only after thresholds are
  validated from representative logs.
- Add operator acknowledgement and conservative recovery logic.
- Conduct tethered, progressively expanded in-water testing.

## 14. Test plan

### 14.1 Unit tests

- Encode `CAN_PACKET_SET_RPM` into golden extended CAN frames, including positive,
  negative, zero, and bounded values.
- Decode valid, truncated, overlong, wrong-ID, and unknown status frames.
- Validate signed eRPM values, byte order, saturation, pole-pair/gear conversion,
  thrust-curve conversion, and NaN/overflow rejection.
- Validate duplicate mapping and invalid configuration pre-arm failures.
- Exercise every health-state transition with deterministic timestamps.
- Confirm that low command never accumulates mechanical-failure evidence.
- Confirm that failed and manually disabled outputs remain zero.
- Verify matrix rank for every supported frame and each single-thruster failure.
- Verify allocator bounds, priorities, saturation flags, and underactuated cases.

### 14.2 Virtual CAN integration

- Run the Linux CAN backend against `vcan0`.
- Simulate all configured VESC IDs at the target command and telemetry rates.
- Inject dropped frames, stale status, wrong direction, zero RPM, bus-off/down events,
  delayed frames, malformed payloads, and duplicate IDs.
- Measure command interval distribution and missed deadlines under CPU and logging load.
- Terminate ArduSub and verify the simulated VESC timeout reaches zero command.

### 14.3 SITL/autotest

- Add ArduSub tests for manual disable, automatic failure injection, allocation changes,
  degraded-mode transitions, and recovery rejection/acceptance.
- Confirm no behavior change when `AP_VESC` is disabled.
- Confirm existing ArduSub frame configurations and motor tests still pass.

### 14.4 Hardware-in-the-loop

1. USB-CAN and one unpowered VESC bus inspection.
2. One powered VESC with no motor.
3. One motor without propeller.
4. All motors without propellers.
5. Propellers in a guarded test tank with the vehicle restrained.
6. Tethered low-gain maneuvering.
7. Deliberate single-thruster disconnects for each physical position.
8. Progressive stabilized and autonomous-mode tests only after log review.

Every stage requires an emergency power-removal method and review of logs before moving
to the next stage.

## 15. Acceptance criteria

The feature is not production-ready until all of the following are demonstrated:

- The Navigator build succeeds with the VESC feature both enabled and disabled.
- Existing PWM-only ArduSub behavior is unchanged when VESC output is disabled.
- Selected VESCs receive correctly encoded signed eRPM commands at the configured rate.
- Measured propeller RPM agrees with commanded eRPM after pole-pair and gear-ratio
  conversion within the validated steady-state tolerance.
- Speed tracking remains stable through zero, startup, bounded command steps, and
  forward/reverse transitions in the validated operating envelope.
- Disarm, emergency stop, driver error, process termination, and USB disconnect all
  result in zero motor torque within the reviewed maximum stopping time.
- No duplicate subsystem writes non-zero commands to the same propulsion output.
- Telemetry values and units match VESC Tool or another independent measurement.
- Fault-state transitions are reproducible from logged inputs.
- Healthy thrusters do not false-trigger across the validated operating envelope.
- Every failed or disabled thruster is held at zero through all modes and motor tests.
- Rank and degraded-axis results match offline matrix analysis.
- Remaining motor commands respect bounds and the approved axis-priority policy.
- Controller integrators do not wind up indefinitely against unavailable authority.
- UI commands are acknowledged by ArduSub and UI state matches ArduSub's applied state.
- Rollback to the prior Navigator binary and parameter set is documented and tested.

## 16. Open design decisions

- Exact VESC hardware and firmware versions to support initially.
- Selected USB-CAN adapter and whether galvanic isolation is mandatory.
- CAN bitrate, command rate, telemetry rates, and measured worst-case utilization.
- VESC speed-PID gains, minimum eRPM, ramp rate, and allowable steady-state tracking error.
- Measured forward and reverse thrust-versus-RPM curves and their fitted exponents.
- Forward and reverse eRPM limits for each thruster type.
- Neutral behavior: coast versus braking, including reverse transition behavior.
- Exact VESC timeout and timeout-brake-current settings.
- Parameter names and the new `AP_CAN::Protocol` enumeration value.
- Whether Navigator enables CAN unconditionally or through a custom build option.
- Standard MAVLink representation for actuator fault state and degraded axes.
- Final allocation algorithm, axis weights, saturation strategy, and mode fallbacks.
- Manufacturing and installation tolerances for the validated 40-degree, 45-degree,
  and 50-degree profiles.
- Recovery policy after intermittent telemetry or mechanical response returns.
- Maximum supported simultaneous failures; initial scope should be one failed thruster.
- Required behavior if the CAN interface fails and all propulsion VESCs disappear at once.

## 17. References

- ArduPilot source: <https://github.com/ArduPilot/ardupilot>
- ArduPilot CAN architecture: <https://ardupilot.org/dev/docs/can-bus.html>
- ArduSub frame configurations: <https://ardupilot.org/sub/docs/sub-frames.html>
- VESC firmware source: <https://github.com/vedderb/bldc>
- VESC CAN implementation: <https://github.com/vedderb/bldc/blob/master/comm/comm_can.c>
- BlueOS advanced usage and custom firmware installation:
  <https://blueos.cloud/docs/stable/usage/advanced/>

## 18. Contribution note

This initial specification was prepared with AI assistance. The human contributors who
implement, review, test, and submit the change remain responsible for validating every
requirement and every line of resulting safety-critical code.
