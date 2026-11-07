# DeepEx VESC maintenance mode (BlueOS extension)

Status: implementation candidate, tested only against a simulated CAN bus and a simulated
BlueOS API. **Not yet validated with real VESCs, VESC Tool or BlueOS.**

Lets VESC Tool on the surface laptop configure the VESCs on `can0` through the tether, without
opening the housings. It complements the CAN propulsion path in
[`ArduSub/VESC_CAN_V1.md`](../../../ArduSub/VESC_CAN_V1.md).

## What it does

A page in the BlueOS sidebar ("VESC Maintenance") with two buttons:

- **Maintenance ON**: refuses if the vehicle is armed (MAVLink HEARTBEAT through mavlink2rest);
  stops the autopilot (`POST /ardupilot-manager/v1.0/stop`); starts the bridge.
- **Maintenance OFF**: stops the bridge and closes the VESC Tool connection; starts the autopilot
  (`POST /ardupilot-manager/v1.0/start`).

Note: the BlueOS stop endpoint also disables "start on boot". If the vehicle is power-cycled in
maintenance, ArduSub does not start until **Maintenance OFF** (or Autopilot Firmware → start) is used.

The bridge (`vesc_can_tcp_bridge.py`) listens on TCP 65102 and speaks the VESC packet protocol to
VESC Tool; on the CAN side it uses the VESC buffer protocol (`PROCESS_SHORT_BUFFER`,
`FILL_RX_BUFFER(_LONG)`, `PROCESS_RX_BUFFER`) with its own controller id (default 120, must be unused).
`COMM_FORWARD_CAN` is routed to the requested id and `COMM_PING_CAN` (CAN scan) is answered by the
bridge with CAN `PING`/`PONG`. Python 3 standard library only (`socket.AF_CAN`).

## Safety interlock

- The bridge never sends motor commands.
- It forwards nothing while motor-command frames (`SET_RPM`, `SET_CURRENT`, `SET_DUTY`, ...) are
  seen on the bus, and drops the VESC Tool client if they appear: ArduSub commanding the VESCs and
  VESC Tool configuring them are mutually exclusive, whatever process or container is running.
- Motor detection started from VESC Tool **does spin the motor**: vehicle secured, propeller clear.
- VESC-side command timeout remains required (see `VESC_CAN_V1.md`).

## Use

1. Disarm. Open the BlueOS page "VESC Maintenance", press **Maintenance ON**.
2. VESC Tool: Connect → TCP, host = vehicle IP (BlueOS default 192.168.2.2), port 65102.
   The default target is controller id 1; **CAN scan** lists ids 1..6 and each can be selected.
3. Terminal, configuration read/write and `dd_cal*` commands (DeepDrive firmware) work as over USB.
4. Disconnect VESC Tool, press **Maintenance OFF**, check the autopilot heartbeat in the page.

## Install (BlueOS)

Image: `ghcr.io/deepex/blueos-vesc-maintenance:0.1.0` (built by
`.github/workflows/deepex_vesc_maintenance.yml`, manual run or tag `vesc-maint-v*`; the GHCR package
must be public or the Pi must be logged in).

BlueOS → Extensions → Installed → **+**:

| Field | Value |
|---|---|
| Extension identifier | `deepex.vesc-maintenance` |
| Extension name | `VESC Maintenance` |
| Docker image | `ghcr.io/deepex/blueos-vesc-maintenance` |
| Docker tag | `0.1.0` |
| Custom settings | `{"NetworkMode": "host", "HostConfig": {"NetworkMode": "host", "RestartPolicy": {"Name": "unless-stopped"}}}` |

Host networking is required: `can0` lives in the host network namespace. The page is served on port
9135 and registered through `/register_service`.

Environment overrides: `WEB_PORT`, `VESC_IFACE`, `VESC_TCP_PORT`, `VESC_TARGET`, `VESC_BRIDGE_ID`,
`VESC_QUIET_S`, `BLUEOS_HOST`.

The bridge also runs stand-alone for bench tests (autopilot stopped by hand):
`python3 vesc_can_tcp_bridge.py --iface can0 --target 3`.

## Tests

`python3 tests/test_bridge.py` (simulated bus with controllers 1..6: firmware query, forwarding,
CAN scan, buffers 5–1500 bytes, interlock) and `python3 tests/test_app.py` (simulated BlueOS:
refusal when armed, stop/start calls, scan through the extension, client closed on OFF).

## Validation still required

- Real VESC Tool session over the tether: scan, terminal, configuration read and write compared with a
  USB dump of the same controller.
- BlueOS endpoints and mavlink2rest HEARTBEAT format on the installed BlueOS version.
- Bus load and timing with six controllers broadcasting status at 20/5 Hz.
- Behaviour when the autopilot is started from another BlueOS page while maintenance is ON (expected:
  motor commands appear only when armed in CAN mode; the interlock then drops the client).
