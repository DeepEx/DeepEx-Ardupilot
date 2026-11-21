# MAVLink Lua Modules

These .lua definition for messages can be generated using pymavlink with the following command:
(change the last path accordingly to place the files where you need them)

```bash
cd ardupilot/modules/mavlink
python ./pymavlink/tools/mavgen.py --lang Lua ./message_definitions/v1.0/all.xml --out ./modules/MAVLink --wire-protocol 2.0
```

Add `--wire-protocol 2.0` to include extension fields in the generated code

## Messages generated for the DeepEx pod battery script

`mavlink_msg_BATTERY_STATUS.lua` and `mavlink_msg_NAMED_VALUE_INT.lua` (used by `applets/pod_battery.lua`) were generated
with exactly the command above from the message definitions of this tree (`modules/mavlink` @ 13f2f735,
`message_definitions/v1.0/all.xml`), and only these two files were added; the existing generated files are unchanged
(the generator output for `mavlink_msgs.lua` and `mavlink_msg_HEARTBEAT.lua` is byte-identical to the committed ones).

Note on MAVLink 2 truncation: the sender drops trailing zero bytes of the payload. In the `mavlink_message_t` string that
the scripting binding hands to Lua, those bytes are already restored as zeros, because the C parser zero-fills the packet
up to the full message length (`mavlink_helpers.h`, "zero-fill the packet to cope with short incoming packets"), so
`mavlink_msgs.decode()` is correct for truncated frames. `applets/pod_battery.lua` nevertheless zero-fills the payload
itself before decoding, so it does not depend on that behaviour.
