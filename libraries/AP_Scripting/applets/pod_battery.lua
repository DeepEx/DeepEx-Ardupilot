--[[
  pod_battery.lua: battery monitors and power state from the two DeepEx pods (ADR-0024)

  Each pod (ESP32 distribution board) sends MAVLink 2 over the vehicle LAN to the Raspberry Pi (via the
  BlueOS router), sysid 1:
    compid 180 = BAT1 (master pod), BATTERY_STATUS id 0  -> battery monitor index 1 (BATT2_)
    compid 181 = BAT2 (slave pod),  BATTERY_STATUS id 1  -> battery monitor index 2 (BATT3_)
  Battery monitor index 0 (BATT_, the primary) is a virtual "most loaded pack": current = the highest current of
  the packs with a fresh, valid measurement; voltage = the highest of their voltages (the bus). ArduSub's own
  current limiter (MOT_BAT_CURR_MAX, AP_Motors6DOF) works on the primary battery only: with MOT_BAT_CURR_MAX = 28
  it keeps every pack under 28 A, with one pack or two (ADR-0016 level 2, SPEC-0013 6.4). When no pack has a valid
  current the primary is unhealthy and the limiter does nothing (owner: loss of the pod data = notify only).
  All three use the "Scripting" battery backend (BATT_MONITOR = BATT2_MONITOR = BATT3_MONITOR = 29). The power mode
  (NAMED_VALUE_INT PWR_MODE / PWR_RSN / PWR_BUDG) is reported as a text message and a DataFlash "PWRM" line.
  The script itself never commands the motors.

  See pod_battery.md for the contract, parameters and vehicle setup.
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: missing-parameter

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local MAV_SEVERITY = {WARNING = 4, INFO = 6}
local SCRIPT_NAME = "PodBatt"

-- ---------------------------------------------------------------- parameters

local PARAM_TABLE_KEY = 118
local PARAM_TABLE_PREFIX = "PBAT_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')

--[[
  // @Param: PBAT_ENABLE
  // @DisplayName: Pod battery script enable
  // @Description: Enable the pod battery script (battery monitors from the DeepEx pods over MAVLink). When 0 the script does nothing and the scripting battery monitors become unhealthy.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PBAT_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: PBAT_TIMEOUT_MS
  // @DisplayName: Pod battery timeout
  // @Description: A pack is unhealthy if no BATTERY_STATUS arrives from its pod for longer than this
  // @Units: ms
  // @Range: 200 10000
  // @User: Advanced
--]]
local PBAT_TIMEOUT_MS = bind_add_param('TIMEOUT_MS', 2, 1000)

--[[
  // @Param: PBAT_SYSID
  // @DisplayName: Pod MAVLink system id
  // @Description: MAVLink system id accepted from the pods (the pods use the vehicle system id, 1). Messages from other system ids are ignored.
  // @Range: 1 255
  // @User: Advanced
--]]
local PBAT_SYSID = bind_add_param('SYSID', 3, 1)

if PBAT_ENABLE:get() == 0 then
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": disabled (PBAT_ENABLE = 0)")
    return
end

-- ---------------------------------------------------------------- contract with the pods

local COMPID_BAT1 = 180   -- MAV_COMP_ID_BATTERY  (master pod)
local COMPID_BAT2 = 181   -- MAV_COMP_ID_BATTERY2 (slave pod)
local COMPID_TO_INDEX = {[COMPID_BAT1] = 0, [COMPID_BAT2] = 1}   -- pod index (= BATTERY_STATUS.id)
local MONITOR_VIRTUAL = 0          -- BATT_  : most loaded pack (input of MOT_BAT_CURR_MAX)
local function monitor_of(idx) return idx + 1 end   -- BAT1 -> BATT2_, BAT2 -> BATT3_
local MONITOR_PARAM = {[0] = "BATT_MONITOR", [1] = "BATT2_MONITOR", [2] = "BATT3_MONITOR"}

local VOLTAGE_INVALID = 65535      -- UINT16_MAX
local CURRENT_INVALID = -1         -- cA
local CONSUMED_INVALID = -1        -- mAh
local TEMPERATURE_INVALID = 32767  -- INT16_MAX, cdegC

local PWR_MODE_NAMES = {[0] = "UNKNOWN", [1] = "DUAL", [2] = "SINGLE_BAT1", [3] = "SINGLE_BAT2", [4] = "NONE"}

local NaN = 0 / 0  -- "not known" for the scripting battery backend (consumed_mah NaN = integrate the current)

local UPDATE_PERIOD_MS = 25        -- 40 Hz
local QUEUE_DEPTH = 25             -- mavlink:init queue depth (the API maximum)
-- A script call is limited to SCR_VM_I_COUNT (10000 by default). CRC checking is mandatory:
-- cap accepted-sender packets at three per call (120/s capacity at 40 Hz, versus 28/s nominal).
-- Foreign senders are dropped before CRC/decoding and have a separate queue-pop bound.
local MAX_DECODES_PER_UPDATE = 3
local MAX_POPS_PER_UPDATE = 60
local STATS_LOG_PERIOD_MS = 1000   -- PBAT log row per pod

-- ---------------------------------------------------------------- MAVLink receive

local BATTERY_STATUS_ID = mavlink_msgs.get_msgid("BATTERY_STATUS")
local NAMED_VALUE_INT_ID = mavlink_msgs.get_msgid("NAMED_VALUE_INT")

local msg_map = {}
msg_map[BATTERY_STATUS_ID] = "BATTERY_STATUS"
msg_map[NAMED_VALUE_INT_ID] = "NAMED_VALUE_INT"

-- initialize MAVLink rx: queue depth and number of rx message ids
mavlink:init(QUEUE_DEPTH, 2)
mavlink:register_rx_msgid(BATTERY_STATUS_ID)
mavlink:register_rx_msgid(NAMED_VALUE_INT_ID)

-- Full payload length of each message, from the generated field definitions.
local full_len = {}
local defs = {}
for msgid, name in pairs(msg_map) do
    local def = require("MAVLink/mavlink_msg_" .. name)
    defs[msgid] = def
    local n = 0
    for _, f in ipairs(def.fields) do
        n = n + string.packsize(f[2]) * (f[3] or 1)
    end
    full_len[msgid] = n
end

-- The fork deliberately delivers BAD_CRC packets to scripting for unknown dialects.
-- Our two messages have known definitions: verify the transmitted header/payload before zero-filling or decoding.
local function valid_crc(raw, def)
    local plen = string.byte(raw, 4)
    if #raw < 12 + plen then return false end
    local magic = string.byte(raw, 3)
    local wire
    if magic == 0xFD then
        wire = string.sub(raw, 4, 12 + plen)
    elseif magic == 0xFE then
        wire = string.char(plen) .. string.sub(raw, 7, 10) .. string.sub(raw, 13, 12 + plen)
    else
        return false
    end
    return mavlink_msgs.generateCRC(wire .. string.char(def.crc_extra)) == string.unpack("<I2", raw)
end

-- Decode only after CRC verification, using the generated field definitions.
local function decode_fields(raw, def)
    local result = {}
    local offset = 13
    for _, v in ipairs(def.fields) do
        if v[3] then
            local arr = {}
            for j = 1, v[3] do
                arr[j], offset = string.unpack(v[2], raw, offset)
            end
            result[v[1]] = arr
        else
            local value
            value, offset = string.unpack(v[2], raw, offset)
            if string.sub(v[2], 2, 2) == 'c' then
                value = (string.gsub(value, "\0", ""))
            end
            result[v[1]] = value
        end
    end
    return result
end

-- MAVLink 2 senders drop the trailing zero bytes of the payload. The C parser already restores them as zeros in the
-- mavlink_message_t that the binding hands over, but do not depend on it: zero-fill the payload up to its full
-- length (and clear anything beyond the transmitted length) before decoding.
local function zero_fill_payload(raw)
    local plen = string.unpack("<B", raw, 4)
    local msgid = string.unpack("<I3", raw, 10)
    local full = full_len[msgid]
    if full == nil or #raw < 12 + full then
        return raw
    end
    if plen >= full then
        return raw
    end
    return string.sub(raw, 1, 12 + plen) .. string.rep("\0", full - plen) .. string.sub(raw, 12 + full + 1)
end

-- ---------------------------------------------------------------- state

-- one entry per pack (battery monitor index 0 and 1)
local packs = {}
for idx = 0, 1 do
    packs[idx] = {
        rx_ms = nil,            -- time of the last BATTERY_STATUS accepted
        v = nil, i = NaN, mah = NaN, temp = NaN,   -- last decoded values (V, A, mAh, degC); nil/NaN = not valid
        healthy = nil,          -- last published health (for transition messages)
        pwr_mode = nil, pwr_rsn = nil,             -- last PWR_MODE / PWR_RSN received
        pwr_budg = nil,                            -- last PWR_BUDG received (A x 10, informative)
        pwr_pub_mode = nil, pwr_pub_rsn = nil,     -- last reported pair
        pwr_pending = false,
        id_mismatch_warned = false,
        n_batt = 0, n_named = 0, n_ignored = 0, n_crc = 0,
    }
end
local virtual = {healthy = nil, src = 255, i = NaN, v = 0}   -- last published most-loaded-pack values
local decode_budget_hits = 0       -- calls that stopped popping because the decode budget was used up
local last_stats_ms = nil

local warned_config = false
local warned_publish = {}

local function now_ms()
    return millis():toint()
end

-- ---------------------------------------------------------------- message handling

local function handle_battery_status(idx, m, t_ms)
    local p = packs[idx]
    if m.id ~= idx then
        -- compid and BATTERY_STATUS.id disagree: a mapping error would swap BAT1 and BAT2, so refuse the message
        p.n_ignored = p.n_ignored + 1
        if not p.id_mismatch_warned then
            p.id_mismatch_warned = true
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: compid %d sent battery id %d, ignored",
                          SCRIPT_NAME, idx == 0 and COMPID_BAT1 or COMPID_BAT2, m.id))
        end
        return
    end
    p.rx_ms = t_ms
    p.n_batt = p.n_batt + 1
    local mv = m.voltages[1]   -- Lua arrays start at 1: voltages[0] of the message
    if mv == nil or mv == VOLTAGE_INVALID then
        p.v = nil
    else
        p.v = mv / 1000.0
    end
    if m.current_battery == CURRENT_INVALID then p.i = NaN else p.i = m.current_battery / 100.0 end
    if m.current_consumed == CONSUMED_INVALID then p.mah = NaN else p.mah = m.current_consumed end
    if m.temperature == TEMPERATURE_INVALID then p.temp = NaN else p.temp = m.temperature / 100.0 end
    -- battery_remaining is -1 (no SOC on board) and is deliberately not used: capacity_remaining_pct is not set
end

local function handle_named_value(idx, m)
    local p = packs[idx]
    if m.name == "PWR_MODE" then
        p.pwr_mode = m.value
    elseif m.name == "PWR_RSN" then
        p.pwr_rsn = m.value
    elseif m.name == "PWR_BUDG" then
        p.pwr_budg = m.value
        p.n_named = p.n_named + 1
        return
    else
        return
    end
    p.n_named = p.n_named + 1
    p.pwr_pending = true
end

local function drain_messages()
    local pbat_sysid = PBAT_SYSID:get()
    local decodes = 0
    for _ = 1, MAX_POPS_PER_UPDATE do
        local raw, _, _ = mavlink:receive_chan()
        if raw == nil then
            return
        end
        -- cheap pre-filter on the raw mavlink_message_t header (sysid at byte 8, compid at byte 9): messages from
        -- anybody else are dropped here without paying for the CRC check and the decode
        local sysid, compid = string.byte(raw, 8, 9)
        local idx = COMPID_TO_INDEX[compid]
        if idx ~= nil and sysid == pbat_sysid then
            local t_ms = now_ms()
            local msgid = string.unpack("<I3", raw, 10)
            local def = defs[msgid]
            if def ~= nil and not valid_crc(raw, def) then
                local p = packs[idx]
                p.n_crc = p.n_crc + 1
                if p.n_crc <= 3 then
                    logger:write("PBAD", "Comp,Msg,Len,Id,Sum", "BHBBH", compid, msgid,
                                 string.byte(raw, 4), msgid == BATTERY_STATUS_ID and string.byte(raw, 45) or 255,
                                 (string.unpack("<I2", raw)))
                end
            elseif def ~= nil then
                local ok, parsed = pcall(decode_fields, zero_fill_payload(raw), def)
                if ok then
                    if msgid == BATTERY_STATUS_ID then
                        handle_battery_status(idx, parsed, t_ms)
                    else
                        handle_named_value(idx, parsed)
                    end
                end
            end
            decodes = decodes + 1
            if decodes >= MAX_DECODES_PER_UPDATE then
                decode_budget_hits = decode_budget_hits + 1
                return
            end
        end
    end
end

-- ---------------------------------------------------------------- publish to the battery monitors

local function pack_healthy(p, t_ms)
    local fresh = p.rx_ms ~= nil and (t_ms - p.rx_ms) <= PBAT_TIMEOUT_MS:get()
    return fresh and p.v ~= nil, fresh
end

local function publish(idx, t_ms)
    local p = packs[idx]
    local timeout = PBAT_TIMEOUT_MS:get()
    local healthy, fresh = pack_healthy(p, t_ms)
    local monitor = monitor_of(idx)

    local state = BattMonitorScript_State()
    state:healthy(healthy)
    state:cell_count(0)
    if healthy then
        state:voltage(p.v)
        state:current_amps(p.i)          -- NaN when the pod reports no valid current
        state:consumed_mah(p.mah)        -- NaN: ArduPilot integrates the current by itself
        state:temperature(p.temp)        -- NaN when not valid
    else
        -- No measurement: 0 V and unhealthy (never a stale number presented as live). Current, mAh and
        -- temperature stay NaN: the backend keeps its previous values and the battery is flagged unhealthy.
        state:voltage(0)
        state:current_amps(NaN)
        state:consumed_mah(NaN)
        state:temperature(NaN)
    end
    -- capacity_remaining_pct is NOT set (the pods have no SOC): it stays at its "invalid" default

    if not battery:handle_scripting(monitor, state) then
        if not warned_publish[monitor] then
            warned_publish[monitor] = true
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: BAT%d not scripting (set %s 29)",
                          SCRIPT_NAME, idx + 1, MONITOR_PARAM[monitor]))
        end
    end

    if healthy ~= p.healthy then
        if p.healthy ~= nil or not healthy then
            if healthy then
                gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: BAT%d pod battery online", SCRIPT_NAME, idx + 1))
            elseif p.rx_ms == nil then
                gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: BAT%d no data from pod yet", SCRIPT_NAME, idx + 1))
            elseif not fresh then
                gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: BAT%d pod silent for >%.0f ms", SCRIPT_NAME, idx + 1, timeout))
            else
                gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: BAT%d voltage not valid", SCRIPT_NAME, idx + 1))
            end
        end
        p.healthy = healthy
    end
end

-- Virtual primary battery: the most loaded pack among those with a fresh voltage AND a valid current.
-- A pack with a NaN current never wins (NaN comparisons are false); with no valid current the primary is unhealthy,
-- so MOT_BAT_CURR_MAX does nothing (AP_Motors6DOF returns before scaling when current_amps() fails).
local function publish_virtual(t_ms)
    local best_i, best_src, bus_v, mah_sum, mah_ok, temp_max = nil, 255, nil, 0, true, nil
    for idx = 0, 1 do
        local p = packs[idx]
        local healthy = pack_healthy(p, t_ms)
        if healthy then
            if bus_v == nil or p.v > bus_v then bus_v = p.v end
            if p.i == p.i then   -- not NaN
                if best_i == nil or p.i > best_i then best_i = p.i; best_src = idx end
            end
            if p.mah == p.mah then mah_sum = mah_sum + p.mah else mah_ok = false end
            if p.temp == p.temp and (temp_max == nil or p.temp > temp_max) then temp_max = p.temp end
        end
    end
    local healthy = best_i ~= nil
    local state = BattMonitorScript_State()
    state:healthy(healthy)
    state:cell_count(0)
    if healthy then
        state:voltage(bus_v)
        state:current_amps(best_i)
        state:consumed_mah(mah_ok and mah_sum or NaN)
        state:temperature(temp_max or NaN)
    else
        state:voltage(0)
        state:current_amps(NaN)
        state:consumed_mah(NaN)
        state:temperature(NaN)
    end
    if not battery:handle_scripting(MONITOR_VIRTUAL, state) then
        if not warned_publish[MONITOR_VIRTUAL] then
            warned_publish[MONITOR_VIRTUAL] = true
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: primary not scripting (set BATT_MONITOR 29)", SCRIPT_NAME))
        end
    end
    if healthy ~= virtual.healthy then
        if virtual.healthy ~= nil or not healthy then
            if healthy then
                gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: pack current available, limiter input on", SCRIPT_NAME))
            else
                gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: no pack current, MOT_BAT_CURR_MAX inactive", SCRIPT_NAME))
            end
        end
        virtual.healthy = healthy
    end
    virtual.src = best_src
    virtual.i = healthy and best_i or NaN
    virtual.v = healthy and bus_v or 0
end

-- ---------------------------------------------------------------- power mode reporting (no action)

local function report_power_state(idx)
    local p = packs[idx]
    if not p.pwr_pending or p.pwr_mode == nil or p.pwr_rsn == nil then
        return
    end
    if p.pwr_mode == p.pwr_pub_mode and p.pwr_rsn == p.pwr_pub_rsn then
        p.pwr_pending = false
        return
    end
    -- DataFlash line at every change: compid, mode, reason
    logger:write("PWRM", "Comp,Mode,Rsn", "BBi", idx == 0 and COMPID_BAT1 or COMPID_BAT2, p.pwr_mode, p.pwr_rsn)
    local mode_name = PWR_MODE_NAMES[p.pwr_mode] or tostring(p.pwr_mode)
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: BAT%d power mode %s reason %d",
                  SCRIPT_NAME, idx + 1, mode_name, p.pwr_rsn))
    p.pwr_pub_mode = p.pwr_mode
    p.pwr_pub_rsn = p.pwr_rsn
    p.pwr_pending = false
end

-- ---------------------------------------------------------------- main loop

local function check_configuration()
    if warned_config then
        return
    end
    for monitor = 0, 2 do
        local name = MONITOR_PARAM[monitor]
        local value = param:get(name)
        if value ~= 29 then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: %s is %s, must be 29 (Scripting)",
                          SCRIPT_NAME, name, tostring(value)))
            warned_config = true
        end
    end
    local imax = param:get("MOT_BAT_CURR_MAX")
    if imax == nil or imax <= 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: MOT_BAT_CURR_MAX is %s: pack current limiter OFF",
                      SCRIPT_NAME, tostring(imax)))
        warned_config = true
    else
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: pack current limit %.1f A (MOT_BAT_CURR_MAX)", SCRIPT_NAME, imax))
    end
end

-- Per pod statistics row (1 Hz): who is healthy and how many messages reached the script (the mavlink queue API has
-- no drop counter, so a loss shows as RxB growing slower than the pod's 10 Hz).
local function log_stats(t_ms)
    if last_stats_ms ~= nil and (t_ms - last_stats_ms) < STATS_LOG_PERIOD_MS then
        return
    end
    last_stats_ms = t_ms
    for idx = 0, 1 do
        local p = packs[idx]
        local age = -1
        if p.rx_ms ~= nil then age = t_ms - p.rx_ms end
        logger:write("PBAT", "Comp,Healthy,AgeMs,RxB,RxN,Ign,Cap", "BBiIIII",
                     idx == 0 and COMPID_BAT1 or COMPID_BAT2, p.healthy and 1 or 0, age,
                     p.n_batt, p.n_named, p.n_ignored, decode_budget_hits)
        -- A separate record preserves the PBAT schema across script restarts without an autopilot reboot.
        logger:write("PBCR", "Comp,Crc", "BI", idx == 0 and COMPID_BAT1 or COMPID_BAT2, p.n_crc)
    end
end

-- Limiter input at the publish rate (40 Hz) only while it changes or once per second: source pack (0 = BAT1, 1 = BAT2,
-- 255 = none), current and bus voltage, and the budget each pod announces (A x 10, -1 = not received).
local last_pmax_ms, last_pmax_src, last_pmax_i = nil, nil, nil
local function log_virtual(t_ms)
    local changed = virtual.src ~= last_pmax_src or
                    (virtual.i == virtual.i and (last_pmax_i == nil or math.abs(virtual.i - last_pmax_i) >= 0.5))
    if not changed and last_pmax_ms ~= nil and (t_ms - last_pmax_ms) < STATS_LOG_PERIOD_MS then
        return
    end
    last_pmax_ms, last_pmax_src, last_pmax_i = t_ms, virtual.src, virtual.i
    logger:write("PMAX", "Src,I,V,Bud1,Bud2", "Bffii", virtual.src, virtual.i, virtual.v,
                 packs[0].pwr_budg or -1, packs[1].pwr_budg or -1)
end

local function update()
    drain_messages()
    local t_ms = now_ms()
    for idx = 0, 1 do
        publish(idx, t_ms)
        report_power_state(idx)
    end
    publish_virtual(t_ms)
    log_virtual(t_ms)
    log_stats(t_ms)
    return update, UPDATE_PERIOD_MS
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: started (timeout %.0f ms)", SCRIPT_NAME, PBAT_TIMEOUT_MS:get()))
check_configuration()

return update, UPDATE_PERIOD_MS
