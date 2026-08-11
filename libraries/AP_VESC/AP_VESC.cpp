/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_VESC_config.h"

#if AP_VESC_ENABLED

#include "AP_VESC.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

#define VESC_OUTPUT_RATE_MIN_HZ 10
#define VESC_OUTPUT_RATE_MAX_HZ 200
#define VESC_OUTPUT_RATE_DEFAULT_HZ 100

const AP_Param::GroupInfo AP_VESC::var_info[] = {
    // @Param: ESC_BM
    // @DisplayName: VESC motor output mask
    // @Description: Motor outputs sent to VESC controllers over CAN
    // @Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4,4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8,8:Motor 9,9:Motor 10,10:Motor 11,11:Motor 12
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ESC_BM", 1, AP_VESC, _esc_mask, 0),

    // @Param: ESC_RT
    // @DisplayName: VESC command rate
    // @Description: Rate at which signed electrical RPM commands are sent to each selected VESC
    // @Units: Hz
    // @Range: 10 200
    // @User: Advanced
    AP_GROUPINFO("ESC_RT", 2, AP_VESC, _output_rate_hz, VESC_OUTPUT_RATE_DEFAULT_HZ),

    // @Param: MAXRPM
    // @DisplayName: VESC maximum electrical RPM
    // @Description: Symmetric magnitude limit for signed VESC electrical RPM commands
    // @Units: RPM
    // @Range: 1000 200000
    // @User: Advanced
    AP_GROUPINFO("MAXRPM", 3, AP_VESC, _max_erpm, 17000),

    // @Param: EXP
    // @DisplayName: VESC thrust to speed exponent
    // @Description: Exponent in the thrust model where thrust is proportional to RPM raised to this value
    // @Range: 1.0 3.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("EXP", 4, AP_VESC, _thrust_exponent, 2.0f),

    // @Param: PPAIRS
    // @DisplayName: VESC motor pole pairs
    // @Description: Number of motor pole pairs used to convert electrical RPM telemetry to mechanical motor RPM
    // @Range: 1 64
    // @User: Advanced
    AP_GROUPINFO("PPAIRS", 5, AP_VESC, _pole_pairs, 7),

    // @Param: IN_MIN
    // @DisplayName: VESC input minimum PWM
    // @Description: Final motor output value corresponding to maximum reverse thrust
    // @Units: PWM
    // @Range: 800 1500
    // @User: Advanced
    AP_GROUPINFO("IN_MIN", 6, AP_VESC, _input_min, 1100),

    // @Param: IN_MID
    // @DisplayName: VESC input neutral PWM
    // @Description: Final motor output value corresponding to zero thrust
    // @Units: PWM
    // @Range: 1200 1800
    // @User: Advanced
    AP_GROUPINFO("IN_MID", 7, AP_VESC, _input_mid, 1500),

    // @Param: IN_MAX
    // @DisplayName: VESC input maximum PWM
    // @Description: Final motor output value corresponding to maximum forward thrust
    // @Units: PWM
    // @Range: 1500 2200
    // @User: Advanced
    AP_GROUPINFO("IN_MAX", 8, AP_VESC, _input_max, 1900),

    // @Param: CMD_TO
    // @DisplayName: VESC output command timeout
    // @Description: Maximum age of final motor output data before the CAN driver commands zero RPM
    // @Units: ms
    // @Range: 20 1000
    // @User: Advanced
    AP_GROUPINFO("CMD_TO", 9, AP_VESC, _command_timeout_ms, 100),

    // @Param: TLM_TO
    // @DisplayName: VESC telemetry timeout
    // @Description: Maximum telemetry age used when reporting whether a VESC is present
    // @Units: ms
    // @Range: 100 5000
    // @User: Advanced
    AP_GROUPINFO("TLM_TO", 10, AP_VESC, _telemetry_timeout_ms, 500),

    // @Param: ID1
    // @DisplayName: Motor 1 VESC controller ID
    // @Description: VESC CAN controller ID for motor 1
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID1", 12, AP_VESC, _controller_id[0], 1),

    // @Param: ID2
    // @DisplayName: Motor 2 VESC controller ID
    // @Description: VESC CAN controller ID for motor 2
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID2", 13, AP_VESC, _controller_id[1], 2),

    // @Param: ID3
    // @DisplayName: Motor 3 VESC controller ID
    // @Description: VESC CAN controller ID for motor 3
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID3", 14, AP_VESC, _controller_id[2], 3),

    // @Param: ID4
    // @DisplayName: Motor 4 VESC controller ID
    // @Description: VESC CAN controller ID for motor 4
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID4", 15, AP_VESC, _controller_id[3], 4),

    // @Param: ID5
    // @DisplayName: Motor 5 VESC controller ID
    // @Description: VESC CAN controller ID for motor 5
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID5", 16, AP_VESC, _controller_id[4], 5),

    // @Param: ID6
    // @DisplayName: Motor 6 VESC controller ID
    // @Description: VESC CAN controller ID for motor 6
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID6", 17, AP_VESC, _controller_id[5], 6),

    // @Param: ID7
    // @DisplayName: Motor 7 VESC controller ID
    // @Description: VESC CAN controller ID for motor 7
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID7", 18, AP_VESC, _controller_id[6], 7),

    // @Param: ID8
    // @DisplayName: Motor 8 VESC controller ID
    // @Description: VESC CAN controller ID for motor 8
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID8", 19, AP_VESC, _controller_id[7], 8),

    // @Param: ID9
    // @DisplayName: Motor 9 VESC controller ID
    // @Description: VESC CAN controller ID for motor 9
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID9", 20, AP_VESC, _controller_id[8], 9),

    // @Param: ID10
    // @DisplayName: Motor 10 VESC controller ID
    // @Description: VESC CAN controller ID for motor 10
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID10", 21, AP_VESC, _controller_id[9], 10),

    // @Param: ID11
    // @DisplayName: Motor 11 VESC controller ID
    // @Description: VESC CAN controller ID for motor 11
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID11", 22, AP_VESC, _controller_id[10], 11),

    // @Param: ID12
    // @DisplayName: Motor 12 VESC controller ID
    // @Description: VESC CAN controller ID for motor 12
    // @Range: 0 254
    // @User: Advanced
    AP_GROUPINFO("ID12", 23, AP_VESC, _controller_id[11], 12),

    // @Param: STOPMS
    // @DisplayName: VESC disarm zero-command flush time
    // @Description: Time to transmit zero RPM after disarming before reporting that VESC power may be removed
    // @Units: ms
    // @Range: 100 2000
    // @User: Advanced
    AP_GROUPINFO("STOPMS", 24, AP_VESC, _disarm_flush_ms, 500),

    // @Param: MODE
    // @DisplayName: VESC propulsion command mode
    // @Description: Select PPM outputs or VESC CAN RPM commands. PPM never transmits motor commands. CAN suppresses PWM on selected motor outputs and never falls back automatically.
    // @Values: 0:PPM,1:CAN
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MODE", 25, AP_VESC, _mode, int8_t(Mode::PPM)),

    // @Param: PROTO
    // @DisplayName: VESC CAN telemetry protocol
    // @Description: STANDARD ignores reserved status fields. DEEPEX_V1 decodes the STATUS_5 reserved field as fault code and warning flags.
    // @Values: 0:STANDARD,1:DEEPEX_V1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PROTO", 26, AP_VESC, _protocol, int8_t(AP_VESC_Protocol::Protocol::STANDARD)),

    // @Param: EXT_TO
    // @DisplayName: VESC extended telemetry timeout
    // @Description: Maximum age of both STATUS_4 and STATUS_5 for command readiness
    // @Units: ms
    // @Range: 200 5000
    // @User: Advanced
    AP_GROUPINFO("EXT_TO", 27, AP_VESC, _extended_timeout_ms, 1000),

    // @Param: ENG_TO
    // @DisplayName: VESC energy telemetry timeout
    // @Description: Maximum age of both STATUS_2 and STATUS_3 when reporting energy telemetry validity
    // @Units: ms
    // @Range: 500 10000
    // @User: Advanced
    AP_GROUPINFO("ENG_TO", 28, AP_VESC, _energy_timeout_ms, 2000),

    AP_GROUPEND
};

AP_VESC::AP_VESC()
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_VESC *AP_VESC::get_vesc(const uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::VESC) {
        return nullptr;
    }
    return static_cast<AP_VESC *>(AP::can().get_driver(driver_index));
}

bool AP_VESC::add_interface(AP_HAL::CANIface *can_iface)
{
    if (_can_iface != nullptr || can_iface == nullptr || !can_iface->is_initialized()) {
        return false;
    }
    if (!can_iface->set_event_handle(&_event_sem)) {
        return false;
    }
    _can_iface = can_iface;
    return true;
}

void AP_VESC::init(const uint8_t driver_index)
{
    if (!_mode_latched) {
        _active_mode = Mode(_mode.get());
        _active_protocol = AP_VESC_Protocol::Protocol(_protocol.get());
        _mode_latched = true;
    }
    if (_initialized || _can_iface == nullptr) {
        return;
    }

    _driver_index = driver_index;
    hal.util->snprintf(_thread_name, sizeof(_thread_name), "VESC_%u", driver_index);
    _initialized = true;
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_VESC::loop, void),
                                      _thread_name,
                                      4096,
                                      AP_HAL::Scheduler::PRIORITY_MAIN,
                                      1)) {
        _initialized = false;
    }
}

bool AP_VESC::motor_is_selected(const uint8_t motor) const
{
    return motor < MAX_ESC && (_esc_mask.get() & (1U << motor)) != 0;
}

uint16_t AP_VESC::expected_mask() const
{
    return uint16_t(_esc_mask.get()) & ((1U << MAX_ESC) - 1U);
}

uint16_t AP_VESC::present_mask() const
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t timeout_ms = MAX(100, _telemetry_timeout_ms.get());
    uint16_t mask = 0;
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        const uint32_t last_telem_ms = _controller_state[motor].last_status1_ms;
        if (motor_is_selected(motor) && last_telem_ms != 0 && now_ms - last_telem_ms <= timeout_ms) {
            mask |= 1U << motor;
        }
    }
    return mask;
}

uint16_t AP_VESC::fault_mask() const
{
    uint16_t mask = 0;
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        if (motor_is_selected(motor) && _controller_state[motor].active_fault) {
            mask |= 1U << motor;
        }
    }
    return mask;
}

uint16_t AP_VESC::diagnostic_flags() const
{
    enum : uint16_t {
        DRIVER_UNAVAILABLE = 1U << 0,
        NO_EXTERNAL_RX = 1U << 1,
        INTERFACE_DOWN = 1U << 2,
        TX_FAILURE = 1U << 3,
        UNEXPECTED_CONTROLLER = 1U << 4,
        REQUIRED_MISSING = 1U << 5,
        ACTIVE_FAULT = 1U << 6,
        COMMAND_TIMEOUT = 1U << 7,
        ZERO_FLUSH_INCOMPLETE = 1U << 8,
    };
    uint16_t flags = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (!_initialized || _can_iface == nullptr) {
        flags |= DRIVER_UNAVAILABLE;
    } else if (_can_iface->is_busoff()) {
        flags |= INTERFACE_DOWN;
    }
    if (_last_external_rx_ms == 0) {
        flags |= NO_EXTERNAL_RX;
    }
    if (_last_tx_failure_ms != 0 &&
        (_last_tx_success_ms == 0 || _last_tx_failure_ms > _last_tx_success_ms) &&
        now_ms - _last_tx_failure_ms < 5000) {
        flags |= TX_FAILURE;
    }
    if (_unexpected_rx_count != 0) {
        flags |= UNEXPECTED_CONTROLLER;
    }
    if ((expected_mask() & ~present_mask()) != 0) {
        flags |= REQUIRED_MISSING;
    }
    if (fault_mask() != 0) {
        flags |= ACTIVE_FAULT;
    }
    if (_command_timeout_active) {
        flags |= COMMAND_TIMEOUT;
    }
    if (!_zero_flush_complete) {
        flags |= ZERO_FLUSH_INCOMPLETE;
    }
    return flags;
}

bool AP_VESC::get_controller_state(const uint8_t motor, ControllerState &state) const
{
    if (motor >= MAX_ESC || !motor_is_selected(motor)) {
        return false;
    }
    state = _controller_state[motor];
    state.configured = _controller_id[motor].get() >= 0 &&
                       _controller_id[motor].get() <= AP_VESC_Protocol::MAX_CONTROLLER_ID;
    state.expected = motor_is_selected(motor);
    state.controller_id = state.configured ? _controller_id[motor].get() : 0;
    state.motor_number = motor + 1;
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t fast_timeout = MAX(100, _telemetry_timeout_ms.get());
    const uint32_t extended_timeout = MAX(200, _extended_timeout_ms.get());
    const uint32_t energy_timeout = MAX(500, _energy_timeout_ms.get());
    const AP_VESC_Protocol::Freshness freshness =
        AP_VESC_Protocol::freshness(now_ms,
                                    state.last_status1_ms,
                                    state.last_status2_ms,
                                    state.last_status3_ms,
                                    state.last_status4_ms,
                                    state.last_status5_ms,
                                    fast_timeout,
                                    extended_timeout,
                                    energy_timeout);
    state.fast_telemetry_valid = freshness.fast;
    state.extended_telemetry_valid = freshness.extended;
    state.energy_telemetry_valid = freshness.energy;
    state.present = state.fast_telemetry_valid;
    state.telemetry_stale = freshness.stale;
    state.command_ready = state.fast_telemetry_valid &&
                          state.extended_telemetry_valid &&
                          !state.active_fault;
    return true;
}

int8_t AP_VESC::motor_for_controller_id(const uint8_t controller_id) const
{
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        if (motor_is_selected(motor) && _controller_id[motor].get() == controller_id) {
            return motor;
        }
    }
    return -1;
}

int8_t AP_VESC::next_selected_motor(const uint16_t mask, const uint8_t start_motor)
{
    for (uint8_t offset = 0; offset < MAX_ESC; offset++) {
        const uint8_t motor = (start_motor + offset) % MAX_ESC;
        if ((mask & (1U << motor)) != 0) {
            return motor;
        }
    }
    return -1;
}

void AP_VESC::update()
{
    if (!AP_VESC_Protocol::can_command_mode(uint8_t(mode()))) {
        return;
    }
    const bool outputs_enabled = hal.util->get_soft_armed() && !SRV_Channels::get_emergency_stop();
    const uint16_t pwm_min = _input_min.get();
    const uint16_t pwm_mid = _input_mid.get();
    const uint16_t pwm_max = _input_max.get();
    const int32_t max_erpm = _max_erpm.get();
    const float exponent = _thrust_exponent.get();

    WITH_SEMAPHORE(_command_sem);
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        _command_erpm[motor] = 0;
        if (!motor_is_selected(motor)) {
            continue;
        }

        const SRV_Channel::Function function = SRV_Channels::get_motor_function(motor);
        uint8_t channel;
        if (!SRV_Channels::find_channel(function, channel)) {
            continue;
        }

        uint16_t pwm = 0;
        const bool have_output = SRV_Channels::get_output_pwm_chan(channel, pwm);
        hal.rcout->write(channel, pwm_mid);
        if (outputs_enabled && have_output) {
            _command_erpm[motor] = AP_VESC_Protocol::pwm_to_erpm(pwm,
                                   pwm_min,
                                   pwm_mid,
                                   pwm_max,
                                   max_erpm,
                                   exponent);
        }
    }
    _last_output_update_ms = AP_HAL::millis();
}

bool AP_VESC::write_frame(const AP_HAL::CANFrame &frame, const uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    const uint64_t deadline_us = AP_HAL::micros64() + timeout_us;
    if (!_can_iface->select(read_select, write_select, &frame, deadline_us) || !write_select) {
        return false;
    }
    return _can_iface->send(frame, deadline_us, AP_HAL::CANIface::AbortOnError) == 1;
}

bool AP_VESC::read_frame(AP_HAL::CANFrame &frame)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    uint64_t timestamp_us;
    AP_HAL::CANIface::CanIOFlags flags {};
    return _can_iface->receive(frame, timestamp_us, flags) == 1;
}

void AP_VESC::send_commands()
{
    if (!AP_VESC_Protocol::can_command_mode(uint8_t(mode()))) {
        _zero_flush_complete = true;
        _readiness_state = ReadinessState::POWERED_OFF;
        return;
    }
    int32_t command_erpm[MAX_ESC] {};
    uint32_t last_output_update_ms;
    {
        WITH_SEMAPHORE(_command_sem);
        last_output_update_ms = _last_output_update_ms;
        memcpy(command_erpm, _command_erpm, sizeof(command_erpm));
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t command_timeout_ms = constrain_int16(_command_timeout_ms.get(), 20, 1000);
    const bool output_fresh = last_output_update_ms != 0 &&
                              now_ms - last_output_update_ms <= command_timeout_ms;
    const bool armed = hal.util->get_soft_armed();
    const uint16_t expected = expected_mask();
    const uint16_t present = present_mask();
    uint16_t ready = 0;
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        ControllerState state {};
        if (get_controller_state(motor, state) && state.command_ready) {
            ready |= 1U << motor;
        }
    }
    const bool tx_healthy = _last_tx_failure_ms == 0 ||
                            (_last_tx_success_ms != 0 && _last_tx_success_ms > _last_tx_failure_ms) ||
                            now_ms - _last_tx_failure_ms >= 5000;
    const AP_VESC_Protocol::CommandConditions command_conditions {
        armed,
        SRV_Channels::get_emergency_stop(),
        output_fresh,
        _can_iface != nullptr,
        _can_iface != nullptr && _can_iface->is_busoff(),
        tx_healthy,
        ready == expected,
    };
    const bool outputs_enabled = AP_VESC_Protocol::allow_nonzero_command(command_conditions);
    _command_timeout_active = armed && !output_fresh;

    if (_last_armed && !armed) {
        _disarm_flush_start_ms = now_ms;
        _zero_flush_success_mask = 0;
        _zero_flush_complete = expected == 0;
    }
    _last_armed = armed;

    const uint32_t flush_ms = constrain_int16(_disarm_flush_ms.get(), 100, 2000);
    if (!armed && !_zero_flush_complete &&
        AP_VESC_Protocol::zero_flush_complete(expected,
                                              _zero_flush_success_mask,
                                              now_ms - _disarm_flush_start_ms,
                                              flush_ms)) {
        _zero_flush_complete = true;
    }
    const bool flushing = !armed && !_zero_flush_complete;

    if (armed) {
        _zero_flush_complete = false;
        _zero_flush_success_mask = 0;
        _readiness_state = outputs_enabled ? ReadinessState::ARMED : ReadinessState::FAULT;
    } else if (flushing && now_ms - _disarm_flush_start_ms < flush_ms) {
        _readiness_state = ReadinessState::DISARM_FLUSH;
    } else if (flushing) {
        _readiness_state = ReadinessState::FAULT;
    } else if (present == 0) {
        _readiness_state = ReadinessState::POWERED_OFF;
    } else if (present != expected) {
        _readiness_state = ReadinessState::WAITING_TELEMETRY;
    } else {
        _readiness_state = ReadinessState::READY;
    }

    if (!outputs_enabled) {
        memset(command_erpm, 0, sizeof(command_erpm));
    }

    // When disarmed, only address controllers which have recently announced
    // themselves. This avoids filling an unpowered CAN bus with failed TX.
    const uint16_t send_mask = (armed || flushing) ? expected : present;
    // Queue one frame per scheduler slot. Bursting the whole mask can leave
    // higher controller IDs behind lower-ID frames in priority-ordered queues.
    const int8_t motor = next_selected_motor(send_mask, _next_command_motor);
    if (motor < 0) {
        return;
    }
    _next_command_motor = (motor + 1) % MAX_ESC;

    const int16_t configured_id = _controller_id[motor].get();
    if (configured_id < 0 || configured_id > AP_VESC_Protocol::MAX_CONTROLLER_ID) {
        _tx_error_count++;
        return;
    }

    AP_HAL::CANFrame frame;
    const uint8_t controller_id = uint8_t(configured_id);
    if (!AP_VESC_Protocol::make_set_rpm_frame(controller_id, command_erpm[motor], frame) ||
        !write_frame(frame, 1000)) {
        _tx_error_count++;
        _last_tx_failure_ms = now_ms;
    } else {
        _last_tx_success_ms = now_ms;
        if (flushing && command_erpm[motor] == 0) {
            _zero_flush_success_mask |= 1U << motor;
        }
    }
}

void AP_VESC::handle_frame(const AP_HAL::CANFrame &frame)
{
    const AP_VESC_Protocol::StatusType type = AP_VESC_Protocol::status_type(frame);
    if (type == AP_VESC_Protocol::StatusType::NONE) {
        return;
    }

    const uint8_t controller_id = frame.id & 0xFF;
    const int8_t motor = motor_for_controller_id(controller_id);
    if (motor < 0) {
        _unexpected_rx_count++;
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    _last_external_rx_ms = now_ms;
    ControllerState &state = _controller_state[motor];
    state.controller_id = controller_id;
    state.motor_number = motor + 1;

    switch (type) {
    case AP_VESC_Protocol::StatusType::STATUS_1: {
        AP_VESC_Protocol::Status1 status {};
        if (!AP_VESC_Protocol::decode_status_1(frame, status)) {
            return;
        }
        state.erpm = status.erpm;
        state.mechanical_rpm = AP_VESC_Protocol::mechanical_rpm(status.erpm, MAX(1, _pole_pairs.get()));
        state.motor_current = status.current;
        state.duty_cycle = status.duty_cycle;
        state.last_status1_ms = now_ms;
#if HAL_WITH_ESC_TELEM
        update_rpm(motor, status.erpm);
#endif
        break;
    }
    case AP_VESC_Protocol::StatusType::STATUS_2: {
        AP_VESC_Protocol::Status2 status {};
        if (!AP_VESC_Protocol::decode_status_2(frame, status)) {
            return;
        }
        state.amp_hours = status.amp_hours;
        state.amp_hours_charged = status.amp_hours_charged;
        state.last_status2_ms = now_ms;
#if HAL_WITH_ESC_TELEM
        AP_ESC_Telem_Backend::TelemetryData telem {};
        telem.consumption_mah = status.amp_hours * 1000.0f;
        update_telem_data(motor, telem, AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
#endif
        break;
    }
    case AP_VESC_Protocol::StatusType::STATUS_3: {
        AP_VESC_Protocol::Status3 status {};
        if (!AP_VESC_Protocol::decode_status_3(frame, status)) {
            return;
        }
        state.watt_hours = status.watt_hours;
        state.watt_hours_charged = status.watt_hours_charged;
        state.last_status3_ms = now_ms;
        break;
    }
    case AP_VESC_Protocol::StatusType::STATUS_4: {
        AP_VESC_Protocol::Status4 status {};
        if (!AP_VESC_Protocol::decode_status_4(frame, status)) {
            return;
        }
        state.mosfet_temperature = status.mosfet_temperature;
        state.motor_temperature = status.motor_temperature;
        state.input_current = status.input_current;
        state.pid_position = status.pid_position;
        state.last_status4_ms = now_ms;
#if HAL_WITH_ESC_TELEM
        AP_ESC_Telem_Backend::TelemetryData telem {};
        telem.temperature_cdeg = status.mosfet_temperature * 100;
        telem.motor_temp_cdeg = status.motor_temperature * 100;
        telem.current = status.input_current;
        update_telem_data(motor, telem,
                          AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                          AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE |
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT);
#endif
        break;
    }
    case AP_VESC_Protocol::StatusType::STATUS_5: {
        AP_VESC_Protocol::Status5 status {};
        if (!AP_VESC_Protocol::decode_status_5(frame, protocol(), status)) {
            return;
        }
        state.tachometer = status.tachometer;
        state.input_voltage = status.input_voltage;
        state.fault_code = status.fault_code;
        state.warning_flags = status.warning_flags;
        state.active_fault = status.fault_code != 0;
        state.last_status5_ms = now_ms;
#if HAL_WITH_ESC_TELEM
        AP_ESC_Telem_Backend::TelemetryData telem {};
        telem.voltage = status.input_voltage;
        update_telem_data(motor, telem, AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
#endif
        break;
    }
    case AP_VESC_Protocol::StatusType::NONE:
        break;
    }
}

void AP_VESC::loop()
{
    uint64_t next_tx_us = AP_HAL::micros64();
    while (true) {
        const uint16_t output_rate_hz = constrain_int16(_output_rate_hz.get(),
                                        VESC_OUTPUT_RATE_MIN_HZ,
                                        VESC_OUTPUT_RATE_MAX_HZ);
        const uint16_t expected = expected_mask();
        const uint16_t send_mask = (hal.util->get_soft_armed() || !_zero_flush_complete) ?
                                   expected : present_mask();
        const uint8_t selected_count = __builtin_popcount(send_mask);
        const uint32_t aggregate_rate_hz = output_rate_hz * MAX(1U, selected_count);
        const uint32_t interval_us = MAX(1U, 1000000U / aggregate_rate_hz);
        const uint64_t now_us = AP_HAL::micros64();
        if (now_us >= next_tx_us) {
            send_commands();
            next_tx_us += interval_us;
            if (now_us >= next_tx_us) {
                next_tx_us = now_us + interval_us;
            }
        }

        AP_HAL::CANFrame frame;
        while (read_frame(frame)) {
            handle_frame(frame);
        }

        const uint64_t wait_start_us = AP_HAL::micros64();
        const uint32_t wait_us = next_tx_us > wait_start_us ?
                                 MIN(1000U, uint32_t(next_tx_us - wait_start_us)) : 1U;
        IGNORE_RETURN(_event_sem.wait(wait_us));
    }
}

bool AP_VESC::pre_arm_check(char *reason, const uint8_t reason_len) const
{
    if (mode() == Mode::PPM) {
        return true;
    }
    if (mode() != Mode::CAN) {
        hal.util->snprintf(reason, reason_len, "invalid command mode");
        return false;
    }
    if (protocol() != AP_VESC_Protocol::Protocol::STANDARD &&
        protocol() != AP_VESC_Protocol::Protocol::DEEPEX_V1) {
        hal.util->snprintf(reason, reason_len, "invalid telemetry protocol");
        return false;
    }
    if (!_initialized || _can_iface == nullptr) {
        hal.util->snprintf(reason, reason_len, "driver not initialized");
        return false;
    }
    if (_can_iface->is_busoff()) {
        hal.util->snprintf(reason, reason_len, "CAN interface down");
        return false;
    }
    if ((_esc_mask.get() & ((1U << MAX_ESC) - 1U)) == 0) {
        hal.util->snprintf(reason, reason_len, "no motor outputs selected");
        return false;
    }
    if ((_esc_mask.get() & ~((1U << MAX_ESC) - 1U)) != 0) {
        hal.util->snprintf(reason, reason_len, "invalid motor output mask");
        return false;
    }
    if (_output_rate_hz < VESC_OUTPUT_RATE_MIN_HZ || _output_rate_hz > VESC_OUTPUT_RATE_MAX_HZ) {
        hal.util->snprintf(reason, reason_len, "invalid output rate");
        return false;
    }
    if (_max_erpm <= 0 || !is_positive(_thrust_exponent) || _pole_pairs <= 0) {
        hal.util->snprintf(reason, reason_len, "invalid motor scaling");
        return false;
    }
    if (_input_min >= _input_mid || _input_mid >= _input_max) {
        hal.util->snprintf(reason, reason_len, "invalid input PWM range");
        return false;
    }
    if (_command_timeout_ms < 20) {
        hal.util->snprintf(reason, reason_len, "invalid command timeout");
        return false;
    }
    if (_telemetry_timeout_ms < 100) {
        hal.util->snprintf(reason, reason_len, "invalid telemetry timeout");
        return false;
    }
    if (_disarm_flush_ms < 100 || _disarm_flush_ms > 2000) {
        hal.util->snprintf(reason, reason_len, "invalid disarm flush time");
        return false;
    }
    if (_last_tx_failure_ms != 0 &&
        (_last_tx_success_ms == 0 || _last_tx_failure_ms > _last_tx_success_ms) &&
        AP_HAL::millis() - _last_tx_failure_ms < 5000) {
        hal.util->snprintf(reason, reason_len, "recent CAN TX failure");
        return false;
    }

    int16_t controller_ids[MAX_ESC] {};
    bool function_assigned[MAX_ESC] {};
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        controller_ids[motor] = _controller_id[motor].get();
        function_assigned[motor] =
            SRV_Channels::function_assigned(SRV_Channels::get_motor_function(motor));
    }
    const AP_VESC_Protocol::ConfigurationResult config =
        AP_VESC_Protocol::validate_configuration(_esc_mask.get(),
                                                 controller_ids,
                                                 function_assigned,
                                                 MAX_ESC);
    switch (config.error) {
    case AP_VESC_Protocol::ConfigurationError::EMPTY_MASK:
        hal.util->snprintf(reason, reason_len, "no motor outputs selected");
        return false;
    case AP_VESC_Protocol::ConfigurationError::INVALID_MASK:
        hal.util->snprintf(reason, reason_len, "invalid motor output mask");
        return false;
    case AP_VESC_Protocol::ConfigurationError::INVALID_ID:
        hal.util->snprintf(reason, reason_len, "motor %u invalid ID", config.motor + 1);
        return false;
    case AP_VESC_Protocol::ConfigurationError::MISSING_FUNCTION:
        hal.util->snprintf(reason, reason_len, "motor %u function missing", config.motor + 1);
        return false;
    case AP_VESC_Protocol::ConfigurationError::DUPLICATE_ID:
        hal.util->snprintf(reason, reason_len, "motors %u/%u duplicate ID",
                           config.motor + 1,
                           config.other_motor + 1);
        return false;
    case AP_VESC_Protocol::ConfigurationError::NONE:
        break;
    }

    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        if (!motor_is_selected(motor)) {
            continue;
        }
        ControllerState state {};
        if (!get_controller_state(motor, state) || !state.fast_telemetry_valid) {
            hal.util->snprintf(reason, reason_len, "motor %u STATUS_1 stale", motor + 1);
            return false;
        }
        if (!state.extended_telemetry_valid) {
            hal.util->snprintf(reason, reason_len, "motor %u STATUS_4/5 stale", motor + 1);
            return false;
        }
        if (state.active_fault) {
            hal.util->snprintf(reason, reason_len, "motor %u fault %u", motor + 1, state.fault_code);
            return false;
        }
    }
    return true;
}

#endif // AP_VESC_ENABLED
