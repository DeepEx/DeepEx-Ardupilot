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

    // @Param: TLM_REQ
    // @DisplayName: Require VESC telemetry before arming
    // @Description: Require fresh VESC status telemetry from every selected controller before arming
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("TLM_REQ", 11, AP_VESC, _require_telemetry, 1),

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

    // @Param: STOP_MS
    // @DisplayName: VESC disarm zero-command flush time
    // @Description: Time to transmit zero RPM after disarming before reporting that VESC power may be removed
    // @Units: ms
    // @Range: 100 2000
    // @User: Advanced
    AP_GROUPINFO("STOP_MS", 24, AP_VESC, _disarm_flush_ms, 500),

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
        const uint32_t last_telem_ms = _last_telem_ms[motor];
        if (motor_is_selected(motor) && last_telem_ms != 0 && now_ms - last_telem_ms <= timeout_ms) {
            mask |= 1U << motor;
        }
    }
    return mask;
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

void AP_VESC::update()
{
    const bool outputs_enabled = hal.util->get_soft_armed() && !SRV_Channels::get_emergency_stop();
    const uint16_t pwm_min = _input_min.get();
    const uint16_t pwm_mid = _input_mid.get();
    const uint16_t pwm_max = _input_max.get();
    const int32_t max_erpm = _max_erpm.get();
    const float exponent = _thrust_exponent.get();

    WITH_SEMAPHORE(_command_sem);
    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        _command_erpm[motor] = 0;
        if (!outputs_enabled || !motor_is_selected(motor)) {
            continue;
        }

        uint16_t pwm = 0;
        const SRV_Channel::Function function = SRV_Channels::get_motor_function(motor);
        if (SRV_Channels::get_output_pwm(function, pwm)) {
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

bool AP_VESC::read_frame(AP_HAL::CANFrame &frame, const uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    bool read_select = true;
    bool write_select = false;
    if (!_can_iface->select(read_select, write_select, nullptr, AP_HAL::micros64() + timeout_us) || !read_select) {
        return false;
    }

    uint64_t timestamp_us;
    AP_HAL::CANIface::CanIOFlags flags {};
    return _can_iface->receive(frame, timestamp_us, flags) == 1;
}

void AP_VESC::send_commands()
{
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
    const bool outputs_enabled = output_fresh && armed &&
                                 !SRV_Channels::get_emergency_stop();

    const uint16_t expected = expected_mask();
    const uint16_t present = present_mask();

    if (_last_armed && !armed) {
        _disarm_flush_start_ms = now_ms;
        _zero_flush_complete = present == 0;
    }
    _last_armed = armed;

    bool flushing = false;
    if (!armed && !_zero_flush_complete) {
        const uint32_t flush_ms = constrain_int16(_disarm_flush_ms.get(), 100, 2000);
        flushing = now_ms - _disarm_flush_start_ms < flush_ms;
        if (!flushing) {
            _zero_flush_complete = true;
        }
    }

    if (armed) {
        _zero_flush_complete = false;
        _readiness_state = present == expected ? ReadinessState::ARMED : ReadinessState::FAULT;
    } else if (flushing) {
        _readiness_state = ReadinessState::DISARM_FLUSH;
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

    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        // When disarmed, only address controllers which have recently announced
        // themselves. This avoids filling an unpowered CAN bus with failed TX.
        const bool should_send = armed ? (expected & (1U << motor)) != 0 :
                                         (present & (1U << motor)) != 0;
        if (!should_send) {
            continue;
        }

        const int16_t configured_id = _controller_id[motor].get();
        if (configured_id < 0 || configured_id > AP_VESC_Protocol::MAX_CONTROLLER_ID) {
            _tx_error_count++;
            continue;
        }

        AP_HAL::CANFrame frame;
        const uint8_t controller_id = uint8_t(configured_id);
        if (!AP_VESC_Protocol::make_set_rpm_frame(controller_id, command_erpm[motor], frame) ||
            !write_frame(frame, 1000)) {
            _tx_error_count++;
        }
    }
}

void AP_VESC::handle_frame(const AP_HAL::CANFrame &frame)
{
    AP_VESC_Protocol::Status1 status {};
    if (!AP_VESC_Protocol::decode_status_1(frame, status)) {
        return;
    }

    const int8_t motor = motor_for_controller_id(status.controller_id);
    if (motor < 0) {
        return;
    }
    _last_telem_ms[motor] = AP_HAL::millis();

#if HAL_WITH_ESC_TELEM
    const int8_t pole_pairs = _pole_pairs.get();
    if (pole_pairs > 0) {
        update_rpm(motor, status.erpm / float(pole_pairs));
    }

    AP_ESC_Telem_Backend::TelemetryData telem {};
    telem.current = status.current;
    update_telem_data(motor, telem, AP_ESC_Telem_Backend::TelemetryType::CURRENT);
#endif
}

void AP_VESC::loop()
{
    uint32_t last_tx_ms = 0;
    while (true) {
        const uint16_t output_rate_hz = constrain_int16(_output_rate_hz.get(),
                                        VESC_OUTPUT_RATE_MIN_HZ,
                                        VESC_OUTPUT_RATE_MAX_HZ);
        const uint32_t interval_ms = MAX(1U, 1000U / output_rate_hz);
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_tx_ms >= interval_ms) {
            last_tx_ms = now_ms;
            send_commands();
        }

        AP_HAL::CANFrame frame;
        while (read_frame(frame, 0)) {
            handle_frame(frame);
        }
        hal.scheduler->delay_microseconds(1000);
    }
}

bool AP_VESC::pre_arm_check(char *reason, const uint8_t reason_len) const
{
    if (!_initialized || _can_iface == nullptr) {
        hal.util->snprintf(reason, reason_len, "driver not initialized");
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

    for (uint8_t motor = 0; motor < MAX_ESC; motor++) {
        if (!motor_is_selected(motor)) {
            continue;
        }
        if (_controller_id[motor] < 0 || _controller_id[motor] > AP_VESC_Protocol::MAX_CONTROLLER_ID) {
            hal.util->snprintf(reason, reason_len, "motor %u invalid ID", motor + 1);
            return false;
        }
        const SRV_Channel::Function function = SRV_Channels::get_motor_function(motor);
        if (!SRV_Channels::function_assigned(function)) {
            hal.util->snprintf(reason, reason_len, "motor %u function missing", motor + 1);
            return false;
        }
        if (_require_telemetry &&
            (_last_telem_ms[motor] == 0 ||
             AP_HAL::millis() - _last_telem_ms[motor] > uint32_t(_telemetry_timeout_ms.get()))) {
            hal.util->snprintf(reason, reason_len, "motor %u telemetry missing", motor + 1);
            return false;
        }
        for (uint8_t other = motor + 1; other < MAX_ESC; other++) {
            if (motor_is_selected(other) && _controller_id[motor] == _controller_id[other]) {
                hal.util->snprintf(reason, reason_len, "motors %u/%u duplicate ID", motor + 1, other + 1);
                return false;
            }
        }
    }
    return true;
}

#endif // AP_VESC_ENABLED
