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

#pragma once

#include "AP_VESC_config.h"

#if AP_VESC_ENABLED

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>

#include "AP_VESC_Protocol.h"

class AP_VESC : public AP_CANDriver, public AP_ESC_Telem_Backend
{
public:
    static constexpr uint8_t MAX_ESC = 12;

    enum class Mode : uint8_t {
        PPM = 0,
        CAN = 1,
    };

    enum class ReadinessState : uint8_t {
        POWERED_OFF = 0,
        WAITING_TELEMETRY = 1,
        READY = 2,
        ARMED = 3,
        DISARM_FLUSH = 4,
        FAULT = 5,
    };

    using ControllerState = AP_VESC_Protocol::ControllerState;

    AP_VESC();

    CLASS_NO_COPY(AP_VESC);

    static const AP_Param::GroupInfo var_info[];

    static AP_VESC *get_vesc(uint8_t driver_index);

    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface *can_iface) override;

    // copy the final motor outputs from SRV_Channels
    void update();

    bool pre_arm_check(char *reason, uint8_t reason_len) const;

    Mode mode() const { return _active_mode; }
    AP_VESC_Protocol::Protocol protocol() const { return _active_protocol; }
    uint16_t expected_mask() const;
    uint16_t present_mask() const;
    uint16_t missing_mask() const { return expected_mask() & ~present_mask(); }
    uint16_t fault_mask() const;
    uint16_t diagnostic_flags() const;
    bool get_controller_state(uint8_t motor, ControllerState &state) const;
    ReadinessState readiness_state() const { return _readiness_state; }
    bool zero_flush_complete() const { return _zero_flush_complete; }

private:
    friend class AP_VESC_SocketCANReceiveTest;
    friend class AP_VESC_CommandSchedulerTest;

    void loop();
    void send_commands();
    void handle_frame(const AP_HAL::CANFrame &frame);

    bool write_frame(const AP_HAL::CANFrame &frame, uint32_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &frame);

    int8_t motor_for_controller_id(uint8_t controller_id) const;
    bool motor_is_selected(uint8_t motor) const;
    static int8_t next_selected_motor(uint16_t mask, uint8_t start_motor);

    AP_HAL::CANIface *_can_iface = nullptr;
    HAL_BinarySemaphore _event_sem;
    HAL_Semaphore _command_sem;

    bool _initialized = false;
    bool _mode_latched = false;
    uint8_t _driver_index = 0;
    char _thread_name[16] {};

    int32_t _command_erpm[MAX_ESC] {};
    uint32_t _last_output_update_ms = 0;
    ControllerState _controller_state[MAX_ESC] {};
    uint32_t _last_external_rx_ms = 0;
    uint32_t _tx_error_count = 0;
    uint32_t _unexpected_rx_count = 0;
    uint32_t _last_tx_failure_ms = 0;
    uint32_t _last_tx_success_ms = 0;
    bool _command_timeout_active = false;
    ReadinessState _readiness_state = ReadinessState::POWERED_OFF;
    bool _last_armed = false;
    bool _zero_flush_complete = true;
    uint32_t _disarm_flush_start_ms = 0;
    uint16_t _zero_flush_success_mask = 0;
    uint8_t _next_command_motor = 0;
    Mode _active_mode = Mode::PPM;
    AP_VESC_Protocol::Protocol _active_protocol = AP_VESC_Protocol::Protocol::STANDARD;

    AP_Int16 _esc_mask;
    AP_Int16 _output_rate_hz;
    AP_Int32 _max_erpm;
    AP_Float _thrust_exponent;
    AP_Int8 _pole_pairs;
    AP_Int16 _input_min;
    AP_Int16 _input_mid;
    AP_Int16 _input_max;
    AP_Int16 _command_timeout_ms;
    AP_Int16 _telemetry_timeout_ms;
    AP_Int16 _disarm_flush_ms;
    AP_Int16 _controller_id[MAX_ESC];
    AP_Int8 _mode;
    AP_Int8 _protocol;
    AP_Int16 _extended_timeout_ms;
    AP_Int16 _energy_timeout_ms;
};

#endif // AP_VESC_ENABLED
