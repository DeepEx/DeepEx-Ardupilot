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

    struct ControllerState {
        int32_t erpm;
        float mechanical_rpm;
        float motor_current;
        float duty_cycle;
        float amp_hours;
        float amp_hours_charged;
        float watt_hours;
        float watt_hours_charged;
        float mosfet_temperature;
        float motor_temperature;
        float input_current;
        float pid_position;
        int32_t tachometer;
        float input_voltage;
        uint32_t last_status1_ms;
        uint32_t last_status2_ms;
        uint32_t last_status3_ms;
        uint32_t last_status4_ms;
        uint32_t last_status5_ms;
        uint8_t controller_id;
        uint8_t motor_number;
        uint8_t fault_code;
        uint8_t warning_flags;
        bool present;
        bool command_ready;
        bool fast_telemetry_valid;
        bool extended_telemetry_valid;
        bool energy_telemetry_valid;
        bool telemetry_stale;
        bool active_fault;
    };

    AP_VESC();

    CLASS_NO_COPY(AP_VESC);

    static const AP_Param::GroupInfo var_info[];

    static AP_VESC *get_vesc(uint8_t driver_index);

    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface *can_iface) override;

    // copy the final motor outputs from SRV_Channels
    void update();

    bool pre_arm_check(char *reason, uint8_t reason_len) const;

    Mode mode() const { return Mode(_mode.get()); }
    AP_VESC_Protocol::Protocol protocol() const { return AP_VESC_Protocol::Protocol(_protocol.get()); }
    uint16_t expected_mask() const;
    uint16_t present_mask() const;
    uint16_t fault_mask() const;
    uint16_t diagnostic_flags() const;
    bool get_controller_state(uint8_t motor, ControllerState &state) const;
    ReadinessState readiness_state() const { return _readiness_state; }
    bool zero_flush_complete() const { return _zero_flush_complete; }

private:
    void loop();
    void send_commands();
    void handle_frame(const AP_HAL::CANFrame &frame);

    bool write_frame(const AP_HAL::CANFrame &frame, uint32_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);

    int8_t motor_for_controller_id(uint8_t controller_id) const;
    bool motor_is_selected(uint8_t motor) const;

    AP_HAL::CANIface *_can_iface;
    HAL_BinarySemaphore _event_sem;
    HAL_Semaphore _command_sem;

    bool _initialized;
    uint8_t _driver_index;
    char _thread_name[16];

    int32_t _command_erpm[MAX_ESC];
    uint32_t _last_output_update_ms;
    ControllerState _controller_state[MAX_ESC];
    uint32_t _last_external_rx_ms;
    uint32_t _tx_error_count;
    uint32_t _unexpected_rx_count;
    uint32_t _last_tx_failure_ms;
    uint32_t _last_tx_success_ms;
    bool _command_timeout_active;
    ReadinessState _readiness_state = ReadinessState::POWERED_OFF;
    bool _last_armed;
    bool _zero_flush_complete = true;
    uint32_t _disarm_flush_start_ms;

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
    AP_Int8 _require_telemetry;
    AP_Int16 _disarm_flush_ms;
    AP_Int16 _controller_id[MAX_ESC];
    AP_Int8 _mode;
    AP_Int8 _protocol;
    AP_Int16 _extended_timeout_ms;
    AP_Int16 _energy_timeout_ms;
};

#endif // AP_VESC_ENABLED
