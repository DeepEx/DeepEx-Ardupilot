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

#include <AP_HAL/CANIface.h>

namespace AP_VESC_Protocol
{

static constexpr uint8_t MAX_CONTROLLER_ID = 254;
static constexpr uint8_t CAN_PACKET_SET_RPM = 3;
static constexpr uint8_t CAN_PACKET_STATUS = 9;
static constexpr uint8_t CAN_PACKET_STATUS_2 = 14;
static constexpr uint8_t CAN_PACKET_STATUS_3 = 15;
static constexpr uint8_t CAN_PACKET_STATUS_4 = 16;
static constexpr uint8_t CAN_PACKET_STATUS_5 = 27;

enum class Protocol : uint8_t {
    STANDARD = 0,
    DEEPEX_V1 = 1,
};

enum class StatusType : uint8_t {
    NONE,
    STATUS_1,
    STATUS_2,
    STATUS_3,
    STATUS_4,
    STATUS_5,
};

struct Status1 {
    uint8_t controller_id;
    int32_t erpm;
    float current;
    float duty_cycle;
};

struct Status2 {
    uint8_t controller_id;
    float amp_hours;
    float amp_hours_charged;
};

struct Status3 {
    uint8_t controller_id;
    float watt_hours;
    float watt_hours_charged;
};

struct Status4 {
    uint8_t controller_id;
    float mosfet_temperature;
    float motor_temperature;
    float input_current;
    float pid_position;
};

struct Status5 {
    uint8_t controller_id;
    int32_t tachometer;
    float input_voltage;
    uint8_t fault_code;
    uint8_t warning_flags;
};

struct Freshness {
    bool fast;
    bool extended;
    bool energy;
    bool stale;
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
    bool configured;
    bool expected;
    bool present;
    bool command_ready;
    bool fast_telemetry_valid;
    bool extended_telemetry_valid;
    bool energy_telemetry_valid;
    bool telemetry_stale;
    bool active_fault;
};

enum class ConfigurationError : uint8_t {
    NONE,
    EMPTY_MASK,
    INVALID_MASK,
    INVALID_ID,
    MISSING_FUNCTION,
    DUPLICATE_ID,
};

struct ConfigurationResult {
    ConfigurationError error;
    uint8_t motor;
    uint8_t other_motor;
};

struct CommandConditions {
    bool armed;
    bool emergency_stop;
    bool command_fresh;
    bool interface_available;
    bool interface_down;
    bool tx_healthy;
    bool all_controllers_ready;
};

static constexpr uint8_t MAVLINK_EXTENSION_LENGTH = 58;

bool make_set_rpm_frame(uint8_t controller_id, int32_t erpm, AP_HAL::CANFrame &frame);
bool decode_status_1(const AP_HAL::CANFrame &frame, Status1 &status);
bool decode_status_2(const AP_HAL::CANFrame &frame, Status2 &status);
bool decode_status_3(const AP_HAL::CANFrame &frame, Status3 &status);
bool decode_status_4(const AP_HAL::CANFrame &frame, Status4 &status);
bool decode_status_5(const AP_HAL::CANFrame &frame, Protocol protocol, Status5 &status);
StatusType status_type(const AP_HAL::CANFrame &frame);
float mechanical_rpm(int32_t erpm, uint8_t pole_pairs);
Freshness freshness(uint32_t now_ms,
                    uint32_t last_status1_ms,
                    uint32_t last_status2_ms,
                    uint32_t last_status3_ms,
                    uint32_t last_status4_ms,
                    uint32_t last_status5_ms,
                    uint32_t fast_timeout_ms,
                    uint32_t extended_timeout_ms,
                    uint32_t energy_timeout_ms);
bool can_command_mode(uint8_t mode);
uint16_t physical_pwm(uint8_t mode, uint16_t requested_pwm, uint16_t neutral_pwm);
bool allow_nonzero_command(const CommandConditions &conditions);
int32_t safe_command_erpm(int32_t requested_erpm, const CommandConditions &conditions);
bool zero_flush_complete(uint16_t expected_mask,
                         uint16_t successful_zero_mask,
                         uint32_t elapsed_ms,
                         uint32_t required_ms);
ConfigurationResult validate_configuration(uint16_t mask,
                                           const int16_t *controller_ids,
                                           const bool *function_assigned,
                                           uint8_t motor_count);
uint16_t freshness_flags(const ControllerState &state);
void pack_mavlink_extension(const ControllerState &state,
                            float (&data)[MAVLINK_EXTENSION_LENGTH]);

int32_t pwm_to_erpm(uint16_t pwm,
                    uint16_t pwm_min,
                    uint16_t pwm_mid,
                    uint16_t pwm_max,
                    int32_t max_erpm,
                    float thrust_exponent);

}
