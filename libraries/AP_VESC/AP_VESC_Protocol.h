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

int32_t pwm_to_erpm(uint16_t pwm,
                    uint16_t pwm_min,
                    uint16_t pwm_mid,
                    uint16_t pwm_max,
                    int32_t max_erpm,
                    float thrust_exponent);

}
