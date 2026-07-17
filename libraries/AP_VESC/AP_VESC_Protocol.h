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

struct Status1 {
    uint8_t controller_id;
    int32_t erpm;
    float current;
    float duty_cycle;
};

bool make_set_rpm_frame(uint8_t controller_id, int32_t erpm, AP_HAL::CANFrame &frame);
bool decode_status_1(const AP_HAL::CANFrame &frame, Status1 &status);

int32_t pwm_to_erpm(uint16_t pwm,
                    uint16_t pwm_min,
                    uint16_t pwm_mid,
                    uint16_t pwm_max,
                    int32_t max_erpm,
                    float thrust_exponent);

}
