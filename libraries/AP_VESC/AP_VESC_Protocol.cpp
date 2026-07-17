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

#include "AP_VESC_Protocol.h"

#include <AP_Math/AP_Math.h>

namespace AP_VESC_Protocol
{

static int32_t get_int32_be(const uint8_t *data)
{
    const uint32_t value = (uint32_t(data[0]) << 24) |
                           (uint32_t(data[1]) << 16) |
                           (uint32_t(data[2]) << 8) |
                           uint32_t(data[3]);
    return int32_t(value);
}

static int16_t get_int16_be(const uint8_t *data)
{
    const uint16_t value = (uint16_t(data[0]) << 8) | uint16_t(data[1]);
    return int16_t(value);
}

bool make_set_rpm_frame(const uint8_t controller_id, const int32_t erpm, AP_HAL::CANFrame &frame)
{
    if (controller_id > MAX_CONTROLLER_ID) {
        return false;
    }

    const uint8_t data[] {
        uint8_t(uint32_t(erpm) >> 24),
        uint8_t(uint32_t(erpm) >> 16),
        uint8_t(uint32_t(erpm) >> 8),
        uint8_t(uint32_t(erpm))
    };
    const uint32_t id = AP_HAL::CANFrame::FlagEFF |
                        (uint32_t(CAN_PACKET_SET_RPM) << 8) |
                        controller_id;
    frame = AP_HAL::CANFrame(id, data, sizeof(data));
    return true;
}

bool decode_status_1(const AP_HAL::CANFrame &frame, Status1 &status)
{
    if (!frame.isExtended() || frame.dlc != 8) {
        return false;
    }

    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if ((id >> 8) != CAN_PACKET_STATUS) {
        return false;
    }

    status.controller_id = id & 0xFF;
    status.erpm = get_int32_be(&frame.data[0]);
    status.current = get_int16_be(&frame.data[4]) * 0.1f;
    status.duty_cycle = get_int16_be(&frame.data[6]) * 0.001f;
    return true;
}

int32_t pwm_to_erpm(const uint16_t pwm,
                    const uint16_t pwm_min,
                    const uint16_t pwm_mid,
                    const uint16_t pwm_max,
                    const int32_t max_erpm,
                    const float thrust_exponent)
{
    if (pwm == 0 || pwm_min >= pwm_mid || pwm_mid >= pwm_max ||
        max_erpm <= 0 || !is_positive(thrust_exponent)) {
        return 0;
    }

    float thrust;
    if (pwm >= pwm_mid) {
        thrust = float(MIN(pwm, pwm_max) - pwm_mid) / (pwm_max - pwm_mid);
    } else {
        thrust = -float(pwm_mid - MAX(pwm, pwm_min)) / (pwm_mid - pwm_min);
    }

    if (is_zero(thrust)) {
        return 0;
    }

    const float erpm_ratio = powf(fabsf(thrust), 1.0f / thrust_exponent);
    return int32_t(roundf(copysignf(erpm_ratio * max_erpm, thrust)));
}

}
