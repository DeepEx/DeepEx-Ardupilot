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

static uint16_t get_uint16_be(const uint8_t *data)
{
    return (uint16_t(data[0]) << 8) | uint16_t(data[1]);
}

static bool status_frame(const AP_HAL::CANFrame &frame,
                         const uint8_t packet_type,
                         const uint8_t expected_dlc,
                         uint8_t &controller_id)
{
    if (!frame.isExtended() || frame.isErrorFrame() || frame.isRemoteTransmissionRequest() ||
        frame.dlc != expected_dlc) {
        return false;
    }

    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if ((id >> 8) != packet_type) {
        return false;
    }
    controller_id = id & 0xFF;
    return controller_id <= MAX_CONTROLLER_ID;
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
    if (!status_frame(frame, CAN_PACKET_STATUS, 8, status.controller_id)) {
        return false;
    }

    status.erpm = get_int32_be(&frame.data[0]);
    status.current = get_int16_be(&frame.data[4]) * 0.1f;
    status.duty_cycle = get_int16_be(&frame.data[6]) * 0.001f;
    return true;
}

bool decode_status_2(const AP_HAL::CANFrame &frame, Status2 &status)
{
    if (!status_frame(frame, CAN_PACKET_STATUS_2, 8, status.controller_id)) {
        return false;
    }
    status.amp_hours = get_int32_be(&frame.data[0]) * 0.0001f;
    status.amp_hours_charged = get_int32_be(&frame.data[4]) * 0.0001f;
    return true;
}

bool decode_status_3(const AP_HAL::CANFrame &frame, Status3 &status)
{
    if (!status_frame(frame, CAN_PACKET_STATUS_3, 8, status.controller_id)) {
        return false;
    }
    status.watt_hours = get_int32_be(&frame.data[0]) * 0.0001f;
    status.watt_hours_charged = get_int32_be(&frame.data[4]) * 0.0001f;
    return true;
}

bool decode_status_4(const AP_HAL::CANFrame &frame, Status4 &status)
{
    if (!status_frame(frame, CAN_PACKET_STATUS_4, 8, status.controller_id)) {
        return false;
    }
    status.mosfet_temperature = get_int16_be(&frame.data[0]) * 0.1f;
    status.motor_temperature = get_int16_be(&frame.data[2]) * 0.1f;
    status.input_current = get_int16_be(&frame.data[4]) * 0.1f;
    status.pid_position = get_int16_be(&frame.data[6]) * 0.02f;
    return true;
}

bool decode_status_5(const AP_HAL::CANFrame &frame, const Protocol protocol, Status5 &status)
{
    if (!frame.isExtended() || frame.isErrorFrame() || frame.isRemoteTransmissionRequest() ||
        (frame.dlc != 6 && frame.dlc != 8)) {
        return false;
    }
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if ((id >> 8) != CAN_PACKET_STATUS_5) {
        return false;
    }
    status.controller_id = id & 0xFF;
    if (status.controller_id > MAX_CONTROLLER_ID) {
        return false;
    }
    status.tachometer = get_int32_be(&frame.data[0]);
    status.input_voltage = get_uint16_be(&frame.data[4]) * 0.1f;
    status.fault_code = 0;
    status.warning_flags = 0;
    if (protocol == Protocol::DEEPEX_V1 && frame.dlc == 8) {
        const uint16_t deepex_status = get_uint16_be(&frame.data[6]);
        status.fault_code = deepex_status & 0xFF;
        status.warning_flags = deepex_status >> 8;
    }
    return true;
}

StatusType status_type(const AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isErrorFrame() || frame.isRemoteTransmissionRequest()) {
        return StatusType::NONE;
    }
    switch ((frame.id & AP_HAL::CANFrame::MaskExtID) >> 8) {
    case CAN_PACKET_STATUS:
        return frame.dlc == 8 ? StatusType::STATUS_1 : StatusType::NONE;
    case CAN_PACKET_STATUS_2:
        return frame.dlc == 8 ? StatusType::STATUS_2 : StatusType::NONE;
    case CAN_PACKET_STATUS_3:
        return frame.dlc == 8 ? StatusType::STATUS_3 : StatusType::NONE;
    case CAN_PACKET_STATUS_4:
        return frame.dlc == 8 ? StatusType::STATUS_4 : StatusType::NONE;
    case CAN_PACKET_STATUS_5:
        return (frame.dlc == 6 || frame.dlc == 8) ? StatusType::STATUS_5 : StatusType::NONE;
    default:
        return StatusType::NONE;
    }
}

float mechanical_rpm(const int32_t erpm, const uint8_t pole_pairs)
{
    return pole_pairs == 0 ? 0.0f : erpm / float(pole_pairs);
}

Freshness freshness(const uint32_t now_ms,
                    const uint32_t last_status1_ms,
                    const uint32_t last_status2_ms,
                    const uint32_t last_status3_ms,
                    const uint32_t last_status4_ms,
                    const uint32_t last_status5_ms,
                    const uint32_t fast_timeout_ms,
                    const uint32_t extended_timeout_ms,
                    const uint32_t energy_timeout_ms)
{
    Freshness result {};
    result.fast = last_status1_ms != 0 && now_ms - last_status1_ms <= fast_timeout_ms;
    result.extended = last_status4_ms != 0 && last_status5_ms != 0 &&
                      now_ms - last_status4_ms <= extended_timeout_ms &&
                      now_ms - last_status5_ms <= extended_timeout_ms;
    result.energy = last_status2_ms != 0 && last_status3_ms != 0 &&
                    now_ms - last_status2_ms <= energy_timeout_ms &&
                    now_ms - last_status3_ms <= energy_timeout_ms;
    result.stale = !result.fast;
    return result;
}

bool can_command_mode(const uint8_t mode)
{
    return mode == 1;
}

uint16_t physical_pwm(const uint8_t mode, const uint16_t requested_pwm, const uint16_t neutral_pwm)
{
    return can_command_mode(mode) ? neutral_pwm : requested_pwm;
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
