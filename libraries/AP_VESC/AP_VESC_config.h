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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_VESC_ENABLED
#define AP_VESC_ENABLED HAL_NUM_CAN_IFACES
#endif

// Default interval of the dedicated per-motor VESC_V1 MAVLink message
// (DEBUG_FLOAT_ARRAY, id 350). One motor is sent per interval in
// round-robin order, so six motors are each refreshed at
// 1000 / (6 * 33) = ~5 Hz. A client can override it with
// MAV_CMD_SET_MESSAGE_INTERVAL.
#ifndef AP_VESC_MAVLINK_V1_INTERVAL_MS
#define AP_VESC_MAVLINK_V1_INTERVAL_MS 33
#endif
