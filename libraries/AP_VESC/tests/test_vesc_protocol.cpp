#include <AP_gtest.h>

#include <AP_VESC/AP_VESC_Protocol.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(VESCProtocol, MakeSetRPMFrame)
{
    AP_HAL::CANFrame frame;

    ASSERT_TRUE(AP_VESC_Protocol::make_set_rpm_frame(42, 17000, frame));
    EXPECT_TRUE(frame.isExtended());
    EXPECT_EQ(frame.id & AP_HAL::CANFrame::MaskExtID, (3U << 8) | 42U);
    ASSERT_EQ(frame.dlc, 4);
    EXPECT_EQ(frame.data[0], 0x00);
    EXPECT_EQ(frame.data[1], 0x00);
    EXPECT_EQ(frame.data[2], 0x42);
    EXPECT_EQ(frame.data[3], 0x68);

    ASSERT_TRUE(AP_VESC_Protocol::make_set_rpm_frame(7, -17000, frame));
    EXPECT_EQ(frame.data[0], 0xFF);
    EXPECT_EQ(frame.data[1], 0xFF);
    EXPECT_EQ(frame.data[2], 0xBD);
    EXPECT_EQ(frame.data[3], 0x98);

    ASSERT_TRUE(AP_VESC_Protocol::make_set_rpm_frame(0, 0, frame));
    EXPECT_EQ(frame.data[0], 0x00);
    EXPECT_EQ(frame.data[1], 0x00);
    EXPECT_EQ(frame.data[2], 0x00);
    EXPECT_EQ(frame.data[3], 0x00);

    EXPECT_FALSE(AP_VESC_Protocol::make_set_rpm_frame(255, 1, frame));
}

TEST(VESCProtocol, DecodeStatus1)
{
    const uint8_t data[] {0x00, 0x00, 0x42, 0x68, 0x00, 0x7B, 0x01, 0xF4};
    const AP_HAL::CANFrame frame(AP_HAL::CANFrame::FlagEFF | (9U << 8) | 42U,
                                 data,
                                 sizeof(data));
    AP_VESC_Protocol::Status1 status {};

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_1(frame, status));
    EXPECT_EQ(status.controller_id, 42);
    EXPECT_EQ(status.erpm, 17000);
    EXPECT_FLOAT_EQ(status.current, 12.3f);
    EXPECT_FLOAT_EQ(status.duty_cycle, 0.5f);
}

TEST(VESCProtocol, DecodeSignedStatus1)
{
    const uint8_t data[] {0xFF, 0xFF, 0xBD, 0x98, 0xFF, 0x85, 0xFE, 0x0C};
    const AP_HAL::CANFrame frame(AP_HAL::CANFrame::FlagEFF | (9U << 8) | 6U,
                                 data,
                                 sizeof(data));
    AP_VESC_Protocol::Status1 status {};

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_1(frame, status));
    EXPECT_EQ(status.controller_id, 6);
    EXPECT_EQ(status.erpm, -17000);
    EXPECT_FLOAT_EQ(status.current, -12.3f);
    EXPECT_FLOAT_EQ(status.duty_cycle, -0.5f);
}

TEST(VESCProtocol, DecodeStatus2And3)
{
    const uint8_t status2_data[] {0x00, 0x00, 0x30, 0x39, 0xFF, 0xFF, 0xE5, 0x8B};
    const uint8_t status3_data[] {0x00, 0x01, 0x09, 0x32, 0x00, 0x00, 0x56, 0xCE};
    AP_VESC_Protocol::Status2 status2 {};
    AP_VESC_Protocol::Status3 status3 {};

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_2(
                    AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (14U << 8) | 2U,
                                     status2_data, sizeof(status2_data)), status2));
    EXPECT_EQ(status2.controller_id, 2);
    EXPECT_NEAR(status2.amp_hours, 1.2345f, 0.00001f);
    EXPECT_NEAR(status2.amp_hours_charged, -0.6773f, 0.00001f);

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_3(
                    AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (15U << 8) | 3U,
                                     status3_data, sizeof(status3_data)), status3));
    EXPECT_EQ(status3.controller_id, 3);
    EXPECT_NEAR(status3.watt_hours, 6.7890f, 0.00001f);
    EXPECT_NEAR(status3.watt_hours_charged, 2.2222f, 0.00001f);
}

TEST(VESCProtocol, DecodeStatus4)
{
    const uint8_t data[] {0x01, 0xC7, 0x01, 0x41, 0xFF, 0x85, 0x04, 0xD2};
    AP_VESC_Protocol::Status4 status {};
    ASSERT_TRUE(AP_VESC_Protocol::decode_status_4(
                    AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (16U << 8) | 4U,
                                     data, sizeof(data)), status));
    EXPECT_FLOAT_EQ(status.mosfet_temperature, 45.5f);
    EXPECT_FLOAT_EQ(status.motor_temperature, 32.1f);
    EXPECT_FLOAT_EQ(status.input_current, -12.3f);
    EXPECT_NEAR(status.pid_position, 24.68f, 0.001f);
}

TEST(VESCProtocol, DecodeStatus5Protocols)
{
    const uint8_t data[] {0xFF, 0xFF, 0xCF, 0xC7, 0x01, 0xF4, 0xA5, 0x07};
    const AP_HAL::CANFrame frame(AP_HAL::CANFrame::FlagEFF | (27U << 8) | 5U,
                                 data, sizeof(data));
    AP_VESC_Protocol::Status5 standard {};
    AP_VESC_Protocol::Status5 deepex {};

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_5(
                    frame, AP_VESC_Protocol::Protocol::STANDARD, standard));
    EXPECT_EQ(standard.controller_id, 5);
    EXPECT_EQ(standard.tachometer, -12345);
    EXPECT_FLOAT_EQ(standard.input_voltage, 50.0f);
    EXPECT_EQ(standard.fault_code, 0);
    EXPECT_EQ(standard.warning_flags, 0);

    ASSERT_TRUE(AP_VESC_Protocol::decode_status_5(
                    frame, AP_VESC_Protocol::Protocol::DEEPEX_V1, deepex));
    EXPECT_EQ(deepex.fault_code, 7);
    EXPECT_EQ(deepex.warning_flags, 0xA5);
}

TEST(VESCProtocol, DecodeStandardSixByteStatus5)
{
    const uint8_t data[] {0x00, 0x00, 0x00, 0x2A, 0x00, 0xFA};
    AP_VESC_Protocol::Status5 status {};
    ASSERT_TRUE(AP_VESC_Protocol::decode_status_5(
                    AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (27U << 8) | 1U,
                                     data, sizeof(data)),
                    AP_VESC_Protocol::Protocol::DEEPEX_V1,
                    status));
    EXPECT_EQ(status.tachometer, 42);
    EXPECT_FLOAT_EQ(status.input_voltage, 25.0f);
    EXPECT_EQ(status.fault_code, 0);
    EXPECT_EQ(status.warning_flags, 0);
}

TEST(VESCProtocol, StatusTypeRejectsMalformedFrames)
{
    const uint8_t data[8] {};
    EXPECT_EQ(AP_VESC_Protocol::status_type(
                  AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (9U << 8), data, sizeof(data))),
              AP_VESC_Protocol::StatusType::STATUS_1);
    EXPECT_EQ(AP_VESC_Protocol::status_type(
                  AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (14U << 8), data, sizeof(data))),
              AP_VESC_Protocol::StatusType::STATUS_2);
    EXPECT_EQ(AP_VESC_Protocol::status_type(
                  AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagERR |
                                   (9U << 8), data, sizeof(data))),
              AP_VESC_Protocol::StatusType::NONE);
    EXPECT_EQ(AP_VESC_Protocol::status_type(
                  AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (9U << 8), data, 7)),
              AP_VESC_Protocol::StatusType::NONE);
}

TEST(VESCProtocol, MechanicalRPM)
{
    EXPECT_FLOAT_EQ(AP_VESC_Protocol::mechanical_rpm(17000, 7), 17000.0f / 7.0f);
    EXPECT_FLOAT_EQ(AP_VESC_Protocol::mechanical_rpm(-17000, 7), -17000.0f / 7.0f);
    EXPECT_FLOAT_EQ(AP_VESC_Protocol::mechanical_rpm(17000, 0), 0.0f);
}

TEST(VESCProtocol, SeparateTelemetryFreshness)
{
    AP_VESC_Protocol::Freshness value =
        AP_VESC_Protocol::freshness(2000, 1900, 500, 600, 1600, 1700, 500, 500, 2000);
    EXPECT_TRUE(value.fast);
    EXPECT_TRUE(value.extended);
    EXPECT_TRUE(value.energy);
    EXPECT_FALSE(value.stale);

    value = AP_VESC_Protocol::freshness(3000, 2900, 500, 600, 1000, 2900, 500, 500, 2000);
    EXPECT_TRUE(value.fast);
    EXPECT_FALSE(value.extended);
    EXPECT_FALSE(value.energy);
    EXPECT_FALSE(value.stale);

    value = AP_VESC_Protocol::freshness(4000, 3000, 3900, 3900, 3900, 3900, 500, 500, 2000);
    EXPECT_FALSE(value.fast);
    EXPECT_TRUE(value.extended);
    EXPECT_TRUE(value.energy);
    EXPECT_TRUE(value.stale);
}

TEST(VESCProtocol, CommandModeAndPhysicalOutputPolicy)
{
    EXPECT_FALSE(AP_VESC_Protocol::can_command_mode(0));
    EXPECT_TRUE(AP_VESC_Protocol::can_command_mode(1));
    EXPECT_FALSE(AP_VESC_Protocol::can_command_mode(2));
    EXPECT_EQ(AP_VESC_Protocol::physical_pwm(0, 1700, 1500), 1700);
    EXPECT_EQ(AP_VESC_Protocol::physical_pwm(1, 1700, 1500), 1500);
}

TEST(VESCProtocol, CommandSafetyConditions)
{
    AP_VESC_Protocol::CommandConditions conditions {
        true,   // armed
        false,  // emergency stop
        true,   // command fresh
        true,   // interface available
        false,  // interface down
        true,   // TX healthy
        true,   // all controllers ready
    };
    EXPECT_TRUE(AP_VESC_Protocol::allow_nonzero_command(conditions));
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(-17000, conditions), -17000);

    conditions.command_fresh = false;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
    conditions.command_fresh = true;
    conditions.armed = false;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
    conditions.armed = true;
    conditions.emergency_stop = true;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
    conditions.emergency_stop = false;
    conditions.interface_available = false;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
    conditions.interface_available = true;
    conditions.interface_down = true;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
    conditions.interface_down = false;
    conditions.all_controllers_ready = false;
    EXPECT_EQ(AP_VESC_Protocol::safe_command_erpm(17000, conditions), 0);
}

TEST(VESCProtocol, ZeroFlushRequiresTimeAndEveryController)
{
    constexpr uint16_t expected = 0x003F;
    EXPECT_FALSE(AP_VESC_Protocol::zero_flush_complete(expected, expected, 499, 500));
    EXPECT_FALSE(AP_VESC_Protocol::zero_flush_complete(expected, 0x001F, 500, 500));
    EXPECT_TRUE(AP_VESC_Protocol::zero_flush_complete(expected, expected, 500, 500));
    EXPECT_TRUE(AP_VESC_Protocol::zero_flush_complete(0, 0, 500, 500));
}

TEST(VESCProtocol, SixMotorConfiguration)
{
    int16_t ids[12] {1, 2, 3, 4, 5, 6};
    bool assigned[12] {true, true, true, true, true, true};

    AP_VESC_Protocol::ConfigurationResult result =
        AP_VESC_Protocol::validate_configuration(0x003F, ids, assigned, 12);
    EXPECT_EQ(result.error, AP_VESC_Protocol::ConfigurationError::NONE);

    ids[5] = 5;
    result = AP_VESC_Protocol::validate_configuration(0x003F, ids, assigned, 12);
    EXPECT_EQ(result.error, AP_VESC_Protocol::ConfigurationError::DUPLICATE_ID);
    EXPECT_EQ(result.motor, 4);
    EXPECT_EQ(result.other_motor, 5);

    ids[5] = 6;
    assigned[3] = false;
    result = AP_VESC_Protocol::validate_configuration(0x003F, ids, assigned, 12);
    EXPECT_EQ(result.error, AP_VESC_Protocol::ConfigurationError::MISSING_FUNCTION);
    EXPECT_EQ(result.motor, 3);

    assigned[3] = true;
    ids[2] = 255;
    result = AP_VESC_Protocol::validate_configuration(0x003F, ids, assigned, 12);
    EXPECT_EQ(result.error, AP_VESC_Protocol::ConfigurationError::INVALID_ID);
    EXPECT_EQ(result.motor, 2);

    ids[2] = 3;
    EXPECT_EQ(AP_VESC_Protocol::validate_configuration(0, ids, assigned, 12).error,
              AP_VESC_Protocol::ConfigurationError::EMPTY_MASK);
    EXPECT_EQ(AP_VESC_Protocol::validate_configuration(0x1000, ids, assigned, 12).error,
              AP_VESC_Protocol::ConfigurationError::INVALID_MASK);
}

TEST(VESCProtocol, MAVLinkExtensionContract)
{
    AP_VESC_Protocol::ControllerState state {};
    state.motor_number = 6;
    state.controller_id = 42;
    state.erpm = -17000;
    state.mechanical_rpm = -17000.0f / 7.0f;
    state.motor_current = -12.3f;
    state.input_current = 10.2f;
    state.duty_cycle = -0.5f;
    state.input_voltage = 50.0f;
    state.mosfet_temperature = 45.5f;
    state.motor_temperature = 32.1f;
    state.amp_hours = 1.2345f;
    state.amp_hours_charged = 0.25f;
    state.watt_hours = 6.789f;
    state.watt_hours_charged = 1.5f;
    state.pid_position = 24.68f;
    state.tachometer = -12345;
    state.fault_code = 7;
    state.warning_flags = 0xA5;
    state.last_status1_ms = 100;
    state.last_status2_ms = 200;
    state.last_status3_ms = 300;
    state.last_status4_ms = 400;
    state.last_status5_ms = 500;
    state.configured = true;
    state.expected = true;
    state.present = true;
    state.fast_telemetry_valid = true;
    state.extended_telemetry_valid = true;
    state.energy_telemetry_valid = true;
    state.command_ready = false;
    state.active_fault = true;

    float data[AP_VESC_Protocol::MAVLINK_EXTENSION_LENGTH];
    AP_VESC_Protocol::pack_mavlink_extension(state, data);
    EXPECT_FLOAT_EQ(data[0], 1);
    EXPECT_FLOAT_EQ(data[1], 6);
    EXPECT_FLOAT_EQ(data[2], 42);
    EXPECT_EQ(uint16_t(data[3]), 16U | 4U | 2U | 1U | 64U | 128U | 256U);
    EXPECT_FLOAT_EQ(data[4], -17000);
    EXPECT_FLOAT_EQ(data[5], -17000.0f / 7.0f);
    EXPECT_FLOAT_EQ(data[6], -12.3f);
    EXPECT_FLOAT_EQ(data[7], 10.2f);
    EXPECT_FLOAT_EQ(data[8], -0.5f);
    EXPECT_FLOAT_EQ(data[9], 50.0f);
    EXPECT_FLOAT_EQ(data[10], 45.5f);
    EXPECT_FLOAT_EQ(data[11], 32.1f);
    EXPECT_FLOAT_EQ(data[12], 1.2345f);
    EXPECT_FLOAT_EQ(data[13], 0.25f);
    EXPECT_FLOAT_EQ(data[14], 6.789f);
    EXPECT_FLOAT_EQ(data[15], 1.5f);
    EXPECT_FLOAT_EQ(data[16], 24.68f);
    EXPECT_FLOAT_EQ(data[17], -12345);
    EXPECT_FLOAT_EQ(data[18], 7);
    EXPECT_FLOAT_EQ(data[19], 0xA5);
    EXPECT_FLOAT_EQ(data[20], 100);
    EXPECT_FLOAT_EQ(data[21], 400);
    EXPECT_FLOAT_EQ(data[22], 500);
    EXPECT_FLOAT_EQ(data[23], 200);
    EXPECT_FLOAT_EQ(data[24], 300);
    for (uint8_t i = 25; i < AP_VESC_Protocol::MAVLINK_EXTENSION_LENGTH; i++) {
        EXPECT_FLOAT_EQ(data[i], 0);
    }

    state.fast_telemetry_valid = false;
    state.extended_telemetry_valid = false;
    state.energy_telemetry_valid = false;
    state.telemetry_stale = true;
    AP_VESC_Protocol::pack_mavlink_extension(state, data);
    EXPECT_EQ(uint16_t(data[3]), 8U | 16U | 64U | 128U | 256U);
    for (uint8_t i = 4; i <= 17; i++) {
        EXPECT_FLOAT_EQ(data[i], 0);
    }
    EXPECT_FLOAT_EQ(data[18], 7);
    EXPECT_FLOAT_EQ(data[19], 0xA5);
    EXPECT_FLOAT_EQ(data[20], 100);
    EXPECT_FLOAT_EQ(data[24], 300);
}

TEST(VESCProtocol, RejectInvalidStatus1)
{
    const uint8_t data[8] {};
    AP_VESC_Protocol::Status1 status {};

    EXPECT_FALSE(AP_VESC_Protocol::decode_status_1(AP_HAL::CANFrame(9U << 8, data, sizeof(data)), status));
    EXPECT_FALSE(AP_VESC_Protocol::decode_status_1(
                     AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (8U << 8), data, sizeof(data)), status));
    EXPECT_FALSE(AP_VESC_Protocol::decode_status_1(
                     AP_HAL::CANFrame(AP_HAL::CANFrame::FlagEFF | (9U << 8), data, 7), status));
}

TEST(VESCProtocol, PWMToERPM)
{
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1500, 1100, 1500, 1900, 17000, 2.0f), 0);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1900, 1100, 1500, 1900, 17000, 2.0f), 17000);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1100, 1100, 1500, 1900, 17000, 2.0f), -17000);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1600, 1100, 1500, 1900, 17000, 2.0f), 8500);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1400, 1100, 1500, 1900, 17000, 2.0f), -8500);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(2200, 1100, 1500, 1900, 17000, 2.0f), 17000);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(800, 1100, 1500, 1900, 17000, 2.0f), -17000);
}

TEST(VESCProtocol, RejectInvalidScaling)
{
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1600, 1500, 1100, 1900, 17000, 2.0f), 0);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1600, 1100, 1500, 1900, 0, 2.0f), 0);
    EXPECT_EQ(AP_VESC_Protocol::pwm_to_erpm(1600, 1100, 1500, 1900, 17000, 0.0f), 0);
}

AP_GTEST_MAIN()
