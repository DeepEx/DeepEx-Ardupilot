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
