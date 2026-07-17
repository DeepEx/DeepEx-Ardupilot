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
