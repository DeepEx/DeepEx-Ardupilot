#include <AP_gtest.h>

#include <AP_VESC/AP_VESC.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class AP_VESC_CommandSchedulerTest
{
public:
    static int8_t next_selected_motor(const uint16_t mask, const uint8_t start_motor)
    {
        return AP_VESC::next_selected_motor(mask, start_motor);
    }
};

TEST(VESCCommandScheduler, VisitsEverySelectedMotor)
{
    constexpr uint16_t mask = 0x000F;
    uint8_t next_motor = 0;
    const uint8_t expected[] {0, 1, 2, 3, 0, 1, 2, 3};

    for (const uint8_t motor : expected) {
        const int8_t selected = AP_VESC_CommandSchedulerTest::next_selected_motor(mask, next_motor);
        ASSERT_GE(selected, 0);
        EXPECT_EQ(selected, motor);
        next_motor = (selected + 1) % AP_VESC::MAX_ESC;
    }
}

TEST(VESCCommandScheduler, SkipsUnselectedMotors)
{
    constexpr uint16_t mask = 0x000C;
    EXPECT_EQ(AP_VESC_CommandSchedulerTest::next_selected_motor(mask, 0), 2);
    EXPECT_EQ(AP_VESC_CommandSchedulerTest::next_selected_motor(mask, 3), 3);
    EXPECT_EQ(AP_VESC_CommandSchedulerTest::next_selected_motor(mask, 4), 2);
    EXPECT_EQ(AP_VESC_CommandSchedulerTest::next_selected_motor(0, 0), -1);
}

AP_GTEST_MAIN()
