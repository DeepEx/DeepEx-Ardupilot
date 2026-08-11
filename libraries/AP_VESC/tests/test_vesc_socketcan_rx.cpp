#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_VESC/AP_VESC.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && HAL_LINUX_USE_VIRTUAL_CAN

#include <AP_HAL_Linux/CANSocketIface.h>

#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class AP_VESC_SocketCANReceiveTest
{
public:
    static void attach(AP_VESC &vesc, AP_HAL::CANIface &can_iface)
    {
        vesc._can_iface = &can_iface;
        vesc._initialized = true;
    }

    static bool read_frame(AP_VESC &vesc, AP_HAL::CANFrame &frame)
    {
        return vesc.read_frame(frame);
    }
};

class ExternalCANSocket
{
public:
    ~ExternalCANSocket()
    {
        if (_fd >= 0) {
            close(_fd);
        }
    }

    bool open(const char *iface_name)
    {
        _fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (_fd < 0) {
            return false;
        }

        ifreq ifr {};
        if (strlen(iface_name) >= sizeof(ifr.ifr_name)) {
            return false;
        }
        strncpy(ifr.ifr_name, iface_name, sizeof(ifr.ifr_name) - 1);
        if (ioctl(_fd, SIOCGIFINDEX, &ifr) < 0) {
            return false;
        }

        sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        return bind(_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0;
    }

    bool send(const uint32_t identifier, const uint8_t *payload, const uint8_t payload_length) const
    {
        if (_fd < 0 || payload_length > CAN_MAX_DLEN) {
            return false;
        }

        can_frame frame {};
        frame.can_id = CAN_EFF_FLAG | identifier;
        frame.can_dlc = payload_length;
        memcpy(frame.data, payload, payload_length);
        return write(_fd, &frame, sizeof(frame)) == sizeof(frame);
    }

private:
    int _fd = -1;
};

TEST(VESCSocketCANReceive, ExternalStatus1ReachesAPVESC)
{
    if (if_nametoindex("vcan0") == 0) {
        GTEST_SKIP() << "requires vcan0; run through run_vesc_socketcan_rx_test.sh";
    }

    static Linux::CANIface can_iface(0);
    ASSERT_TRUE(can_iface.init(500000));

    ExternalCANSocket external;
    ASSERT_TRUE(external.open("vcan0"));

    const uint32_t identifier = (9U << 8) | 2U;
    const uint8_t payload[8] {};
    ASSERT_TRUE(external.send(identifier, payload, sizeof(payload)));

    bool read_select = true;
    bool write_select = false;
    ASSERT_TRUE(can_iface.select(read_select, write_select, nullptr, AP_HAL::micros64()));
    EXPECT_FALSE(read_select);

    AP_VESC vesc;
    AP_VESC_SocketCANReceiveTest::attach(vesc, can_iface);

    AP_HAL::CANFrame frame;
    ASSERT_TRUE(AP_VESC_SocketCANReceiveTest::read_frame(vesc, frame));
    EXPECT_TRUE(frame.isExtended());
    EXPECT_EQ(frame.id & AP_HAL::CANFrame::MaskExtID, identifier);
    EXPECT_EQ(frame.id & 0xFFU, 2U);
    EXPECT_EQ(frame.dlc, sizeof(payload));
    EXPECT_EQ(memcmp(frame.data, payload, sizeof(payload)), 0);

    AP_VESC_Protocol::Status1 status {};
    EXPECT_TRUE(AP_VESC_Protocol::decode_status_1(frame, status));

    ASSERT_TRUE(external.send(identifier, payload, sizeof(payload)));
    uint64_t timestamp_us;
    AP_HAL::CANIface::CanIOFlags flags {};
    ASSERT_EQ(can_iface.receive(frame, timestamp_us, flags), 1);
    EXPECT_EQ(flags & AP_HAL::CANIface::Loopback, 0);
}

#else

TEST(VESCSocketCANReceive, RequiresLinuxVirtualCAN)
{
    GTEST_SKIP() << "requires a Linux build configured for virtual CAN";
}

#endif

AP_GTEST_MAIN()
