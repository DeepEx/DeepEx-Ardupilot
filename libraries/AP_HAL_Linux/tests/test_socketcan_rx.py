#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

import errno
import os
import socket
import struct
import subprocess
import unittest

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_RAW_RECV_OWN_MSGS = 4
CAN_FRAME = struct.Struct("=IB3x8s")
INTERFACE = os.environ.get("ARDUPILOT_VCAN_IFACE", "vcan_vesc")


def pack_frame(identifier, payload):
    return CAN_FRAME.pack(CAN_EFF_FLAG | identifier, len(payload), payload.ljust(8, b"\0"))


def unpack_frame(frame):
    identifier, dlc, payload = CAN_FRAME.unpack(frame)
    return identifier, payload[:dlc]


class SocketCANReceiveTest(unittest.TestCase):
    def setUp(self):
        self.external_rx = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.local_tx = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.external_rx.settimeout(1)
        self.local_tx.settimeout(1)
        self.local_tx.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 1)
        self.external_rx.bind((INTERFACE,))
        self.local_tx.bind((INTERFACE,))

    def tearDown(self):
        self.external_rx.close()
        self.local_tx.close()

    def test_external_extended_frame_and_signed_payload(self):
        identifier = (9 << 8) | 6
        payload = struct.pack(">ihh", -17000, -123, 500)
        self.local_tx.send(pack_frame(identifier, payload))

        frame, _ancillary, flags, _address = self.external_rx.recvmsg(CAN_FRAME.size)
        received_id, received_payload = unpack_frame(frame)
        self.assertEqual(flags & socket.MSG_CONFIRM, 0)
        self.assertNotEqual(received_id & CAN_EFF_FLAG, 0)
        self.assertEqual(received_id & CAN_EFF_MASK, identifier)
        self.assertEqual(received_payload, payload)
        self.assertEqual(struct.unpack(">ihh", received_payload), (-17000, -123, 500))

    def test_local_tx_confirmation_is_not_external_rx(self):
        identifier = (3 << 8) | 1
        payload = struct.pack(">i", 17000)
        self.local_tx.send(pack_frame(identifier, payload))

        local_frame, _ancillary, local_flags, _address = self.local_tx.recvmsg(CAN_FRAME.size)
        external_frame, _ancillary, external_flags, _address = self.external_rx.recvmsg(CAN_FRAME.size)
        self.assertNotEqual(local_flags & socket.MSG_CONFIRM, 0)
        self.assertEqual(external_flags & socket.MSG_CONFIRM, 0)
        self.assertEqual(unpack_frame(local_frame), unpack_frame(external_frame))

    def test_interface_down_fails_safely(self):
        if os.geteuid() != 0:
            self.skipTest("interface-down check requires root in an isolated test environment")
        subprocess.run(["ip", "link", "set", INTERFACE, "down"], check=True)
        try:
            with self.assertRaises(OSError) as raised:
                self.local_tx.send(pack_frame((3 << 8) | 1, struct.pack(">i", 0)))
            self.assertIn(raised.exception.errno, (errno.ENETDOWN, errno.ENODEV, errno.ENXIO))
        finally:
            subprocess.run(["ip", "link", "set", INTERFACE, "up"], check=True)


if __name__ == "__main__":
    unittest.main()
