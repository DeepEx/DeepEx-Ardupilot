#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
Check that the VESC / VESS DataFlash messages declared in AP_VESC.cpp can be
decoded, and that a SET_RPM command, its VESC record and the controller
telemetry are mutually consistent.

The message definitions are taken from the C++ source so that this test fails
if the format string and the documented labels drift apart.
"""

import os
import re
import struct
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "modules", "mavlink"))

from pymavlink import DFReader  # noqa: E402

SOURCE = os.path.join(ROOT, "libraries", "AP_VESC", "AP_VESC.cpp")
PROTOCOL = os.path.join(ROOT, "libraries", "AP_VESC", "AP_VESC_Protocol.h")

FMT_TYPE_BYTES = {
    'B': 'B', 'H': 'H', 'I': 'I', 'Q': 'Q', 'b': 'b', 'h': 'h', 'i': 'i', 'q': 'q',
}
FMT_ID = 128


def parse_definitions():
    with open(SOURCE) as f:
        text = f.read()
    defs = {}
    pattern = re.compile(
        r'WriteStreaming\(\s*"(\w+)",\s*"([^"]+)",\s*"([^"]+)",\s*"([^"]+)",\s*"([^"]+)"', re.S)
    for name, labels, units, mults, fmt in pattern.findall(text):
        defs[name] = (labels, units, mults, fmt)
    return defs


def parse_enum(name):
    with open(PROTOCOL) as f:
        text = f.read()
    body = re.search(r'enum (?:class )?%s[^{]*\{(.*?)\};' % name, text, re.S).group(1)
    values = {}
    for key, expr in re.findall(r'(\w+)\s*=\s*([^,/]+?)\s*(?:,|//|$)', body, re.M):
        shift = re.fullmatch(r'1U?\s*<<\s*(\d+)', expr)
        values[key] = (1 << int(shift.group(1))) if shift else int(expr)
    return values


def build_log(defs, records):
    out = bytearray()
    ids = {}
    out += bytes([0xA3, 0x95, FMT_ID])
    out += struct.pack('<BB4s16s64s', FMT_ID, 89, b'FMT', b'BBnNZ', b'Type,Length,Name,Format,Columns')
    for i, (name, (labels, units, mults, fmt)) in enumerate(defs.items()):
        ids[name] = 200 + i
        length = 3 + struct.calcsize('<' + fmt)
        out += bytes([0xA3, 0x95, FMT_ID])
        out += struct.pack('<BB4s16s64s', ids[name], length, name.encode(), fmt.encode(), labels.encode())
    for name, values in records:
        fmt = defs[name][3]
        out += bytes([0xA3, 0x95, ids[name]])
        out += struct.pack('<' + fmt, *values)
    return bytes(out)


def set_rpm_payload(erpm):
    return struct.pack('>i', erpm)


class VESCLogTest(unittest.TestCase):
    def setUp(self):
        self.defs = parse_definitions()

    def test_definitions_consistent(self):
        self.assertIn('VESC', self.defs)
        self.assertIn('VESS', self.defs)
        for name, (labels, units, mults, fmt) in self.defs.items():
            count = len(labels.split(','))
            self.assertEqual(len(units), count, name)
            self.assertEqual(len(mults), count, name)
            self.assertEqual(len(fmt), count, name)
            self.assertTrue(set(fmt) <= set(FMT_TYPE_BYTES), name)
            self.assertLessEqual(len(name), 4)
            self.assertLessEqual(3 + struct.calcsize('<' + fmt), 255)

    def decode(self, records):
        data = build_log(self.defs, records)
        with tempfile.NamedTemporaryFile(suffix='.BIN', delete=False) as f:
            f.write(data)
            path = f.name
        try:
            log = DFReader.DFReader_binary(path, zero_time_base=True)
            try:
                msgs = []
                while True:
                    m = log.recv_match(type=['VESC', 'VESS'])
                    if m is None:
                        break
                    msgs.append(m)
                return msgs
            finally:
                log.close()
        finally:
            os.unlink(path)

    def test_set_rpm_record_telemetry_chain(self):
        reason = parse_enum('BlockReason')
        tx = parse_enum('TxResult')
        armed_ok = 0
        blocked = reason['BLOCK_DISARMED']
        accepted = tx['ACCEPTED']
        rejected = tx['REJECTED']
        # (request, final, block, tx result, telemetry erpm, age)
        scenario = [
            (8500, 8500, armed_ok, accepted, 8420, 12),
            (-8500, -8500, armed_ok, accepted, -8390, 9),
            (8500, 0, blocked, accepted, 40, 15),
            (17000, 0, reason['BLOCK_TX_UNHEALTHY'], rejected, 0, 65535),
        ]
        records = []
        for i, (req, fin, blk, res, rx, age) in enumerate(scenario):
            records.append(('VESC', (1000000 + i * 100000, 2, 2, 1600, req, fin, blk, 3, res, i, rx, age)))
        records.append(('VESS', (2000000, 4, 396, 80)))
        msgs = self.decode(records)
        self.assertEqual([m.get_type() for m in msgs], ['VESC'] * 4 + ['VESS'])
        for m, (req, fin, blk, res, rx, age) in zip(msgs, scenario):
            self.assertEqual(m.Mot, 2)
            self.assertEqual(m.Req, req)
            self.assertEqual(m.Cmd, fin)
            self.assertEqual(m.Blk, blk)
            self.assertEqual(m.Tx, res)
            self.assertEqual(m.Rx, rx)
            self.assertEqual(m.RxAge, age)
            # the logged final command is what is encoded in the SET_RPM frame
            self.assertEqual(struct.unpack('>i', set_rpm_payload(m.Cmd))[0], m.Cmd)
            if m.Blk != 0:
                self.assertEqual(m.Cmd, 0)
            # telemetry follows the command within the test tolerance when not blocked
            if m.Blk == 0:
                self.assertLess(abs(m.Rx - m.Cmd), 0.02 * abs(m.Cmd))
        self.assertEqual(msgs[-1].Rec + msgs[-1].Sup, 400)

    def test_record_size_budget(self):
        labels, units, mults, fmt = self.defs['VESC']
        size = 3 + struct.calcsize('<' + fmt)
        # six motors at the nominal 10 Hz per-motor rate
        self.assertLessEqual(size * 6 * 10, 2500)


if __name__ == '__main__':
    unittest.main()
