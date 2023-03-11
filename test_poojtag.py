#!/usr/bin/env python3

import logging
import unittest
from binascii import hexlify, unhexlify

from poojtag import OCD, TAP, IDCODE, IDCODE_implicit, AVR_Prog_Enter, AVR_Prog_Leave

class test_OCD(unittest.TestCase):
    def test_cmd_shift_idcode(self):
        logging.basicConfig(level=logging.INFO)
        ocd = OCD()
        tap = TAP()
        istr = IDCODE()
        data, reslen = ocd.cmd_shift(tap.send_seq(istr.seq(tap)))
        self.assertEqual(reslen, 14)
        self.assertEqual(data, unhexlify('05 00 58 00 ff 20 06 00 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 60'.replace(' ', '')))

class test_TAP(unittest.TestCase):
    def test_idcode_send(self):
        logging.basicConfig(level=logging.INFO)
        tap = TAP()
        istr = IDCODE()
        seq = tap.send_seq(istr.seq(tap))
        self.assertEqual(len(seq), 88)
        expected_tdi = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        expected_tms = [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]
        expected_seq = list(zip(expected_tdi, expected_tms))
        self.assertEqual(seq, expected_seq)

    def test_avr_prog_enter_send(self):
        logging.basicConfig(level=logging.DEBUG)
        tap = TAP()
        istr = AVR_Prog_Enter()
        seq = tap.send_seq(istr.seq(tap))
        self.assertEqual(len(seq), 56)
        expected_tdi = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0]
        expected_tms = [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]
        expected_seq = list(zip(expected_tdi, expected_tms))
        self.assertEqual(seq, expected_seq)

    def test_avr_prog_leave_send(self):
        logging.basicConfig(level=logging.DEBUG)
        tap = TAP()
        istr = AVR_Prog_Leave()
        seq = tap.send_seq(istr.seq(tap))
        self.assertEqual(len(seq), 9 + 97)
        expected_tdi = [0, 0, 0, 0, 0, 0, 0, 0, 0] + [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0]
        expected_tms = [1, 1, 1, 1, 1, 1, 1, 1, 0] + [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0]
        expected_seq = list(zip(expected_tdi, expected_tms))
        self.assertEqual(seq, expected_seq)

if __name__ == '__main__':
    unittest.main()
