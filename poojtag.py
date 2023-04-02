#!/usr/bin/env python3

# Copyright (c) 2023 Laurent Ghigonis <ooookiwi@gmail.com>
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

VERSION = "v20230402"
DESCRIPTION = "poojtag %s - pure python JTAG over BusPirate-like OCD protocol" % VERSION

""" Data flow overview between poojtag.py classes, BusPirate and the JTAG port

    BP       Instruction   TAP                   OCD          BP     pySerial    BusPirate              JTAG port
-----------------------------------------------------------------------------------------------------------------
--> command  seq           send_seq,set_state    cmd_shift    _sr    write       [applies TDI,TMS]  --> JTAG port
<-- command  dec           recv_seq              decode_tdo   _sr    read        [reads TDO]        <-- JTAG port
"""

import re
import sys
import time
import inspect
import argparse
import logging
import logging
import itertools
from struct import pack, unpack
from datetime import datetime
from binascii import hexlify, unhexlify
from functools import reduce
from textwrap import wrap
from logging import debug, info, warning, error

import serial # pySerial
from bitstring import BitStream, BitArray

LOG = 'poojtag.log'

def bstobl(bs):
    """ bit string to bit list """
    return list(map(lambda b: int(b), [*bs]))

def bstobl_rev(bs):
    """ bit string to bit list """
    return bstobl(bs)[::-1]


class TAP(object):
    """ JTAG Test Access Port """

    # states, values are arbitrary
    STATE_UNKNOWN          = -1
    STATE_TEST_LOGIC_RESET = 0
    STATE_RUN_TEST_IDLE    = 1
    # DR
    STATE_SELECT_DR_SCAN   = 10
    STATE_CAPTURE_DR       = 11
    STATE_SHIFT_DR         = 12
    STATE_EXIT1_DR         = 13
    STATE_PAUSE_DR         = 14
    STATE_EXIT2_DR         = 15
    STATE_UPDATE_DR        = 16
    # IR
    STATE_SELECT_IR_SCAN   = 20
    STATE_CAPTURE_IR       = 21
    STATE_SHIFT_IR         = 22
    STATE_EXIT1_IR         = 23
    STATE_PAUSE_IR         = 24
    STATE_EXIT2_IR         = 25
    STATE_UPDATE_IR        = 26

    STATES = {
        -1: 'STATE_UNKNOWN',
        0: 'STATE_TEST_LOGIC_RESET',
        1: 'STATE_RUN_TEST_IDLE',
        10: 'STATE_SELECT_DR_SCAN',
        11: 'STATE_CAPTURE_DR',
        12: 'STATE_SHIFT_DR',
        13: 'STATE_EXIT1_DR',
        14: 'STATE_PAUSE_DR',
        15: 'STATE_EXIT2_DR',
        16: 'STATE_UPDATE_DR',
        20: 'STATE_SELECT_IR_SCAN',
        21: 'STATE_CAPTURE_IR',
        22: 'STATE_SHIFT_IR',
        23: 'STATE_EXIT1_IR',
        24: 'STATE_PAUSE_IR',
        25: 'STATE_EXIT2_IR',
        26: 'STATE_UPDATE_IR',
    }

    STATES_CHANGE = {
        STATE_UNKNOWN: { 0: STATE_UNKNOWN, 1: STATE_UNKNOWN },
        STATE_TEST_LOGIC_RESET: { 0: STATE_RUN_TEST_IDLE, 1: STATE_TEST_LOGIC_RESET },
        STATE_RUN_TEST_IDLE: { 0: STATE_RUN_TEST_IDLE, 1: STATE_SELECT_DR_SCAN },
        # DR
        STATE_SELECT_DR_SCAN: { 0: STATE_CAPTURE_DR, 1: STATE_SELECT_IR_SCAN },
        STATE_CAPTURE_DR: { 0: STATE_SHIFT_DR, 1: STATE_EXIT1_DR },
        STATE_SHIFT_DR: { 0: STATE_SHIFT_DR, 1: STATE_EXIT1_DR },
        STATE_EXIT1_DR: { 0: STATE_PAUSE_DR, 1: STATE_UPDATE_DR },
        STATE_PAUSE_DR: { 0: STATE_PAUSE_DR, 1: STATE_EXIT2_DR },
        STATE_EXIT2_DR: { 0: STATE_SHIFT_DR, 1: STATE_UPDATE_DR },
        STATE_UPDATE_DR: { 0: STATE_RUN_TEST_IDLE, 1: STATE_SELECT_DR_SCAN },
        # IR
        STATE_SELECT_IR_SCAN: { 0: STATE_CAPTURE_IR, 1: STATE_TEST_LOGIC_RESET },
        STATE_CAPTURE_IR: { 0: STATE_SHIFT_IR, 1: STATE_EXIT1_IR },
        STATE_SHIFT_IR: { 0: STATE_SHIFT_IR, 1: STATE_EXIT1_IR },
        STATE_EXIT1_IR: { 0: STATE_PAUSE_IR, 1: STATE_UPDATE_IR },
        STATE_PAUSE_IR: { 0: STATE_PAUSE_IR, 1: STATE_EXIT2_IR },
        STATE_EXIT2_IR: { 0: STATE_SHIFT_IR, 1: STATE_UPDATE_IR },
        STATE_UPDATE_IR: { 0: STATE_RUN_TEST_IDLE, 1: STATE_SELECT_DR_SCAN },
    }
            

    # instructions
    INST_EXTEST = 0x0
    INST_IDCODE = 0x1
    INST_SAMPLE_PRELOAD = 0x2
    INST_BYPASS = 0xF

    # TDO decoding
    SKIP_TDO = 0
    DECODE_TDO = 1

    def __init__(self):
        self.state = self.STATE_UNKNOWN

    def reset(self):
        if self.state == self.STATE_TEST_LOGIC_RESET:
            return []
        self.state = self.STATE_TEST_LOGIC_RESET
        return [(0, 1, TAP.SKIP_TDO)] * 8 # TDI=0 TMS=1 * 8

    def idle(self):
        s = []
        if self.state == self.STATE_TEST_LOGIC_RESET:
            s += [(0, 0, TAP.SKIP_TDO)]
        elif self.state != self.STATE_RUN_TEST_IDLE:
            debug("TAP.idle: we are reseting from state %s" % self.STATES[self.state])
            s += self.reset() + [(0, 0, TAP.SKIP_TDO)]
        self.state = TAP.STATE_RUN_TEST_IDLE
        return s

    def set_state(self, state):
        seq = self.idle()
        if state == self.STATE_SHIFT_DR:
            seq += zip([0]*3, [1, 0, 0], [TAP.SKIP_TDO]*3)
            self.state = self.STATE_SHIFT_DR
        elif state == self.STATE_SHIFT_IR:
            seq += zip([0]*4, [1, 1, 0, 0], [TAP.SKIP_TDO]*4)
            self.state = self.STATE_SHIFT_IR
        else:
            raise Exception("unsupported state %d" % state)
        return seq

    def shift_ir(self, istr):
        # prepare to shift instruction
        s = self.set_state(TAP.STATE_SHIFT_IR)
        # shift instruction and go back to STATE_RUN_TEST_IDLE:
        # TDI: reverse instruction and add 0,0
        # TMS: [0, 0, 0] to shift 3 LSB bits, [1] to shift MSB and set STATE_EXIT1_DR, and [1, 0] for STATE_RUN_TEST_IDLE
        # decode TDO: 6 & SKIP_TDO
        s += list(zip(bstobl_rev("{:04b}".format(istr))+[0,0], bstobl('000110'), [TAP.SKIP_TDO]*6))
        debug("shift_ir: %s" % s)
        self.state = TAP.STATE_RUN_TEST_IDLE
        return s

    def shift_dr(self, count, tdi=0, decode_tdo=False):
        """ shift Data Register and provide a fixed value for TDI """
        return self.shift_dr_tdi([tdi]*count, decode_tdo=decode_tdo)

    def shift_dr_int(self, value, size, decode_tdo_int=0):
        """ shift Data Register for {size} bits of value {value} """
        tdi = bstobl_rev('{:0{size}b}'.format(value, size=size))
        decode_tdo = bstobl_rev('{:0{size}b}'.format(decode_tdo_int, size=size))
        return self.shift_dr_tdi(tdi, decode_tdo=decode_tdo)

    def shift_dr_int_list(self, intlist, size, decode_tdo_intlist=False):
        s = list()
        if type(decode_tdo_intlist) is not list:
            decode_tdo_intlist = [decode_tdo_intlist] * len(intlist)
        for i, i_dec  in zip(intlist, decode_tdo_intlist):
            s += self.shift_dr_int(i, size, decode_tdo_int=i_dec)
        return s

    def shift_dr_tdi(self, tdi, decode_tdo=False):
        """ shift Data Register and provide a TDI sequence """
        count = len(tdi)
        if type(decode_tdo) is list:
            if len(decode_tdo) != len(tdi):
                raise Exception("invalid decode_tdo len %d vs %d for tdi" % (len(decode_tdo), len(tdi)))
        else:
            decode_tdo = [1]*count if decode_tdo else [0]*count

        s = self.set_state(TAP.STATE_SHIFT_DR)
        s += list(zip(tdi[:-1], [0]*(count-1), decode_tdo[:-1]))
        s += [(tdi[-1], 1, decode_tdo[-1])] # last bit is recorded when exiting to STATE_EXIT1_DR
        s += [(0, 1, TAP.SKIP_TDO)] # STATE_UPDATE_DR
        s += [(0, 0, TAP.SKIP_TDO)] # STATE_RUN_TEST_IDLE
        self.state = TAP.STATE_RUN_TEST_IDLE
        return s

    def send_seq(self, seq):
        debug("send_seq len        %d" % len(seq))
        debug("send_seq TDI        %s" % [s[0] for s in seq])
        debug("send_seq TMS        %s" % [s[1] for s in seq])
        debug("send_seq decode_TDO %s" % [s[2] for s in seq])
        self.last_send = seq
        # return only TDI and TDO components
        return list(map(lambda e: e[0:2], seq))

    def recv_seq(self, tdo):
        out_tdo = list()
        if len(self.last_send) != len(tdo):
            error("len(last_send)=%d != len(tdo)=%d" % (len(self.last_send), len(tdo)))
            exit(1)
        debug("recv_seq: decode_tdo=%s" % list(map(lambda s: str(s[2]), self.last_send)))
        debug("recv_seq:        tdo=%s [%d]" % (tdo, len(tdo)))
        for [tdi, tms, decode_tdo], tdo in zip(self.last_send, tdo):
            if decode_tdo:
                out_tdo.append(tdo)
        debug("recv_seq:    out_tdo=%s [%d]" % (out_tdo, len(out_tdo)))
        if len(out_tdo) == 0:
            return BitStream()
        return BitStream('0b'+''.join(out_tdo))

    @classmethod
    def sim_seq(cls, seq, state):
        """ simulate the state and register changes when running a TDI,TMS,decode_tdo sequence """
        info("dump sequence [%d] : %s" % (len(seq), seq))
        info("tdi = %s" % ([e[0] for e in seq]))
        info("tms = %s" % ([e[1] for e in seq]))
        ir = ''
        dr = ''
        state_unknown_count = 0
        for idx, (tdi, tms, decode_tdo) in enumerate(seq, 1):
            extra = ''
            if state == TAP.STATE_UNKNOWN:
                state_unknown_count += 1
                if state_unknown_count == 8:
                    state = TAP.STATE_TEST_LOGIC_RESET
                    state_unknown_count = 0
            if state == TAP.STATE_SHIFT_DR:
                dr = str(tdi) + dr
            elif state == TAP.STATE_EXIT1_DR:
                extra = 'dr=%s [%d] 0x%x' % (dr, len(dr), int(dr, 2))
                dr = ''
            elif state == TAP.STATE_SHIFT_IR:
                ir = str(tdi) + ir
            elif state == TAP.STATE_EXIT1_IR:
                extra = 'ir=%s [%d] 0x%x' % (ir, len(ir), int(ir, 2))
                ir = ''
            info("%02d/%02d %22s  tdi=%s tms=%s decode_tdo=%s %s" % (idx, len(seq), TAP.STATES[state], tdi, tms, decode_tdo, extra))
            state = TAP.STATES_CHANGE[state][tms]
        info("end       %22s" % (TAP.STATES[state]))
        return state

class TAP_ATmega1248P(TAP):
    """ JTAG Test Access Port - specifics of Atmel AVR ATmega1284P, from ATmega1284P Datasheet 8059D–AVR–11/09 """

    # 25.1 Program And Data Memory Lock Bits
    LOCK_BITS = [ ("LB1", "lock Flash/EEPROM read"), ("LB2", "lock Flash/EEPROM write"), ("BLB01", ""), ("BLB02", ""), ("BLB11", ""), ("BLB12", ""), ("", "1"), ("", "1") ]
    # 25.2 Fuse Bits
    EXTENDED_FUSE_BYTE = [ ("BODLEVEL0", ""), ("BODLEVEL1", ""), ("BODLEVEL2", ""), ("", "1"), ("", "1"), ("", "1"), ("", "1"), ("", "1") ]
    FUSE_HIGH_BYTE = [ ("BOOTRST", ""), ("BOOTSZ0", ""), ("BOOTSZ1", ""), ("EESAVE", ""), ("WDTON", ""), ("SPIEN", ""), ("JTAGEN", ""), ("OCDEN", "") ]
    FUSE_LOW_BYTE = [ ("CKSEL0", ""), ("CKSEL1", ""), ("CKSEL2", ""), ("CKSEL3", ""), ("SUT0", ""), ("SUT1", ""), ("CKOUT", ""), ("CKDIV8", "") ]

    # 25.10.2 AVR_RESET (0xC)
    INST_AVR_RESET = 0xC
    # 25.10.3 PROG_ENABLE (0x4)
    INST_PROG_ENABLE = 0x4
    # 25.10.4 PROG_COMMANDS (0x5)
    INST_PROG_COMMANDS = 0x5
    # 25.10.5 PROG_PAGELOAD (0x6)
    INST_PROG_PAGELOAD = 0x6
    # 25.10.6 PROG_PAGEREAD (0x7)
    INST_PROG_PAGEREAD = 0x7

    REG_PROG_CMD_SIZE = 15
    REG_PROG_ENABLE_SIZE = 16

    # 25.10.9 Programming Enable Register
    PROGRAMMING_ENABLE_SIGNATURE = 0b1010001101110000

    # 25.10.10 Programming Command Register
    # from Table 25-18. JTAG Programming Instruction
    # TDI Sequence, TDO Sequence
    # Set a = address high bits, b = address low bits, c = address extended bits, H = 0 - Low byte, 1 - High Byte, o = data out, i = data in, x = don’t care
    # 1a. Chip Erase
    PINST_1A_CHIP_ERASE = [ 0b010001110000000,  0b011000110000000, 0b011001110000000, 0b011001110000000 ]
    # 1b. Poll for Chip Erase Complete
    # 0110011_10000000 xxxxxox_xxxxxxxx
    PINST_1B_POLL_FOR_CHIP_ERASE_COMPLETE = 0b011001110000000
    # 2a. Enter Flash Write
    # 0100011_00010000 xxxxxxx_xxxxxxxx
    PINST_2A_ENTER_FLASH_WRITE = 0b010001100010000
    # 2b. Load Address Extended High Byte (10)
    # 0001011_cccccccc xxxxxxx_xxxxxxxx
    PINST_2B_LOAD_ADDRESS_EXTENDED_HIGH_BYTE_PREFIX = 0b0001011
    # 2c. Load Address High Byte
    # 0000111_aaaaaaaa xxxxxxx_xxxxxxxx
    PINST_2C_LOAD_ADDRESS_HIGH_BYTE_PREFIX = 0b0000111
    # 2d. Load Address Low Byte
    # 0000011_bbbbbbbb xxxxxxx_xxxxxxxx
    PINST_2D_LOAD_ADDRESS_LOW_BYTE_PREFIX = 0b0000011
    # 2e. Load Data Low Byte
    # 0010011_iiiiiiii xxxxxxx_xxxxxxxx
    PINST_2E_LOAD_DATA_LOW_BYTE_PREFIX = 0b0010011
    # 2f. Load Data High Byte
    # 0010111_iiiiiiii xxxxxxx_xxxxxxxx
    PINST_2F_LOAD_DATA_HIGH_BYTE_PREFIX = 0b0010111
    # 2g. Latch Data (1)
    # 0110111_00000000 xxxxxxx_xxxxxxxx
    # 1110111_00000000 xxxxxxx_xxxxxxxx
    # 0110111_00000000 xxxxxxx_xxxxxxxx
    PINST_2G_LATCH_DATA = [ 0b011011100000000, 0b111011100000000, 0b011011100000000 ]
    # 2h. Write Flash Page (1)
    # 0110111_00000000 xxxxxxx_xxxxxxxx
    # 0110101_00000000 xxxxxxx_xxxxxxxx
    # 0110111_00000000 xxxxxxx_xxxxxxxx
    # 0110111_00000000 xxxxxxx_xxxxxxxx
    PINST_2H_WRITE_FLASH_PAGE = [ 0b011011100000000, 0b011010100000000, 0b011011100000000, 0b011011100000000 ]
    # 2i. Poll for Page Write Complete (2)
    # 0110111_00000000 xxxxxox_xxxxxxxx
    PINST_2I_POLL_FOR_PAGE_WRITE_COMPLETE = 0b011011100000000
    # 3a. Enter Flash Read
    # 0100011_00000010 xxxxxxx_xxxxxxxx
    PINST_3A_ENTER_FLASH_READ = 0b010001100000010
    # 3b. Load Address Extended High Byte
    # 0001011_cccccccc xxxxxxx_xxxxxxxx
    PINST_3B_LOAD_ADDRESS_EXTENDID_HIGH_BYTE_PREFIX = 0b0001011
    # 3c. Load Address High Byte
    # 0000111_aaaaaaaa xxxxxxx_xxxxxxxx
    PINST_3C_LOAD_ADDRESS_HIGH_BYTE_PREFIX = 0b0000111
    # 3d. Load Address Low Byte
    # 0000011_bbbbbbbb xxxxxxx_xxxxxxxx
    PINST_3D_LOAD_ADDRESS_LOW_BYTE_PREFIX = 0b0000011
    # 3e. Read Data Low and High Byte
    # 0110010_00000000 xxxxxxx_xxxxxxxx 
    # 0110110_00000000 xxxxxxx_oooooooo Low byte 
    # 0110111_00000000 xxxxxxx_oooooooo High byte
    PINST_3E_READ_DATA_LOW_AND_HIGH_BYTE = [ 0b011011000000000, 0b011011000000000, 0b011011100000000 ]
    PINST_3E_READ_DATA_LOW_AND_HIGH_BYTE_DECODE_MASK = [ 0b000000000000000, 0b000000011111111, 0b000000011111111 ]
    # 8a. Enter Fuse/Lock Bit Read
    # 0100011_00000100 xxxxxxx_xxxxxxxx
    PINST_8A_ENTER_FUSE_LOCK_BIT_READ = 0b010001100000100
    # 8b. Read Extended Fuse Byte(6)
    # 0111010_00000000 xxxxxxx_xxxxxxxx
    # 0111011_00000000 xxxxxxx_oooooooo
    PINST_8B_READ_EXTENDED_FUSE_BYTE = [ 0b011101000000000, 0b011101100000000 ]
    PINST_8B_READ_EXTENDED_FUSE_BYTE_DECODE_MASK = [ 0b000000000000000, 0b000000011111111 ]
    # 8c. Read Fuse High Byte(7)
    # 0111110_00000000 xxxxxxx_xxxxxxxx
    # 0111111_00000000 xxxxxxx_oooooooo
    PINST_8C_READ_FUSE_HIGH_BYTE = [ 0b011111000000000, 0b011111100000000 ]
    PINST_8C_READ_FUSE_HIGH_BYTE_DECODE_MASK = [ 0b000000000000000, 0b000000011111111 ]
    # 8d. Read Fuse Low Byte(8)
    # 0110010_00000000 xxxxxxx_xxxxxxxx
    # 0110011_00000000 xxxxxxx_oooooooo
    PINST_8D_READ_FUSE_LOW_BYTE = [ 0b011001000000000, 0b011001100000000 ]
    PINST_8D_READ_FUSE_LOW_BYTE_DECODE_MASK = [ 0b000000000000000, 0b000000011111111 ]
    # 8e. Read Lock Bits(9)
    # 0110110_00000000 xxxxxxx_xxxxxxxx (5)
    # 0110111_00000000 xxxxxxx_xxoooooo
    PINST_8E_READ_LOCK_BITS = [ 0b011011000000000, 0b011011100000000 ]
    PINST_8E_READ_LOCKS_BITS_DECODE_MASK = [ 0b000000000000000, 0b000000011111111 ]
    # 8f. Read Fuses and Lock Bits
    # 0111010_00000000 xxxxxxx_xxxxxxxx (5)
    # 0111110_00000000 xxxxxxx_oooooooo Fuse Ext. byte
    # 0110010_00000000 xxxxxxx_oooooooo Fuse High byte
    # 0110110_00000000 xxxxxxx_oooooooo Fuse Low byte
    # 0110111_00000000 xxxxxxx_oooooooo Lock bits
    PINST_8F_READ_FUSES_AND_LOCK_BITS = [ 0b011101000000000, 0b011111000000000, 0b011001000000000, 0b011011000000000, 0b011011100000000 ]
    PINST_8F_READ_FUSES_AND_LOCK_BITS_DECODE_MASK = [ 0b000000000000000, 0b000000011111111, 0b000000011111111, 0b000000011111111, 0b000000011111111 ]
    # 9a. Enter Signature Byte Read
    # 0100011_00001000 xxxxxxx_xxxxxxxx
    PINST_9A_ENTER_SIGNATURE_BYTE_READ = 0b010001100001000
    # 9b. Load Address Byte
    # 0000011_bbbbbbbb xxxxxxx_xxxxxxxx
    PINST_9B_LOAD_ADDRESS_BYTE_PREFIX = 0b0000011
    # 9c. Read Signature Byte 
    # 0110010_00000000 xxxxxxx_xxxxxxxx 
    # 0110011_00000000 xxxxxxx_oooooooo
    PINST_9C_READ_SIGNATURE_BYTE = [ 0b011001000000000, 0b011001100000000 ]
    PINST_9C_READ_SIGNATURE_BYTE_DECODE_MASK = [ 0b000000000000000, 0b000000011111111 ]
    # 11a. Load No Operation Command
    # 0100011_00000000 xxxxxxx_xxxxxxxx 
    # 0110011_00000000 xxxxxxx_xxxxxxxx
    PINST_11A_LOAD_NO_OP = [ 0b010001100000000, 0b011001100000000 ]

    def prog_mode(self):
        pass

class TAP_AVR_Private(TAP):
    """ JTAG Test Access Port - AVR Private commands, credits to Free AVR ICE for documenting them incompletely
    ressources:
    * http://cvs.savannah.nongnu.org/viewvc/freeice/freeice/gipper/firmware/jtag_avr_ocd.c?revision=1.3&view=markup
    * https://download-mirror.savannah.gnu.org/releases/freeice/AVR-OCD-Documentation.html
    * https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2009/jgs33_rrw32/Final%20Paper/Documentation.html
    * https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2009/jgs33_rrw32/Final%20Paper/debugger.c """

    # On-Chip Debug Specific JTAG Instructions
    # OCD Force Break (0x8)
    INST_OCD_FORCE_BREAK = 0x8
    # OCD Run (0x9)
    INST_OCD_RUN = 0x9
    # OCD Execute AVR Instruction (0xA)
    # instructions can be 16bit or 32bit long
    # use 0x0A, SDR 0xFFFF0000 to read PC on TDO (actually returns PC+2 or PC+4)
    INST_OCD_EXEC = 0xA
    # OCD Access OCD Registers (0xB)
    # there are total 16 Addressable Registers
    # after IR next DRSHIFT is RW Flag (1=Write) + 4 Bits Address
    # those Data in Instruction is 21 (5 + 16) bits
    # note for read operation OCD Address need to be pre latched!
    INST_OCD_REGISTERS = 0xB

    OPCODE_READ_PC_JTAG = 0xFFFF0000
    OPCODE_READ_PC_JTAG_LEN = 32

    BREAK_RUN = 1
    BREAK_STOP = 0

    # OCD Registers
    OCDR_FLAG_READ = 0x0
    OCDR_FLAG_WRITE = 0x1
    OCDR_FLAG_LEN = 1
    OCDR_ADDRESS_LEN = 4
    #OCDR_DATA_LEN = 16
    OCDR_DATA_LEN = 21
    # Register 0 PSB0
    OCDR_0_PSB0 = 0x00
    # Register 1 PSB1
    OCDR_1_PSB1 = 0x01
    # Register 2 PDMSB
    OCDR_2_PDMSB = 0x02
    # Register 3 PDSB
    OCDR_3_PDSB = 0x03
    # Register 8 Break Control Register (BCR)
    OCDR_8_BCR = 0x08
    # Register 9 - Break Status Register (BSR)
    OCDR_9_BSR = 0x09
    # Register C - OCDR Readback
    #   Bit      rw     Description 
    #   D15-8    rw     OCDR 7..0 
    #   D7-0     r      unused (read as 0)
    OCDR_C_READBACK = 0x0C
    # Register D - Control and Status Register
    #   Bit      rw 	Description
    #   D15      rw 	1=Enable OCDR
    #   D14      rw 	1=?
    #   D13-D5   r
    #   D4       r 	    1=OCDR written by AVR and not read by OCD
    #   D3       r      1=Reset not active
    #   D2       r      1=Reset not active
    #   D1-0     r
    OCDR_D_CONTROL_STATUS = 0x0D

class Instruction(object):
    """ implements the logic to execute a JTAG instruction and process the result """
    def seq(self, tap):
        info("Instruction has no seq() function")
        return []

    def dec(self, tdo):
        return tdo.bin

class BYPASS(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP.INST_BYPASS) \
            + tap.shift_dr(32, tdi=1, decode_tdo=True)

    def dec(self, tdo):
        return tdo.bin

class SAMPLE_PRELOAD(Instruction):
    def __init__(self, chain_len=64):
        self.chain_len = chain_len

    def seq(self, tap):
        return tap.shift_ir(TAP.INST_SAMPLE_PRELOAD) \
            + tap.shift_dr(self.chain_len, decode_tdo=True)

class IDCODE_implicit(Instruction):
    def seq(self, tap):
        return tap.shift_dr(64, decode_tdo=True)

    def dec(self, tdo):
        res = list()
        while tdo.pos < tdo.len:
            val = bytes()
            for i in range(4):
                bits = tdo.read(min(8, tdo.len-tdo.pos))
                bits.reverse()
                bits.append(8 - bits.len) # fill the byte with zeros
                val += bits.bytes
            res.append(hexlify(val[::-1]))
        return res

class IDCODE(IDCODE_implicit):
    def seq(self, tap):
        return tap.shift_ir(TAP.INST_IDCODE) + IDCODE_implicit.seq(self, tap)

class AVR_Reset(Instruction):
    def __init__(self, tdi=1):
        self.tdi = tdi

    def seq(self, tap):
        return tap.shift_ir(TAP_ATmega1248P.INST_AVR_RESET) \
            + tap.shift_dr(1, tdi=self.tdi, decode_tdo=True)

class AVR_Prog_Enter(Instruction):
    def seq(self, tap):
        s = AVR_Reset(tdi=1).seq(tap)
        s += tap.shift_ir(TAP_ATmega1248P.INST_PROG_ENABLE)
        s += tap.shift_dr_int(TAP_ATmega1248P.PROGRAMMING_ENABLE_SIGNATURE, TAP_ATmega1248P.REG_PROG_ENABLE_SIZE)
        return s

class AVR_Prog_Leave(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP_ATmega1248P.INST_PROG_COMMANDS) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_11A_LOAD_NO_OP, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_ir(TAP_ATmega1248P.INST_PROG_ENABLE) \
            + tap.shift_dr(TAP_ATmega1248P.REG_PROG_ENABLE_SIZE, tdi=0) \
            + AVR_Reset(tdi=0).seq(tap)

class AVR_Read_Signature(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP_ATmega1248P.INST_PROG_COMMANDS) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_9A_ENTER_SIGNATURE_BYTE_READ, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_9B_LOAD_ADDRESS_BYTE_PREFIX<<8 | 0x00, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE, TAP_ATmega1248P.REG_PROG_CMD_SIZE, TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE_DECODE_MASK) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_9B_LOAD_ADDRESS_BYTE_PREFIX<<8 | 0x01, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE, TAP_ATmega1248P.REG_PROG_CMD_SIZE, TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE_DECODE_MASK) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_9B_LOAD_ADDRESS_BYTE_PREFIX<<8 | 0x02, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE, TAP_ATmega1248P.REG_PROG_CMD_SIZE, TAP_ATmega1248P.PINST_9C_READ_SIGNATURE_BYTE_DECODE_MASK)

    def dec(self, tdo):
        return [ "0x%02x" % int(v[::-1], 2) for v in wrap(tdo.bin, 8) ]

class AVR_Read_Fuses_and_Lock_bits(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP_ATmega1248P.INST_PROG_COMMANDS) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_8A_ENTER_FUSE_LOCK_BIT_READ, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_8F_READ_FUSES_AND_LOCK_BITS, TAP_ATmega1248P.REG_PROG_CMD_SIZE, TAP_ATmega1248P.PINST_8F_READ_FUSES_AND_LOCK_BITS_DECODE_MASK)

    def dec(self, tdo):
        b = wrap(tdo.bin, 8)
        s = "1 for disabled, 0 for enabled\n"
        s += "Fuse Ext. byte : %s\n" % b[0]
        for n, (name, desc) in enumerate(TAP_ATmega1248P.EXTENDED_FUSE_BYTE):
            s += "%30s : %s\n" % ("%s (%s)" % (name, desc), b[0][n])
        s += "Fuse High byte : %s\n" % b[1]
        for n, (name, desc) in enumerate(TAP_ATmega1248P.FUSE_HIGH_BYTE):
            s += "%30s : %s\n" % ("%s (%s)" % (name, desc), b[1][n])
        s += "Fuse Low byte  : %s\n" % b[2]
        for n, (name, desc) in enumerate(TAP_ATmega1248P.FUSE_LOW_BYTE):
            s += "%30s : %s\n" % ("%s (%s)" % (name, desc), b[2][n])
        s += "Fuse Lock bits : %s\n" % b[3]
        for n, (name, desc) in enumerate(TAP_ATmega1248P.LOCK_BITS):
            s += "%30s : %s\n" % ("%s (%s)" % (name, desc), b[3][n])
        return s[:-1]

class AVR_Read_Flash_Byte(Instruction):
    def __init__(self, address=0x0):
        self.address = address

    def seq(self, tap):
        return tap.shift_ir(TAP_ATmega1248P.INST_PROG_COMMANDS) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_3A_ENTER_FLASH_READ, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_3B_LOAD_ADDRESS_EXTENDID_HIGH_BYTE_PREFIX<<8 | 0x00, TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_3C_LOAD_ADDRESS_HIGH_BYTE_PREFIX << 8 |  ((self.address >> 8) & 0xff), TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int(TAP_ATmega1248P.PINST_3D_LOAD_ADDRESS_LOW_BYTE_PREFIX  << 8 |  ( self.address       & 0xff), TAP_ATmega1248P.REG_PROG_CMD_SIZE) \
            + tap.shift_dr_int_list(TAP_ATmega1248P.PINST_3E_READ_DATA_LOW_AND_HIGH_BYTE, TAP_ATmega1248P.REG_PROG_CMD_SIZE, TAP_ATmega1248P.PINST_3E_READ_DATA_LOW_AND_HIGH_BYTE_DECODE_MASK)

    def dec(self, tdo):
        return '0x'+tdo.hex

class AVR_OCD_Break(Instruction):
    def __init__(self, run_or_stop=None):
        self.mode = run_or_stop

    def seq(self, tap):
        s = []
        if self.mode is not None:
            s = tap.shift_dr(1, tdi=self.mode, decode_tdo=1) # BREAK_RUN (1) or BREAK_STOP (0)
        return tap.shift_ir(TAP_AVR_Private.INST_OCD_FORCE_BREAK) + s

class AVR_OCD_Run(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP_AVR_Private.INST_OCD_RUN)

class AVR_OCD_Read_Register_Private(Instruction):
    def __init__(self, register=TAP_AVR_Private.OCDR_D_CONTROL_STATUS):
        self.register = register

    def seq(self, tap):
        return tap.shift_ir(TAP_AVR_Private.INST_OCD_REGISTERS) \
            + tap.shift_dr_int(self.register<<1, TAP_AVR_Private.OCDR_FLAG_LEN+TAP_AVR_Private.OCDR_ADDRESS_LEN) \
            + tap.shift_dr_tdi(bstobl('0{:04b}'.format(0xc)+'0'*16), 1)

class AVR_OCD_Read_PC_Private(Instruction):
    def seq(self, tap):
        return tap.shift_ir(TAP_AVR_Private.INST_OCD_EXEC) \
            + tap.shift_dr_int(TAP_AVR_Private.OPCODE_READ_PC_JTAG, TAP_AVR_Private.OPCODE_READ_PC_JTAG_LEN, 0xffffffff)

class Sleep(Instruction):
    def __init__(self, usec=1000):
        warn("when used with a real device, it seems the sleep is breaking pySerial connection")
        self.usec = usec

    def seq(self, tap):
        time.sleep(self.usec/1000)
        return []

class OCD(object):
    """ BusPirate protocol to drive JTAG, used in OpenOCD mode """
    CMD_PORT_MODE = 0x01
    CMD_FEATURE = 0x02
    CMD_TAP_SHIFT = 0x05

    PORT_MODE_HIZ = 0
    PORT_MODE_JTAG = 1
    PORT_MODE_JTAG_OD = 2

    FEATURE_LED = 0x01
    FEATURE_VREG = 0x02
    FEATURE_TRST = 0x04
    FEATURE_SRST = 0x08
    FEATURE_PULLUP = 0x10

    def __init__(self):
        pass

    def enter_bbio(self):
        return b'\x00' * 21

    def enter_ocd(self):
        return b'\x06'

    def cmd_port_mode(self, mode):
        return pack('BB', self.CMD_PORT_MODE, mode)

    def cmd_feature(self, features, opts):
        return pack('BBB', self.CMD_FEATURE, features, opts)

    def cmd_shift(self, sequence):
        """ converts sequence of (tdi, tms) boolean values to a packed form bits_count[1],bits_count[0],[tmi_8bits,tms_8bits,...] """
        def seq2byte(seq):
            x = BitArray(seq)
            x.append(8 - x.len) # fill the byte with zeros
            x.reverse()
            return x.uint
        sequence_len = len(sequence)
        buf = bytearray()
        while len(sequence) > 0:
            seq8 = sequence[:8]
            tdi8 = seq2byte(map(lambda s: s[0], seq8))
            tms8 = seq2byte(map(lambda s: s[1], seq8))
            buf.extend((tdi8, tms8))
            sequence = sequence[8:]
        cmd_buf = pack('BBB', self.CMD_TAP_SHIFT, sequence_len >> 8, sequence_len % 2**8) + buf
        response_len = int(3 + ((len(cmd_buf) - 3) / 2))
        debug("cmd_shift [%d] %s" % (len(cmd_buf), hexlify(cmd_buf, ' ', 1)))
        debug("cmd_shift response_len %d" % (response_len))
        return cmd_buf, response_len

    def decode_tdo(self, tdo):
        """ converts received tdo bytes into sequence of tdo boolean values """
        debug("decode_tdo [%d] %s" % (len(tdo), hexlify(tdo, ' ', 1)))
        bitlen = unpack('H', tdo[1:3][::-1])[0]
        bits = [ [ bit for bit in '{:08b}'.format(byte)[::-1] ] for byte in tdo[3:] ]
        sequence = list(itertools.chain.from_iterable(bits))[:bitlen]
        debug("decode_tdo [%d] %s" % (len(sequence), sequence))
        return sequence

class BP(object):
    """ Communication with BusPirate """
    SPEED = 115200

    def __init__(self, device):
        self.ser = serial.Serial(device, self.SPEED)
        self.ocd = OCD()
        self.tap = TAP()

        # open binary mode then OpenOCD mode
        self.ser.write(self.ocd.enter_bbio())
        self.ser.write(self.ocd.enter_ocd())
        while True:
            ret = self.ser.read(4)
            if ret == b'BBIO':
                ret = self.ser.read(1)
                if ret != b'1':
                    error("invalid answer from BusPirate: BBIO version 0x%s" % hexlify(ret))
                    sys.exit(1)
            elif ret == b'OCD1':
                debug("BP OCD mode confirmed")
                break
            else:
                error("invalid answer from BusPirate: %s" % ret)
                sys.exit(1)

        # setup port mode
        self.ser.write(self.ocd.cmd_port_mode(OCD.PORT_MODE_JTAG))
        self.ser.write(self.ocd.cmd_feature(OCD.FEATURE_PULLUP, 0x0))

        # set the tap in STATE_RUN_TEST_IDLE
        data, reslen = self.ocd.cmd_shift(self.tap.send_seq(self.tap.idle()))
        self._sr(data, reslen)

        info("BP initialized")

    def close(self):
        debug("BP exiting")

    def _sr(self, data, read_count=0):
        if data:
            self.ser.write(data)
        if read_count > 0:
            res = self.ser.read(read_count)
            return res

    def instruction(self, istr):
        data, reslen = self.ocd.cmd_shift(self.tap.send_seq(istr.seq(self.tap)))
        recv = self._sr(data, reslen)
        return istr.dec(bp.tap.recv_seq(self.ocd.decode_tdo(recv)))

INSTRUCTIONS = { m[0]: m[1] for m in inspect.getmembers(sys.modules[__name__],
        lambda member: inspect.isclass(member) and (member.__base__ == Instruction or member.__base__.__base__ == Instruction)) }
ACTIONS = [ 'interactive' ] + list(INSTRUCTIONS.keys())
EPILOG = "Instructions:\n"
for name, istr in INSTRUCTIONS.items():
    extra = ""
    if '__code__' in dir(istr.__init__) and istr.__init__.__code__.co_argcount > 1:
        istr_args = [ "%s=%s" % (arg[0], arg[1]) for arg in zip(istr.__init__.__code__.co_varnames[1:], istr.__init__.__defaults__) ]
        extra = "[:%s]" % ','.join(istr_args)
    EPILOG += "%s%s\n" % (name, extra)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=DESCRIPTION, epilog=EPILOG, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('device', help='Serial port to BusPirate-compatible device eg. /dev/ttyUSB0, or \'pretend\' to only display the JTAG sequence')
    parser.add_argument('-d', '--debug', action='store_true', help='show debug output')
    parser.add_argument('actions', action='store', nargs='+', metavar='{interactive,<instruction>[:arg]}')

    # parse arguments
    args = parser.parse_args()
    actions = list()
    for action in args.actions:
        reg = re.match(r"(?P<action>[A-Za-z-_]*):?(?P<arg>[x0-9A-Fa-f,]*)?", action).groupdict()
        if reg['action'] not in ACTIONS:
            parser.error("invalid action %s" % action)
        if action != reg['action'] and not reg['arg']:
            parser.error("invalid parameter for action %s" % action)
        actions.append((reg['action'], [ int(a, base=0) for a in reg['arg'].split(',')] if reg['arg'] else []))

    # initialize logging to file and stdout
    class Formatter(logging.Formatter):
        def format(self, record):
            if record.levelno == logging.INFO:
                self._style._fmt = "%(message)s"
            else:
                self._style._fmt = "%(levelname)s %(module)s: %(message)s"
            return super().format(record)
    handler_file = logging.FileHandler(LOG, mode='a')
    handler_file.setFormatter(Formatter())
    handler_console = logging.StreamHandler(sys.stdout)
    handler_console.setFormatter(Formatter())
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO, handlers=[handler_file, handler_console])

    info("starting at %s, logging to %s, using device '%s', actions: %s" % (datetime.now(), LOG, args.device, args.actions))

    # initialize the device
    if args.device == 'pretend':
        tap = TAP()
        state = TAP.sim_seq(tap.idle(), TAP.STATE_UNKNOWN)
    else:
        bp = BP(args.device)

    # run actions
    for action, action_arg in actions:
        if action == 'interactive':
            import IPython; from IPython import embed; embed()
        elif action in INSTRUCTIONS.keys():
            if args.device == 'pretend':
                istr = INSTRUCTIONS[action](*action_arg)
                state = TAP.sim_seq(istr.seq(tap), state)
            else:
                info("%s%s: %s" % (action, action_arg, bp.instruction(INSTRUCTIONS[action](*action_arg))))

    # close device
    if args.device != 'pretend':
        bp.close()

    info("stopping at %s" % datetime.now())
