### poojtag - pure python JTAG over BusPirate-like OCD protocol

poojtag uses python objects to describe JTAG instructions in an attempt to ease experimentation with new instructions, for both encoding of TDI/TMS sequence and TDO decoding.

Instruction implementation can be tested (test_poojtag.py) and loosely simulated (TAP.sim_seq()).

poojtag has a focus on Atmel AVR chips.

### Example

#### Running the IDCODE command

```
$ ./poojtag.py /dev/ttyACM0 IDCODE
Password:
starting at 2023-03-11 03:00:25.000688, logging to poojtag.log, using device '/dev/ttyACM0', actions: ['IDCODE']
BP initialized
IDCODE[]: [b'00000000', b'00000000']
stopping at 2023-03-11 03:00:25.009613
```

#### Pretending to run the IDCODE command

```
$ ./poojtag.py pretend IDCODE
starting at 2023-03-11 03:01:17.326655, logging to poojtag.log, using device 'pretend', actions: ['IDCODE']
dump sequence [9] : [(0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 0, 0)]
tdi = [0, 0, 0, 0, 0, 0, 0, 0, 0]
tms = [1, 1, 1, 1, 1, 1, 1, 1, 0]
01/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
02/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
03/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
04/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
05/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
06/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
07/09          STATE_UNKNOWN  tdi=0 tms=1 decode_tdo=0
08/09 STATE_TEST_LOGIC_RESET  tdi=0 tms=1 decode_tdo=0
09/09 STATE_TEST_LOGIC_RESET  tdi=0 tms=0 decode_tdo=0
end          STATE_RUN_TEST_IDLE
dump sequence [79] : [(0, 1, 0), (0, 1, 0), (0, 0, 0), (0, 0, 0), (1, 0, 0), (0, 0, 0), (0, 0, 0), (0, 1, 0), (0, 1, 0), (0, 0, 0), (0, 1, 0), (0, 0, 0), (0, 0, 0), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1),(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 1, 1), (0, 1, 0), (0, 0, 0)]
tdi = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
tms = [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]
01/79    STATE_RUN_TEST_IDLE  tdi=0 tms=1 decode_tdo=0 
02/79   STATE_SELECT_DR_SCAN  tdi=0 tms=1 decode_tdo=0
03/79   STATE_SELECT_IR_SCAN  tdi=0 tms=0 decode_tdo=0 
04/79       STATE_CAPTURE_IR  tdi=0 tms=0 decode_tdo=0 
05/79         STATE_SHIFT_IR  tdi=1 tms=0 decode_tdo=0 
06/79         STATE_SHIFT_IR  tdi=0 tms=0 decode_tdo=0 
07/79         STATE_SHIFT_IR  tdi=0 tms=0 decode_tdo=0 
08/79         STATE_SHIFT_IR  tdi=0 tms=1 decode_tdo=0 
09/79         STATE_EXIT1_IR  tdi=0 tms=1 decode_tdo=0 ir=0001 [4] 0x1
10/79        STATE_UPDATE_IR  tdi=0 tms=0 decode_tdo=0
11/79    STATE_RUN_TEST_IDLE  tdi=0 tms=1 decode_tdo=0
12/79   STATE_SELECT_DR_SCAN  tdi=0 tms=0 decode_tdo=0 
13/79       STATE_CAPTURE_DR  tdi=0 tms=0 decode_tdo=0
14/79         STATE_SHIFT_DR  tdi=0 tms=0 decode_tdo=1 
15/79         STATE_SHIFT_DR  tdi=0 tms=0 decode_tdo=1 
[...]
75/79         STATE_SHIFT_DR  tdi=0 tms=0 decode_tdo=1 
76/79         STATE_SHIFT_DR  tdi=0 tms=0 decode_tdo=1 
77/79         STATE_SHIFT_DR  tdi=0 tms=1 decode_tdo=1 
78/79         STATE_EXIT1_DR  tdi=0 tms=1 decode_tdo=0 dr=0000000000000000000000000000000000000000000000000000000000000000 [64] 0x0
79/79        STATE_UPDATE_DR  tdi=0 tms=0 decode_tdo=0 
end          STATE_RUN_TEST_IDLE
stopping at 2023-03-11 03:01:17.335403
```

### Compatibility

This python object oriented jtag tool is compatible with any JTAG controller supporting the BusPirate JTAG OCD protocol.

It has been tested on:
* BusPirate
* HydraBus

### Dependencies

* pySerial
* python-bitstring

### Alternatives

* Use urjtag with busblaster or other UUSB to JTAG hardware cable
* Use OpenOCD, that can have less target chip support than urjtag
* Develop some code over https://github.com/eblot/pyftdi and use a compatible FTDI chip for the bridge

### Ressources

#### python libraries

* FTDI device driver written in pure Python
https://github.com/eblot/pyftdi
Suported FTDI devices include:
UART and GPIO bridges
    FT232R (single port, 3Mbps)
    FT230X/FT231X/FT234X (single port, 3Mbps)
UART, GPIO and multi-serial protocols (SPI, I2C, JTAG) bridges
    FT2232C/D (dual port, clock up to 6 MHz)
    FT232H (single port, clock up to 30 MHz)
    FT2232H (dual port, clock up to 30 MHz)
    FT4232H (quad port, clock up to 30 MHz)

* python-bsdl-parser
https://github.com/raczben/python-bsdl-parser
This is a Grako-based parser for IEEE 1149.1 Boundary-Scan Description Language (BSDL) files.

* pyBSDL, 
https://github.com/BCadet/pyBSDL
uses pyftdi and integrates code from python-bsdl-parser
