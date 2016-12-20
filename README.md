# RFM69OOK_mbed
A general OOK transceiver library for RFM69 for the MBED platform. Everything ported except for interrupt handling.

Code originally based on https://github.com/kobuki/RFM69OOK/ by Kobuki. Code is largely the same with modifications to support MBED instead of Arduino.

Random_Transmit_Example transmits 101010 each time a character is sent via USB UART. This example is based on LPC11U24 board however it should be easily portabel to other MBED devices.
