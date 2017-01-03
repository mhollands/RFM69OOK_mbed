# RFM69OOK_mbed
A general OOK transceiver library for RFM69 on the MBED platform.

Code originally based on https://github.com/kobuki/RFM69OOK/ by Kobuki. Code is largely the same with modifications to support MBED instead of Arduino.

Random_Transmit_Example transmits 101010 each time a character is sent via USB UART. LED is turned on when serial character is receieved. LED is turned off everytime the RFM69 toggles the data pin in receive mode. This example is based on LPC11U24 board however it should be easily portable to other MBED devices.
