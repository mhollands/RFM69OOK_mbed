// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
//Modified for MBED platform by Matt Hollands. projects.matthollands.com

#include <RFM69OOK.h>
#include <RFM69OOKregisters.h>
#include "mbed.h"
#include <SPI.h>
volatile char RFM69OOK::_mode;  // current transceiver state
volatile int RFM69OOK::RSSI;    // most accurate RSSI during reception (closest to the reception)
RFM69OOK* RFM69OOK::selfPointer;
SPI spi(p11, p12, p13); // define the SPI interface to the Radio (mosi, miso, sclk)
DigitalOut cs(RF69OOK_SPI_CS);
DigitalInOut irq(RF69OOK_IRQ_PIN);

bool RFM69OOK::initialize()
{
  const char CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_OFF | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, 0x03}, // bitrate: 32768 Hz
    /* 0x04 */ { REG_BITRATELSB, 0xD1},
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_4}, // BW: 10.4 kHz
    /* 0x1B */ { REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000 },
    /* 0x1D */ { REG_OOKFIX, 100 }, // Fixed threshold value (in dB) in the OOK demodulator
    /* 0x29 */ { REG_RSSITHRESH, 140 }, // RSSI threshold in dBm = -(REG_RSSITHRESH / 2)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
    {255, 0}
  };
  cs = DigitalOut(_slaveSelectPin, 1);
  spi.format(8,0); //8 bits per transaction, mode 0 (polarity = 0, phase = 0)
  spi.frequency(1000000); //1Mhz

  for (char i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69OOK_MODE_STANDBY);
    while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

  selfPointer = this;
  return true;
}

// Poll for OOK signal
bool RFM69OOK::poll()
{
  return irq;
}

// Send a 1 or 0 signal in OOK mode
void RFM69OOK::send(bool signal)
{
  irq = signal;
}

// Turn the radio into transmission mode
void RFM69OOK::transmitBegin()
{
  setMode(RF69OOK_MODE_TX);
  //detachInterrupt(_interruptNum); // not needed in TX mode (interrupts not yet supported)
  irq.output(); //set interrupt pin as an output
}

// Turn the radio back to standby
void RFM69OOK::transmitEnd()
{
  irq.input(); //set interrupt pin as an input
  setMode(RF69OOK_MODE_STANDBY);
}

// Turn the radio into OOK listening mode
void RFM69OOK::receiveBegin()
{
  irq.input(); //set interrupt pin as an input
  //attachInterrupt(_interruptNum, RFM69OOK::isr0, CHANGE); // generate interrupts in RX mode (interrupts not yet supported)
  setMode(RF69OOK_MODE_RX);
}

// Turn the radio back to standby
void RFM69OOK::receiveEnd()
{
  setMode(RF69OOK_MODE_STANDBY);
  //detachInterrupt(_interruptNum); // make sure there're no surprises (interrupts not yet supported)
}

// Handle pin change interrupts in OOK mode (interrupts not yet supported)
void RFM69OOK::interruptHandler()
{
  if (userInterrupt != NULL) (*userInterrupt)();
}

// Set a user interrupt for all transfer methods in receive mode
// call with NULL to disable the user interrupt handler (interrupts not yet supported)
void RFM69OOK::attachUserInterrupt(void (*function)())
{
  userInterrupt = function;
}

// return the frequency (in Hz)
uint32_t RFM69OOK::getFrequency()
{
  return RF69OOK_FSTEP * (((uint32_t)readReg(REG_FRFMSB)<<16) + ((uint16_t)readReg(REG_FRFMID)<<8) + readReg(REG_FRFLSB));
}

// Set literal frequency using floating point MHz value
void RFM69OOK::setFrequencyMHz(float f)
{
  setFrequency(f * 1000000);
}

// set the frequency (in Hz)
void RFM69OOK::setFrequency(uint32_t freqHz)
{
  // TODO: p38 hopping sequence may need to be followed in some cases
  freqHz /= RF69OOK_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
}

// Set bitrate
void RFM69OOK::setBitrate(uint32_t bitrate)
{
  bitrate = 32000000 / bitrate; // 32M = XCO freq.
  writeReg(REG_BITRATEMSB, bitrate >> 8);
  writeReg(REG_BITRATELSB, bitrate);
}

// set OOK bandwidth
void RFM69OOK::setBandwidth(uint8_t bw)
{
  writeReg(REG_RXBW, readReg(REG_RXBW) & 0xE0 | bw);
}

// set RSSI threshold
void RFM69OOK::setRSSIThreshold(int8_t rssi)
{
  writeReg(REG_RSSITHRESH, (-rssi << 1));
}

// set OOK fixed threshold
void RFM69OOK::setFixedThreshold(uint8_t threshold)
{
  writeReg(REG_OOKFIX, threshold);
}

// set sensitivity boost in REG_TESTLNA
// see: http://www.sevenwatt.com/main/rfm69-ook-dagc-sensitivity-boost-and-modulation-index
void RFM69OOK::setSensitivityBoost(uint8_t value)
{
  writeReg(REG_TESTLNA, value);
}

void RFM69OOK::setMode(char newMode)
{
    if (newMode == _mode) return;

    switch (newMode) {
        case RF69OOK_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) setHighPowerRegs(true);
            break;
        case RF69OOK_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) setHighPowerRegs(false);
            break;
        case RF69OOK_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69OOK_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69OOK_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default: return;
    }

    // waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    while (_mode == RF69OOK_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

    _mode = newMode;
}

void RFM69OOK::sleep() {
  setMode(RF69OOK_MODE_SLEEP);
}

// set output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
void RFM69OOK::setPowerLevel(char powerLevel)
{
  _powerLevel = powerLevel;
  writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | (_powerLevel > 31 ? 31 : _powerLevel));
}

void RFM69OOK::isr0() { selfPointer->interruptHandler(); }

int8_t RFM69OOK::readRSSI(bool forceTrigger) {
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // Wait for RSSI_Ready
  }
  return -(readReg(REG_RSSIVALUE) >> 1);
}

char RFM69OOK::readReg(char addr)
{
  select();
  spi.write(addr & 0x7F);
  char regval = spi.write(0x00);
  unselect();
  return regval;
}

void RFM69OOK::writeReg(char addr, char value)
{
  select();
  spi.write(addr | 0x80);
  spi.write(value);
  unselect();
}

// Select the transceiver
void RFM69OOK::select() {
  //noInterrupts(); (interrupts not yet supported)
  cs = 0;
}

/// UNselect the transceiver chip
void RFM69OOK::unselect() {
  cs = 1;
  // interrupts(); (interrupts not yet supported)
}

void RFM69OOK::setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

void RFM69OOK::setHighPowerRegs(bool onOff) {
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void RFM69OOK::setCS(PinName newSPISlaveSelect) {
  _slaveSelectPin = newSPISlaveSelect;
  cs = DigitalOut(_slaveSelectPin, 1);
}

// for debugging
void RFM69OOK::readAllRegs(Serial pc)
{
  char regVal;
  for (char regAddr = 1; regAddr <= 0x4F; regAddr++) {
    regVal = readReg(regAddr);
    pc.printf("0x%X", regAddr);
    pc.printf(" - ");
    pc.printf("0x%X", regVal);
    pc.printf(" - ");
    pc.printf("0x%X", regVal);
  }
}

char RFM69OOK::readTemperature(char calFactor)  // returns centigrade
{
  setMode(RF69OOK_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
}                                                            // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69OOK::rcCalibration()
{
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}