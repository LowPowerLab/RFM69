// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
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
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include "RFM69.h"
#include "RFM69registers.h"
#include <SPI.h>

uint8_t RFM69::DATA[RF69_MAX_DATA_LEN+1];
uint8_t RFM69::_mode;        // current transceiver state
uint8_t RFM69::DATALEN;
uint16_t RFM69::SENDERID;
uint16_t RFM69::TARGETID;     // should match _address
uint8_t RFM69::PAYLOADLEN;
uint8_t RFM69::ACK_REQUESTED;
uint8_t RFM69::ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
int16_t RFM69::RSSI;          // most accurate RSSI during reception (closest to the reception)
volatile bool RFM69::_haveData;
RFM69 *RFM69::_instance = nullptr;

RFM69::RFM69(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW_HCW, SPIClass *spi) {
  _instance = this;
  _slaveSelectPin = slaveSelectPin;
  _interruptPin = interruptPin;
  _mode = RF69_MODE_STANDBY;
  _spyMode = false;
  _powerLevel = 31;
  _isRFM69HW = isRFM69HW_HCW;
  _spi = spi;
#if defined(RF69_LISTENMODE_ENABLE)
  _isHighSpeed = true;
  _haveEncryptKey = false;
  uint32_t rxDuration = DEFAULT_LISTEN_RX_US;
  uint32_t idleDuration = DEFAULT_LISTEN_IDLE_US;
  listenModeSetDurations(rxDuration, idleDuration);
#endif
}

bool RFM69::initialize(uint8_t freqBand, uint16_t nodeID, uint8_t networkID) {
  _interruptNum = digitalPinToInterrupt(_interruptPin);
  if (_interruptNum == (uint8_t)NOT_AN_INTERRUPT) return false;
#ifdef RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    _interruptNum = _interruptPin;
#endif
  const uint8_t CONFIG[][2] = {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
    //* 0x31 */ { REG_SYNCVALUE3, 0xAA },
    //* 0x31 */ { REG_SYNCVALUE4, 0xBB },
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_OFF | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);
  if (_spi == nullptr) {
    _spi = &SPI;
  }
  _spi->begin();

#ifdef SPI_HAS_TRANSACTION
  _settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
#endif

  uint32_t start = millis();
  uint8_t timeout = 50;
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  if (millis()-start >= timeout) return false;
  start = millis();
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);
  if (millis()-start >= timeout) return false;

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  encrypt(0);

  setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW (at this point _isRFM69HW may not be explicitly set by constructor and setHighPower() may not have been called yet (ie called after initialize() call)
  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
  if (millis()-start >= timeout) return false;
  attachInterrupt(_interruptNum, RFM69::isr0, RISING);

  _address = nodeID;
  _networkID = networkID;
#if defined(RF69_LISTENMODE_ENABLE)
  selfPointer = this;
  _freqBand = freqBand;
#endif
  return true;
}

uint8_t RFM69::getVersion() {
  return readReg(REG_VERSION);
}

// return the frequency (in Hz)
uint32_t RFM69::getFrequency() {
  return RF69_FSTEP * (((uint32_t) readReg(REG_FRFMSB) << 16) + ((uint16_t) readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void RFM69::setFrequency(uint32_t freqHz) {
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    setMode(RF69_MODE_SYNTH);
  }
  setMode(oldMode);
}

// return the frequency deviation (in Hz)
uint32_t RFM69::getFrequencyDeviation() {
return RF69_FSTEP * ((readReg(REG_FDEVMSB) << 8) | readReg(REG_FDEVLSB));
}

uint32_t RFM69::getBitRate() {
  return RF69_FXOSC / ((readReg(REG_BITRATEMSB) << 8) | readReg(REG_BITRATELSB));
}

void RFM69::setMode(uint8_t newMode) {
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

// put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void RFM69::sleep() {
  setMode(RF69_MODE_SLEEP);
}

// set this node's address
void RFM69::setAddress(uint16_t addr) {
  _address = addr;
  writeReg(REG_NODEADRS, _address); // unused in packet mode
}

uint16_t RFM69::getAddress() {
  return _address;
}

// set this node's network id
void RFM69::setNetwork(uint8_t networkID) {
  _networkID = networkID;
  writeReg(REG_SYNCVALUE2, networkID);
}

uint8_t RFM69::getNetwork() {
  return _networkID;
}

// set user's ISR callback
void RFM69::setIsrCallback(void (*callback)()) { 
  _isrCallback = callback;
}

// Control transmitter output power (this is NOT a dBm value!)
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//   - for RFM69 W/CW the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//   - for RFM69 HW/HCW the range is from 0-22 [-2dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
//   - the HW/HCW 0-24 range is split into 3 REG_PALEVEL parts:
//     -  0-15 = REG_PALEVEL 16-31, ie [-2 to 13dBm] & PA1 only
//     - 16-19 = REG_PALEVEL 26-29, ie [12 to 15dBm] & PA1+PA2
//     - 20-23 = REG_PALEVEL 28-31, ie [17 to 20dBm] & PA1+PA2+HiPower (HiPower is only enabled before going in TX mode, ie by setMode(RF69_MODE_TX)
// The HW/HCW range overlaps are to smooth out transitions between the 3 PA domains, based on actual current/RSSI measurements
// Any changes to this function also demand changes in dependent function setPowerDBm()
void RFM69::setPowerLevel(uint8_t powerLevel) {
  uint8_t PA_SETTING;
  if (_isRFM69HW) {
    if (powerLevel > 23) powerLevel = 23;
    _powerLevel =  powerLevel;

    //now set Pout value & active PAs based on _powerLevel range as outlined in summary above
    if (_powerLevel < 16) {
      powerLevel += 16;
      PA_SETTING = RF_PALEVEL_PA1_ON; // enable PA1 only
    } else {
      if (_powerLevel < 20)
        powerLevel += 10;
      else 
        powerLevel += 8;
      PA_SETTING = RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON; // enable PA1+PA2
    }
    setHighPowerRegs(true); //always call this in case we're crossing power boundaries in TX mode
  } else { //this is a W/CW, register value is the same as _powerLevel
    if (powerLevel > 31) powerLevel = 31;
    _powerLevel =  powerLevel;
    PA_SETTING = RF_PALEVEL_PA0_ON; // enable PA0 only
  }

  //write value to REG_PALEVEL
  writeReg(REG_PALEVEL, PA_SETTING | powerLevel);
}

uint8_t RFM69::getOutputPower() {
  // _RegisterBits(_REG_PA_LEVEL, offset=0, bits=5)
  return readReg(REG_PALEVEL) & 0x1F;
}

// return stored _powerLevel
uint8_t RFM69::getPowerLevel() { return _powerLevel; }

// Set TX Output power in dBm:
// [-18..+13]dBm in RFM69 W/CW
// [ -2..+20]dBm in RFM69 HW/HCW
int8_t RFM69::setPowerDBm(int8_t dBm) {
  if (_isRFM69HW) {
    //fix any out of bounds
    if (dBm < -2) dBm=-2;
    else if (dBm>20) dBm=20;

    //map dBm to _powerLevel according to implementation in setPowerLevel()
    if (dBm < 12) setPowerLevel(2+dBm);
    else if (dBm < 16) setPowerLevel(4+dBm);
    else setPowerLevel(3+dBm);
  } else { //W/CW
    if (dBm < -18) dBm=-18;
    else if (dBm > 13) dBm=13;
    setPowerLevel(18+dBm);
  }
  return dBm;
}

double RFM69::dBm_to_mW(uint8_t dBm) {
  return pow(10, (dBm / 10.0));
}

bool RFM69::canSend() {
  if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI() < CSMA_LIMIT) { // if signal stronger than -100dBm is detected assume channel activity
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69::send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK) {
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS){
      receiveDone();
#ifdef ESP8266
      delay(1); // Give esp8266-based boards to handle background tasks. Seems to work better than yield();
#endif
  }
  sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
bool RFM69::sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
  uint32_t sentTime;
  for (uint8_t i = 0; i <= retries; i++) {
    send(toAddress, buffer, bufferSize, true);
    sentTime = millis();
    while (millis() - sentTime < retryWaitTime) {
      if (ACKReceived(toAddress)) return true;
    }
  }
  return false;
}

// should be polled immediately after sending a packet with ACK request
bool RFM69::ACKReceived(uint16_t fromNodeID) {
  if (receiveDone())
    return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
bool RFM69::ACKRequested() {
  return ACK_REQUESTED && (TARGETID == _address);
}

// should be called immediately after reception in case sender wants ACK
void RFM69::sendACK(const void* buffer, uint8_t bufferSize) {
  ACK_REQUESTED = 0;   // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
  uint16_t sender = SENDERID;
  int16_t _RSSI = RSSI; // save payload received RSSI value
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS){
      receiveDone();
#ifdef ESP8266
      delay(1); // Give esp8266-based boards to handle background tasks. Seems to work better than yield().
#endif
  }
  SENDERID = sender;    // TWS: Restore SenderID after it gets wiped out by receiveDone()
  sendFrame(sender, buffer, bufferSize, false, true);
  RSSI = _RSSI; // restore payload RSSI
}

// internal function
void RFM69::sendFrame(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK) {
  //NOTE: overridden in RFM69_ATC!
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

  if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; // assign last 2 bits of address if > 255
  if (_address > 0xFF) CTLbyte |= (_address & 0x300) >> 8;   // assign last 2 bits of address if > 255

  // write to FIFO
  select();
  _spi->transfer(REG_FIFO | 0x80);
  _spi->transfer(bufferSize + 3);
  _spi->transfer((uint8_t)toAddress);
  _spi->transfer((uint8_t)_address);
  _spi->transfer(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    _spi->transfer(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  //uint32_t txStart = millis();
  //while (digitalRead(_interruptPin) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
  while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for PacketSent
  setMode(RF69_MODE_STANDBY);
}

// internal function - interrupt gets called when a packet is received
void RFM69::interruptHandler() {
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
    setMode(RF69_MODE_STANDBY);
    select();
    _spi->transfer(REG_FIFO & 0x7F);
    PAYLOADLEN = _spi->transfer(0);
    PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
    TARGETID = _spi->transfer(0);
    SENDERID = _spi->transfer(0);
    uint8_t CTLbyte = _spi->transfer(0);
    TARGETID |= (uint16_t(CTLbyte) & 0x0C) << 6; // 10 bit address (most significant 2 bits stored in bits(2,3) of CTL byte
    SENDERID |= (uint16_t(CTLbyte) & 0x03) << 8; // 10 bit address (most sifnigicant 2 bits stored in bits(0,1) of CTL byte

    if(!(_spyMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in spy mode
       || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      PAYLOADLEN = 0;
      unselect();
      receiveBegin();
      return;
    }

    DATALEN = PAYLOADLEN - 3;
    ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
    ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag
    uint8_t _pl = _powerLevel; // interruptHook() can change _powerLevel so remember it
    interruptHook(CTLbyte);    // TWS: hook to derived class interrupt function

    for (uint8_t i = 0; i < DATALEN; i++) DATA[i] = _spi->transfer(0);

    DATA[DATALEN] = 0; // add null at end of string // add null at end of string
    unselect();
    setMode(RF69_MODE_RX);
    if (_pl != _powerLevel) setPowerLevel(_powerLevel); // set new _powerLevel if changed
  }
  RSSI = readRSSI();
}

// internal function
ISR_PREFIX void RFM69::isr0() {
  _haveData = true;
  if (_instance->_isrCallback)
    _instance->_isrCallback();
}

// internal function
void RFM69::receiveBegin() {
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
#if defined(RF69_LISTENMODE_ENABLE)
  RF69_LISTEN_BURST_REMAINING_MS = 0;
#endif
  RSSI = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool RFM69::receiveDone() {
  if (_haveData) {
  	_haveData = false;
  	interruptHandler();
  }
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0) {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  } else if (_mode == RF69_MODE_RX) { // already in RX no payload yet
    return false;
  }
  receiveBegin();
  return false;
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69::encrypt(const char* key) {
#if defined(RF69_LISTENMODE_ENABLE)
  _haveEncryptKey = key;
#endif
  setMode(RF69_MODE_STANDBY);
  uint8_t validKey = key != 0 && strlen(key)!=0;
  if (validKey) {
#if defined(RF69_LISTENMODE_ENABLE)
    memcpy(_encryptKey, key, 16);
#endif
    select();
    _spi->transfer(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      _spi->transfer(key[i]);
    unselect();
  }
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69::readRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger) {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t RFM69::readReg(uint8_t addr) {
  select();
  _spi->transfer(addr & 0x7F);
  uint8_t regval = _spi->transfer(0);
  unselect();
  return regval;
}

void RFM69::writeReg(uint8_t addr, uint8_t value) {
  select();
  _spi->transfer(addr | 0x80);
  _spi->transfer(value);
  unselect();
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void RFM69::select() {
#if defined (SPCR) && defined (SPSR)
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
#endif

#ifdef SPI_HAS_TRANSACTION
  _spi->beginTransaction(_settings);
#else
  // set RFM69 SPI settings explicitly
  _spi->setDataMode(SPI_MODE0);
  _spi->setBitOrder(MSBFIRST);
  #if defined(__STM32F1__)
    _spi->setClockDivider(SPI_CLOCK_DIV8);
  #elif defined(__arm__)
    _spi->setClockDivider(SPI_CLOCK_DIV16);
  #else
    _spi->setClockDivider(SPI_CLOCK_DIV2);
  #endif
#endif
  digitalWrite(_slaveSelectPin, LOW);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void RFM69::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
#ifdef SPI_HAS_TRANSACTION
  _spi->endTransaction();
#endif  
  // restore SPI settings to what they were before talking to RFM69
#if defined (SPCR) && defined (SPSR)
  SPCR = _SPCR;
  SPSR = _SPSR;
#endif
}

// true = disable ID filtering to capture all packets on network, regardless of TARGETID
// false (default) = enable node/broadcast ID filtering to capture only frames sent to this/broadcast address
void RFM69::spyMode(bool onOff) {
  _spyMode = onOff;
  //writeReg(REG_PACKETCONFIG1, (readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

bool RFM69::getSpyMode() {
  return _spyMode;
}

bool RFM69::isSyncOn() {
  return readReg(REG_SYNCCONFIG) >> 7;
}

uint8_t RFM69::getSyncSize() {
  return (readReg(REG_SYNCCONFIG) & 0b00111000) >> 3;
}

bool RFM69::isCrcOn() {
  return (readReg(REG_PACKETCONFIG1) & 0b00010000) >> 4;
}

bool RFM69::isAesOn() {
  return readReg(REG_PACKETCONFIG2) & 0b00000001;
}

// for RFM69 HW/HCW only: you must call setHighPower(true) after initialize() or else transmission won't work
void RFM69::setHighPower(bool _isRFM69HW_HCW) {
  _isRFM69HW = _isRFM69HW_HCW;
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON); // disable OverCurrentProtection for HW/HCW
  setPowerLevel(_powerLevel);
}

bool RFM69::isHighPower() {
 return _isRFM69HW;
}

// internal function - for HW/HCW only:
// enables HiPower for 18-20dBm output
// should only be used with PA1+PA2
void RFM69::setHighPowerRegs(bool enable) {
  if (!_isRFM69HW || _powerLevel<20) enable=false;
  writeReg(REG_TESTPA1, enable ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, enable ? 0x7C : 0x70);
}

// set the slave select (CS) pin 
void RFM69::setCS(uint8_t newSPISlaveSelect) {
  _slaveSelectPin = newSPISlaveSelect;
  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);
}

// set the IRQ pin
bool RFM69::setIrq(uint8_t newIRQPin) {
  uint8_t _newInterruptNum = digitalPinToInterrupt(newIRQPin);
  if (_newInterruptNum == (uint8_t)NOT_AN_INTERRUPT) return false;
#ifdef RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
  _newInterruptNum = newIRQPin;
#endif

  // disconnect from existing IRQ pin
  detachInterrupt( _interruptNum );
  _interruptNum = _newInterruptNum;
  attachInterrupt(_interruptNum, RFM69::isr0, RISING);
  return true;
}

// for debugging
#define REGISTER_DETAIL 0
#if REGISTER_DETAIL
// SERIAL PRINT
// replace Serial.print("string") with SerialPrint("string")
#define SerialPrint(x) SerialPrint_P(PSTR(x))
void SerialWrite(uint8_t c) {
    Serial.write(c);
}

void SerialPrint_P(PGM_P str, void (*f)(uint8_t) = SerialWrite ) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) (*f)(c);
}
#endif

void RFM69::readAllRegs() {
  uint8_t regVal;

#if REGISTER_DETAIL 
  int capVal;

  //... State Variables for intelligent decoding
  uint8_t modeFSK = 0;
  int bitRate = 0;
  int freqDev = 0;
  long freqCenter = 0;
#endif
  
  Serial.println("Address - HEX - BIN");
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++) {
    select();
    _spi->transfer(regAddr & 0x7F); // send address + r/w bit
    regVal = _spi->transfer(0);
    unselect();

    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal, HEX);
    Serial.print(" - ");
    Serial.println(regVal, BIN);

#if REGISTER_DETAIL 
    switch (regAddr){
      case 0x1 : {
        SerialPrint("Controls the automatic Sequencer ( see section 4.2 )\nSequencerOff : ");
        if (0x80 & regVal) {
          SerialPrint("1 -> Mode is forced by the user\n");
        } else {
          SerialPrint("0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n");
        }
        
        SerialPrint("\nEnables Listen mode, should be enabled whilst in Standby mode:\nListenOn : ");
        if (0x40 & regVal) {
          SerialPrint("1 -> On\n");
        } else {
          SerialPrint("0 -> Off ( see section 4.3)\n");
        }
        
        SerialPrint("\nAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n");
        if (0x20 & regVal) {
          SerialPrint("ERROR - ListenAbort should NEVER return 1 this is a write only register\n");
        }
        
        SerialPrint("\nTransceiver's operating modes:\nMode : ");
        capVal = (regVal >> 2) & 0x7;
        if (capVal == 0b000) {
          SerialPrint("000 -> Sleep mode (SLEEP)\n");
        } else if (capVal == 0b001) {
          SerialPrint("001 -> Standby mode (STDBY)\n");
        } else if (capVal == 0b010) {
          SerialPrint("010 -> Frequency Synthesizer mode (FS)\n");
        } else if (capVal == 0b011) {
          SerialPrint("011 -> Transmitter mode (TX)\n");
        } else if (capVal == 0b100) {
          SerialPrint("100 -> Receiver Mode (RX)\n");
        } else {
          Serial.print(capVal, BIN);
          SerialPrint(" -> RESERVED\n");
        }
        SerialPrint("\n");
        break;
      }
        
      case 0x2 : {
        SerialPrint("Data Processing mode:\nDataMode : ");
        capVal = (regVal >> 5) & 0x3;
        if (capVal == 0b00) {
          SerialPrint("00 -> Packet mode\n");
        } else if (capVal == 0b01) {
          SerialPrint("01 -> reserved\n");
        } else if (capVal == 0b10) {
          SerialPrint("10 -> Continuous mode with bit synchronizer\n");
        } else if (capVal == 0b11) {
          SerialPrint("11 -> Continuous mode without bit synchronizer\n");
        }
        
        SerialPrint("\nModulation scheme:\nModulation Type : ");
        capVal = (regVal >> 3) & 0x3;
        if (capVal == 0b00) {
          SerialPrint("00 -> FSK\n");
          modeFSK = 1;
        } else if (capVal == 0b01) {
          SerialPrint("01 -> OOK\n");
        } else if (capVal == 0b10) {
          SerialPrint("10 -> reserved\n");
        } else if (capVal == 0b11) {
          SerialPrint("11 -> reserved\n");
        }
        
        SerialPrint("\nData shaping: ");
        if (modeFSK) {
          SerialPrint("in FSK:\n");
        } else {
          SerialPrint("in OOK:\n");
        }
        SerialPrint("ModulationShaping : ");
        capVal = regVal & 0x3;
        if (modeFSK) {
          if (capVal == 0b00) {
            SerialPrint("00 -> no shaping\n");
          } else if (capVal == 0b01) {
            SerialPrint("01 -> Gaussian filter, BT = 1.0\n");
          } else if (capVal == 0b10) {
            SerialPrint("10 -> Gaussian filter, BT = 0.5\n");
          } else if (capVal == 0b11) {
            SerialPrint("11 -> Gaussian filter, BT = 0.3\n");
          }
        } else {
          if (capVal == 0b00) {
            SerialPrint("00 -> no shaping\n");
          } else if (capVal == 0b01) {
            SerialPrint("01 -> filtering with f(cutoff) = BR\n");
          } else if (capVal == 0b10) {
            SerialPrint("10 -> filtering with f(cutoff) = 2*BR\n");
          } else if (capVal == 0b11) {
            SerialPrint("ERROR - 11 is reserved\n");
          }
        }
        
        SerialPrint("\n");
        break;
      }
        
      case 0x3 : {
        bitRate = (regVal << 8);
        break;
      }
      
      case 0x4 : {
        bitRate |= regVal;
        SerialPrint("Bit Rate (Chip Rate when Manchester encoding is enabled)\nBitRate : ");
        unsigned long val = 32UL * 1000UL * 1000UL / bitRate;
        Serial.println(val);
        SerialPrint("\n");
        break;
      }
      
      case 0x5 : {
        freqDev = ((regVal & 0x3f) << 8);
        break;
      }
      
      case 0x6 : {
        freqDev |= regVal;
        SerialPrint("Frequency deviation\nFdev : ");
        unsigned long val = RF69_FSTEP * freqDev;
        Serial.println(val);
        SerialPrint("\n");
        break;
      }
      
      case 0x7 : {
        unsigned long tempVal = regVal;
        freqCenter = (tempVal << 16);
        break;
      }
     
      case 0x8 : {
        unsigned long tempVal = regVal;
        freqCenter = freqCenter | (tempVal << 8);
        break;
      }

      case 0x9 : {        
        freqCenter = freqCenter | regVal;
        SerialPrint("RF Carrier frequency\nFRF : ");
        unsigned long val = RF69_FSTEP * freqCenter;
        Serial.println(val);
        SerialPrint("\n");
        break;
      }

      case 0xa : {
        SerialPrint("RC calibration control & status\nRcCalDone : ");
        if (0x40 & regVal) {
            SerialPrint("1 -> RC calibration is over\n");
        } else {
            SerialPrint("0 -> RC calibration is in progress\n");
        }
    
        SerialPrint("\n");
        break;
      }

      case 0xb : {
        SerialPrint("Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\nAfcLowBetaOn : ");
        if (0x20 & regVal) {
            SerialPrint("1 -> Improved AFC routine\n");
        } else {
            SerialPrint("0 -> Standard AFC routine\n");
        }
        SerialPrint("\n");
        break;
      }
      
      case 0xc : {
        SerialPrint("Reserved\n\n");
        break;
      }

      case 0xd : {
        byte val;
        SerialPrint("Resolution of Listen mode Idle time (calibrated RC osc):\nListenResolIdle : ");
        val = regVal >> 6;
        if (val == 0b00) {
          SerialPrint("00 -> reserved\n");
        } else if (val == 0b01) {
          SerialPrint("01 -> 64 us\n");
        } else if (val == 0b10) {
          SerialPrint("10 -> 4.1 ms\n");
        } else if (val == 0b11) {
          SerialPrint("11 -> 262 ms\n");
        }
        
        SerialPrint("\nResolution of Listen mode Rx time (calibrated RC osc):\nListenResolRx : ");
        val = (regVal >> 4) & 0x3;
        if (val == 0b00 ) {
          SerialPrint("00 -> reserved\n");
        } else if (val == 0b01) {
          SerialPrint("01 -> 64 us\n");
        } else if (val == 0b10) {
          SerialPrint("10 -> 4.1 ms\n");
        } else if (val == 0b11) {
          SerialPrint("11 -> 262 ms\n");
        }

        SerialPrint("\nCriteria for packet acceptance in Listen mode:\nListenCriteria : ");
        if (0x8 & regVal) {
          SerialPrint("1 -> signal strength is above RssiThreshold and SyncAddress matched\n");
        } else {
          SerialPrint("0 -> signal strength is above RssiThreshold\n");
        }
        
        SerialPrint("\nAction taken after acceptance of a packet in Listen mode:\nListenEnd : ");
        val = (regVal >> 1) & 0x3;
        if (val == 0b00) {
          SerialPrint("00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n");
        } else if (val == 0b01) {
          SerialPrint("01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n");
        } else if (val == 0b10) {
          SerialPrint("10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n");
        } else if (val == 0b11) {
          SerialPrint("11 -> Reserved\n");
        }

        SerialPrint("\n");
        break;
      }
      
      default : {
      }
    }
#endif
  }
  unselect();
}

void RFM69::readAllRegsCompact() {
  // Print the header row and first register entry
  Serial.println(); Serial.print("     ");
  for (uint8_t reg = 0x00; reg < 0x10; reg++) {
    Serial.print(reg, HEX);
    Serial.print("  ");
  }
  Serial.println();
  Serial.print("00: -- ");

  // Loop over the registers from 0x01 to 0x7F and print their values
  for (uint8_t reg = 0x01; reg < 0x80; reg++) {
    if (reg % 16 == 0) {    // Print the header column entries
      Serial.println();
      Serial.print(reg, HEX);
      Serial.print(": ");
    }

    // Print the actual register values
    uint8_t ret = readReg(reg);
    if (ret < 0x10) Serial.print("0");  // Handle values less than 10
    Serial.print(ret, HEX);
    Serial.print(" ");
  }
}

uint8_t RFM69::readTemperature(uint8_t calFactor) { // returns centigrade
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69::rcCalibration() {
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

//===================================================================================================================
// radio300KBPS() - switch radio to max bitrate
//===================================================================================================================
void RFM69::set300KBPS() {
  writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
  writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
  writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
  writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
  writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
  writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
  writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
  writeReg(0x37, 0b10010000); //DC=WHITENING, CRCAUTOOFF=0
  //                ^^->DC: 00=none, 01=manchester, 10=whitening
}

//=============================================================================
// setLNA() - disable the AGC and set a manual gain to attenuate input signal
// Makes receiver hear a "weaker" signal.
// Use this function to simulate a receiver "distance" from a transmitter
// newReg should be: (see table 26 RegLna 0x18 values)
//  000 - gain set by the internal AGC loop (when bits 
//  001 - G1 = highest gain
//  010 - G2 = highest gain 6 dB
//  011 - G3 = highest gain 12 dB
//  100 - G4 = highest gain 24 dB
//  101 - G5 = highest gain 36 dB
//  110 - G6 = highest gain 48 dB
//  111 - reserved
//=============================================================================
uint8_t RFM69::setLNA(uint8_t newReg) {
  byte oldReg;
  oldReg = readReg(REG_LNA);
  writeReg(REG_LNA, ((newReg & 7) | (oldReg & ~7))); // just control the LNA Gain bits for now
  return oldReg;  // return the original value in case we need to restore it
}

// ListenMode sleep/timer - see ListenModeSleep example for proper usage!
void RFM69::listenModeSleep(uint16_t millisInterval) {
  setMode( RF69_MODE_STANDBY );
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  detachInterrupt( _interruptNum );
  //attachInterrupt( _interruptNum, delayIrq, RISING);
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11);
  writeReg(REG_BITRATEMSB, RF_BITRATEMSB_200000);
  writeReg(REG_BITRATELSB, RF_BITRATELSB_200000);
  writeReg(REG_FDEVMSB, RF_FDEVMSB_100000);
  writeReg(REG_FDEVLSB, RF_FDEVLSB_100000);
  writeReg(REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0);

  uint8_t idleResol;
  uint32_t divisor;
  uint32_t microInterval = millisInterval * 1000L;

  if(microInterval > 255 * 4100L) {
    idleResol = RF_LISTEN1_RESOL_IDLE_262000;
    divisor = 262000;
  } else if(microInterval > 255 * 64L) {
    idleResol = RF_LISTEN1_RESOL_IDLE_4100;
    divisor = 4100;
  } else {
    idleResol = RF_LISTEN1_RESOL_IDLE_64;
    divisor = 64;
  }

  writeReg(REG_LISTEN1, RF_LISTEN1_RESOL_RX_64 | idleResol | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_10);
  writeReg(REG_LISTEN2, (microInterval + (divisor >> 1 ) ) / divisor);
  writeReg(REG_LISTEN3, 4);
  writeReg(REG_RSSITHRESH, 255);
  writeReg(REG_RXTIMEOUT2, 1);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY );
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY | RF_OPMODE_LISTEN_ON );
  attachInterrupt(_interruptNum, delayIrq, RISING);
  //must call sleep + interrupt handler 3 times here, then endListenModeSleep() - see ListenModeSleep example!
}

//=============================================================================
// endListenModeSleep() - called by listenModeSleep()
//=============================================================================
void RFM69::endListenModeSleep() {
  detachInterrupt( _interruptNum );
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY);
  writeReg(REG_RXTIMEOUT2, 0);
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
}

//=============================================================================
// delayIRQ() - called by listenModeSleep()
//=============================================================================
void RFM69::delayIrq() { return; }

//=============================================================================
//                     ListenMode specific functions  
//=============================================================================
#if defined(RF69_LISTENMODE_ENABLE)
RFM69* RFM69::selfPointer=0;
volatile uint16_t RFM69::RF69_LISTEN_BURST_REMAINING_MS = 0;

//=============================================================================
// reinitRadio() - use base class initialization with saved values
//=============================================================================
bool RFM69::reinitRadio() {
  bool haveEncryptKey = _haveEncryptKey;
  if (!initialize(_freqBand, _address, _networkID)) return false;
  if (haveEncryptKey) encrypt(_encryptKey); // Restore the encryption key if necessary
  if (_isHighSpeed) writeReg(REG_LNA, (readReg(REG_LNA) & ~0x3) | RF_LNA_GAINSELECT_AUTO);
  return true;
}

static uint32_t getUsForResolution(uint8_t resolution) {
  switch (resolution) {
    case RF_LISTEN1_RESOL_RX_64:
    case RF_LISTEN1_RESOL_IDLE_64:
      return 64;
    case RF_LISTEN1_RESOL_RX_4100:
    case RF_LISTEN1_RESOL_IDLE_4100:
      return 4100;
    case RF_LISTEN1_RESOL_RX_262000:
    case RF_LISTEN1_RESOL_IDLE_262000:
      return 262000;
    default:
      // Whoops
      return 0;
  }
}

static uint32_t getCoefForResolution(uint8_t resolution, uint32_t duration) {
  uint32_t resolDuration = getUsForResolution(resolution);
  uint32_t result = duration / resolDuration;

  // If the next-higher coefficient is closer, use that
  if (abs(duration - ((result + 1) * resolDuration)) < abs(duration - (result * resolDuration)))
    return result + 1;

  return result;
}

static bool chooseResolutionAndCoef(uint8_t *resolutions, uint32_t duration, uint8_t& resolOut, uint8_t& coefOut) {
  for (int i = 0; resolutions[i]; i++) {
    uint32_t coef = getCoefForResolution(resolutions[i], duration);
    if (coef <= 255) {
      coefOut = coef;
      resolOut = resolutions[i];
      return true;
    }
  }

  // out of range
  return false;
}

bool RFM69::listenModeSetDurations(uint32_t& rxDuration, uint32_t& idleDuration) {
  uint8_t rxResolutions[] = { RF_LISTEN1_RESOL_RX_64, RF_LISTEN1_RESOL_RX_4100, RF_LISTEN1_RESOL_RX_262000, 0 };
  uint8_t idleResolutions[] = { RF_LISTEN1_RESOL_IDLE_64, RF_LISTEN1_RESOL_IDLE_4100, RF_LISTEN1_RESOL_IDLE_262000, 0 };

  if (!chooseResolutionAndCoef(rxResolutions, rxDuration, _rxListenResolution, _rxListenCoef))
    return false;

  if (!chooseResolutionAndCoef(idleResolutions, idleDuration, _idleListenResolution, _idleListenCoef))
    return false;

  rxDuration = getUsForResolution(_rxListenResolution) * _rxListenCoef;
  idleDuration = getUsForResolution(_idleListenResolution) * _idleListenCoef;
  _listenCycleDurationUs = rxDuration + idleDuration;

  return true;
}

void RFM69::listenModeGetDurations(uint32_t &rxDuration, uint32_t &idleDuration) {
  rxDuration = getUsForResolution(_rxListenResolution) * _rxListenCoef;
  idleDuration = getUsForResolution(_idleListenResolution) * _idleListenCoef;
}

void RFM69::listenModeReset(void) {
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RF69_LISTEN_BURST_REMAINING_MS = 0;
}

//=============================================================================
// irq handler, simply calls listenModeInterruptHandler method so internal methods can be accessed easily
//=============================================================================
ISR_PREFIX void RFM69::listenModeIrq() { selfPointer->listenModeInterruptHandler(); }

//=============================================================================
// listenModeInterruptHandler() - only called by listen irq handler
//=============================================================================
void RFM69::listenModeInterruptHandler(void) {
  if (DATALEN != 0) return;

  listenModeReset();
  noInterrupts();
  select();

  union {                       // union to simplify addressing of long and short parts of time offset
    uint32_t l;
    uint8_t  b[4];
  } burstRemaining;

  burstRemaining.l = 0;

  _spi->transfer(REG_FIFO & 0x7F);
  PAYLOADLEN = _spi->transfer(0);
  PAYLOADLEN = PAYLOADLEN > 64 ? 64 : PAYLOADLEN; // precaution
  TARGETID = _spi->transfer(0);
  if(!(_spyMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in spy mode
     || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this library's extra fields
  {
    listenModeReset();
    goto out;
  }

  // We've read the target, and will read the sender id and two time offset bytes for a total of 4 bytes
  DATALEN = PAYLOADLEN - 4;
  SENDERID = _spi->transfer(0);
  burstRemaining.b[0] =  _spi->transfer(0);  // and get the time remaining
  burstRemaining.b[1] =  _spi->transfer(0);
  RF69_LISTEN_BURST_REMAINING_MS = burstRemaining.l;

  for (uint8_t i = 0; i < DATALEN; i++)
    DATA[i] = _spi->transfer(0);

  if (DATALEN < RF69_MAX_DATA_LEN)
    DATA[DATALEN] = 0; // add null at end of string

out:
  unselect();
  interrupts();
}

//=============================================================================
// listenModeStart() - switch radio to Listen Mode in prep for sleep until burst
//=============================================================================
void RFM69::listenModeStart(void) {
  while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  listenModeReset();

  detachInterrupt(_interruptNum);
  attachInterrupt(_interruptNum, listenModeIrq, RISING);
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
  writeReg(REG_FRFMSB, readReg(REG_FRFMSB) + 1);
  writeReg(REG_FRFLSB, readReg(REG_FRFLSB));      // MUST write to LSB to affect change!

  listenModeApplyHighSpeedSettings();

  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON);
  writeReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
  writeReg(REG_SYNCVALUE1, 0x5A);
  writeReg(REG_SYNCVALUE2, 0x5A);
  writeReg(REG_LISTEN1, _rxListenResolution | _idleListenResolution | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_10);
  writeReg(REG_LISTEN2, _idleListenCoef);
  writeReg(REG_LISTEN3, _rxListenCoef);
  writeReg(REG_RSSITHRESH, 180);
  writeReg(REG_RXTIMEOUT2, 75);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_ON  | RF_OPMODE_STANDBY);
}

//=============================================================================
// listenModeEnd() - exit listen mode and reinit the radio
//=============================================================================
void RFM69::listenModeEnd(void) {
  detachInterrupt(_interruptNum);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY);
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_STANDBY);
  writeReg(REG_RXTIMEOUT2, 0);
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  listenModeReset();
  reinitRadio();
}

void RFM69::listenModeApplyHighSpeedSettings() {
  if (!_isHighSpeed) return;
  writeReg(REG_BITRATEMSB, RF_BITRATEMSB_200000);
  writeReg(REG_BITRATELSB, RF_BITRATELSB_200000);
  writeReg(REG_FDEVMSB, RF_FDEVMSB_100000);
  writeReg(REG_FDEVLSB, RF_FDEVLSB_100000);
  writeReg(REG_RXBW, RF_RXBW_DCCFREQ_000 | RF_RXBW_MANT_20 | RF_RXBW_EXP_0);
  
  // Force LNA to the highest gain
  //writeReg(REG_LNA, (readReg(REG_LNA) << 2) | RF_LNA_GAINSELECT_MAX);
}

//=============================================================================
// sendBurst() - send a burst of packets to a sleeping listening node (or all)
//=============================================================================
void RFM69::listenModeSendBurst(uint8_t targetNode, const void* buffer, uint8_t size) {
  detachInterrupt(_interruptNum);
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON );
  writeReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
  writeReg(REG_SYNCVALUE1, 0x5A);
  writeReg(REG_SYNCVALUE2, 0x5A);
  listenModeApplyHighSpeedSettings();
  writeReg(REG_FRFMSB, readReg(REG_FRFMSB) + 1);
  writeReg(REG_FRFLSB, readReg(REG_FRFLSB));      // MUST write to LSB to affect change!

  union { // union to simplify addressing of long and short parts of time offset
    int32_t l;
    uint8_t b[4];
  } timeRemaining;

  uint16_t cycleDurationMs = _listenCycleDurationUs / 1000;
  timeRemaining.l = cycleDurationMs;

#ifdef RF69_WL_DEBUG
  Serial.print("Sending burst for ");
  Serial.print(cycleDurationMs, DEC);
  Serial.println(" ms");
#endif

  setMode(RF69_MODE_TX);
  uint32_t numSent = 0;
  uint32_t startTime = millis();

  while(timeRemaining.l > 0) {
    noInterrupts();
    // write to FIFO
    select();
    _spi->transfer(REG_FIFO | 0x80);
    _spi->transfer(size + 4);      // two bytes for target and sender node, two bytes for the burst time remaining
    _spi->transfer(targetNode);
    _spi->transfer(_address);

    // We send the burst time remaining with the packet so the receiver knows how long to wait before trying to reply
    _spi->transfer(timeRemaining.b[0]);
    _spi->transfer(timeRemaining.b[1]);

    for (uint8_t i = 0; i < size; i++) {
      _spi->transfer(((uint8_t*) buffer)[i]);
    }

    unselect();
    interrupts();

    while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) != 0x00);  // make sure packet is sent before putting more into the FIFO
    timeRemaining.l = cycleDurationMs - (millis() - startTime);
  }

  setMode(RF69_MODE_STANDBY);
  reinitRadio();
}
#endif