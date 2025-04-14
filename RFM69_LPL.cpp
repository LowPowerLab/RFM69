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
#include "RFM69_LPL.h"
#include "RFM69_LPL_registers.h"

uint8_t RFM69::DATA[RF69_MAX_DATA_LEN+1];
uint8_t RFM69::_mode;        // current transceiver state
uint8_t RFM69::DATALEN;
uint16_t RFM69::SENDERID;
uint16_t RFM69::TARGETID;     // should match _address
uint8_t RFM69::PAYLOADLEN;
uint8_t RFM69::ACK_REQUESTED;
uint8_t RFM69::ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
int16_t RFM69::RSSI;          // most accurate RSSI during reception (closest to the reception)

bool RFM69::initialize (uint8_t freqBand, uint16_t ID, uint8_t networkID) {
  _address = ID;
  _retrycount = 0;
  
  // 10 MHz, i.e. 30 MHz / 3 (or 4 MHz if clock is still at 12 MHz)
  spi_init();
  
  uint32_t start = millis();
  uint8_t timeout = 50;
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  if (millis()-start >= timeout) return false;
  start = millis();
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);
  if (millis()-start >= timeout) return false;

  const uint8_t configRegs [] = {
    0x01, 0x04, 
    0x02, 0x00,
    0x03, 0x02, // MSB 55555 bits/s
    0x04, 0x40, // LSB 55555 bits/s
    0x05, 0x03, // MSB FDEV 50000
    0x06, 0x33, // LSB FDEV 50000
    0x07, (uint8_t)(freqBand == RF69_868MHZ?0xD9:0x6C), // MSB 433 Mhz
    0x08, (uint8_t)(freqBand == RF69_868MHZ?0x00:0x40), // MID 433 Mhz
    0x09, (uint8_t)(freqBand == RF69_868MHZ?0x00:0x00), // LSB 433 Mhz
    0x19, 0x42, // REG_RXBW (BitRate < 2 * RxBw)
    0x25, 0x40, // DIO0 is the only IRQ we're using
    0x26, 0x07, // DIO5 ClkOut disable for power saving
    0x28, 0x10, // REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN writing to this bit ensures that the FIFO & status flags are reset
    0x29, 0xDC, // REG_RSSITHRESH must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    0x2E, 0x88, // RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0
    0x2F, 0x2D, // attempt to make this compatible with sync1 byte of RFM12B lib
    0x30, networkID,  // networkID
    0x37, 0x90, // RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF
    0x38, 66,   // in variable length mode: the max frame size, not used in TX
    0x3C, 0x8F, // TX on FIFO not empty
    0x3D, 0x10, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    0x6F, 0x30, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    0
  };

  configure(configRegs);
  
  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  encrypt(0);
  
  uint8_t powerLevel = 25; // could be up to 31... -18 + dBm with PA0 = 7dBm
  if (powerLevel>31) powerLevel = 31;
  writeReg(0x11,RF_PALEVEL_PA0_ON | powerLevel); 
  
  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
  if (millis()-start >= timeout)
    return false;
  
  return true;
}

void RFM69::setPins (uint8_t _SSpin, uint8_t _MOSIpin, uint8_t _MISOpin, uint8_t _SCKpin) {
  SSpin = _SSpin;
  MOSIpin = _MOSIpin;
  MISOpin = _MISOpin;
  SCKpin = _SCKpin;
  
  #ifdef __AVR_DB__
  if (MOSIpin==PIN_PA4 && MISOpin==PIN_PA5 && SCKpin==PIN_PA6) {
    SPI_PORT = 0;
  } else if (MOSIpin==PIN_PC0 && MISOpin==PIN_PC1 && SCKpin==PIN_PC2) {
    SPI_PORT = 1;
  }
  #endif
}

void RFM69::setMode (uint8_t newMode) {

  if (newMode == _mode)
    return;

  writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | newMode);
  
  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode

  uint32_t start = millis();
  uint8_t timeout = 100;

  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {
    if (millis()-start >= timeout) {
      // setMode(RF69_MODE_STANDBY);
      return;
    }
  }

  _mode = newMode;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void RFM69::sleep() {
  setMode(RF69_MODE_SLEEP);
}

//set this node's address
void RFM69::setAddress(uint16_t addr)
{
  _address = addr;
  writeReg(REG_NODEADRS, _address); //unused in packet mode
}

bool RFM69::canSend() 
{
  if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI() < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69::send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS){
      receiveDone();
  }
  sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

void RFM69::send_csma(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t custom_csma_limit=0)
{
  if (custom_csma_limit > 0) {
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    uint32_t now = millis();
    while (!canSend() && millis() - now < custom_csma_limit){
        receiveDone();
    }
  }
  sendFrame(toAddress, buffer, bufferSize, false, false);
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
bool RFM69::sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
  uint32_t sentTime;
  for (uint8_t i = 0; i <= retries; i++)
  {
    _retrycount = i;
    send(toAddress, buffer, bufferSize, true);
    sentTime = millis();
    while (millis() - sentTime < retryWaitTime)
    {
      if (ACKReceived(toAddress)) return true;
    }
  }
  return false;
}

uint8_t RFM69::retry_count() {
  return _retrycount;
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
  }
  SENDERID = sender;    // TWS: Restore SenderID after it gets wiped out by receiveDone()
  sendFrame(sender, buffer, bufferSize, false, true);
  RSSI = _RSSI; // restore payload RSSI
}

// internal function
void RFM69::sendFrame(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo

  uint32_t start = millis();
  uint8_t timeout = 100;

  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) { // wait for ModeReady
    if (millis()-start >= timeout) {
      setMode(RF69_MODE_STANDBY);
      return;
    }
  }
  
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

  if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; //assign last 2 bits of address if > 255
  if (_address > 0xFF) CTLbyte |= (_address & 0x300) >> 8;   //assign last 2 bits of address if > 255

  // write to FIFO
  select();
  spi_transfer(REG_FIFO | 0x80);
  spi_transfer(bufferSize + 3);
  spi_transfer((uint8_t)toAddress);
  spi_transfer((uint8_t)_address);
  spi_transfer(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    spi_transfer(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  start = millis();
  while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00) { // wait for packet sent
    if (millis()-start >= timeout) {
      setMode(RF69_MODE_STANDBY);
      return;
    }
  }
  setMode(RF69_MODE_STANDBY);
}

// internal function - interrupt gets called when a packet is received
void RFM69::interruptHandler() {
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    select();
    spi_transfer(REG_FIFO & 0x7F);
    PAYLOADLEN = spi_transfer(0);
    PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
    TARGETID = spi_transfer(0);
    SENDERID = spi_transfer(0);
    uint8_t CTLbyte = spi_transfer(0);
    TARGETID |= (uint16_t(CTLbyte) & 0x0C) << 6; //10 bit address (most significant 2 bits stored in bits(2,3) of CTL byte
    SENDERID |= (uint16_t(CTLbyte) & 0x03) << 8; //10 bit address (most sifnigicant 2 bits stored in bits(0,1) of CTL byte

    if(!(TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in spy mode
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
    // uint8_t _pl = _powerLevel; //interruptHook() can change _powerLevel so remember it
    // interruptHook(CTLbyte);    // TWS: hook to derived class interrupt function

    for (uint8_t i = 0; i < DATALEN; i++) DATA[i] = spi_transfer(0);

    DATA[DATALEN] = 0; // add null at end of string // add null at end of string
    unselect();
    setMode(RF69_MODE_RX);
    // if (_pl != _powerLevel) setPowerLevel(_powerLevel); //set new _powerLevel if changed
    RSSI = readRSSI();
  }
  
}

void RFM69::receiveBegin() {
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RSSI = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  // writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool RFM69::receiveDone() {
  interruptHandler();
  
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    return false;
  }
  receiveBegin();
  return false;
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69::encrypt(const char* key) {

  setMode(RF69_MODE_STANDBY);
  uint8_t validKey = key != 0 && strlen(key)!=0;
  if (validKey)
  {
    select();
    spi_transfer(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      spi_transfer(key[i]);
    unselect();
  }
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69::readRSSI(bool forceTrigger) {
  
  int16_t rssi = 0;
  if (forceTrigger)
  {
    uint32_t start = millis();
    uint8_t timeout = 100;
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00) { // wait for RSSI_Ready
      if (millis()-start >= timeout) {
        // setMode(RF69_MODE_STANDBY);
        return 0;
      }
    }
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t RFM69::readReg(uint8_t addr)
{
  select();
  spi_transfer(addr & 0x7F);
  uint8_t regval = spi_transfer(0);
  unselect();
  return regval;
}

void RFM69::writeReg(uint8_t addr, uint8_t value)
{
  select();
  spi_transfer(addr | 0x80);
  spi_transfer(value);
  unselect();
}

void RFM69::select() {
  #ifdef __AVR_ATmega328P__
  PORTB &= ~(1 << PORTB2); // faster direct port manipulation, may not be necessary
  #endif
  
  #ifdef __AVR_DB__
    #ifdef PIN_PB5 //PIN_PB5 is specific to DB48 and not defined on DB28
    if (SSpin==PIN_PB5) {
    digitalWriteFast(PIN_PB5, 0);
    }
    #endif
    if (SSpin==PIN_PA7) {
    digitalWriteFast(PIN_PA7, 0);
    }
    else if (SSpin==PIN_PC3) {
    digitalWriteFast(PIN_PC3, 0);
    }
  #endif

  #ifdef __AVR_ATtinyX4__
    if (SSpin==PIN_PB1) {
    digitalWrite(PIN_PB1, 0);
    }
  #endif
}

void RFM69::unselect() {
  #ifdef __AVR_ATmega328P__
  PORTB |= (1 << PORTB2); // faster direct port manipulation, may not be necessary
  #endif
  
  #ifdef __AVR_DB__
    #ifdef PIN_PB5  //PIN_PB5 is specific to DB48 and not defined on DB28
    if (SSpin==PIN_PB5) {
      digitalWriteFast(PIN_PB5, 1);
    }
    #endif
    if (SSpin==PIN_PA7) {
      digitalWriteFast(PIN_PA7, 1);
    } 
    else if (SSpin==PIN_PC3) {
      digitalWriteFast(PIN_PC3, 1);
    }
  #endif

  #ifdef __AVR_ATtinyX4__
    if (SSpin==PIN_PB1) {
    digitalWrite(PIN_PB1, 1);
    }
  #endif
}

void RFM69::spi_init() {
  pinMode(SSpin, OUTPUT);
  pinMode(MOSIpin, OUTPUT);
  pinMode(MISOpin, INPUT);
  pinMode(SCKpin, OUTPUT);

  unselect();
  
  #ifdef __AVR_ATmega328P__
  SPCR = _BV(SPE) | _BV(MSTR);
  SPSR |= _BV(SPI2X);
  #endif
  
  #ifdef __AVR_DB__
  if (SPI_PORT==0) {
    PORTMUX.SPIROUTEA = SPI_MUX | (PORTMUX.SPIROUTEA & (~PORTMUX_SPI0_gm));
    SPI0.CTRLB |= (SPI_SSD_bm);
    SPI0.CTRLA |= (SPI_ENABLE_bm | SPI_MASTER_bm);
  } else {
    PORTMUX.SPIROUTEA = SPI_MUX | (PORTMUX.SPIROUTEA & (~PORTMUX_SPI1_gm));
    SPI1.CTRLB |= (SPI_SSD_bm);
    SPI1.CTRLA |= (SPI_ENABLE_bm | SPI_MASTER_bm);
  }
  #endif
  
  #ifdef __AVR_ATtinyX4__
    USICR = bit(USIWM0);
  #endif
}

uint8_t RFM69::spi_transfer (uint8_t data) {  
  #ifdef __AVR_ATmega328P__
  SPDR = data;
  while ((SPSR & (1<<SPIF)) == 0)
    ;
  return SPDR;
  #endif
  
  #ifdef __AVR_DB__
  asm volatile("nop");
  if (SPI_PORT==0) {
    SPI0.DATA = data;
    while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
    return SPI0.DATA;                             // read data back
  } else {
    SPI1.DATA = data;
    while ((SPI1.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
    return SPI1.DATA;                             // read data back
  }
  return 0;
  #endif

  #ifdef __AVR_ATtinyX4__
    USIDR = data;
      byte v1 = bit(USIWM0) | bit(USITC);
      byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
    #if F_CPU <= 5000000
      //unroll if clock is under 2.5 MHz
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
      USICR = v1; USICR = v2;
    #else
      for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
      }
    #endif
    return USIDR;
  #endif
}

void RFM69::configure (const uint8_t* p) {
  while (true) {
    uint8_t cmd = p[0];
    if (cmd == 0)
      break;
    writeReg(cmd, p[1]);
    p += 2;
  }
}


