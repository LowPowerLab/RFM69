// **********************************************************************************
// Automatic Transmit Power Control class derived from RFM69 library.
// Discussion and details in this forum post: https://lowpowerlab.com/forum/index.php/topic,688.0.html
// **********************************************************************************
// Copyright Thomas Studwell (2014,2015)
// Adjustments by Felix Rusu, LowPowerLab.com
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
#include "RFM69_ATC.h"
#include "RFM69.h"   // include the RFM69 library files as well
#include "RFM69registers.h"
#include <SPI.h>

volatile uint8_t RFM69_ATC::ACK_RSSI_REQUESTED;  // new type of flag on ACK_REQUEST

//=============================================================================
// initialize() - some extra initialization before calling base class
//=============================================================================
bool RFM69_ATC::initialize(uint8_t freqBand, uint16_t nodeID, uint8_t networkID) {
  _targetRSSI = 0;        // TomWS1: default to disabled
  _ackRSSI = 0;           // TomWS1: no existing response at init time
  ACK_RSSI_REQUESTED = 0; // TomWS1: init to none
  _transmitLevelStep = 1; //increment 1 step at a time by default
  return RFM69::initialize(freqBand, nodeID, networkID);  // use base class to initialize most everything
}

//=============================================================================
// sendAck() - updated to call new sendFrame with additional parameters
//=============================================================================
// should be called immediately after reception in case sender wants ACK
void RFM69_ATC::sendACK(const void* buffer, uint8_t bufferSize) {
  ACK_REQUESTED = 0;   // TomWS1 added to make sure we don't end up in a timing race and infinite loop sending Acks
  uint16_t sender = SENDERID;
  int16_t _RSSI = RSSI; // save payload received RSSI value
  bool sendRSSI = ACK_RSSI_REQUESTED;  
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) receiveDone();
  SENDERID = sender;    // TomWS1: Restore SenderID after it gets wiped out by receiveDone()
  sendFrame(sender, buffer, bufferSize, false, true, sendRSSI, _RSSI);   // TomWS1: Special override on sendFrame with extra params
  RSSI = _RSSI; // restore payload RSSI
}

//=============================================================================
// sendFrame() - the basic version is used to match the RFM69 prototype so we can extend it
//=============================================================================
// this sendFrame is generally called by the internal RFM69 functions.  Simply transfer to our modified version.
void RFM69_ATC::sendFrame(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK) {
  sendFrame(toAddress, buffer, bufferSize, requestACK, sendACK, false, 0);  // default sendFrame
}

//=============================================================================
// sendFrame() - the new one with additional parameters.  This packages recv'd RSSI with the packet, if required.
//=============================================================================
void RFM69_ATC::sendFrame(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK, bool sendRSSI, int16_t lastRSSI) {
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

  bufferSize += (sendACK && sendRSSI)?1:0;  // if sending ACK_RSSI then increase data size by 1
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // write to FIFO
  select();
  _spi->transfer(REG_FIFO | 0x80);
  _spi->transfer(bufferSize + 3);
  _spi->transfer((uint8_t)toAddress); //lower 8bits
  _spi->transfer((uint8_t)_address);  //lower 8bits

  // CTL (control byte)
  uint8_t CTLbyte=0x0;
  if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; //assign last 2 bits of address if > 255
  if (_address > 0xFF) CTLbyte |= (_address & 0x300) >> 8;   //assign last 2 bits of address if > 255
  if (sendACK) {                   // TomWS1: adding logic to return ACK_RSSI if requested
    _spi->transfer(CTLbyte | RFM69_CTL_SENDACK | (sendRSSI?RFM69_CTL_RESERVE1:0));  // TomWS1  TODO: Replace with EXT1
    if (sendRSSI) {
      _spi->transfer(abs(lastRSSI)); //RSSI dBm is negative expected between [-100 .. -20], convert to positive and pass along as single extra header byte
      bufferSize -=1;              // account for the extra ACK-RSSI 'data' byte
    }
  }
  else if (requestACK) {  // TODO: add logic to request ackRSSI with ACK - this is when both ends of a transmission would dial power down. May not work well for gateways in multi node networks
    _spi->transfer(CTLbyte | (_targetRSSI ? RFM69_CTL_REQACK | RFM69_CTL_RESERVE1 : RFM69_CTL_REQACK));
  }
  else _spi->transfer(CTLbyte);

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

//=============================================================================
// interruptHook() - gets called by the base class interrupt handler right after the header is fetched.
//=============================================================================
void RFM69_ATC::interruptHook(uint8_t CTLbyte) {
  ACK_RSSI_REQUESTED = CTLbyte & RFM69_CTL_RESERVE1; // TomWS1: extract the ACK RSSI request bit (could potentially merge with ACK_REQUESTED)
  // TomWS1: now see if this was an ACK with an ACK_RSSI response
  if (ACK_RECEIVED && ACK_RSSI_REQUESTED) {
    // the next two bytes contain the ACK_RSSI (assuming the datalength is valid)
    if (DATALEN >= 1) {
      _ackRSSI = -1 * _spi->transfer(0); //rssi was sent as single byte positive value, get the real value by * -1
      DATALEN -= 1;   // and compensate data length accordingly
      // TomWS1: Now dither transmitLevel value (register update occurs later when transmitting);
      if (_targetRSSI != 0) {
        uint8_t maxLevel = _isRFM69HW ? 23 : 31;
        if (_ackRSSI < _targetRSSI && _powerLevel < maxLevel) {
          _powerLevel += _transmitLevelStep;
        }
        else if (_ackRSSI > _targetRSSI && _powerLevel > 0)
          _powerLevel--;
      }
    }
  }
}

//=============================================================================
//  sendWithRetry() - overrides the base to allow increasing power when repeated ACK requests fail
//=============================================================================
bool RFM69_ATC::sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
  uint32_t sentTime;
  for (uint8_t i = 0; i <= retries; i++) {
    send(toAddress, buffer, bufferSize, true);
    sentTime = millis();
    uint8_t maxLevel = _isRFM69HW ? 23 : 31;
    while (millis() - sentTime < retryWaitTime)
      if (ACKReceived(toAddress)) return true;
    if (_powerLevel < maxLevel) {
      setPowerLevel(_powerLevel + _transmitLevelStep);
    }
  }

  return false;
}

//=============================================================================
//  receiveBegin() - need to clear out our flag before calling base class.
//=============================================================================
void RFM69_ATC::receiveBegin() {
  ACK_RSSI_REQUESTED = 0;
  RFM69::receiveBegin();
}

//=============================================================================
// enableAutoPower() - call with target RSSI, use 0 to disable (default), any other value with turn on autotransmit control.
//=============================================================================
// TomWS1: New methods to address autoPower control
void  RFM69_ATC::enableAutoPower(int16_t targetRSSI){    // TomWS1: New method to enable/disable auto Power control
  _targetRSSI = targetRSSI;         // no logic here, just set the value (if non-zero, then enabled), caller's responsibility to use a reasonable value
}

//=============================================================================
// getAckRSSI() - returns the RSSI value ack'd by the far end.
//=============================================================================
int16_t  RFM69_ATC::getAckRSSI(void){                     // TomWS1: New method to retrieve the ack'd RSSI (if any)
  return (_targetRSSI==0?0:_ackRSSI);
}