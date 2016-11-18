// **********************************************************************************
// Library for OTA wireless programming of Moteinos using an RFM69 transceiver
// **********************************************************************************
// Hardware requirements:
//   - DualOptiboot bootloader - ships with all Moteinos
//   - SPI "Flash MEM" chip on Moteino (optional)
// Library requirements:
//   - RFM69      - get library at: https://github.com/LowPowerLab/RFM69
//   - SPIFLash.h - get it here: http://github.com/LowPowerLab/SPIFlash
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
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
#ifndef RFM69_OTA_H
#define RFM69_OTA_H
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
#else
  #define LED           9 // Moteinos have LEDs on D9
#endif
#define SHIFTCHANNEL 1000000 //amount to shift frequency of HEX transmission to keep original channel free of the HEX transmission traffic

#ifndef DEFAULT_TIMEOUT
  #define DEFAULT_TIMEOUT 3000
#endif

#ifndef ACK_TIMEOUT
  #define ACK_TIMEOUT 20
#endif

#include <RFM69.h>
#include <SPIFlash.h>

//functions used in the REMOTE node
void CheckForWirelessHEX(RFM69 radio, SPIFlash flash, uint8_t DEBUG=false, uint8_t LEDpin=LED);
void HandleHandshakeACK(RFM69 radio, SPIFlash flash, uint8_t flashCheck=true);
void resetUsingWatchdog(uint8_t DEBUG=false);
uint8_t HandleWirelessHEXData(RFM69 radio, uint8_t remoteID, SPIFlash flash, uint8_t DEBUG=false, uint8_t LEDpin=LED);

#ifdef SHIFTCHANNEL
uint8_t HandleWirelessHEXDataWrapper(RFM69 radio, uint8_t remoteID, SPIFlash flash, uint8_t DEBUG=false, uint8_t LEDpin=LED);
#endif

//functions used in the MAIN node
uint8_t CheckForSerialHEX(uint8_t* input, uint8_t inputLen, RFM69 radio, uint8_t targetID, uint16_t TIMEOUT=DEFAULT_TIMEOUT, uint16_t ACKTIMEOUT=ACK_TIMEOUT, uint8_t DEBUG=false);
uint8_t HandleSerialHandshake(RFM69 radio, uint8_t targetID, uint8_t isEOF, uint16_t TIMEOUT=DEFAULT_TIMEOUT, uint16_t ACKTIMEOUT=ACK_TIMEOUT, uint8_t DEBUG=false);
uint8_t HandleSerialHEXData(RFM69 radio, uint8_t targetID, uint16_t TIMEOUT=DEFAULT_TIMEOUT, uint16_t ACKTIMEOUT=ACK_TIMEOUT, uint8_t DEBUG=false);
#ifdef SHIFTCHANNEL
uint8_t HandleSerialHEXDataWrapper(RFM69 radio, uint8_t targetID, uint16_t TIMEOUT=DEFAULT_TIMEOUT, uint16_t ACKTIMEOUT=ACK_TIMEOUT, uint8_t DEBUG=false);
#endif
uint8_t waitForAck(RFM69 radio, uint8_t fromNodeID, uint16_t ACKTIMEOUT=ACK_TIMEOUT);

uint8_t validateHEXData(void* data, uint8_t length);
uint8_t prepareSendBuffer(char* hexdata, uint8_t*buf, uint8_t length, uint16_t seq);
uint8_t sendHEXPacket(RFM69 radio, uint8_t remoteID, uint8_t* sendBuf, uint8_t hexDataLen, uint16_t seq, uint16_t TIMEOUT=DEFAULT_TIMEOUT, uint16_t ACKTIMEOUT=ACK_TIMEOUT, uint8_t DEBUG=false);
uint8_t BYTEfromHEX(char MSB, char LSB);
uint8_t readSerialLine(char* input, char endOfLineChar=10, uint8_t maxLength=115, uint16_t timeout=1000);
void PrintHex83(uint8_t* data, uint8_t length);

#endif