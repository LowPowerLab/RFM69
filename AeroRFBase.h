/*
 * AeroRFBase.h
 *
 *  Created on: Oct 28, 2017
 *      Author: Joel Blackthorne
 *
 *
 *  This module is based on the RFM69HW_HCW HopeRF transceiver
 *  and will only work with this specific module.
 *
 *  Extends work Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
 */
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


#ifndef AERORFBASE_H_
#define AERORFBASE_H_

#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <RFM69.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
//#define NETWORKID     106  //the same on all nodes that talk to each other
//
////Node IDS determine the type of device, so the ide must be in the correct range
////Nodes 1-30 = Sensors
////Nodes 31 + are tags
//#define NODEID        1    //unique for each node on same network
//#define NODEID_SENSOR_CUTT_OFF 30 //max number for a sensor node id

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "Aer0RadioFreq915" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
//*********************************************************************************************
#define SERIAL_EN     //comment out if you don't want any serial verbose output
#define SERIAL_BAUD   115200

//*********************************************************************************************
//Configure the distance ping
//*********************************************************************************************
#define ACK_TIME       30  // # of ms to wait for an ack packet

//ATC Notes: DO NOT Enable Automatic Transmission Control!
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL


#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif


class AeroRFBase {
public:
	AeroRFBase(uint8_t networkId, uint8_t nodeId);
	virtual ~AeroRFBase();
	uint8_t getNodeId();
	uint8_t getNetworkId();
	void run_cycle();
	bool initialize(uint8_t networkId, uint8_t nodeId);
	RFM69 radio;
private:
	uint8_t _nodeId;
	uint8_t _networkId;
};

#endif /* AERORFBASE_H_ */
