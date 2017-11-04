/*
 * AeroRFBase.cpp
 *
 *  Created on: Oct 28, 2017
 *      Author: Joel Blackthorne
 *
 *  AeroRFLib is a radio library designed for long-distance, high-frequency,
 *  distance measurement as part of the AeroTracker project.
 *
 *
 *  This module is based on the RFM69HW_HCW and RFM69HW_HW HopeRF transceivers
 *  operating at 915mhz and will only work with those specific modules.
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

#include "AeroRFBase.h"
#include "SPI.h"           //included with Arduino IDE install (www.arduino.cc)

AeroRFBase::AeroRFBase(uint8_t networkId, uint8_t nodeId) {
	this->_networkId = networkId;
	this->_nodeId = nodeId;
}

AeroRFBase::~AeroRFBase() {
	// TODO Auto-generated destructor stub
}

/*
* The NodeID is the unique identifier for each
* radio.  Sensors are kept in the lower range,
* from 1 to 30 (defined by NODEID_SENSOR_CUTT_OFF).
* All node ids > NODEID_SENSOR_CUTT_OFF are considered
* target tags.
*/
uint8_t AeroRFBase::getNodeId() {
	return this->_nodeId;
}

uint8_t AeroRFBase::getNetworkId() {
	return this->_networkId;
}

bool AeroRFBase::initialize() {
	bool rval = true;
#ifdef STATUS_LED
	pinMode(STATUS_LED, OUTPUT);
	this->blink(STATUS_LED);
#endif


	//Initialize the radio
	radio.initialize(FREQUENCY,this->_nodeId,this->_networkId);
	radio.setHighPower(); //must include this only for RFM69HW/HCW!

	//Init the serial bus
	Serial.begin(SERIAL_BAUD);

	//Output debugging only if serial is enabled
	#ifdef DEBUG_EN
		char buff[50];
		sprintf(buff, "\nTransmitting at %d Mhz...", int(radio.getFrequency()/1000000));
		DEBUGln(buff);
	#endif
	#ifdef STATUS_LED
		this->blink(STATUS_LED);
	#endif
	return rval;
}

/*
 * Single run cycle to be executed in main loop.
 */
void AeroRFBase::run_cycle() {
}

/*
 * Blinks a LED pin.
 */
void AeroRFBase::blink(uint8_t pin) {
#ifdef STATUS_LED
	if (this->_blink_on){
		this->_blink_on = false;
		digitalWrite(pin,LOW);
	}
	else{
		this->_blink_on = true;
		digitalWrite(pin,HIGH);
	}
	//This delay can help with debugging
//	delay(BLINK_DELAY);
#endif
}
