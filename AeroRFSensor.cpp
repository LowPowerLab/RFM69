/*
 * AeroRFSensor.cpp
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

#include "AeroRFSensor.h"

AeroRFSensor::AeroRFSensor(uint8_t networkId, uint8_t nodeId): AeroRFBase::AeroRFBase(networkId, nodeId) {
}

void AeroRFSensor::run_cycle() {
	AeroRFBase::run_cycle();
	if (radio.receiveDone()){
		this->print_packet(radio.SENDERID, radio.RSSI);
		#ifdef DEBUG_EN
			this->print_debug(radio.SENDERID, radio.RSSI);
		#endif
		#ifdef STATUS_LED
			this->blink(STATUS_LED);
		#endif
	}
}

bool AeroRFSensor::initialize(){
	AeroRFBase::initialize();
	radio.promiscuous(1);
	radio.setListenOnly();
	return true;
}

//Output debugging only if serial is enabled
void AeroRFSensor::print_debug(uint8_t tagId, int16_t rssi) {
	#ifdef SERIAL_EN
	char debug_buff[50];
	sprintf(debug_buff, "Tag: %d RSSI: %d", tagId, rssi);
	DEBUGln(debug_buff);
	#endif
}

void AeroRFSensor::print_packet(uint8_t tagId, int16_t rssi) {
	#ifdef SERIAL_EN
		char pkt[3];
		pkt[0] = tagId;
		pkt[1] = rssi & 0xFF;
		pkt[2] = rssi >> 8;
		SERPRINT(pkt);
	#endif
}
