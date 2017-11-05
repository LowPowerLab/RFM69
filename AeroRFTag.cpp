/*
 * AeroRFTag.cpp
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

#include <AeroRFTag.h>

AeroRFTag::AeroRFTag(uint8_t networkId, uint8_t nodeId) : AeroRFBase::AeroRFBase(networkId, nodeId){
	this->_packet.nodeId = nodeId;
}

/*
 * Redefintion of base method for the tag functionality.
 */
void AeroRFTag::run_cycle() {
	AeroRFBase::run_cycle();
	radio.send(DEFAULT_SEND_ADDRESS, (const void*)(&this->_packet), this->_packetSize, false);
	this->incr_ping_packet();
	//Output debugging only if serial is enabled
	#ifdef SERIAL_EN
	char buff[50];
	sprintf(buff, "Ping from Tag: %d Count: %d",
			this->_packet.nodeId,
			this->_packet.counter);
	DEBUGln(buff);
	#endif

#ifdef STATUS_LED
	this->blink(STATUS_LED);
#endif
	delayMicroseconds(100); //TODO - Put in variable value
}

bool AeroRFTag::initialize() {
	AeroRFBase::initialize();
	radio.setSendOnly();
	this->_packetSize = sizeof(this->_packet);
	return true;
}

void AeroRFTag::incr_ping_packet() {
	this->_packet.counter++;
}
