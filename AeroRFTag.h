/*
 * AeroRFTag.h
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

#ifndef AERORFTAG_H_
#define AERORFTAG_H_

#include <AeroRFBase.h>

#define DEFAULT_SEND_ADDRESS 1
#define PING_DELAY_MS 20

typedef struct{
	uint8_t nodeId;
	uint16_t counter;
} PingPacket;

class AeroRFTag: public AeroRFBase {
public:
	AeroRFTag(uint8_t networkId, uint8_t nodeId);
	void run_cycle();
	bool initialize();
private:
	PingPacket _packet;
	uint8_t _packetSize;
	void incr_ping_packet();
};

#endif /* AERORFTAG_H_ */
