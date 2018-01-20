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

AeroRFSensor::AeroRFSensor(): AeroRFBase::AeroRFBase() {
	last_command = 0; //initialize
}

void AeroRFSensor::run_cycle() {
	AeroRFBase::run_cycle();

	return;
	switch (this->last_command){
	case 0:
		this->print_info();
		break;
	case CMD_LISTEN_START:
		this->check_radio();
		break;
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
	SER_WRITE(DATA_PREFIX_CHAR);
	SER_WRITE(DATA_PREFIX_CHAR);
	SER_WRITE(tagId);
	SER_WRITE(((uint8_t)rssi & 0xFF));
	SER_WRITE((rssi >> 8));
	#endif
}

//Checks hardware serial to see if a request
//for a command packet has been received
//
//Command packets can be at most 2 bytes
void AeroRFSensor::check_for_command_packet() {
	bool command_started = false;
	char nchar = 0;
	while (Serial.available()){
		nchar = (char)Serial.read();
		if (nchar == CMD_START){
			command_started = true;
		}
		else if (command_started){
			//Full command is now received, so process
			this->process_command(nchar);
			command_started = false;
		}
		else{
			command_started = false;
		}
	}
}

//Performs a read on the radio
void AeroRFSensor::check_radio() {
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

//Executes a command immediately if applicable
void AeroRFSensor::process_command(char cmd) {
	switch (cmd){
	case CMD_IDENTIFY:
		this->send_identification();
		break;
	}
	this->last_command = cmd;
}

//Sends a sensor identification over serial
//
//Identification packet is 4 bytes in the form:
// <begin response>
// <begin response>
// <network id>
// <node id>
void AeroRFSensor::send_identification() {
	SER_WRITE(CMD_RESPONSE);
	SER_WRITE(CMD_RESPONSE);
	SER_WRITE(this->getNetworkId());
	SER_WRITE(this->getNodeId());
}
