/*
 * AeroRFSensor.cpp
 *
 *  Created on: Oct 28, 2017
 *      Author: midian
 */

#include "AeroRFSensor.h"


void AeroRFSensor::run_cycle() {
	AeroRFBase::run_cycle();
	if (radio.receiveDone()){
//			int16_t rssi = radio.RSSI;
//			uint8_t tag_id = radio.SENDERID;

		//Output debugging only if serial is enabled
		#ifdef SERIAL_EN
		char buff[50];
		sprintf(buff, "\nTag: %d RSSI: %d", radio.RSSI, radio.SENDERID);
		DEBUGln(buff);
		#endif
	}
}

bool AeroRFSensor::initialize(uint8_t networkId, uint8_t nodeId){
	AeroRFBase::initialize(networkId, nodeId);
	radio.promiscuous(1);
	radio.setListenOnly();
	return true;
}
