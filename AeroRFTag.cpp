/*
 * AeroRFTag.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: midian
 */

#include <AeroRFTag.h>

/*
 * Redefintion of base method for the tag functionality.
 */
void AeroRFTag::run_cycle() {
	AeroRFBase::run_cycle();
	radio.send(DEFAULT_SEND_ADDRESS, (const void*)(&this->_packet), this->_packetSize, false);
	delayMicroseconds(25000); //TODO - Put in variable value
}

bool AeroRFTag::initialize(uint8_t networkId, uint8_t nodeId) {
	AeroRFBase::initialize(networkId, nodeId);
	radio.setSendOnly();
	this->_packetSize = sizeof(this->_packet);
	return true;
}
