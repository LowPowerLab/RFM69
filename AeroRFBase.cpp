/*
 * AeroRFBase.cpp
 *
 *  Created on: Oct 28, 2017
 *      Author: midian
 */

#include "AeroRFBase.h"
#include "SPI.h"           //included with Arduino IDE install (www.arduino.cc)

AeroRFBase::AeroRFBase(uint8_t networkId, uint8_t nodeId) {
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
	return _nodeId;
}

uint8_t AeroRFBase::getNetworkId() {
	return _networkId;
}

bool AeroRFBase::initialize(uint8_t networkId, uint8_t nodeId) {
	bool rval = true;
	this->_nodeId = nodeId;
	this->_networkId = networkId;


	//Initialize the radio
	radio.initialize(FREQUENCY,nodeId,networkId);
	radio.setHighPower(); //must include this only for RFM69HW/HCW!

	//Output debugging only if serial is enabled
	#ifdef SERIAL_EN
	char buff[50];
	sprintf(buff, "\nTransmitting at %d Mhz...", int(radio.getFrequency()/1000000));
	DEBUGln(buff);
	#endif

	return rval;
}

/*
 * Single run cycle to be executed in main loop.
 */
void AeroRFBase::run_cycle() {
}
