/*
 * AeroRFSensor.h
 *
 *  Created on: Oct 28, 2017
 *      Author: midian
 */

#ifndef AERORFSENSOR_H_
#define AERORFSENSOR_H_

#include "AeroRFBase.h"

class AeroRFSensor: public AeroRFBase {
public:
	void run_cycle();
	bool initialize(uint8_t networkId, uint8_t nodeId);
};

#endif /* AERORFSENSOR_H_ */
