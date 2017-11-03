/*
 * AeroRFTag.h
 *
 *  Created on: Oct 30, 2017
 *      Author: midian
 */

#ifndef AERORFTAG_H_
#define AERORFTAG_H_

#include <AeroRFBase.h>

#define DEFAULT_SEND_ADDRESS 1

typedef struct{
	uint8_t nodeId;
	uint16_t counter;
} PingPacket;

class AeroRFTag: public AeroRFBase {
public:
	void run_cycle();
	bool initialize(uint8_t networkId, uint8_t nodeId);
private:
	PingPacket _packet;
	uint8_t _packetSize;
};

#endif /* AERORFTAG_H_ */
