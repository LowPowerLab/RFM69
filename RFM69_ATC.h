// **********************************************************************************
// Automatic Transmit Power Control class derived from RFM69 library.
// Discussion and details in this forum post: https://lowpowerlab.com/forum/index.php/topic,688.0.html
// **********************************************************************************
// Copyright Thomas Studwell (2014,2015)
// Adjustments by Felix Rusu, LowPowerLab.com
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
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
#ifndef RFM69_ATC_h
#define RFM69_ATC_h
#include "RFM69.h"

#define RFM69_CTL_RESERVE1  0x20

class RFM69_ATC: public RFM69 {
  public:
    static volatile uint8_t ACK_RSSI_REQUESTED;  // new flag in CTL byte to request RSSI with ACK (could potentially be merged with ACK_REQUESTED)

    RFM69_ATC(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, SPIClass *spi=nullptr) :
      RFM69(slaveSelectPin, interruptPin, isRFM69HW, spi) {
    }

    bool initialize(uint8_t freqBand, uint16_t ID, uint8_t networkID=1);
    void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=RFM69_ACK_TIMEOUT);
    void enableAutoPower(int16_t targetRSSI=-90);  // TWS: New method to enable/disable auto Power control
    int16_t getTargetRssi();

    int16_t getAckRSSI(void);       // TWS: New method to retrieve the ack'd RSSI (if any)
    int16_t _targetRSSI;     // if non-zero then this is the desired end point RSSI for our transmission
    uint8_t _transmitLevelStep;  // saved powerLevel in case we do auto power adjustment, this value gets dithered

  protected:
    void interruptHook(uint8_t CTLbyte);
    void sendFrame(uint16_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);  // Need this one to match the RFM69 prototype.
    void sendFrame(uint16_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK, bool sendRSSI, int16_t lastRSSI);
    void receiveBegin();

    int16_t _ackRSSI;         // this contains the RSSI our destination Ack'd back to us (if we enabledAutoPower)
    uint8_t _PA_Reg;          // saved and derived PA control bits so we don't have to spend time reading back from SPI port
};

#endif
