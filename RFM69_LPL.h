// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
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
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
  
#define RF69_MAX_DATA_LEN         61
#define CSMA_LIMIT               -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP           0<<2
#define RF69_MODE_STANDBY         1<<2
#define RF69_MODE_TX              3<<2
#define RF69_MODE_RX              4<<2

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define RF69_BROADCAST_ADDR       0
#define RF69_CSMA_LIMIT_MS        1000

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK         0x80
#define RFM69_CTL_REQACK          0x40

#define RFM69_ACK_TIMEOUT   30  // 30ms roundtrip req for 61byte packets

class RFM69 {
  public:
    static uint8_t DATA[RF69_MAX_DATA_LEN+1]; // RX/TX payload buffer, including end of string NULL char
    static uint8_t DATALEN;
    static uint16_t SENDERID;
    static uint16_t TARGETID; // should match _address
    static uint8_t PAYLOADLEN;
    static uint8_t ACK_REQUESTED;
    static uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
    static int16_t RSSI; // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
    static uint8_t _mode; // should be protected?
    
    
    bool initialize(uint8_t freqBand, uint16_t ID, uint8_t networkID=1);
    void setPins (uint8_t _SSpin, uint8_t _MOSIpin, uint8_t _MISOpin, uint8_t _SCKpin);
    void setAddress(uint16_t addr);
    void sleep();
    bool canSend(); 
    void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=RFM69_ACK_TIMEOUT);    
    void send_csma(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t custom_csma_limit=0);

    bool receiveDone();
    bool ACKReceived(uint16_t fromNodeID);
    bool ACKRequested();
    void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    uint8_t retry_count();
    
    void encrypt (const char* key);
    int16_t readRSSI(bool forceTrigger=false);
    
    uint8_t readReg (uint8_t addr);
    void writeReg (uint8_t addr, uint8_t value);
    
    void setMode (uint8_t newMode);
    void select();
    void unselect();
    
    int receive ();
    
  protected:
    void spi_init();
    void interruptHandler();
    void sendFrame(uint16_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);
    uint8_t spi_transfer (uint8_t data);
    void configure (const uint8_t* p);
    void receiveBegin();
    uint8_t _address;
    uint8_t _retrycount;
        
    uint8_t SSpin = 10;
    uint8_t MOSIpin = 11;
    uint8_t MISOpin = 12;
    uint8_t SCKpin = 13;
    uint8_t SPI_PORT = 0;  
};
