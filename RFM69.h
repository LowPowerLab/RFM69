// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H
// **********************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/3.0/
// 2013-06-14 (C) felix@lowpowerlab.com
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#define MAX_DATA_LEN         61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead)
#define SPI_CS               SS // SS is the SPI slave select pin, for instance D10 on atmega328
#define RF69_IRQ_PIN          2 // INT0 on AVRs should be connected to DIO0
                                // ex on Atmega328 it's D2
#define CSMA_LIMIT          -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP       0 // XTAL OFF
#define	RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_SYNTH	      2 // PLL ON
#define RF69_MODE_RX          3 // RX MODE
#define RF69_MODE_TX		      4 // TX MODE

//available frequency bands
#define RF69_315MHZ     31  // non trivial values to avoid misconfiguration
#define RF69_433MHZ     43
#define RF69_868MHZ     86
#define RF69_915MHZ     91

#define null            0
#define COURSE_TEMP_COEF  -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

class RFM69 {
  public:
    static volatile byte DATA[MAX_DATA_LEN];          // recv/xmit buf, including hdr & crc bytes
    static volatile byte DATALEN;
    static volatile byte SENDERID;
    static volatile byte TARGETID; //should match _address
    static volatile byte PAYLOADLEN;
    static volatile byte ACK_REQUESTED;
    static volatile byte ACK_RECEIVED; /// Should be polled immediately after sending a packet with ACK request
    static volatile byte _mode; //should be protected?
    
    RFM69(byte slaveSelectPin=SPI_CS, byte interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _mode = RF69_MODE_STANDBY;
      _promiscuousMode = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
    }

    bool initialize(byte freqBand, byte ID, byte networkID=1);
    void setAddress(byte addr);
    bool canSend();
    void send(byte toAddress, const void* buffer, byte bufferSize, bool requestACK=false);
    bool sendWithRetry(byte toAddress, const void* buffer, byte bufferSize, byte retries=2, byte retryWaitTime=15);
    bool receiveDone();
    bool ACKReceived(byte fromNodeID);
    void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    void setFrequency(uint32_t FRF);
    void encrypt(const char* key);
    void setCS(byte newSPISlaveSelect);
    int readRSSI(bool forceTrigger=false);
    void promiscuous(bool onOff=true);
    void setHighPower(bool onOFF=true); //have to call it after initialize for RFM69HW
    void setPowerLevel(byte level); //reduce/increase transmit power level
    void sleep();
    byte readTemperature(byte calFactor=0); //get CMOS temperature (8bit)
    void rcCalibration(); //calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    byte readReg(byte addr);
    void writeReg(byte addr, byte val);
    void readAllRegs();
    
  protected:
    static void isr0();
    void virtual interruptHandler();
    void sendFrame(byte toAddress, const void* buffer, byte size, bool requestACK=false, bool sendACK=false);

    static RFM69* selfPointer;
    byte _slaveSelectPin;
    byte _interruptPin;
    byte _address;
    bool _promiscuousMode;
    byte _powerLevel;
    bool _isRFM69HW;

    void receiveBegin();
    void setMode(byte mode);
    void setHighPowerRegs(bool onOff);
    void select();
    void unselect();
};

#endif