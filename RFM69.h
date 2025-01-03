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
#ifndef RFM69_h
#define RFM69_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <SPI.h>

//////////////////////////////////////////////////////////////////////
//Platform and digitalPinToInterrupt definitions credit to RadioHead//
//////////////////////////////////////////////////////////////////////
// Select platform automatically, if possible
#ifndef RF69_PLATFORM
 #if (MPIDE>=150 && defined(ARDUINO))
  // Using ChipKIT Core on Arduino IDE
  #define RF69_PLATFORM RF69_PLATFORM_CHIPKIT_CORE
 #elif defined(MPIDE)
  // Uno32 under old MPIDE, which has been discontinued:
  #define RF69_PLATFORM RF69_PLATFORM_UNO32
 #elif defined(NRF51)
  #define RF69_PLATFORM RF69_PLATFORM_NRF51
 #elif defined(NRF52)
  #define RF69_PLATFORM RF69_PLATFORM_NRF52
 #elif defined(ESP8266)
  #define RF69_PLATFORM RF69_PLATFORM_ESP8266
 #elif defined(ESP32)
  #define RF69_PLATFORM RF69_PLATFORM_ESP32
 #elif defined(ARDUINO)
  #define RF69_PLATFORM RF69_PLATFORM_ARDUINO
 #elif defined(AVR_ATtinyxy7) || defined(AVR_ATtinyxy6) // MegaTinyCore AVR Series-1
  #define RF69_PLATFORM RF69_PLATFORM_ARDUINO
 #elif defined(__MSP430G2452__) || defined(__MSP430G2553__)
  #define RF69_PLATFORM RF69_PLATFORM_MSP430
 #elif defined(MCU_STM32F103RE)
  #define RF69_PLATFORM RF69_PLATFORM_STM32
 #elif defined(STM32F2XX)
  #define RF69_PLATFORM RF69_PLATFORM_STM32F2
 #elif defined(USE_STDPERIPH_DRIVER)
  #define RF69_PLATFORM RF69_PLATFORM_STM32STD
 #elif defined(RASPBERRY_PI)
  #define RF69_PLATFORM RF69_PLATFORM_RASPI
 #elif defined(__unix__) // Linux
  #define RF69_PLATFORM RF69_PLATFORM_UNIX
 #elif defined(__APPLE__) // OSX
  #define RF69_PLATFORM RF69_PLATFORM_UNIX
 #else
  #error Platform not defined! 	
 #endif
#endif

#if defined(ESP8266) || defined(ESP32)
  #define ISR_PREFIX ICACHE_RAM_ATTR
#else
  #define ISR_PREFIX
#endif

// digitalPinToInterrupt is not available prior to Arduino 1.5.6 and 1.0.6
// See http://arduino.cc/en/Reference/attachInterrupt
#ifndef NOT_AN_INTERRUPT
 #define NOT_AN_INTERRUPT -1
#endif
#ifndef digitalPinToInterrupt
 #if (RF69_PLATFORM == RF69_PLATFORM_ARDUINO) && !defined(__arm__)
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   // Arduino Mega, Mega ADK, Mega Pro
   // 2->0, 3->1, 21->2, 20->3, 19->4, 18->5
   #define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))
  #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
   // Arduino 1284 and 1284P - See Maniacbug and Optiboot
   // 10->0, 11->1, 2->2
   #define digitalPinToInterrupt(p) ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
  #elif defined(__AVR_ATmega32U4__)
   // Leonardo, Yun, Micro, Pro Micro, Flora, Esplora
   // 3->0, 2->1, 0->2, 1->3, 7->4
   #define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
  #else
   // All other arduino except Due:
   // Serial Arduino, Extreme, NG, BT, Uno, Diecimila, Duemilanove, Nano, Menta, Pro, Mini 04, Fio, LilyPad, Ethernet etc
   // 2->0, 3->1
   #define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))
  #endif
 #elif (RF69_PLATFORM == RF69_PLATFORM_UNO32) || (RF69_PLATFORM == RF69_PLATFORM_CHIPKIT_CORE)
  // Hmmm, this is correct for Uno32, but what about other boards on ChipKIT Core?
  #define digitalPinToInterrupt(p) ((p) == 38 ? 0 : ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 735 ? 4 : NOT_AN_INTERRUPT)))))
 #else
  // Everything else (including Due and Teensy) interrupt number the same as the interrupt pin number
  #define digitalPinToInterrupt(p) (p)
 #endif
#elif defined(__SAMD21__) || defined (__SAMD51__) //Arduino.h in most/all cores wrongly "#define digitalPinToInterrupt(P) (P)" after calling variant.h
  #define digitalPinToInterrupt(P)   (g_APinDescription[P].ulExtInt)
#endif

// On some platforms, attachInterrupt() takes a pin number, not an interrupt number
#if (defined(TEENSYDUINO))||((RF69_PLATFORM == RF69_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_SAM_DUE)))
 #define RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
#endif
////////////////////////////////////////////////////

#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          7
#elif defined(__STM32F1__) || defined(STM32F1)
  #define RF69_IRQ_PIN          PA3
  #define RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
#elif defined(MOTEINO_M0)
  #define RF69_IRQ_PIN          9
#elif defined(__SAMD51__)
  #define RF69_IRQ_PIN          18
#elif defined(ARDUINO_SAMD_ZERO) //includes Feather SAMD
  #define RF69_IRQ_PIN          3
#elif defined(ESP8266)
  #define RF69_IRQ_PIN          4
#elif defined(ESP32)
  #define RF69_IRQ_PIN          2
#else
  #define RF69_IRQ_PIN          2
#endif

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ             31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ             43
#define RF69_868MHZ             86
#define RF69_915MHZ             91

#define null                    0
#define COURSE_TEMP_COEF        -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR     0
#define RF69_CSMA_LIMIT_MS      1000
#define RF69_TX_LIMIT_MS        1000
#define RF69_FSTEP              61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK       0x80
#define RFM69_CTL_REQACK        0x40

#define RFM69_ACK_TIMEOUT       30  // 30ms roundtrip req for 61byte packets

// Native hardware ListenMode is experimental
// It was determined to be buggy and unreliable, see https://lowpowerlab.com/forum/low-power-techniques/ultra-low-power-listening-mode-for-battery-nodes/msg20261/#msg20261
// uncomment to try ListenMode, adds ~1K to compiled size
// FYI - 10bit addressing is not supported in ListenMode
//#define RF69_LISTENMODE_ENABLE

#if defined(RF69_LISTENMODE_ENABLE)
  // By default, receive for 256uS in listen mode and idle for ~1s
  #define  DEFAULT_LISTEN_RX_US   256
  #define  DEFAULT_LISTEN_IDLE_US 1000000
#endif

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

    RFM69(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW, uint8_t interruptNum __attribute__((unused))) //interruptNum is now deprecated
                : RFM69(slaveSelectPin, interruptPin, isRFM69HW){};

    RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW_HCW=false, SPIClass *spi=nullptr);

    bool initialize(uint8_t freqBand, uint16_t ID, uint8_t networkID=1);
    void setAddress(uint16_t addr);
    void setNetwork(uint8_t networkID);
    void setIsrCallback(void (*callback)());
    virtual bool canSend();
    virtual void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    virtual bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=RFM69_ACK_TIMEOUT);
    virtual bool receiveDone();
    bool ACKReceived(uint16_t fromNodeID);
    bool ACKRequested();
    virtual void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);
    void encrypt(const char* key);
    void setCS(uint8_t newSPISlaveSelect);
    bool setIrq(uint8_t newIRQPin);
    int16_t readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
    void spyMode(bool onOff=true);
    //void promiscuous(bool onOff=true); //replaced with spyMode()
    virtual void setHighPower(bool _isRFM69HW_HCW=true); // has to be called after initialize() for RFM69 HW/HCW
    virtual void setPowerLevel(uint8_t level); // reduce/increase transmit power level
    virtual int8_t setPowerDBm(int8_t dBm); // reduce/increase transmit power level, in dBm

    virtual uint8_t getPowerLevel(); // get powerLevel	
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]
    void set300KBPS();
    uint8_t setLNA(uint8_t newReg);

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();
    void readAllRegsCompact();

    // ListenMode sleep/timer
    void listenModeSleep(uint16_t millisInterval);
    void endListenModeSleep();
    virtual void setMode(uint8_t mode);
    virtual void select();
    virtual void unselect();

  protected:
    static void isr0();
    void interruptHandler();
    virtual void interruptHook(uint8_t CTLbyte __attribute__((unused))) {};
    static volatile bool _haveData;
    static RFM69 *_instance;
    virtual void sendFrame(uint16_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);

    // for ListenMode sleep/timer
    static void delayIrq();

    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
    uint16_t _address;
    bool _spyMode;
    uint8_t _powerLevel;
    bool _isRFM69HW;
    SPIClass *_spi;
    void (*_isrCallback)() = nullptr;
#if defined (SPCR) && defined (SPSR)
    uint8_t _SPCR;
    uint8_t _SPSR;
#endif
#ifdef SPI_HAS_TRANSACTION
  SPISettings _settings;
#endif

    virtual void receiveBegin();
    //virtual void setMode(uint8_t mode);
    virtual void setHighPowerRegs(bool enable);
    //virtual void select();
    //virtual void unselect();

#if defined(RF69_LISTENMODE_ENABLE)
  static RFM69* selfPointer;
  //=============================================================================
  //                     ListenMode specific declarations  
  //=============================================================================
  public:
    // When we receive a packet in listen mode, this is the time left in the sender's burst.
    // You need to wait at least this long before trying to reply.
    static volatile uint16_t RF69_LISTEN_BURST_REMAINING_MS;
    
    void listenModeStart(void);
    void listenModeEnd(void);
    void listenModeHighSpeed(bool highSpeed) { _isHighSpeed = highSpeed; }
    
    // rx and idle duration in microseconds
    bool listenModeSetDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // The values passed to listenModeSetDurations() may be slightly different to accomodate
    // what is allowed by the radio. This function returns the actual values used.
    void listenModeGetDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // This repeatedly sends the message to the target node for the duration
    // of an entire listen cycle. The amount of time remaining in the burst
    // is transmitted to the receiver, and it is expected that the receiver
    // wait for the burst to end before attempting a reply.
    // See RF69_LISTEN_BURST_REMAINING_MS above.
    void listenModeSendBurst(uint8_t targetNode, const void* buffer, uint8_t size);

  protected:
    void listenModeInterruptHandler(void);
    void listenModeApplyHighSpeedSettings();
    void listenModeReset(); //resets variables used on the receiving end
    bool reinitRadio(void);
    static void listenModeIrq();

    bool _isHighSpeed;
    bool _haveEncryptKey;
    char _encryptKey[16];

    // Save these so we can reinitialize the radio after sending a burst
    // or exiting listen mode.
    uint8_t _freqBand;
    uint8_t _networkID;
    uint8_t _rxListenCoef;
    uint8_t _rxListenResolution;
    uint8_t _idleListenCoef;
    uint8_t _idleListenResolution;
    uint32_t _listenCycleDurationUs;
#endif
};

#endif
