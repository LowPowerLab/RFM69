// **********************************************************************************
// Sample RFM69 sender/node sketch with radio listen mode sleep
// Saves additional 2-3uA over WDT sleep
// **********************************************************************************
// Copyright Felix Rusu 2020, http://www.LowPowerLab.com/contact
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
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <LowPower.h>
#include <PString.h>   //easy string manipulator: http://arduiniana.org/libraries/pstring/
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID          123   //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID       100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID       1
#define FREQUENCY       RF69_915MHZ //match the RFM69 version! Others: RF69_433MHZ, RF69_868MHZ
//#define FREQUENCY_EXACT 916000000
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY      "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC      //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -90
// **********************************************************************************
//to avoid the buggy listen mode high resolution timer TRANSMITPERIOD should always be > 262ms
#define TRANSMITPERIOD  3000 //sleep time in ms
//#define WDTSLEEP      //uncomment to sleep with WDT instead (compare sleep currents!)
// **********************************************************************************
#define SERIAL_EN      //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
  #define DEBUGFlush() Serial.flush()
#else
  #define DEBUG(input)
  #define DEBUGln(input)
  #define DEBUGFlush()
#endif
// **********************************************************************************
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char buff[61]; //61 max payload for radio
PString Pbuff(buff, sizeof(buff));
// **********************************************************************************

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif

  if (!radio.initialize(FREQUENCY,NODEID,NETWORKID))
    DEBUG("radio.init() FAIL");
  else
    DEBUG("radio.init() SUCCESS");

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif

#ifdef ENCRYPTKEY
  radio.encrypt(ENCRYPTKEY);
#endif

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  Pbuff = F("Transmitting at ");
  Pbuff.print(radio.getFrequency());
  Pbuff.print(F("Hz..."));
  DEBUGln(buff);
}

uint32_t packetCounter=0;
void loop() {
  Pbuff = F("PACKET #");
  Pbuff.print(packetCounter++);
  DEBUGln(buff);
  digitalWrite(LED_BUILTIN, HIGH);
  radio.sendWithRetry(GATEWAYID, buff, Pbuff.length());
  digitalWrite(LED_BUILTIN, LOW);
  DEBUGFlush();

#ifdef WDTSLEEP
  radio.sleep();
  LowPower.longPowerDown(TRANSMITPERIOD);
#else
  if (TRANSMITPERIOD%262 && TRANSMITPERIOD > 262*2)
  {
    DEBUG("Sleeping "); DEBUGln(TRANSMITPERIOD-TRANSMITPERIOD%262-262); DEBUGFlush();
    radio.listenModeSleep(TRANSMITPERIOD-TRANSMITPERIOD%262-262);
    DEBUG("Sleeping "); DEBUGln(TRANSMITPERIOD%262 + 262); DEBUGFlush();
    radio.listenModeSleep(TRANSMITPERIOD%262 + 262);
  }
  else {
    DEBUG("Sleeping "); DEBUGln(TRANSMITPERIOD); DEBUGFlush();
    radio.listenModeSleep(TRANSMITPERIOD);
  }

  //wakeup (must reinit)
  radio.RFM69::initialize(FREQUENCY,NODEID,NETWORKID); //call base init!
  #ifdef ENCRYPTKEY
    radio.encrypt(ENCRYPTKEY);
  #endif
  #ifdef FREQUENCY_EXACT
    radio.setFrequency(FREQUENCY_EXACT);
  #endif
#endif
}
