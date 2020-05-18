// **********************************************************************************
// Sample RFM69 sender/node sketch for DS18B20
// Periodically wakes up, reads the sensor, and goes back to sleep
// Compensates sleep time for the conversion time which can take up to 700ms on this sensor
//
//                  ************************************
//                  **** IMPORTANT NOTES ON DS18B20 ****
//                  ************************************
//
// DS18B20 requires a pullup on the data pin (2.2K-10K)
// Adjust DS18B20 read resolution based on required accuracy and conversion time
// Radio & MCU are WDT slept during conversion but DS18B20 still uses ~1mA during conversion
// Conversion times: ~95ms @9bit, ~180ms @10bit, 370ms @11bit, ~650ms @12bit)
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
//*********************************************************************************************
#include <RFM69.h>      //https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>  //included in RFM69
#include <LowPower.h>   //https://www.github.com/lowpowerlab/lowpower
#include <OneWire.h>    //https://github.com/PaulStoffregen/OneWire
//*********************************************************************************************
#define DALLAS_PIN        7
#define DALLAS_RESOLUTION 12
#define TRANSMITPERIOD    5000  //read and send data every this many ms (WDT sleep)
//*********************************************************************************************
#define NODEID           789
#define NETWORKID        100
#define GATEWAYID        1   //as a rule of thumb the gateway ID should always be 1
#define FREQUENCY        RF69_915MHZ  //match the RFM69 version! Others: RF69_433MHZ, RF69_868MHZ
//#define FREQUENCY_EXACT 916000000
#define ENCRYPTKEY       "sampleEncryptKey" //same 16 characters on all nodes, comment this line to disable encryption
#define IS_RFM69HW_HCW   //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
#define ENABLE_ATC       //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI         -90
//*********************************************************************************************
#define SERIAL_BAUD      115200 //comment out to turn off serial output
#ifdef SERIAL_BAUD
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
  #define DEBUGHEX(x)    Serial.print(x, HEX)
  #define DEBUGflush()   Serial.flush()
#else
  #define DEBUG(input)
  #define DEBUGln(input)
  #define DEBUGflush()
  #define DEBUGHEX(x)
#endif

#define LED_HIGH digitalWrite(LED_BUILTIN, HIGH)
#define LED_LOW digitalWrite(LED_BUILTIN, LOW)
//*********************************************************************************************
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

int HighByte, LowByte, TReading, SignBit, Tc_100, Fh_100, Whole, Fract;
char buff[61]; //max packet size is 61 with encryption enabled
char Fstr[10];
byte buffLen;
float fahrenheit, celsius;
uint32_t time=0, now=0, LASTPACKETTIME=-1;
uint16_t conversionTime=0;

void setup() {
#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED_BUILTIN, OUTPUT);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HCW
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
#endif
  radio.sleep();
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
}

void loop() {
  fahrenheit = 0;
  celsius = 0;
  getTemp(DALLAS_PIN, DALLAS_RESOLUTION);

  if (fahrenheit !=0) {
    dtostrf(fahrenheit, 3,2, Fstr);
    sprintf(buff, "F:%s", Fstr);
    buffLen = strlen(buff);

    LED_HIGH;
    DEBUG("Sending '"); DEBUG(buff); DEBUG("' .. ");
    if (radio.sendWithRetry(GATEWAYID, buff, buffLen))
      DEBUG("ok!");
    else DEBUG("nok...");
    radio.sleep();
    DEBUGln();
    LED_LOW;
  }
  else DEBUGln("Nothing to send: fahrenheit=0");

  DEBUGflush();
  LowPower.longPowerDown(TRANSMITPERIOD - conversionTime);
  DEBUGln("WAKEUP");
}

void getTemp(byte dataPin, byte resolution) {
  byte dsaddr[8]; // Device identifier
  OneWire myds(dataPin);
  getFirstDSAddr(myds,dsaddr);
  conversionTime=0;

  DEBUG(F("DallasAddress:"));
  for (int j=0;j<8;j++) {
    if (dsaddr[j] < 16) DEBUG('0');
    DEBUGHEX(dsaddr[j]);
  }
  DEBUGln();
  getdstemp(myds, dsaddr, resolution);
}

void getFirstDSAddr(OneWire myds, byte firstAddr[]){
  byte i;
  byte present = 0;
  byte addr[8];
  float celsius, fahrenheit;
  int length = 8;

  //DEBUGln("Looking for 1-Wire devices...");
  while(myds.search(addr)) {
    for( i = 0; i < 8; i++) firstAddr[i]=addr[i];
    if (OneWire::crc8( addr, 7) != addr[7]) {
        DEBUGln("FAIL:INVALID CRC");
        return;
    }
    return;
  }
}

float getdstemp(OneWire myds, byte addr[8], byte resolution) {
  byte present = 0;
  byte data[12];
  byte type_s;
  uint32_t starttime=millis();
  uint32_t sleeptime=0;

  switch (addr[0]) {
    case 0x10:
      //DEBUGln(F("  Chip = DS18S20"));  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //DEBUGln(F("  Chip = DS18B20"));
      type_s = 0;
      break;
    case 0x22:
      //DEBUGln(F("  Chip = DS1822"));
      type_s = 0;
      break;
    default:
      DEBUGln(F("Device is not a DS18x20 family device."));
  }

  // Set reading resolution
  byte resbyte = 0x1F;
  if (resolution == 12) resbyte = 0x7F;
  else if (resolution == 11) resbyte = 0x5F;
  else if (resolution == 10) resbyte = 0x3F;

  // Set configuration
  myds.reset();
  myds.select(addr);
  myds.write(0x4E);         // Write scratchpad
  myds.write(0);            // TL
  myds.write(0);            // TH
  myds.write(resbyte);         // Configuration Register
  myds.write(0x48);         // Copy Scratchpad
  myds.reset();
  myds.select(addr);
  myds.write(0x44,1);         // start conversion, with parasite power on at the end

  // Sleep while waiting for conversion to complete
  while (!myds.read()) {
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
    sleeptime+=15;
  }

  conversionTime = millis() - starttime+sleeptime;
  DEBUG("Conversion took: "); DEBUG(conversionTime); DEBUGln(" ms");

  present = myds.reset();
  myds.select(addr);    
  myds.write(0xBE);         // Read Scratchpad

  DEBUG("Raw Scratchpad Data: ");
  for (byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = myds.read();
    DEBUGHEX(data[i]);
  }
  DEBUGln();

  // convert the data to actual temperature
  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    } else {
      byte cfg = (data[4] & 0x60);
      // default is 12 bit resolution: ~650 ms conversion time
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution: ~95ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res: ~180ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res: ~370ms
    }
  }

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return conversionTime;
}
