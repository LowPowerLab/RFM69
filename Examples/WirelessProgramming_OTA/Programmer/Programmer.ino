// **********************************************************************************
// This sketch is an example of how wireless programming can be achieved with a Moteino
// that was loaded with a custom 1k bootloader (DualOptiboot) that is capable of loading
// a new sketch from an external SPI flash chip
// This is the GATEWAY node, it does not need a custom Optiboot nor any external FLASH memory chip
// (ONLY the target node will need those)
// The sketch includes logic to receive the new sketch from the serial port (from a host computer) and 
// transmit it wirelessly to the target node
// The handshake protocol that receives the sketch from the serial port 
// is handled by the SPIFLash/WirelessHEX69 library, which also relies on the RFM69 library
// These libraries and custom 1k Optiboot bootloader for the target node are at: http://github.com/lowpowerlab
// **********************************************************************************
// (C) 2020 Felix Rusu, LowPowerLab LLC, http://www.LowPowerLab.com/contact
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
#define SKETCH_VERSION "1.0"
// **********************************************************************************
#include <RFM69.h>      //https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>  //https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>  //https://github.com/lowpowerlab/RFM69
#include <EEPROMex.h>   //http://playground.arduino.cc/Code/EEPROMex
#include <Streaming.h>  //easy C++ style output operators: http://arduiniana.org/libraries/streaming/
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
//#define IS_RFM69HW_HCW  //uncomment for RFM69HW/HCW! Leave out for RFM69W/CW!
//*********************************************************************************************
#define NODEID_DEFAULT     1023
#define NETWORKID_DEFAULT  100
#define FREQUENCY_DEFAULT  915000000
#define VALID_FREQUENCY(freq) ((freq >= 430000000 && freq <= 435000000) || (freq >= 860000000 && freq <= 870000000) || (freq >= 902000000 && freq <= 928000000))
#define ENCRYPTKEY_DEFAULT ""
//*********************************************************************************************
#define DEBUG_MODE  false  //'true' = verbose output from programming sequence, ~12% slower OTA!
#define SERIAL_BAUD 115200
#define ACK_TIME    50  // # of ms to wait for an ack
#define TIMEOUT     3000
//*********************************************************************************************
RFM69_ATC radio;
struct config {
  uint8_t NETWORKID;
  uint16_t NODEID;
  uint32_t FREQUENCY;
  uint8_t BR300KBPS;
  char ENCRYPTKEY[17]; //16+nullptr
} CONFIG;

char c = 0;
char input[64]; //serial input buffer
uint16_t targetID=0;
//*********************************************************************************************
void initRadio() {
  radio.initialize(RF69_915MHZ, CONFIG.NODEID, CONFIG.NETWORKID);
  radio.encrypt(CONFIG.ENCRYPTKEY);
  radio.setFrequency(CONFIG.FREQUENCY);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  if (CONFIG.BR300KBPS) {
    radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
    radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
    radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
    radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
    radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
    radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  delay(100);

  EEPROM.setMaxAllowedWrites(10000);
  EEPROM.readBlock(0, CONFIG);

  Serial.println("START OTA programmer...");
  Serial << F("SKETCH_VERSION:") << SKETCH_VERSION << endl;

  //if EEPROM is empty or has invalid values, set/write defaults
  if (resetEEPROMCondition()) resetEEPROM();

  initRadio();

  pinMode(LED_BUILTIN, OUTPUT);
  printSettings();
}

void loop(){
  byte inputLen = readSerialLine(input, 10, 64, 100); //readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=1000);

  if (inputLen > 0) {
    boolean configChanged=false;
    char* colon = strchr(input, ':');

    if (strstr(input, "EEPROMRESET")==input) {
      resetEEPROM();
    } else if (strstr(input, "SETTINGS?")==input) {
      printSettings();
    } else if (strstr(input, "NETWORKID:")==input && strlen(colon+1)>0) {
      uint8_t newNetId = atoi(++colon); //extract ID from message
      if (newNetId <= 255) {
        CONFIG.NETWORKID=newNetId; 
        configChanged=true;
      } else {
        Serial << F("Invalid networkId:") << newNetId << endl;
      }
    } else if (strstr(input, "NODEID:")==input && strlen(colon+1)>0) {
      uint16_t newId = atoi(++colon); //extract ID from message
      if (newId <= 1023) {
        CONFIG.NODEID=newId; 
        configChanged=true;
      } else {
        Serial << F("Invalid nodeId:") << newId << endl;
      }
    } else if (strstr(input, "FREQUENCY:")==input && strlen(colon+1)>0) {
      uint32_t newFreq = atol(++colon); //extract ID from message
      if (VALID_FREQUENCY(newFreq)) {
        CONFIG.FREQUENCY=newFreq; 
        configChanged=true;
      } else {
        Serial << F("Invalid frequency:") << newFreq << endl;
      }
    } else if (strstr(input, "ENCRYPTKEY:")==input) {
      if (strlen(colon+1)==16) {
        strcpy(CONFIG.ENCRYPTKEY, colon+1);
        configChanged=true;
      } else if (strlen(colon+1)==0) {
        strcpy(CONFIG.ENCRYPTKEY, "");
        configChanged=true;
      }
      else Serial << F("Invalid encryptkey length:") << colon+1 << "(" << strlen(colon+1) << F("expected:16)") << endl;
    } else if (strstr(input, "BR300KBPS:")==input && strlen(colon+1)>0) {
      uint8_t newBR = atoi(++colon); //extract ID from message
      if (newBR==0 || newBR==1) {
        CONFIG.BR300KBPS=newBR; 
        configChanged=true;
      } else {
        Serial << F("Invalid BR300KBPS:") << newBR << endl;
      }
    } else if (inputLen==4 && strstr(input, "FLX?")==input) {
      if (targetID==0)
        Serial.println("TO?");
      else
        CheckForSerialHEX((byte*)input, inputLen, radio, targetID, TIMEOUT, ACK_TIME, DEBUG_MODE);
    } else if (strstr(input, "TO:")==input && strlen(colon+1)>0) {
      uint16_t newTarget=atoi(++colon);
      if (newTarget>0 && newTarget <=1023)
      {
        targetID = newTarget;
        Serial << F("TO:") << targetID << F(":OK") << endl;
      }
      else Serial << input << F(":INV") << endl;
    } else Serial << F("UNKNOWN_CMD: ") << input << endl; //echo back un

    if (configChanged) {
      EEPROM.writeBlock(0, CONFIG); //save changes to EEPROM
      printSettings();
      initRadio();
    }
  }

  if (radio.receiveDone())
  {
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    
    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    
    Serial.println();
  }
  Blink(1); //heartbeat
}

boolean resetEEPROMCondition() {
  //conditions for resetting EEPROM:
  return CONFIG.NETWORKID > 255 ||
    CONFIG.NODEID > 1023 ||
    !VALID_FREQUENCY(CONFIG.FREQUENCY);
}

void resetEEPROM() {
  Serial.println("Resetting EEPROM to default values...");
  CONFIG.NETWORKID=NETWORKID_DEFAULT;
  CONFIG.NODEID=NODEID_DEFAULT;
  CONFIG.FREQUENCY=FREQUENCY_DEFAULT;
  CONFIG.BR300KBPS=false;
  strcpy(CONFIG.ENCRYPTKEY, ENCRYPTKEY_DEFAULT);
  EEPROM.writeBlock(0, CONFIG);
}

void printSettings() {
  Serial << endl << F("NETWORKID:") << CONFIG.NETWORKID << endl;
  Serial << F("NODEID:") << CONFIG.NODEID << endl;
  Serial << F("FREQUENCY:") << CONFIG.FREQUENCY << endl;
  Serial << F("BR300KBPS:") << CONFIG.BR300KBPS << endl;
  Serial << F("ENCRYPTKEY:") << CONFIG.ENCRYPTKEY << endl;
}

void Blink(int DELAY_MS)
{
  digitalWrite(LED_BUILTIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(LED_BUILTIN,LOW);
}
