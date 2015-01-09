// **********************************************************************************************************
// GarageMote garage door controller base receiver sketch that works with Moteinos equipped with HopeRF RFM69W/RFM69HW
// Can be adapted to use Moteinos using RFM12B
// This is the sketch for the base, not the controller itself, and meant as another example on how to use a
// Moteino as a gateway/base/receiver
// 2014-07-14 (C) felix@lowpowerlab.com, http://www.LowPowerLab.com
// **********************************************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code/library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/4.0/
// **********************************************************************************

#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define NODEID          1
#define NETWORKID     200
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "thisIsEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED             9
#define FLASH_CS        8
#define SERIAL_BAUD 115200
#define SERIAL_EN     //comment out if you don't want any serial verbose output
#define ACK_TIME       30  // # of ms to wait for an ack
//*****************************************************************************************************************************

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
SPIFlash flash(FLASH_CS, 0xEF30); //EF40 for 16mbit windbond chip

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
    DEBUGln("SPI Flash Init FAIL! (is chip present?)");
}

byte ackCount=0;
byte inputLen=0;
char input[64];
byte buff[61];
String inputstr;
void loop() {
  inputLen = readSerialLine(input);
  inputstr = String(input);
  inputstr.toUpperCase();
  
  if (inputLen > 0)
  {
    if (inputstr.equals("KEY?"))
    {
      DEBUG("ENCRYPTKEY:");
      DEBUG(ENCRYPTKEY);
    }
    
    byte targetId = inputstr.toInt(); //extract ID if any
    byte colonIndex = inputstr.indexOf(":"); //find position of first colon
    if (targetId > 0) inputstr = inputstr.substring(colonIndex+1); //trim "ID:" if any
    if (targetId > 0 && targetId != NODEID && targetId != RF69_BROADCAST_ADDR && colonIndex>0 && colonIndex<4 && inputstr.length()>0)
    {
      
      inputstr.getBytes(buff, 61);
      //DEBUGln((char*)buff);
      //DEBUGln(targetId);
      //DEBUGln(colonIndex);
      if (radio.sendWithRetry(targetId, buff, inputstr.length()))
      {
        DEBUGln("ACK:OK");
      }
      else
        DEBUGln("ACK:NOK");
    }
  }

  if (radio.receiveDone())
  {
    int rssi = radio.RSSI;
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    if (radio.DATALEN > 0)
    {
      for (byte i = 0; i < radio.DATALEN; i++)
        DEBUG((char)radio.DATA[i]);
      DEBUG("   [RSSI:");DEBUG(rssi);DEBUG("]");
    }

    CheckForWirelessHEX(radio, flash, false); //non verbose DEBUG

    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      DEBUG("[ACK-sent]");
    }
    DEBUGln();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

//readSerialLine already defined in WirelessHEX69
// reads a line feed (\n) terminated line from the serial stream
// returns # of bytes read, up to 255
// timeout in ms, will timeout and return after so long
//byte readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=10);
//byte readSerialLine(char* input, char endOfLineChar, byte maxLength, uint16_t timeout)
//{
//  byte inputLen = 0;
//  Serial.setTimeout(timeout);
//  inputLen = Serial.readBytesUntil(endOfLineChar, input, maxLength);
//  input[inputLen]=0;//null-terminate it
//  Serial.setTimeout(0);
//  //Serial.println();
//  return inputLen;
//}