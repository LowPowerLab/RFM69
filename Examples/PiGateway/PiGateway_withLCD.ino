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
#include "ST7036.h"        //get it from here: https://bitbucket.org/fmalpartida/st7036-display-driver/src/
#include "LCD_C0220BiZ.h"  //get it from here: https://bitbucket.org/fmalpartida/st7036-display-driver/src/
#include <Wire.h>          //comes with Arduino

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
#define BACKLIGHTPIN    5 //3=R,5=G,6=B
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

//initialize LCD
ST7036 lcd = ST7036(2, 20, 0x78, BACKLIGHTPIN); //row count, column count, I2C addr, pin for backlight PWM
byte battChar[8] = {0b00000,0b01110,0b11111,0b11111,0b11111,0b11111,0b11111,0};
byte rssiChar[8] = {0b00000,0b00100,0b10101,0b01110,0b00100,0b00100,0b00100,0};

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  char buff[50];
  sprintf(buff, "\nListening @ %dmhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
    DEBUGln("SPI Flash Init FAIL! (is chip present?)");
    
  lcd.init();
  lcd.setContrast(10);
  lcd.clear();
  lcd.load_custom_character(0, battChar);
  lcd.load_custom_character(1, rssiChar);
  lcd.setCursor(0,0);
  lcd.print(buff);
}

byte ackCount=0;
byte inputLen=0;
char input[64];
byte buff[61];
char LO[20];
char BAT[20];
char temp[25];
String inputstr;
void loop() {
  inputLen = readSerialLine(input, 10, 64, 10); //readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=10);
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
    
    lcd.clear();
    lcd.setCursor(0,0);

    //if (radio.DATALEN < RF69_MAX_DATA_LEN) radio.DATA[radio.DATALEN]=0;
    byte matches = sscanf((const char*)radio.DATA, "%s BAT:%s", LO, BAT);
    if (matches==2)
    {
      lcd.print(LO);
      lcd.setCursor(0,14);
      lcd.print(char(0));
      lcd.setCursor(0,15);
      lcd.print(BAT);
    }
    else lcd.print((const char*)radio.DATA);
    
    lcd.setCursor(1,14);
    lcd.print(char(1));
    lcd.setCursor(1,16);
    lcd.print(rssi);
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
