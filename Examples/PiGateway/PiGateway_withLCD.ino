// **********************************************************************************************************
// Moteino gateway/base sketch that works with Moteinos equipped with HopeRF RFM69 transceivers (W/CW/HW/HCW)
// This sketch uses a 2x20 newhaven RGB backlight LCD to display incoming messages from end nodes
// This is a basic gateway sketch that receives packets from end node Moteinos, formats them as ASCII strings
//      with the end node [ID] and passes them to Pi/host computer via serial port
//     (ex: "messageFromNode" from node 123 gets passed to serial as "[123] messageFromNode")
// It also listens to serial messages that should be sent to listening end nodes
//     (ex: "123:messageToNode" sends "messageToNode" to node 123)
// Make sure to adjust the settings to match your transceiver settings (frequency, HW etc).
// 2014 (C) Felix Rusu, http://www.LowPowerLab.com
// **********************************************************************************************************
// License: GPL 3.0, please see the License.txt file in library for details
// https://www.gnu.org/licenses/gpl-3.0.en.html
// **********************************************************************************************************

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
#define NODEID          1 //the ID of this node
#define NETWORKID     200 //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey" //identical 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define ACK_TIME       30  // # of ms to wait for an ack

//*****************************************************************************************************************************
// Serial baud rate must match your Pi/host computer serial port baud rate!
#define SERIAL_EN     //comment out if you don't want any serial verbose output
#define SERIAL_BAUD  19200
//*****************************************************************************************************************************

#define BACKLIGHTPIN    5 //3=R,5=G,6=B

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); //EF40 for 16mbit windbond chip

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
  sprintf(buff, "\nTransmitting at %d Mhz...", radio.getFrequency()/1000000);
  
  DEBUGln(buff);

  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
  {
    DEBUGln("SPI FlashMEM not found (is chip onboard?)");
  }
    
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