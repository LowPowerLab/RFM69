// **********************************************************************************************************
// Moteino gateway/base sketch that works with Moteinos equipped with RFM69 transceiver
// This is a basic gateway sketch that receives packets from end node Moteinos, formats them as ASCII strings
//      with the end node [ID] and passes them to Pi/host computer via serial port
//     (ex: "messageFromNode" from node 123 gets passed to serial as "[123] messageFromNode")
// It also listens to serial messages that should be sent to listening end nodes
//     (ex: "123:messageToNode" sends "messageToNode" to node 123)
// Make sure to adjust the settings to match your transceiver settings (frequency, HW etc).
// **********************************************************************************
// Copyright Felix Rusu 2020, http://www.LowPowerLab.com/contact
// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID          1 //the ID of this node
#define NETWORKID     200 //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey" //identical 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW  //required for RFM69HW/HCW, comment out for RFM69W/CW!
#define ACK_TIME       30  // # of ms to wait for an ack packet
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -90  //target RSSI for RFM69_ATC (recommended > -80)
//*****************************************************************************************************************************
// Serial baud rate must match your Pi/host computer serial port baud rate!
#define DEBUG_EN     //comment out if you don't want any serial verbose output
#define SERIAL_BAUD   19200
//*****************************************************************************************************************************
#ifdef DEBUG_EN
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#define LED_HIGH digitalWrite(LED_BUILTIN, HIGH)
#define LED_LOW digitalWrite(LED_BUILTIN, LOW)

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit Windbond FlashMEM chip

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif
  
  char buff[50];
  sprintf(buff, "\nDEBUG:Transmitting at %d Mhz...", radio.getFrequency()/1000000);
  DEBUGln(buff);

  if (flash.initialize())
  {
    DEBUGln("DEBUG:SPI Flash Init OK!");
  }
  else
  {
    DEBUGln("DEBUG:SPI FlashMEM not found (is chip onboard?)");
  }
}

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
    byte targetId = inputstr.toInt(); //extract ID if any
    byte colonIndex = inputstr.indexOf(":"); //find position of first colon

    if (targetId > 0)
    {
      inputstr = inputstr.substring(colonIndex+1); //trim "ID:" if any
    }

    if (targetId > 0 && targetId != NODEID && targetId != RF69_BROADCAST_ADDR && colonIndex>0 && colonIndex<4 && inputstr.length()>0)
    {
      inputstr.getBytes(buff, 61);
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
    LED_HIGH;
    int rssi = radio.RSSI;
    Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
    if (radio.DATALEN > 0)
    {
      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i]);
      Serial.print("   [RSSI:");Serial.print(rssi);Serial.print(']');
    }
    Serial.println();
    
    CheckForWirelessHEX(radio, flash, false); //non verbose DEBUG

    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUGln("DEBUG:ACK-sent");
    }
    
    LED_LOW;
  }
}