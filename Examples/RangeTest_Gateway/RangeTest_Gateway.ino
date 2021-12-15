//*********************************************************************************************
// RangeTest_Gateway (to be used with an RFGateway or similar Moteino)
// Adapted from Gateway firmware sketch
// Acts as a listener for testing RF performance, in tandem with a RangeTest_Transmitter
//*********************************************************************************************
// Copyright Felix Rusu 2021, http://www.LowPowerLab.com/contact
//*********************************************************************************************
#include <RFM69.h>     //https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h> //https://www.github.com/lowpowerlab/rfm69
#include <U8g2lib.h>   //https://github.com/olikraus/u8g2/wiki/u8g2reference fonts:https://github.com/olikraus/u8g2/wiki/fntlistall
#include <Wire.h>      //i2c scanner: https://playground.arduino.cc/Main/I2cScanner
#include <PString.h>   //easy string manipulator: http://arduiniana.org/libraries/pstring/
//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID          1    //unique for each node on same network
#define NETWORKID       100  //the same on all nodes that talk to each other
#define FREQUENCY       RF69_915MHZ  //others: RF69_433MHZ, RF69_868MHZ, match to radio variant on your Moteino/board
#define ENCRYPTKEY      "sampleEncryptKey"
#define IS_RFM69HW_HCW  //assumes RFM69 HCW/HW, remove if you have RFM69 W/CW
#define ENABLE_ATC      //comment out to disable AUTO TRANSMISSION CONTROL (ie. always transmit at max power)
//*********************************************************************************************
#define SERIAL_BAUD   500000      //this works with 0% error on all LowPowerLab boards, for SAMD it is irrelevant
#define LED           LED_BUILTIN
#define BUZZER        6
//***********************************************************************************************************
#define OLED_BAUD                   1600000 //fast i2c clock
#define OLED_ADDRESS                0x3C    //i2c address on most small OLEDs
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//***********************************************************************************************************
#define LED_HIGH digitalWrite(LED, HIGH)
#define LED_LOW digitalWrite(LED, LOW)

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#ifdef SERIAL_BAUD
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
  #define DEBUGHex(input) Serial.print(input, HEX)
  #define DEBUGFlush()   Serial.flush()
#else
  #define DEBUG(input)
  #define DEBUGln(input)
  #define DEBUGHex(input)
  #define DEBUGFlush()
#endif

#define MSG_MAX_LEN   30  //don't need more for testing purposes
#define HISTORY_LEN   9   //hold this many past messages - to display on OLED at small font size

byte OLEDfound=false;
char buff[61];
PString Pbuff(buff, sizeof(buff)); //easy string manipulator
byte ackCount=0;
byte lastMessageIndex = HISTORY_LEN;
byte historyLength = 0;
uint16_t senderID;
int rxRSSI;

typedef struct {
  char msg[MSG_MAX_LEN];
  int rssi;
  uint16_t from;
  byte pinged;
  byte pingedOK;
} Message;
Message * messageHistory = new Message[HISTORY_LEN];

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower();
#endif
  radio.encrypt(ENCRYPTKEY);
  DEBUG(F("Listening at ")); DEBUG(radio.getFrequency()); DEBUGln(F(" Hz.."));
  DEBUGln(buff);

  Pbuff = F("Listening at ");
  Pbuff.print(radio.getFrequency());
  Pbuff.print(F("Hz..."));
  DEBUGln(buff);

  delay(1000); //Wire apparently needs 50ms
  Wire.begin();
  Wire.beginTransmission(OLED_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0) {
    DEBUG(F("OLED FOUND at 0x")); DEBUGln(OLED_ADDRESS);
    u8g2.begin();
    u8g2.setDisplayRotation(U8G2_R2); //if required (inside/custom mount?)
    u8g2.setBusClock(OLED_BAUD);
    OLEDfound = true;
  } else { 
    DEBUGln(F("NO OLED found..."));
  }

  DEBUGFlush();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13B_tf);
  u8g2.setCursor(0,10); u8g2.print("OLED PKT CAPTURE");  
  u8g2.sendBuffer();
  delay(1000);
}

void loop() {
  if (radio.receiveDone()) {
    LED_HIGH;
    senderID = radio.SENDERID;
    rxRSSI = radio.RSSI;

    DEBUG('[');DEBUG(senderID);DEBUG(F("] "));
    DEBUG((char*)radio.DATA);
    DEBUG(F(" [RX_RSSI:"));DEBUG(rxRSSI);DEBUG(F("]"));

    Pbuff = "";
    Pbuff.print(radio.SENDERID);
    Pbuff.print(' ');
    Pbuff.print((char*)radio.DATA);
    Pbuff.print(' ');
    Pbuff.print(radio.RSSI);

    if (radio.ACKRequested()) {
      radio.sendACK();

      if (++ackCount%3==0) {
        DEBUG(F(" Pinging ["));
        DEBUG(senderID);
        DEBUG(F("]:"));
        delay(3); //need this when sending right after reception .. ?
        //if (radio.sendWithRetry(senderID, "ACK TEST", 8, 0)) { // 0 = only 1 attempt, no retries
        if (radio.sendWithRetry(senderID, "ACK TEST", 8)) {
          DEBUG(F("OK!"));
          Pbuff.print(" OK");
        } else {
          DEBUG(F("nothing"));
          Pbuff.print(" NOK");
        }
      }
    }
    DEBUGln();
    saveToHistory(buff);
    DEBUGln();
    if (OLEDfound) draw();
    DEBUGln();
    LED_LOW;
  }

  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void draw() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tf);

  byte offset=0;
  for (byte i=lastMessageIndex+1; i<historyLength; i++) {
    offset = 7 + (i-lastMessageIndex-1)*7;
    u8g2.setCursor(0, offset); u8g2.print(messageHistory[i].msg);
    //DEBUG(F("L1_i=")); DEBUG(i); DEBUG(F("_offset=")); DEBUGln(offset);
  }
  offset+=7;
  for (byte i=0; i<=lastMessageIndex; i++) {
    u8g2.setCursor(0, offset + i*7); u8g2.print(messageHistory[i].msg);
    //DEBUG(F("L2_i=")); DEBUG(i); DEBUG(F("_offset=")); DEBUG(offset); DEBUG(F("_x=")); DEBUGln(offset + i*7);
  }

  u8g2.sendBuffer();
}

void saveToHistory(char * msg) {
  byte length = strlen(msg);
  if (length == 0) return;

  if (historyLength < HISTORY_LEN) historyLength++;
  if (lastMessageIndex >= HISTORY_LEN-1) //reset pointed in circular buffer/array
    lastMessageIndex = 0;
  else
    lastMessageIndex++;

  strcpy(messageHistory[lastMessageIndex].msg, msg);
  DEBUG(F("HIST ADDED ["));
  DEBUG(lastMessageIndex);
  DEBUG(F("]["));
  DEBUG((char*)messageHistory[lastMessageIndex].msg);
  DEBUG(F("]"));
}
