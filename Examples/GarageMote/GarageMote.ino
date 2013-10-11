// **********************************************************************************************************
// GarageMote garage door controller sketch that works with Moteinos equipped with HopeRF RFM69W/RFM69HW
// Can be adapted to use Moteinos using RFM12B
// 2013-09-13 (C) felix@lowpowerlab.com, http://www.LowPowerLab.com
// **********************************************************************************************************
// It uses 2 hall effect sensors (and magnets mounted on the garage belt/chain) to detect the position of the
// door, and a small signal relay to be able to toggle the garage opener.
// Implementation details are posted at the LowPowerLab blog
// Door status is reported via RFM69 to a base Moteino, and visually on the onboard Moteino LED:
//    - solid ON - door is in open position
//    - solid OFF - door is in closed position
//    - blinking - door is not in either open/close position
//    - pulsing - door is in motion
// **********************************************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code/library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/3.0/
// **********************************************************************************************************

#include <RFM69.h>  //install this library in your Arduino library directory from https://github.com/LowPowerLab/RFM69
#include <SPI.h>

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define GATEWAYID   1
#define NODEID      99
#define NETWORKID   100
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "thisIsEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!

#define HALLSENSOR1          A0
#define HALLSENSOR1_EN        4
#define HALLSENSOR2          A1
#define HALLSENSOR2_EN        5

#define RELAYPIN1             6
#define RELAYPIN2             7
#define RELAY_PULSE_MS      250  //just enough that the opener will pick it up

#define DOOR_MOVEMENT_TIME 14000 // this has to be at least as long as the max between [door opening time, door closing time]
                                 // my door opens and closes in about 12s
#define STATUS_CHANGE_MIN  1500  // this has to be at least as long as the delay 
                                 // between a opener button press and door movement start
                                 // most garage doors will start moving immediately (within half a second)
//*****************************************************************************************************************************
#define HALLSENSOR_OPENSIDE   0
#define HALLSENSOR_CLOSEDSIDE 1

#define STATUS_CLOSED        0
#define STATUS_CLOSING       1
#define STATUS_OPENING       2
#define STATUS_OPEN          3
#define STATUS_UNKNOWN       4

#define LED                  9   //pin connected to onboard LED
#define LED_PULSE_PERIOD  5000   //5s seems good value for pulsing/blinking (not too fast/slow)
#define SERIAL_BAUD     115200
#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

void setStatus(byte newSTATUS, boolean reportStatus=true);
byte STATUS;
long lastStatusTimestamp=0;
byte lastRequesterNodeID=0;
long ledPulseTimestamp=0;
int ledPulseValue=0;
boolean ledPulseDirection=false; //false=down, true=up
RFM69 radio;

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  pinMode(HALLSENSOR1, INPUT);
  pinMode(HALLSENSOR2, INPUT);
  pinMode(HALLSENSOR1_EN, OUTPUT);
  pinMode(HALLSENSOR2_EN, OUTPUT);
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower(); //must uncomment only for RFM69HW!
  radio.encrypt(KEY);

  char buff[50];
  sprintf(buff, "GarageMote : %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
    setStatus(STATUS_OPEN);
  if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
    setStatus(STATUS_CLOSED);
  else setStatus(STATUS_UNKNOWN);
}

long doorPulseCount = 0;
char input;

void loop()
{
  if (Serial.available())
    input = Serial.read();
    
  if (input=='r')
  {
    Serial.println("Relay test...");
    pulseRelay();
    input = 0;
  }
    
  // UNKNOWN => OPEN/CLOSED
  if (STATUS == STATUS_UNKNOWN && millis()-lastStatusTimestamp>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
  }

  // OPEN => CLOSING  
  if (STATUS == STATUS_OPEN && millis()-lastStatusTimestamp>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==false)
      setStatus(STATUS_CLOSING);
  }

  // CLOSED => OPENING  
  if (STATUS == STATUS_CLOSED && millis()-lastStatusTimestamp>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==false)
      setStatus(STATUS_OPENING);
  }

  // OPENING/CLOSING => OPEN (when door returns to open due to obstacle or toggle action)
  //                 => CLOSED (when door closes normally from OPEN)
  //                 => UNKNOWN (when more time passes than normally would for a door up/down movement)
  if ((STATUS == STATUS_OPENING || STATUS == STATUS_CLOSING) && millis()-lastStatusTimestamp>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    else if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
    else if (millis()-lastStatusTimestamp>DOOR_MOVEMENT_TIME)
      setStatus(STATUS_UNKNOWN);
  }
  
  if (radio.receiveDone())
  {
    byte newStatus=STATUS;
    boolean reportStatusRequest=false;
    lastRequesterNodeID = radio.SENDERID;
    DEBUG('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    //check for an OPEN/CLOSE/STATUS request
    if (radio.DATA[0]=='O' && radio.DATA[1]=='P' && radio.DATA[2]=='N')
    {
      if (millis()-lastStatusTimestamp > STATUS_CHANGE_MIN && (STATUS == STATUS_CLOSED || STATUS == STATUS_CLOSING || STATUS == STATUS_UNKNOWN))
        newStatus = STATUS_OPENING;
      //else radio.Send(requester, "INVALID", 7);
    }
    if (radio.DATA[0]=='C' && radio.DATA[1]=='L' && radio.DATA[2]=='S')
    {
      if (millis()-lastStatusTimestamp > STATUS_CHANGE_MIN && (STATUS == STATUS_OPEN || STATUS == STATUS_OPENING || STATUS == STATUS_UNKNOWN))
        newStatus = STATUS_CLOSING;
      //else radio.Send(requester, "INVALID", 7);
    }
    if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='S')
    {
      reportStatusRequest = true;
    }

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.readRSSI());DEBUG("]");
    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }
    
    //now take care of the request, if not invalid
    if (STATUS != newStatus)
    {
      pulseRelay();
      setStatus(newStatus);
    }
    if (reportStatusRequest)
    {
      reportStatus();
    }
      
    DEBUGln();
  }
  
  //use LED to visually indicate STATUS
  if (STATUS == STATUS_OPEN || STATUS == STATUS_CLOSED) //solid ON/OFF
  {
    digitalWrite(LED, STATUS == STATUS_OPEN ? LOW : HIGH);
  }
  if (STATUS == STATUS_OPENING || STATUS == STATUS_CLOSING) //pulse
  {
    if (millis()-ledPulseTimestamp > LED_PULSE_PERIOD/256)
    {
      ledPulseValue = ledPulseDirection ? ledPulseValue + LED_PULSE_PERIOD/256 : ledPulseValue - LED_PULSE_PERIOD/256;

      if (ledPulseDirection && ledPulseValue > 255)
      {
        ledPulseDirection=false;
        ledPulseValue = 255;
      }
      else if (!ledPulseDirection && ledPulseValue < 0)
      {
        ledPulseDirection=true;
        ledPulseValue = 0;
      }
      
      analogWrite(LED, ledPulseValue);
      ledPulseTimestamp = millis();
    }
  }
  if (STATUS == STATUS_UNKNOWN) //blink
  {
    if (millis()-ledPulseTimestamp > LED_PULSE_PERIOD/20)
    {
      ledPulseDirection = !ledPulseDirection;
      digitalWrite(LED, ledPulseDirection ? HIGH : LOW);
      ledPulseTimestamp = millis();
    }
  }
}

//returns TRUE if magnet is next to sensor, FALSE if magnet is away
boolean hallSensorRead(byte which)
{
  //while(millis()-lastStatusTimestamp<STATUS_CHANGE_MIN);
  digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, HIGH); //turn sensor ON
  delay(1); //wait a little
  byte reading = digitalRead(which ? HALLSENSOR2 : HALLSENSOR1);
  digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, LOW); //turn sensor OFF
  return reading==0;
}

void setStatus(byte newSTATUS, boolean reportStatusRequest)
{
  if (STATUS != newSTATUS) lastStatusTimestamp = millis();
  STATUS = newSTATUS;
  DEBUGln(STATUS==STATUS_CLOSED ? "CLOSED" : STATUS==STATUS_CLOSING ? "CLOSING" : STATUS==STATUS_OPENING ? "OPENING" : STATUS==STATUS_OPEN ? "OPEN" : "UNKNOWN");
  if (reportStatusRequest)
    reportStatus();
}

boolean reportStatus()
{
  if (lastRequesterNodeID == 0) return false;
  char buff[10];
  sprintf(buff, STATUS==STATUS_CLOSED ? "CLOSED" : STATUS==STATUS_CLOSING ? "CLOSING" : STATUS==STATUS_OPENING ? "OPENING" : STATUS==STATUS_OPEN ? "OPEN" : "UNKNOWN");
  byte len = strlen(buff);
  return radio.sendWithRetry(lastRequesterNodeID, buff, len);
}

void pulseRelay()
{
  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);
  delay(RELAY_PULSE_MS);
  digitalWrite(RELAYPIN1, LOW);
  digitalWrite(RELAYPIN2, LOW);
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
