// Sample RFM69 sender/node sketch for the SonarMote - Distance tracker
// Can be used for inventory control - ex to measure distance in a multi lane cigarette pack rack
// http://lowpowerlab.com/sonarmote
// Ultrasonic sensor (HC-SR04) connected to D6 (Trig), D7 (Echo), and power enabled through D5
// This sketch sleeps the Moteino and sensor most of the time. It wakes up every few seconds to take
//   a distance reading. If it detects an approaching object (car) it increases the sampling rate
//   and starts lighting up the LED (from green to yellow to red to blinking red). Once there is no more
//   motion the LED is turned off and the cycle is back to a few seconds in between sensor reads.
// Button is connected on D3. Holding the button for a few seconds enters the "red zone adjust" mode (RZA).
//   By default the red zone limit is at 25cm (LED turns RED below this and starts blinking faster and faster).
//   In RZA, readings are taken for 5 seconds. In this time you have the chance to set a new red zone limit.
//   Valid new red zone readings are between the RED__LIMIT_UPPER (default 25cm) and MAX_ADJUST_DISTANCE (cm).
//   In RZA mode the BLU Led blinks fast to indicate new red limit distance. It blinks slow if the readings are invalid
//   If desired this value could be saved to EEPROM to persist if unit is turned off
// Get the RFM69 at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
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
#include <RFM69.h>     //https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h> //https://github.com/lowpowerlab/rfm69
#include <LowPower.h>  //https://github.com/lowpowerlab/lowpower
#include <SPIFlash.h>  //https://www.github.com/lowpowerlab/spiflash
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        22   //unique for each node on same network
#define GATEWAYID     1    //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
//#define FREQUENCY_EXACT 916000000
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Remove/comment if you have RFM69W/CW!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send
#define READ_SAMPLES 3
#define HISTERESIS   1.3  //(cm) only send a message when new reading is this many centimeters different
#define DIST_READ_LOOPS 2 //read distance every this many sleeping loops (ie if sleep time is 8s then 2 loops => a read occurs every 16s)
//*********************************************************************************************
#define BUZZER_ENABLE  //uncomment this line if you have the BUZZER soldered and want the sketch to make sounds
//#define SERIAL_EN      //uncomment if you want serial debugging output
//*********************************************************************************************
#define TRIG           6  // digital pin wired to TRIG pin of ultrasonic sensor
#define ECHO           7  // digital pin wired to ECHO pin of ultrasonic sensor
#define SENSOR_EN      5  // digital pin that enables power to ultrasonic sensor
#define BUZZER         4  // digital pin that is connected to onboard buzzer
#define MAX_DISTANCE 150  // maximum valid distance
#define MIN_DISTANCE   2  // minimum valid distance
#define MAX_ADJUST_DISTANCE (MAX_DISTANCE-GRN_LIMIT_UPPER)   //this is the amount by which the RED_LIMIT_UPPER can by increased
//*********************************************************************************************
#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
  #define DEBUGFlush() Serial.flush()
#else
  #define DEBUG(input)
  #define DEBUGln(input)
  #define DEBUGFlush()
#endif
//*********************************************************************************************
#define BATT_MONITOR      A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_READ_LOOPS   SENDLOOPS  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
#define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
//*********************************************************************************************
byte sendLen;
byte sendLoops=0;
byte distReadLoops=0;
byte battReadLoops=0;
float distance=0;
float prevDistance=0;
float batteryVolts = 5;
char buff[50]; //this is just an empty string used as a buffer to place the payload for the radio
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
char* DISTstr="99999.99cm"; //longest distance reading message = 5chars
float readDistance(byte samples=1);  //take 1 samples by default
SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

void setup() {
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD); // Open serial monitor at 115200 baud to see ping results.
#endif

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
#ifdef ENCRYPTKEY
  radio.encrypt(ENCRYPTKEY);
#endif
#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif
#ifdef ENABLE_ATC
    radio.enableAutoPower();
#endif
  sprintf(buff, "\nTransmitting at %l Hz, id:%d nid:%d gid:%d", radio.getFrequency(), NODEID, NETWORKID, GATEWAYID);
  DEBUG(buff);
  for (byte i=0;i<strlen(ENCRYPTKEY);i++) DEBUG(ENCRYPTKEY[i]);
  DEBUGln();
  radio.sleep();

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_EN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(SENSOR_EN, LOW);
#ifdef BUZZER_ENABLE
  pinMode(BUZZER, OUTPUT);
  buzzer(50,2,100);
#endif

  if (flash.initialize()) flash.sleep();
  radio.sendWithRetry(GATEWAYID, "START", 5);
  DEBUGFlush();
  readDistance(); //first reading seems to always be low
  readBattery();
}

void loop() {
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (distReadLoops--<=0)
  {
    distance = readDistance(READ_SAMPLES);
    distReadLoops = DIST_READ_LOOPS-1;
  }

  float diff = distance - prevDistance;
  if ((diff > HISTERESIS || diff < -HISTERESIS) || (sendLoops--<=0)) //only send a new message if the distance has changed more than the HISTERESIS or if sendloops has expired
  {
    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
      DISTstr = "0"; // zero, out of range
    else dtostrf(distance,3,2, DISTstr);

    if (diff > HISTERESIS || diff < -HISTERESIS)
      sprintf(buff, "%scm BAT:%s", DISTstr, BATstr); //send both distance and battery readings
    else
      sprintf(buff, "BAT:%s", BATstr); //distance has not changed significantly so only send last battery reading
    sendLen = strlen(buff);

    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG(buff);
    if (radio.sendWithRetry(GATEWAYID, buff, sendLen))
    {
      prevDistance = distance;
      DEBUG(" - ACK:OK! RSSI:");
      DEBUGln(radio.RSSI);
    }
    else DEBUGln(" - ACK:NOK...");
    digitalWrite(LED_BUILTIN, LOW);
    sendLoops = SENDLOOPS-1; //reset send loop counter
  }

  radio.sleep();
  DEBUGFlush();

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //sleep the microcontroller
}

float readDistance(byte samples)
{
  if (samples == 0) samples = 1;
  if (samples > 10) samples = 10;
  digitalWrite(SENSOR_EN, HIGH);
  //need about 60-75ms after power up before HC-SR04 will be usable, so just sleep in the meantime
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  PING();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  unsigned long duration = 0;
  for (byte i=0; i<samples; i++)
  {
    duration += PING();
    if (samples > 1) LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  }
  digitalWrite(SENSOR_EN, LOW);
  return microsecondsToCentimeters(duration / samples);
}

long PING()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH);
}

void readBattery()
{
  unsigned int readings=0;
  for (byte i=0; i<5; i++) //take several samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
}

float microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74.0 / 2.0f;
}

float microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return (float)microseconds / 29.0f / 2.0f;
}

#ifdef BUZZER_ENABLE
void buzzer(byte soundTime, byte repeats, byte repeatsDelay)
{
  for (byte i=0;i<=repeats;i++)
  {
    tone(BUZZER, 4500); //4500hz makes a nice audible sound from a 3.3v Moteino digital pin
    delay(soundTime);
    noTone(BUZZER);
    if (repeats>0) delay(repeatsDelay);
  }
}
#endif