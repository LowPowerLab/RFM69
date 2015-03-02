// Sample RFM69 sender/node sketch for the SonarMote - Distance tracker
// Can be used for inventory control - ex to measure distance in a multi lane cigarette pack rack
// More info/photos at: http://lowpowerlab.com/sonar
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
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
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
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <SPI.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h> //get it here: https://github.com/LowPowerLab/SPIFlash
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        22    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send
#define READ_SAMPLES 3
//*********************************************************************************************
//#define BUZZER_ENABLE  //uncomment this line if you have the BUZZER soldered and want the sketch to make sounds
#define SERIAL_EN         //uncomment if you want serial debugging output
//*********************************************************************************************
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8
#endif
#define TRIG           6  // digital pin wired to TRIG pin of ultrasonic sensor
#define ECHO           7  // digital pin wired to ECHO pin of ultrasonic sensor
#define SENSOR_EN      5  // digital pin that enables power to ultrasonic sensor
#define BUZZER         4  // digital pin that is connected to onboard buzzer
#define MAX_DISTANCE 150  // maximum valid distance
#define MIN_DISTANCE   2  // minimum valid distance
#define MAX_ADJUST_DISTANCE (MAX_DISTANCE-GRN_LIMIT_UPPER)   //this is the amount by which the RED_LIMIT_UPPER can by increased

//  
#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif

#define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_CYCLES   SENDLOOPS  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
#define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_LOW      3.3

byte sendLen;
byte sendLoops=SENDLOOPS;
float distance=0;
float prevDistance=0;
float batteryVolts = 5;
char buff[50]; //this is just an empty string used as a buffer to place the payload for the radio
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
char* DISTstr="99999.99cm"; //longest distance reading message = 5chars
void checkBattery(byte samples=10);    //take 10 samples by default
float readDistance(byte samples=1);    //take 1 samples by default
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
RFM69 radio;

void setup() {
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD); // Open serial monitor at 115200 baud to see ping results.
#endif

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  //sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  sprintf(buff, "\nTransmitting at %d Mhz, id:%d nid:%d gid:%d", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915, NODEID, NETWORKID, GATEWAYID);
  DEBUG(buff);
  for (byte i=0;i<strlen(ENCRYPTKEY);i++) DEBUG(ENCRYPTKEY[i]);
  DEBUGln();
  radio.sleep();
  if (flash.initialize()) flash.sleep();

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(SENSOR_EN, OUTPUT);
  digitalWrite(SENSOR_EN, LOW);
#ifdef BUZZER_ENABLE
  pinMode(BUZZER, OUTPUT);
  buzzer(50,2,100);
#endif

  SERIALFLUSH();
  readDistance(); //first reading seems to always be low
}

void loop() {
  checkBattery();
  distance = readDistance(READ_SAMPLES);

  float diff = distance - prevDistance;
  if ((diff > 1 || diff < -1) || (--sendLoops==0)) //only send a new message if the distance has changed by at least 1cm
  {
    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
      DISTstr = "0"; // zero, out of range
    else dtostrf(distance,3,2, DISTstr);

    sprintf(buff, "%scm BAT:%s", DISTstr, BATstr);
    sendLen = strlen(buff);

    digitalWrite(LED, HIGH);
    DEBUG(buff);
    if (radio.sendWithRetry(GATEWAYID, buff, sendLen))
    {
      prevDistance = distance;
      DEBUG(" - ACK:OK! RSSI:");
      DEBUGln(radio.RSSI);
    }
    else DEBUGln(" - ACK:NOK...");
    digitalWrite(LED, LOW);
    sendLoops = SENDLOOPS; //reset loop counter
  }
  radio.sleep();
  SERIALFLUSH();

//  if (radio.sendWithRetry(1, "123 TEST", 8))
//  {
//    //prevDistance = distance;
//    DEBUG(" - ACK:OK! RSSI:");
//    DEBUGln(radio.RSSI);
//  }
//  else DEBUGln(" - ACK:NOK...");
//  SERIALFLUSH();

  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life
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

byte cycleCount=BATT_CYCLES;
void checkBattery(byte samples)
{
  if (cycleCount++ == BATT_CYCLES) //only read battery every BATT_CYCLES sleep cycles
  {
    unsigned int readings=0;
    for (byte i=0; i<samples; i++) //take 10 samples, and average
      readings+=analogRead(BATT_MONITOR);
    batteryVolts = BATT_FORMULA(readings / 10.0);
    dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
    if (batteryVolts <= BATT_LOW) BATstr = "LOW";
    cycleCount = 0;
  }
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

void Blink(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(2);
  digitalWrite(pin, LOW);
}
