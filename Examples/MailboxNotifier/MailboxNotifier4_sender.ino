// Sample RFM69 sender/node sketch for mailbox motion sensor
// http://www.lowpowerlab.com/mailbox
// PIR motion sensor connected to D3 (INT1)
// When RISE happens on D3, the sketch transmits a "MOTION" msg to receiver Moteino and goes back to sleep
// It then wakes up every 32 seconds and sends a message indicating when the last
//    motion event happened (days, hours, minutes, seconds ago) and the battery level
// In sleep mode, Moteino + PIR motion sensor use about ~78uA
// Make sure you adjust the settings in the configuration section below !!!

// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
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

#include <RFM69.h>    //get it here: https://github.com/LowPowerLab/RFM69
#include <SPI.h>      //get it here: https://github.com/LowPowerLab/SPIFlash
#include <LowPower.h> //get library from: https://github.com/LowPowerLab/LowPower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        55    //unique for each node on same network
#define NETWORKID     250  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SENDEVERYXLOOPS   4 //each loop sleeps 8 seconds, so send status message every this many loops (default "4" = 32 seconds)
//*********************************************************************************************

#define MOTIONPIN     1  //hardware interrupt 1 (D3)
#define BATTERYSENSE  A7 //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define LED           9  // Moteinos have LEDs on D9
//#define BLINK_EN         //uncomment to make LED flash when messages are sent, leave out if you want low power

#define SERIAL_BAUD   115200
//#define SERIAL_EN      //uncomment this line to enable serial IO debug messages, leave out if you want low power
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
volatile boolean motionDetected=false;

void setup() {
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  pinMode(MOTIONPIN, INPUT);
  attachInterrupt(MOTIONPIN, motionIRQ, RISING);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
}

void motionIRQ()
{
  motionDetected=true;
  //DEBUGln("I");
}

char sendBuf[32];
byte sendLen;
byte sendLoops=0;
unsigned long MLO=0; //MailLastOpen (ago, in ms)
unsigned long now = 0, time=0, lastSend = 0, temp = 0;

void loop() {
  now = millis();
  int batteryReading = analogRead(BATTERYSENSE);
  if (motionDetected && time-MLO > 20000) //avoid duplicates in 20second intervals
  {
    MLO = time; //save timestamp of event
    sprintf(sendBuf, "MOTION BAT:%i", batteryReading);
    sendLen = strlen(sendBuf);
    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
     DEBUGln(" ok!");
     #ifdef BLINK_EN
       Blink(LED,3);
     #endif
    }
    else DEBUGln(" nothing...");
    radio.sleep();
    motionDetected=false;
  }
  else sendLoops++;
  
  //send readings every SENDEVERYXLOOPS
  if (sendLoops>=SENDEVERYXLOOPS)
  {
    sendLoops=0;
    
    char periodO='X', periodC='X';
    unsigned long lastOpened = (time - MLO) / 1000; //get seconds
    unsigned long LO = lastOpened;
    char* MLOstr="LO:99d23h59m";
    char* BATstr="BAT:1024";
    char* BATactual="BATA:5.00v";
    
    if (lastOpened <= 59) periodO = 's'; //1-59 seconds
    else if (lastOpened <= 3599) { periodO = 'm'; lastOpened/=60; } //1-59 minutes
    else if (lastOpened <= 259199) { periodO = 'h'; lastOpened/=3600; } // 1-71 hours
    else if (lastOpened >= 259200) { periodO = 'd'; lastOpened/=86400; } // >=3 days

    if (periodO == 'd')
      sprintf(MLOstr, "LO:%ldd%ldh", lastOpened, (LO%86400)/3600);
    else if (periodO == 'h')
      sprintf(MLOstr, "LO:%ldh%ldm", lastOpened, (LO%3600)/60);
    else sprintf(MLOstr, "LO:%ld%c", lastOpened, periodO);

    //sprintf(BATstr, "BAT:%i", batteryReading);
    float battV = ((float)batteryReading * 3.3 * 9)/(1023*2.976);
    dtostrf(battV, 3,2, BATactual);
    sprintf(sendBuf, "%s BAT:%sv", MLOstr, BATactual);
    sendLen = strlen(sendBuf);
    radio.send(GATEWAYID, sendBuf, sendLen);
    radio.sleep();
    DEBUG(sendBuf); DEBUG(" ("); DEBUG(sendLen); DEBUGln(")"); 
    lastSend = time;
    #ifdef BLINK_EN
      Blink(LED, 5);
    #endif
    delay(10); motionDetected=false; //fix PIR false positive glitch
  }

  DEBUGln("LOOP");
  time = time + 8000 + millis()-now + 480; //correct millis() resonator drift, may need to be tweaked to be accurate
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}