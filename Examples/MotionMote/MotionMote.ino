// Sample RFM69 sender/node sketch for the MotionMote
// http://lowpowerlab.com/motion
// PIR motion sensor connected to D3 (INT1)
// When RISE happens on D3, the sketch transmits a "MOTION" msg to receiver Moteino and goes back to sleep
// In sleep mode, Moteino + PIR motion sensor use about ~78uA
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
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

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        88    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
//*********************************************************************************************

#define ACK_TIME      30 // max # of ms to wait for an ack
#define ONBOARDLED     9  // Moteinos have LEDs on D9
#define EXTLED         5  // MotionOLEDMote has an external LED on D5
#define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define MOTIONPIN     1 //hardware interrupt 1 (D3)

//#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

RFM69 radio;
volatile boolean motionDetected=false;
float batteryVolts = 5;
char* BATstr="BAT:5.00v";
char sendBuf[32];
byte sendLen;
float battAvg = 0;                    // Keeping a running average of samples taken in the timer interrupt
byte battCount = 0;                   // Count of samples taken;
byte reportCount = 75;                // The count at which to report

void setup() {
  Serial.begin(SERIAL_BAUD);
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
  pinMode(ONBOARDLED, OUTPUT);
  pinMode(EXTLED, OUTPUT);
}

void motionIRQ()
{
  motionDetected=true;
}

void loop() {
  if (motionDetected)
  {
    digitalWrite(EXTLED, HIGH);
    //sprintf(sendBuf, "MOTION ............................................ BAT:%sv", BATstr);
    sprintf(sendBuf, "MOTION");
    sendLen = strlen(sendBuf);

    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
      DEBUG("MOTION ACK:OK! RSSI:");
      DEBUG(radio.RSSI);
    }
    else DEBUG("MOTION ACK:NOK...");

    radio.sleep();
    //Blink(EXTLED,50); //Blink(ONBOARDLED,3);
    motionDetected=false;
    digitalWrite(EXTLED, LOW);
  }
  checkBatt();        // Check the battery voltage before powering down
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void checkBatt() {
  if (battCount < reportCount) {
    batteryVolts = analogRead(A7) * 0.00322 * 1.42;
    battAvg = ((battAvg * battCount) + batteryVolts) / (battCount + 1);
    
    DEBUG("battCount:\t"); DEBUGln(battCount);
    DEBUG("batteryVolts:\t"); DEBUGln(batteryVolts);
    DEBUG("battAvg:\t"); DEBUGln(battAvg);
    DEBUGln("-------");

    ++battCount;
    
    if (battCount >= reportCount) {
      digitalWrite(EXTLED, HIGH);
      
      dtostrf(battAvg, 3,2, BATstr);
      sprintf(sendBuf, "BAT,%s;", BATstr);
      sendLen = strlen(sendBuf);
      if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
      {
        DEBUG("BAT ACK:OK! RSSI:");
        DEBUG(radio.RSSI);
        Blink(ONBOARDLED,50);
      }
      else {
        DEBUG("BAT ACK:NOK...");
        Blink(ONBOARDLED,50); Blink(ONBOARDLED,50);
      }
      radio.sleep();
  
      DEBUG(" VIN: ");
      DEBUGln(BATstr);
      battCount = battAvg = 0;
      digitalWrite(EXTLED, LOW);
    }
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
