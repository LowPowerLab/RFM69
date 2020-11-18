// **********************************************************************************
// Sample RFM69 sender/node sketch for the MotionMote R4 With Panasonic PIR sensors
// http://lowpowerlab.com/motion
// PIR motion sensor connected to D3 (INT1)
// When RISE happens on D3, the sketch transmits a "MOTION" msg to receiver Moteino Gateway and goes back to sleep
// In sleep mode, Moteino + PIR motion sensor use ~2uA
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// (C) 2020, Felix Rusu, LowPowerLab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Foundation; either version 3 of the License, or        
// Public License as published by the Free Software       
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
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        123   //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -90
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack
#define ONBOARDLED     9  // same as LED_BUILTIN on Moteinos (D9)
#define PIR_POWER      7  // PIR is powered from D7
#define MOTION_PIN     3  // PIR output
#define MOTION_IRQ     1  // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION
#define DUPLICATE_INTERVAL 20000 //avoid duplicates in 55second intervals (ie mailman sometimes spends 30+ seconds at mailbox)
#define BATT_INTERVAL  300000  // read and report battery voltage every this many ms (approx)
const uint16_t INTERNAL_AREF_V = 1100; //measured internal 1.1v bandgap

#define LED_PWR 6
#define LED_GND 5
#define LED_HIGH digitalWrite(LED_PWR, HIGH)
#define LED_LOW digitalWrite(LED_PWR, LOW)

#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#define DEBUGFlush() { Serial.flush(); }
#else
#define DEBUG(input);
#define DEBUGln(input);
#define DEBUGFlush();
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

volatile boolean motionDetected=false;
float batteryVolts = 5;
float temp = 0;
char BATstr[10]; //longest battery voltage reading message = 9chars
char TEMPstr[10];
char sendBuf[32];
byte sendLen;

void motionIRQ(void);
void checkBattery(void);

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower(); //for RFM69HCW only!
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  pinMode(MOTION_PIN, INPUT);
  attachInterrupt(MOTION_IRQ, motionIRQ, RISING);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
  pinMode(ONBOARDLED, OUTPUT);
  radio.sendWithRetry(GATEWAYID, "START", 5);

#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  if (flash.initialize()) flash.sleep();
  pinMode(PIR_POWER, OUTPUT);
  digitalWrite(PIR_POWER, HIGH);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_GND, OUTPUT);
}

void motionIRQ() {
  motionDetected=true;
  DEBUGln("IRQ");
}

uint16_t batteryReportCycles=0;
uint32_t time=0, now=0, MLO=0, BLO=0;
byte motionRecentlyCycles=0;
void loop() {
  now = millis();
  checkBattery();
  //DEBUG("Slept: ");DEBUG(now-lastSleepTime);DEBUGln("ms");

  if (motionDetected && (time-MLO > DUPLICATE_INTERVAL))
  {
    LED_HIGH; //digitalWrite(LED, HIGH);
    MLO = time; //save timestamp of event
    sprintf(sendBuf, "MOTION BAT:%sv F:%s", BATstr, TEMPstr);
    DEBUG(sendBuf);
    sendLen = strlen(sendBuf);

    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
      DEBUG("..OK! RSSI:");
      DEBUG(radio.RSSI);
      batteryReportCycles = 0;
    }
    else DEBUG("..NOK..");

    radio.sleep();
    LED_LOW; //digitalWrite(LED, LOW);
  }
  else if (time-BLO > BATT_INTERVAL)
  {
    sprintf(sendBuf, "BAT:%sv F:%s", BATstr, TEMPstr);
    sendLen = strlen(sendBuf);
    BLO = time;
    DEBUGln(sendBuf);
    radio.sendWithRetry(GATEWAYID, sendBuf, sendLen);
    radio.sleep();
    batteryReportCycles=0;
  }
  
  DEBUGFlush();

  //while motion recently happened sleep for small slots of time to better approximate last motion event
  //this helps with debouncing a "MOTION" event more accurately for sensors that fire the IRQ very rapidly (ie panasonic sensors)
  if (motionDetected ||motionRecentlyCycles>0)
  {
    if (motionDetected) motionRecentlyCycles=8;
    else motionRecentlyCycles--;
    motionDetected=false; //do NOT move this after the SLEEP line below or motion will never be detected
    time = time + 250 + millis()-now; //correct millis() resonator drift, may need to be tweaked to be accurate
    radio.sleep();
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    DEBUGln("WAKEUP250ms");
  }
  else
  {
    time = time + 8000 + millis()-now /*+ 480*/; //correct millis() resonator drift, may need to be tweaked to be accurate
    radio.sleep();
    //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //watchdog sleep uses extra ~4uA!
    sleep(8000);
    DEBUGln("WAKEUP8s");
  }
  batteryReportCycles++;
}

uint32_t BLR=0;
void checkBattery()
{
  if (time-BLR > 30000) //only read battery every 30s or so
  {
    BLR = time;
    long vavg = 0;
    temp = 0;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // Ref to Vcc. Measure internal 1.1V ref
    for (int j = 0; j < 10; j++)
    {                                     // Read a few times to get ADC to settle
      ADCSRA |= _BV(ADSC);                // Start conversion
      temp +=  radio.readTemperature(-1); // Temperature. -1 = user cal factor, adjust for correct ambient 
      while (bit_is_set(ADCSRA,ADSC));    // measuring
      if (j > 4) {                        // Skip the first 5 Vcc readings, take the next 5
        vavg = vavg + (((INTERNAL_AREF_V * 1024L) / ADC) + 5L);
      }
    }
    batteryVolts = (vavg/5.0)/1000.0;
    temp /= 10;
    dtostrf(temp, 3,2, TEMPstr);
    dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  }
}


void sleep(uint32_t sleepTime) {
  DEBUGFlush();
  if (sleepTime < 262) { //sleeps just the MCU, using WDT (radio is not touched)
    longPowerDown(sleepTime);
  } else { //sleeps MCU using the radio timer - should not be used if radio needs to be in RX mode!
    uint32_t freq = radio.getFrequency();
    if (sleepTime%262 && sleepTime > 262*2) {
      DEBUG("Sleeping "); DEBUGln(sleepTime-sleepTime%262-262); DEBUGFlush();
      listenModeSleep(sleepTime-sleepTime%262-262);
      DEBUG("Sleeping "); DEBUGln(sleepTime%262 + 262); DEBUGFlush();
      listenModeSleep(sleepTime%262 + 262);
    } else {
      DEBUG("Sleeping "); DEBUGln(sleepTime); DEBUGFlush();
      listenModeSleep(sleepTime);
    }

    //WAKEUP happens here (must reinit!)
    radio.RFM69::initialize(FREQUENCY,NODEID,NETWORKID); //call base init!
    #ifdef ENCRYPTKEY
      radio.encrypt(ENCRYPTKEY);
    #endif
    radio.setFrequency(freq);
  }
}

void listenModeSleep(uint16_t millisInterval) {
  radio.listenModeSleep(millisInterval);
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  radio.endListenModeSleep();
}

void longPowerDown(uint32_t sleepTime) {
  do {
    if (sleepTime > 8000)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      sleepTime-=8000;
    }
    else if (sleepTime > 4000)
    {
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
      sleepTime-=4000;
    }
    else if (sleepTime > 2000)
    {
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      sleepTime-=2000;
    }
    else if (sleepTime > 1000)
    {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      sleepTime-=1000;
    }
    else if (sleepTime > 512)
    {
      LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
      sleepTime-=512;
    }
    else if (sleepTime > 256)
    {
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      sleepTime-=256;
    }
    else if (sleepTime > 128)
    {
      LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
      sleepTime-=128;
    }
    else if (sleepTime > 64)
    {
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
      sleepTime-=64;
    }
    else if (sleepTime > 32)
    {
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
      sleepTime-=32;
    }
    else if (sleepTime > 16)
    {
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      sleepTime-=16;
    }
    else
    {
      sleepTime=0;
    }
  } while(sleepTime);
}