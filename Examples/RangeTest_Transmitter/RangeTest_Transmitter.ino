//*********************************************************************************************
// RangeTest_Transmitter (to be used with a MotionMote or similar Moteino)
// Adapted from MotionMote firmware sketch
// Acts as a continuous transmitter for testing RF performance, in tandem with a RangeTest_Gateway
//*********************************************************************************************
// Copyright Felix Rusu 2021, http://www.LowPowerLab.com/contact
//*********************************************************************************************
#include <RFM69.h>    //https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//https://www.github.com/lowpowerlab/rfm69
#include <LowPower.h> //https://github.com/lowpowerlab/lowpower
#include <SPIFlash.h> //https://www.github.com/lowpowerlab/spiflash
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID         2    //unique for each node on same network
#define NETWORKID      100  //the same on all nodes that talk to each other
#define GATEWAYID      1
#define FREQUENCY      RF69_915MHZ  //others: RF69_433MHZ, RF69_868MHZ, match to radio variant on your Moteino/board
#define ENCRYPTKEY     "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW //assumes RFM69 HCW/HW, remove if you have RFM69 W/CW
#define ENABLE_ATC     //comment out to disable AUTO TRANSMISSION CONTROL (ie. always transmit at max power)
#define ATC_RSSI       -90 //noise floor is at -100dB, keep above by a margin of 10db
//*********************************************************************************************
//************ GPIO & MISC ********************************************************************
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack
#define ONBOARDLED     9  // MotionMote onboard LED on D9
#define PIR_POWER      7  //PIR is powered from D7
#define RFM_RST        A0 //used to reset the radio module at power up to ensure a clean start
#define MOTION_PIN     3  // D3
#define MOTION_IRQ     1  // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION
#define LED_PWR        6  // separate MotionMote LED powered from D6
#define LED_GND        5  // separate MotionMote LED ground on D5
#define LED_HIGH digitalWrite(LED_PWR, HIGH)
#define LED_LOW digitalWrite(LED_PWR, LOW)
#define DUPLICATE_INTERVAL 10000  //avoid duplicates in 55second intervals (ie mailman sometimes spends 30+ seconds at mailbox)
#define BATT_INTERVAL       4000  // read and report battery voltage every this many ms (approx)
#define INTERNAL_AREF_V     1100  //=1.1v internal bandgap. can be adjusted to more or less to be more accurate
//*********************************************************************************************
#define DEBUG_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#ifdef DEBUG_EN
  #define SERIAL_BAUD   500000
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
  #define DEBUGHEX(input, param) Serial.print(input, param)
  #define DEBUGFlush() Serial.flush()
#else
  #define DEBUG(input)
  #define DEBUGln(input)
  #define DEBUGHEX(input, param)
  #define DEBUGFlush();
#endif
//*********************************************************************************************
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
void motionIRQ(void);
void checkBattery(void);

void setup() {
#ifdef DEBUG_EN
  Serial.begin(SERIAL_BAUD);
#endif

  if (flash.initialize()) {
    DEBUGln(F("SPI Flash Init OK!"));
    flash.sleep(); //safe to call because initialize() wakes it up
  }
  else DEBUGln(F("SPI Flash MEM FAIL!"));

  pinMode(MOTION_PIN, INPUT);
  pinMode(ONBOARDLED, OUTPUT);
  pinMode(PIR_POWER, OUTPUT);
  pinMode(RFM_RST, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_GND, OUTPUT);
  digitalWrite(PIR_POWER, HIGH);
  digitalWrite(RFM_RST, LOW);
  attachInterrupt(MOTION_IRQ, motionIRQ, RISING);
  digitalWrite(RFM_RST, HIGH); delay(100); digitalWrite(RFM_RST, LOW);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower();
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  DEBUG("Transmitting at "); DEBUG(radio.getFrequency()); DEBUGln(" Hz..");

  radio.sendWithRetry(GATEWAYID, "START", 5);
}

void motionIRQ() {
  motionDetected=true;
  DEBUGln("IRQ");
}

uint32_t time=0, now=0, MLO=0, BLO=0;
byte motionRecentlyCycles=0;
byte ackCount=0;
byte packetCounter=0;
void loop() {
  now = millis();
  checkBattery();
  //DEBUG("Slept: ");DEBUG(now-lastSleepTime);DEBUGln("ms");

  if (motionDetected && (time-MLO > DUPLICATE_INTERVAL)) {
    packetCounter++;
    LED_HIGH; //digitalWrite(LED, HIGH);
    MLO = BLO = time; //save timestamp of event
    sprintf(sendBuf, "%d MOTION X:%d", packetCounter, radio.getPowerLevel());
    DEBUG(sendBuf);

    if (radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf))) {
      DEBUG("..OK! RSSI:");
      DEBUG(radio.RSSI);
      listen_a_little();
    } else DEBUG("..NOK..");

    radio.sleep();
    LED_LOW;
  } else if (time-BLO > BATT_INTERVAL) {
    packetCounter++;
    sprintf(sendBuf, "%d V:%s F:%s X:%d", packetCounter, BATstr, TEMPstr, radio.getPowerLevel());
    BLO = time;

    DEBUG(sendBuf);
    if (radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf))) {
      DEBUG("..OK!");
      listen_a_little();
    } else DEBUG("..NOK..");
  }
  DEBUGln(); DEBUGFlush();

  //while motion recently happened sleep for small slots of time to better approximate last motion event
  //this helps with debouncing a "MOTION" event more accurately for sensors that fire the IRQ very rapidly (ie panasonic sensors)
  if (motionDetected || motionRecentlyCycles>0) {
    if (motionDetected) motionRecentlyCycles=8;
    else motionRecentlyCycles--;
    motionDetected=false; //do NOT move this after the SLEEP line below or motion will never be detected
    time = time + 250 + millis()-now; //correct millis() resonator drift, may need to be tweaked to be accurate
    radio.sleep();
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    DEBUGln("WAKEUP250ms");
  } else {
    time = time + 4000 + millis()-now /*+ 480*/; //correct millis() resonator drift, may need to be tweaked to be accurate
    radio.sleep();
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); //watchdog sleep uses extra ~4uA!
    //sleep(4000);
    DEBUGln("WAKEUP4s");
  }
}

uint32_t BLR=0;
void checkBattery()
{
  if (time-BLR > 3) //only read battery every 30s or so
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
    dtostrf(temp, 2,0, TEMPstr);
    dtostrf(batteryVolts, 3,1, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  }
}


void sleep(uint32_t sleepTime) {
  DEBUGFlush();
  if (sleepTime < 262) { //sleeps just the MCU, using WDT (radio is not touched)
    LowPower.longPowerDown(sleepTime);
  } else { //sleeps MCU using the radio timer - should not be used if radio needs to be in RX mode!
    uint32_t freq = radio.getFrequency();
    uint32_t remainingSleepTime = sleepTime;
    
    while (remainingSleepTime) { //split into sleep(60s) calls if > 60s sleep
      if (remainingSleepTime > 65500) {
        sleepTime = 65500;
        remainingSleepTime -= 65500;
      } else {
        sleepTime = remainingSleepTime;
        remainingSleepTime = 0;
      }

      if (sleepTime%262 && sleepTime > 262*2) {
        DEBUG("Sleep "); DEBUGln(sleepTime-sleepTime%262-262); DEBUGFlush();
        listenModeSleep(sleepTime-sleepTime%262-262);
        DEBUG("Sleep "); DEBUGln(sleepTime%262 + 262); DEBUGFlush();
        listenModeSleep(sleepTime%262 + 262);
      } else {
        DEBUG("Sleep "); DEBUGln(sleepTime); DEBUGFlush();
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
}

void listenModeSleep(uint16_t millisInterval) {
  radio.listenModeSleep(millisInterval);
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
  radio.endListenModeSleep();
}

void listen_a_little() {
  radio.receiveDone(); //radio RX!
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

  if (radio.receiveDone()) {
    DEBUG(" [");DEBUG(radio.SENDERID);DEBUG("] ");
    DEBUG((char*)radio.DATA);
    DEBUG("[RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");

    if (radio.ACKRequested()) {
      radio.sendACK();
      DEBUG("[ACK sent]");
    }
    time += 10;
  } else time += 60;
}
