// **********************************************************************************************************
// MightyHat gateway base unit sketch that works with MightyHat with onboard RFM69W/RFM69HW
// This will relay all RF data over serial to the host computer (RaspberryPi, PC etc) and vice versa
// http://LowPowerLab.com/MightyHat
// Copyright http://www.LowPowerLab.com (2015)
// Also see http://LowPowerLab.com/gateway
//**********************************************************************************
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

#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include "U8glib.h"        //https://bintray.com/olikraus/u8glib/Arduino
                           //u8g compared to adafruit lib: https://www.youtube.com/watch?v=lkWZuAnHa2Y
                           //draing bitmaps: https://www.coconauts.net/blog/2015/01/19/easy-draw-bitmaps-arduino/
//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define NODEID          1
#define NETWORKID     100
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*****************************************************************************************************************************
#define ACK_TIME       30  // # of ms to wait for an ack
#define SERIAL_BAUD 115200
#define DEBUG_EN     //comment out if you don't want any serial verbose output (keep out in real use)

#define BTN_LED_RED     9
#define BTN_LED_GRN     6  // This will indicate when Pi has power
#define POWER_LED_RED()    { digitalWrite(BTN_LED_RED, HIGH); digitalWrite(BTN_LED_GRN, LOW); }
#define POWER_LED_GRN()    { digitalWrite(BTN_LED_RED, LOW);  digitalWrite(BTN_LED_GRN, HIGH); }
#define POWER_LED_ORANGE() { digitalWrite(BTN_LED_RED, HIGH); digitalWrite(BTN_LED_GRN, HIGH); }
#define POWER_LED_OFF()    { digitalWrite(BTN_LED_RED, LOW);  digitalWrite(BTN_LED_GRN, LOW); }
#define ON              1
#define OFF             0

#define FLASH_CS        8

#define BUZZER              5     // Buzzer attached to D5 (PWM pin required for tones)
#define BUTTON             A2     // Power button pin
#define LATCH_EN            4
#define LATCH_VAL           7
#define SIG_SHUTOFF        A3     // Signal to Pi to ask for a shutdown
#define SIG_BOOTOK         A6     // Signal from Pi that it's OK to cutoff power
                                  // !!NOTE!! Originally this was D7 but it was moved to A0 at least temporarily.
                                  // On MightyBoost R1 you need to connect D7 and A0 with a jumper wire.
                                  // The explanation for this is given here: http://lowpowerlab.com/mightyboost/#source
#define BATTERYSENSE       A7     // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
                                  // hence the actual input voltage = analogRead(A7) * 0.00322 (3.3v/1024) * 1.47 (10k+4.7k voltage divider ratio)
                                  // when plugged in this should be 4.80v, nothing to worry about
                                  // when on battery power this should decrease from 4.15v (fully charged Lipoly) to 3.3v (discharged Lipoly)
                                  // trigger a shutdown to the target device once voltage is around 3.4v to allow 30sec safe shutdown

#define BATTERY_VOLTS(analog_reading) analog_reading * 0.00322 * 1.51 // 100/66 is the inverse ratio of the voltage divider ( Batt > 1MEG > A7 > 2MEG > GND )
#define LOWBATTERYTHRESHOLD  3.65  // a shutdown will be triggered to the target device when battery voltage drops below this (Volts)
#define CHARGINGTHRESHOLD     4.3
#define RESETHOLDTIME         500 // Button must be hold this many mseconds before a reset is issued (should be much less than SHUTDOWNHOLDTIME)
#define SHUTDOWNHOLDTIME     2000 // Button must be hold this many mseconds before a shutdown sequence is started (should be much less than ForcedShutoffDelay)
#define ShutoffTriggerDelay  6000 // will start checking the SIG_BOOTOK line after this long
#define RESETPULSETIME        500 // When reset is issued, the SHUTOFF signal is held HIGH this many ms
#define ForcedShutoffDelay   7500 // when SIG_BOOTOK==0 (PI in unknown state): if button is held
                                  // for this long, force shutdown (this should be less than RecycleTime)
#define ShutdownFinalDelay   4500 // after shutdown signal is received, delay for this long
                                  // to allow all PI LEDs to stop activity (pulse LED faster)
#define RecycleTime         60000 // window of time in which SIG_BOOTOK is expected to go HIGH
                                  // should be at least 3000 more than Min
                                  // if nothing happens after this window, if button is 
                                  // still pressed, force cutoff power, otherwise switch back to normal ON state
#define PRINTPERIOD         1500

#ifdef DEBUG_EN
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
#else
  #define DEBUG(input)
  #define DEBUGln(input)
#endif

//******************************************** BEGIN LCD PRIMITIVES ********************************************************************************
#define PIN_LCD_CS    LATCH_VAL //Pin 2 on LCD, lcd CS is shared with Latch value pin since they are both outputs and never HIGH at the same time
#define PIN_LCD_RST   A1 //Pin 1 on LCD
#define PIN_LCD_DC    A0 //Pin 3 on LCD
#define PIN_LCD_LIGHT 3 //Backlight pin
#define xbmp_logo_width 67
#define xbmp_logo_height 36
#define LCD_BACKLIGHT(x) { if (x) analogWrite(PIN_LCD_LIGHT, 100); else analogWrite(PIN_LCD_LIGHT, 0); }

const uint8_t xbmp_logo[] PROGMEM = {
   0x00, 0x00, 0x00, 0xfe, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x40,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x30, 0x3c, 0x00, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x42, 0x80, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x42, 0x80, 0x10,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0xc2, 0xc0, 0x10, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x30, 0x3c, 0x21, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x00, 0x1e, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x1e, 0x00,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x3c, 0x1e, 0x0f, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x30, 0x42, 0x9e, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30,
   0xc2, 0xff, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x42, 0x9e, 0x10,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x3c, 0x1e, 0x0f, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x30, 0x00, 0x1e, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x00, 0x1e, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x3c, 0x21, 0x0f,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0xc2, 0xc0, 0x10, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x30, 0x42, 0x80, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x42, 0x80, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x3c, 0x00, 0x0f,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x1f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x03, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x18, 0xc0, 0x00, 0x03, 0x00, 0xb0,
   0x01, 0x00, 0x00, 0x18, 0xc0, 0x00, 0xc3, 0xd9, 0xb6, 0x39, 0xdb, 0x9c,
   0x1b, 0xcf, 0x03, 0x63, 0xdb, 0xb6, 0x6d, 0xdb, 0xb6, 0x19, 0xd8, 0x06,
   0x63, 0xdb, 0xf6, 0x6c, 0xdb, 0xbe, 0x19, 0xde, 0x06, 0x63, 0x33, 0x33,
   0x6c, 0x66, 0x86, 0x19, 0xdb, 0x06, 0x63, 0x33, 0x33, 0x6c, 0x66, 0x86,
   0x19, 0xdb, 0x06, 0xcf, 0x31, 0x33, 0x38, 0x66, 0xbc, 0x79, 0xde, 0x03 };
   
#define xbmp_batt_width 9
#define xbmp_batt_height 6
const uint8_t xbmp_batt_c[] PROGMEM = { 0xff, 0x00, 0xbf, 0x00, 0x9f, 0x01, 0x8f, 0x01, 0x87, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_x[] PROGMEM = { 0xff, 0x00, 0xa5, 0x00, 0x81, 0x01, 0x99, 0x01, 0xa5, 0x00, 0xff, 0x00 };

const uint8_t xbmp_batt_0[] PROGMEM = {  };
const uint8_t xbmp_batt_1[] PROGMEM = { 0xff, 0x00, 0x83, 0x00, 0x83, 0x01, 0x83, 0x01, 0x83, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_2[] PROGMEM = { 0xff, 0x00, 0x87, 0x00, 0x87, 0x01, 0x87, 0x01, 0x87, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_3[] PROGMEM = { 0xff, 0x00, 0x8f, 0x00, 0x8f, 0x01, 0x8f, 0x01, 0x8f, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_4[] PROGMEM = { 0xff, 0x00, 0x9f, 0x00, 0x9f, 0x01, 0x9f, 0x01, 0x9f, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_5[] PROGMEM = { 0xff, 0x00, 0xbf, 0x00, 0xbf, 0x01, 0xbf, 0x01, 0xbf, 0x00, 0xff, 0x00 };
const uint8_t xbmp_batt_6[] PROGMEM = { 0xff, 0x00, 0xff, 0x00, 0xff, 0x01, 0xff, 0x01, 0xff, 0x00, 0xff, 0x00 };


#define xbmp_rssi_width 7
#define xbmp_rssi_height 6
const uint8_t xbmp_rssi_1[] PROGMEM = { 0x40, 0x10, 0x00, 0x04, 0x04, 0x05 };
const uint8_t xbmp_rssi_2[] PROGMEM = { 0x40, 0x10, 0x10, 0x14, 0x14, 0x15 };
const uint8_t xbmp_rssi_3[] PROGMEM = { 0x40, 0x50, 0x50, 0x54, 0x54, 0x55 };
const uint8_t xbmp_rssi_0[] PROGMEM = { 0x40, 0x10, 0x00, 0x04, 0x00, 0x01 };
//******************************************** END LCD PRIMITIVES ********************************************************************************

//general variables
byte ackCount=0;
String inputstr;
byte inputLen=0;
char BATstr[5];
char BATvstr[6];
char RSSIstr[] = "-100dBm";
char input[64];
byte temp[61];
char lcdbuff[80];
int lastValidReading = 1;
unsigned long lastValidReadingTime = 0;
unsigned long NOW=0;
byte PowerState = OFF;
long lastPeriod = -1;
int rssi=0;
float systemVoltage = 5;
boolean batteryLow=false;
boolean batteryLowShutdown=false;

//SPI devices (RFM69, FLASH MEM, LCD)
U8GLIB_PCD8544 lcd(PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);
SPIFlash flash(FLASH_CS, 0xEF30); //EF30 for 4mbit Windbond FLASH MEM 
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

//******************************************** BEGIN LCD FUNCTIONS ********************************************************************************
void drawLogo() {
  lcd.firstPage();
  do {
    lcd.drawXBMP((84-xbmp_logo_width)/2, (48-xbmp_logo_height)/2, xbmp_logo_width, xbmp_logo_height, xbmp_logo); //tutorial: https://www.coconauts.net/blog/2015/01/19/easy-draw-bitmaps-arduino/
  } while(lcd.nextPage());
}

void clearDisplay() {
  lcd.firstPage();
  do{}while(lcd.nextPage());
}

void refreshLCD() {
  
  byte lcdwidth = lcd.getWidth();
  byte lcdheight = lcd.getHeight();
  char c;
  byte i,pos,swidth;
  byte * bmpPtr;
      
  //u8glib picture loop
  lcd.firstPage();
  do {
    lcd.setFont(u8g_font_profont10);
    lcd.setFontRefHeightText();
    lcd.setFontPosTop();
    byte fontheight = lcd.getFontAscent()-lcd.getFontDescent();
    char * textp = lcdbuff;
    byte textLength = strlen(textp);
    byte line=0;
    byte done = false;

    while(textLength && !done)
    {
      //DEBUGln(textLength);
      for (i=1;i<=textLength;i++)
      {
        c = textp[i];
        textp[i]=0;
        swidth = lcd.getStrWidth(textp);
        textp[i] = c;
        if (swidth > lcdwidth || c=='\n') { pos = i-1; break; }
        else if (i==textLength) { done = true; }
      }
      if (!done)
      {
        c = textp[pos];
        textp[pos]=0;
      }
      lcd.drawStr(0, line * fontheight, textp);
      if (done) break;
      textp[pos] = c;
      textp += pos;
      textLength -= pos;
      line++;
    }

    lcd.setFontPosBaseline();

    //battery
    if (systemVoltage >= 4.3) bmpPtr = (byte*)xbmp_batt_c;
    else if (systemVoltage >= 4) bmpPtr = (byte*)xbmp_batt_6;
    else if (systemVoltage >= 3.9) bmpPtr = (byte*)xbmp_batt_5;
    else if (systemVoltage >= 3.8) bmpPtr = (byte*)xbmp_batt_4;
    else if (systemVoltage >= 3.7) bmpPtr = (byte*)xbmp_batt_3;
    else if (systemVoltage >= 3.6) bmpPtr = (byte*)xbmp_batt_2;
    else if (systemVoltage >= 3.5) bmpPtr = (byte*)xbmp_batt_1;
    else bmpPtr = (byte*)xbmp_batt_x;
    lcd.drawXBMP(lcdwidth-xbmp_batt_width, lcdheight-xbmp_batt_height, xbmp_batt_width, xbmp_batt_height, bmpPtr);
    
    if (systemVoltage >= CHARGINGTHRESHOLD)
      sprintf(BATvstr, "CHRG"); //sprintf(BATvstr, "%sv", BATstr);
    else {
      sprintf(BATvstr, "%sv", BATstr);
    }
    lcd.drawStr(50, 48, BATvstr);

    //rssi
    if (rssi > -70) bmpPtr = (byte*)xbmp_rssi_3;
    else if (rssi > -80) bmpPtr = (byte*)xbmp_rssi_2;
    else if (rssi > -90) bmpPtr = (byte*)xbmp_rssi_1;
    else if (rssi > -95) bmpPtr = (byte*)xbmp_rssi_0;
    lcd.drawXBMP(0, lcdheight-xbmp_rssi_height, xbmp_rssi_width, xbmp_rssi_height, bmpPtr);
    lcd.drawStr(xbmp_rssi_width+1, 48, RSSIstr);

  } while(lcd.nextPage());
}
//******************************************** END LCD FUNCTIONS ********************************************************************************


//parse through any serial commands from the host (Pi)
void handleSerialInput() {
  inputLen = readSerialLine(input, 10, 64, 10); //readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=10);
  inputstr = String(input);
  inputstr.toUpperCase();
  
  if (inputLen > 0)
  {
    if (inputstr.equals("BEEP")) Beep(10, false);
    if (inputstr.equals("BEEP2")) Beep(10, true);
    if (inputstr.equals("RAM")) { DEBUG(F("Free RAM bytes: "));DEBUGln(checkFreeRAM()); }
    
    if (inputstr.equals("KEY?"))
    {
      Serial.print(F("ENCRYPTKEY:"));
      Serial.print(ENCRYPTKEY);
    }

    byte targetId = inputstr.toInt(); //extract ID if any
    byte colonIndex = inputstr.indexOf(":"); //find position of first colon
    if (targetId > 0) inputstr = inputstr.substring(colonIndex+1); //trim "ID:" if any
    if (targetId > 0 && targetId != NODEID && targetId != RF69_BROADCAST_ADDR && colonIndex>0 && colonIndex<4 && inputstr.length()>0)
    {
      inputstr.getBytes(temp, 61);
      if (radio.sendWithRetry(targetId, temp, inputstr.length()))
        Serial.println(F("ACK:OK"));
      else
        Serial.println(F("ACK:NOK"));
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

void setupPowerControl(){
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SIG_BOOTOK, INPUT);
  pinMode(SIG_SHUTOFF, OUTPUT);
  pinMode(BTN_LED_RED, OUTPUT);
  pinMode(BTN_LED_GRN, OUTPUT);
  pinMode(LATCH_EN, OUTPUT);
  //digitalWrite(LATCH_EN, LOW);
  pinMode(LATCH_VAL, OUTPUT);
  pinMode(BATTERYSENSE, INPUT);
  digitalWrite(SIG_SHUTOFF, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
}

void handlePowerControl() {
  int reading = digitalRead(BUTTON);
  NOW = millis();
  digitalWrite(SIG_SHUTOFF, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  
  //artificial power ON after a low battery shutdown
  if (PowerState == OFF && batteryLowShutdown && systemVoltage >= CHARGINGTHRESHOLD)
    reading = HIGH;
  
  if ((PowerState == ON && batteryLow) || (reading != lastValidReading && NOW - lastValidReadingTime > 200))
  {
    lastValidReading = reading;
    lastValidReadingTime = NOW;
    
    if ((PowerState == ON && batteryLow) || reading == LOW)
    {
      radio.sleep();
      //make sure the button is held down for at least 'RESETHOLDTIME' before taking action (this is to avoid accidental button presses and consequently Pi shutdowns)
      NOW = millis();
      while (!batteryLow && (PowerState == ON && millis()-NOW < RESETHOLDTIME)) { delay(10); if (digitalRead(BUTTON) != 0) return; }

      //RESETHOLDTIME is satisfied, now check if button still held until SHUTDOWNHOLDTIME is satisfied
      POWER_LED_ORANGE(); //make the button LED orange to show something's going on
      while (!batteryLow && (PowerState == ON && millis()-NOW < SHUTDOWNHOLDTIME))
      {
        if (digitalRead(BUTTON) != 0)
        {
          if (BOOTOK())       //SIG_BOOTOK is HIGH so Pi is running the shutdowncheck.sh script, ready to intercept the RESET PULSE
          {
            sprintf(lcdbuff, "Rebooting Pi..");
            refreshLCD();
        
            digitalWrite(SIG_SHUTOFF, HIGH);
            delay(RESETPULSETIME);
            digitalWrite(SIG_SHUTOFF, LOW);
            //DEBUGln("SIG_SHUTOFF - HIGH>delay>LOW");

            NOW = millis();
            boolean recycleDetected=false;
            while (millis()-NOW < RecycleTime) //blink LED while waiting for BOOTOK to go high
            {
              //blink 3 times and pause
              POWER_LED_OFF(); //digitalWrite(POWER_LED, LOW);
              delay(100);
              POWER_LED_ORANGE(); //digitalWrite(POWER_LED, HIGH);
              delay(100);
              POWER_LED_OFF(); //digitalWrite(POWER_LED, LOW);
              delay(100);
              POWER_LED_ORANGE(); //digitalWrite(POWER_LED, HIGH);
              delay(100);
              POWER_LED_OFF(); //digitalWrite(POWER_LED, LOW);
              delay(100);
              POWER_LED_ORANGE(); //digitalWrite(POWER_LED, HIGH);
              delay(500);

              if (!BOOTOK()) recycleDetected = true;
              else if (BOOTOK() && recycleDetected)
                return;
            }
            return; //reboot pulse sent but it appears a reboot failed; exit all checks
          }
          else return; //ignore everything else (button was held for RESETHOLDTIME, but SIG_BOOTOK was LOW)
        }
      }
      
      //SIG_BOOTOK must be HIGH when Pi is ON. During boot, this will take a while to happen (till it executes the "shutdowncheck" script)
      //so I dont want to cutoff power before it had a chance to fully boot up
      if ((batteryLow || PowerState == ON) && BOOTOK())
      {
        if (batteryLow) {
          sprintf(lcdbuff, "Battery low! Shutting down Pi..");
          batteryLowShutdown = true;
        }
        else {
          sprintf(lcdbuff, "Shutting down Pi..");
        }
        refreshLCD();
        // signal Pi to shutdown
        digitalWrite(SIG_SHUTOFF, HIGH);
        //DEBUGln("SIG_SHUTOFF - HIGH - if(batteryLow || (PowerState == 1 && BOOTOK())");

        //now wait for the Pi to signal back
        NOW = millis();
        float in, out;
        boolean forceShutdown = true;

        POWER_LED_OFF();
        while (millis()-NOW < RecycleTime)
        {
          if (in > 6.283) in = 0;
          in += .00628;

          out = sin(in) * 127.5 + 127.5;
          analogWrite(BTN_LED_RED, out);
          delayMicroseconds(1500);
          
          //account for force-shutdown action (if button held for ForcedShutoffDelay, then force shutdown regardless)
          if (millis()-NOW <= (ForcedShutoffDelay-SHUTDOWNHOLDTIME) && digitalRead(BUTTON) != 0)
            forceShutdown = false;
          if (millis()-NOW >= (ForcedShutoffDelay-SHUTDOWNHOLDTIME) && forceShutdown)
          {
            PowerState = OFF;
            POWER_LED_OFF(); //digitalWrite(POWER_LED, PowerState); //turn off LED to indicate power is being cutoff
            POWER(PowerState);
            break;
          }

          if (millis() - NOW > ShutoffTriggerDelay)
          {
            // Pi signaling OK to turn off
            if (!BOOTOK())
            {
              PowerState = OFF;
              POWER_LED_OFF(); //digitalWrite(POWER_LED, PowerState); //turn off LED to indicate power is being cutoff
              NOW = millis();
              while (millis()-NOW < ShutdownFinalDelay)
              {
                if (in > 6.283) in = 0;
                in += .00628;
                out = sin(in) * 127.5 + 127.5;
                analogWrite(BTN_LED_RED,out);
                delayMicroseconds(300);
              }

              POWER(PowerState);
              break;
            }
          }
        }

        // last chance: if power still on but button still pressed, force cutoff power
        if (PowerState == ON && digitalRead(BUTTON) == 0)
        {
          PowerState = OFF;
          POWER(PowerState);
        }
        
        digitalWrite(SIG_SHUTOFF, LOW);
        //DEBUGln("SIG_SHUTOFF - LOW");
      }
      else if (PowerState == ON && !BOOTOK())
      {
        sprintf(lcdbuff, "Forced shutdown..");
        refreshLCD();
        
        NOW = millis();
        unsigned long NOW2 = millis();
        int analogstep = 255 / ((ForcedShutoffDelay-SHUTDOWNHOLDTIME)/100); //every 500ms decrease LED intensity
        while (digitalRead(BUTTON) == 0)
        {
          if (millis()-NOW2 > 100)
          {
            analogWrite(BTN_LED_RED, 255 - ((millis()-NOW)/100)*analogstep);
            NOW2 = millis();
          }
          if (millis()-NOW > ForcedShutoffDelay-SHUTDOWNHOLDTIME)
          {
            //TODO: add blinking here to signal final shutdown delay
            PowerState = OFF;
            POWER(PowerState);
            break;
          }
        }
      }
      else if (PowerState == OFF)
      {
        PowerState = ON;
        batteryLowShutdown=false;
        POWER(PowerState);
      }
    }

    if (PowerState == ON) POWER_LED_GRN() else POWER_LED_OFF(); //digitalWrite(POWER_LED, PowerState);
  }
}

boolean BOOTOK() {
  return analogRead(SIG_BOOTOK) > 800;
}

void POWER(uint8_t ON_OFF) {
  digitalWrite(LATCH_EN, HIGH);
  delay(5);
  digitalWrite(LATCH_VAL, ON_OFF);
  digitalWrite(LATCH_EN, LOW);
}

void Beep(byte theDelay, boolean twoSounds)
{
  if (theDelay > 20) theDelay = 20;
  tone(BUZZER, 4200); //4200
  delay(theDelay);
  noTone(BUZZER);
  delay(10);
  if (twoSounds)
  {
    tone(BUZZER, 4500); //4500
    delay(theDelay);
    noTone(BUZZER);
  }
}

int checkFreeRAM()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

boolean readBattery() {
  //periodically read the battery voltage
  int currPeriod = millis()/PRINTPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod=currPeriod;
    systemVoltage = BATTERY_VOLTS(analogRead(BATTERYSENSE));
    dtostrf(systemVoltage, 3,2, BATstr);
    batteryLow = systemVoltage < LOWBATTERYTHRESHOLD;
    //DEBUG("VBAT: ");DEBUG(systemVoltage);
    return true; //signal that batt has been read
  }
  return false;
}

void setup() {
  setupPowerControl();
  Serial.begin(SERIAL_BAUD);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);

#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif

  sprintf(lcdbuff, "Listening @ %dmhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(lcdbuff);
  if (flash.initialize()) DEBUGln("SPI Flash Init OK!");
  else DEBUGln(F("SPI Flash MEM not found, skipping..."));

#ifdef ENABLE_ATC
  DEBUGln(F("RFM69_ATC Enabled (Auto Transmission Control)"));
#endif

  //LCD backlight
  pinMode(PIN_LCD_LIGHT, OUTPUT);
  LCD_BACKLIGHT(1);
  lcd.setRot180();
  drawLogo();
  delay(2000);
  readBattery();
  Serial.print(F("Free RAM bytes: "));Serial.println(checkFreeRAM());
  refreshLCD();
  delay(1500);
}

float tempf;
boolean newPacketReceived;
void loop() {
  handlePowerControl(); //checks any button presses and takes action
  handleSerialInput();  //checks for any serial input from the Pi computer

  //process any received radio packets
  newPacketReceived = false;
  if (radio.receiveDone())
  {
    rssi = radio.RSSI;
    if (radio.DATALEN > 0)
    {
      sprintf(lcdbuff, "[%d] %s", radio.SENDERID, radio.DATA);
      sprintf(RSSIstr, "%ddBm", rssi);
      Serial.print(lcdbuff);
      Serial.print(F("   [RSSI:"));Serial.print(rssi);Serial.print(']');
    }

    //check if the packet is a wireless programming request
    CheckForWirelessHEX(radio, flash, false); //non verbose DEBUG

    //respond to any ACK if requested
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      DEBUG(F("[ACK-sent]"));
    }
    Serial.println();
    Blink(LED,2);
    newPacketReceived = true;
  }
  if (readBattery() || newPacketReceived) refreshLCD();
  //LCD_BACKLIGHT(batteryLow ? 0 : 1);
}
