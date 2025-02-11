// Sample RFM69 sketch, with optional encryption, Automatic Transmission Control
// This will only init the radio module and print the values of all getter functions
// No data is transmitted or received. The device will only blink the onboard LED.
// **********************************************************************************
// Copyright Felix Rusu 2025, http://www.LowPowerLab.com/contact
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
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
// Address IDs are 10bit, meaning usable ID range is 1..1023
// Address 0 is special (broadcast), messages to address 0 are received by all *listening* nodes (ie. active RX mode)
// Gateway ID should be kept at ID=1 for simplicity, although this is not a hard constraint
//*********************************************************************************************
#define NODEID        1    // keep UNIQUE for each node on same network
#define NETWORKID     100  // keep IDENTICAL on all nodes that talk to each other

//*********************************************************************************************
// Frequency should be set to match the radio module hardware tuned frequency,
// otherwise if say a "433mhz" module is set to work at 915, it will work but very badly.
// Moteinos and RF modules from LowPowerLab are marked with a colored dot to help identify their tuned frequency band,
// see this link for details: https://lowpowerlab.com/guide/moteino/transceivers/
// The below examples are predefined "center" frequencies for the radio's tuned "ISM frequency band".
// You can always set the frequency anywhere in the "frequency band", ex. the 915mhz ISM band is 902..928mhz.
//*********************************************************************************************
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define FREQUENCY_EXACT 916000000 // you may define an exact frequency/channel in Hz
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************
#define SERIAL_BAUD    115200
#define DEBUG(input)   Serial.print(input)
#define DEBUGln(input) Serial.println(input)

#define BLINK_INTERVAL 1000      //< [ms] LED heartbeat blink interval

uint32_t last_blink = 0;

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  DEBUGln("START SETUP TEST!");

  if (!radio.initialize(FREQUENCY,NODEID,NETWORKID))
    DEBUGln("radio.init() FAIL");
  else
    DEBUGln("radio.init() SUCCESS");

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); // must include this only for RFM69HW/HCW!
#endif

#ifdef ENCRYPTKEY
  radio.encrypt(ENCRYPTKEY);
#endif

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); // set frequency to some custom frequency
#endif

// Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
// For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
// For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
// Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  test_config();
  radio.readAllRegs();
}

void loop() {
  uint32_t now = millis();

  if (now > (last_blink + BLINK_INTERVAL)) {
    last_blink = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void test_config() {
  /*
  RFM version: 36
  Node address: 2
  Node network: 100
  Frequency: 433MHz
  Frequency deviation: 49987Hz
  Bit rate: 55555bits/sec
  Spy mode active: no
  HCW module: yes
  TX Power 20dBM, 100.00mW
  Temperature: 24 Celsius
  Sync active: yes
  Sync size: 1
  CRC on: yes
  AES on: no
  Target RSSI:-80
  */
  DEBUG("RFM version: "); DEBUGln(radio.getVersion());
  DEBUG("Node address: "); DEBUGln(radio.getAddress());
  DEBUG("Node network: "); DEBUGln(radio.getNetwork());
  DEBUG("Frequency: "); DEBUG(radio.getFrequency()/1000000L); DEBUGln("MHz");
  DEBUG("Frequency deviation: "); DEBUG(radio.getFrequencyDeviation()); DEBUGln("Hz");
  DEBUG("Bit rate: "); DEBUG(radio.getBitRate()); DEBUGln("bits/sec");
  DEBUG("Spy mode active: "); DEBUGln(radio.getSpyMode()==1 ? "yes":"no");
  DEBUG("HCW module: "); DEBUGln(radio.isHighPower()==1 ? "yes":"no");

  uint8_t tx_power = radio.getPowerLevel();
  DEBUG("TX Power "); DEBUG(tx_power); DEBUG("dBM, "); DEBUG(radio.dBm_to_mW(tx_power)); DEBUGln("mW");
  DEBUG("Temperature: "); DEBUG(radio.readTemperature()); DEBUGln(" Celsius");
  DEBUG("Sync active: "); DEBUGln(radio.isSyncOn()==1 ? "yes":"no");
  DEBUG("Sync size: "); DEBUGln(radio.getSyncSize());
  DEBUG("CRC on: "); DEBUGln(radio.isCrcOn()==1 ? "yes":"no");
  DEBUG("AES on: "); DEBUGln(radio.isAesOn()==1 ? "yes":"no");

#ifdef ENABLE_ATC
  DEBUG("Target RSSI by ATC: "); DEBUGln(radio.getTargetRssi());
#endif
}
