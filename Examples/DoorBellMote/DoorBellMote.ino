// **********************************************************************************
// DoorBellMote sketch works with Moteinos equipped with RFM69W/RFM69HW
// Can be adapted to use Moteinos/Arduinos using RFM12B or other RFM69 variants (RFM69CW, RFM69HCW)
// http://www.LowPowerLab.com/
// 2015-04-13 (C) Felix Rusu of http://www.LowPowerLab.com/
// **********************************************************************************
// It detects current flow at the doorbell transformer and send a message each time to the gateway
// It can trigger doorbell through a relay powered from pins D6+D7
// Deploy and forget: wirelessly programmable via Moteino + WirelessHEX69 library
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
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define GATEWAYID   1
#define NODEID      133
#define NETWORKID   100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define CHIMEPIN              4 // active HIGH chime signal from detector H11AA1 circuit
#define RELAYPIN1             6
#define RELAYPIN2             7
#define RELAY_PULSE_MS      250  //just enough that the doorbell chimne will trigger
#define RINGDELAY          3000
//*****************************************************************************************************************************
#define LED                  9   //pin connected to onboard LED
#define SERIAL_BAUD     115200
#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;
/////////////////////////////////////////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0xEF30 for windbond 4mbit flash (Moteino OEM)
/////////////////////////////////////////////////////////////////////////////
SPIFlash flash(8, 0xEF30); //regular Moteinos have FLASH MEM on D8, MEGA has it on D15

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  pinMode(CHIMEPIN, INPUT);
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

  char buff[50];
  sprintf(buff, "DoorBellMote : %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
}

uint32_t doorPulseCount = 0;
uint32_t lastStatusTimestamp=0;
uint32_t LEDCYCLETIMER=0;
byte LEDSTATE=LOW;
char input;
byte ring=false;

void loop()
{
  if (Serial.available())
    input = Serial.read();
    
  if (input=='r')
  {
    DEBUGln("Relay test...");
    pulseRelay();
    input = 0;
  }
 
  if (millis()-(lastStatusTimestamp)>RINGDELAY)
  {
    delay(50); //some basic debouncing
    if (digitalRead(CHIMEPIN) == HIGH)
    {
      lastStatusTimestamp = millis();
      radio.sendWithRetry(GATEWAYID, "RING", 4);
      Blink(LED,20);
      Blink(LED,20);
      Blink(LED,20);
    }
  }

  if (radio.receiveDone())
  {
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    //
    if (radio.DATALEN==4)
      if (radio.DATA[0]=='R' && radio.DATA[1]=='I' && radio.DATA[2]=='N' && radio.DATA[3]=='G')
        ring = true;
    
    // wireless programming token check
    // DO NOT REMOVE, or GarageMote will not be wirelessly programmable any more!
    CheckForWirelessHEX(radio, flash, true);

    //first send any ACK to request
    DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUG("]");
    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUG(" - ACK sent.");
    }

    if (ring)
    {
      pulseRelay();
      ring = false;
    }

    DEBUGln();
  }
  
  if (millis() - LEDCYCLETIMER > 3000)
  {
    LEDCYCLETIMER = millis();
    LEDSTATE = !LEDSTATE;
    digitalWrite(LED, LEDSTATE);
  }
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
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}
