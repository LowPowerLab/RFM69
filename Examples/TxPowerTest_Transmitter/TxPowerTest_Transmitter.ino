// TxPowerTest_Transmitter for RFM69 transceiver radios
// This turns on the Transmitter (unmodulated carrier) continuously
// Should be used experimentally to measure power output and current of the transmitter
// Use on a frequency that does not interfere with any other known active frequencies
// Ensure settings match with the Receiver sketch if you use that to measure the RSSI
// Trasmitter is toggled with 't' (or tactile/SPST button that pulls A0 to GND)
// Transmitter power is controlled with +,- in steps, or <,> in dBm
// **********************************************************************************
// Copyright Felix Rusu 2021, http://www.LowPowerLab.com/contact
// **********************************************************************************
#include <RFM69.h>      //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>

#define NODEID        123   //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all nodes that talk to each other (range up to 255)

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define FREQUENCY_EXACT 915500000
#define IS_RFM69HW_HCW   //uncomment only for RFM69HCW! Leave out if you have RFM69CW!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI -90

#define SERIAL_BAUD   500000
#define DEBUG(input)   Serial.print(input)
#define DEBUGln(input) Serial.println(input)
#define DEBUGHEX(input, param) Serial.print(input, param)

#ifdef ENABLE_ATC
  RFM69_ATC radio/*(10,11) FOR M0 radio add-on board*/;  //#if defined (__AVR_ATmega32U4__) RFM69_ATC radio(8,7);
#else
  RFM69 radio;  //#if defined (__AVR_ATmega32U4__) RFM69 radio(8,7);
#endif

int MODE;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(0, OUTPUT); digitalWrite(0, LOW);
  pinMode(A0, INPUT_PULLUP);

  DEBUGln("START RFM69_NODE_TX_TEST!");

  if (!radio.initialize(FREQUENCY,NODEID,NETWORKID))
    DEBUGln("radio.init() FAIL");
  else
    DEBUGln("radio.init() SUCCESS");

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //only for RFM69HW/HCW!
#endif

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  char buff[50];
  sprintf(buff, "\nTransmitting at %lu Mhz...", radio.getFrequency()/1000000L);
  DEBUGln(buff);

  DEBUGln("Use:\n+,- to adjust power in _powerLevel steps");
  DEBUGln("<,> to adjust power in dBm");

#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  MODE = RF69_MODE_SLEEP;
  radio.setMode(MODE);
}

int8_t dBm=-18; //start at minimum possible value for W/CW, gets bumped by library to -2 for HW/HCW
void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'r') //d=dump register values
      radio.readAllRegsCompact();

    if (input == 'R') //d=dump register values
      radio.readAllRegs();

    if (input=='+') {
      radio.setPowerLevel(radio.getPowerLevel()+1);
      DEBUG("_powerLevel=");DEBUGln(radio.getPowerLevel());
    }
    if (input=='-') {
      if (radio.getPowerLevel()>0) {
        radio.setPowerLevel(radio.getPowerLevel()-1);
      }
      DEBUG("_powerLevel=");DEBUGln(radio.getPowerLevel());
    }
    if (input=='<') {
      dBm = radio.setPowerDBm(dBm-1);
      DEBUG("POWER=");DEBUG(dBm);DEBUG(" (dBm); _powerLevel=");DEBUGln(radio.getPowerLevel());
    }
    if (input=='>') {
      dBm = radio.setPowerDBm(dBm+1);
      DEBUG("POWER=");DEBUG(dBm);DEBUG(" (dBm); _powerLevel=");DEBUGln(radio.getPowerLevel());
    }

    //transmit mode toggle (enables unmodulated carrier at current power level)
    if (input=='t') {
      if (MODE == RF69_MODE_TX) {
        MODE = RF69_MODE_SLEEP;
        DEBUG("RADIO_MODE = 0/SLEEP; _powerLevel=");DEBUGln(radio.getPowerLevel());
      }
      else {
        MODE = RF69_MODE_TX;
        DEBUG("RADIO_MODE = 4/TX;    _powerLevel=");DEBUGln(radio.getPowerLevel());
      }
    }
  }
  delay(200);
  if (MODE == RF69_MODE_TX) radio.setMode(MODE);
  else if (digitalRead(A0)==LOW) radio.setMode(RF69_MODE_TX);
  else radio.setMode(RF69_MODE_SLEEP);
}
