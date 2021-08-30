// TxPowerMeasurement_Receiver for RFM69 transceiver radios
// This will switch on the Receiver and continuously sample RSSI
// It also dials the AGC gain to the max value to simulate distance,
//   useful when bench testing to avoid RSSI saturation at high TX power levels
// **********************************************************************************
// Copyright Felix Rusu 2021, http://www.LowPowerLab.com/contact
// **********************************************************************************
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69Registers.h> //get it here: https://www.github.com/lowpowerlab/rfm69

#define NODEID        1   //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all nodes that talk to each other (range up to 255)

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define FREQUENCY_EXACT 915500000
#define IS_RFM69HW_HCW   //uncomment only for RFM69HCW! Leave out if you have RFM69CW!

#define SERIAL_BAUD   500000
#define DEBUG(input)   Serial.print(input)
#define DEBUGln(input) Serial.println(input)
#define DEBUGHEX(input, param) Serial.print(input, param)

RFM69 radio;  //#if defined (__AVR_ATmega32U4__) RFM69 radio(8,7);

int MODE;
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  DEBUGln("START RFM69_NODE_RX_TEST!");

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

  DEBUG("\nListening at %lu Mhz..."); DEBUGln(radio.getFrequency()/1000000L);

  //when bench testing, simulate "distance" between TX/RX radios
  radio.setLNA(RF_LNA_GAINSELECT_MAXMINUS48);
  radio.receiveDone(); //enable RX mode
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'r') //d=dump register values
      radio.readAllRegsCompact();

    if (input == 'R') //d=dump register values
      radio.readAllRegs();
  }
  delay(200);
  Serial.println(radio.readRSSI());
}
