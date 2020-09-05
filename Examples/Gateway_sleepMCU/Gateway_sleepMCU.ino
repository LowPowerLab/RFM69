// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// MCU in SLEEP_FOREVER, radio in active RX mode
// Passes through any wireless received messages to the serial port & responds to ACKs
// Library and code by Felix Rusu - felix@lowpowerlab.com
//****************************************************************************************************************
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>
#include <LowPower.h> //get it here: https://www.github.com/lowpowerlab/lowpower
//****************************************************************************************************************
#define NODEID        1    //keep Gateways at ID=1
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define FREQUENCY_EXACT 917000000 //uncomment to set to a specific frequency
#define IS_RFM69HW_HCW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //AUTO TRANSMISSION CONTROL - comment out to disable it
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SERIAL_BAUD   115200
//****************************************************************************************************************
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char buff[61];
void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
#ifdef ENCRYPTKEY
  radio.encrypt(ENCRYPTKEY);
#endif

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif

  sprintf(buff, "\nListening at %lu Mhz...", radio.getFrequency()/1000000L);

#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif

  pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t packetCount = 0;

void loop() {
  //ensure radio in RX mode before sleeping
  radio.receiveDone(); //this checks if a packet is ready and processes it (saves it in lib buffer), or else puts radio in RX mode

  Serial.println("FOREVER SLEEP...");
  Serial.flush();

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //sleep the MCU

  if (radio.receiveDone()) //WAKEUP by radio interrupt!
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("#[");
    Serial.print(++packetCount);
    Serial.print(']');
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");

    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);

    Serial.print("  [RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested()) radio.sendACK();

    Serial.println();
    digitalWrite(LED_BUILTIN, LOW);
  }
}