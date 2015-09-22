// Sample RFM69 random temperature sender at fixed intervals (F)
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 library at: https://github.com/LowPowerLab/
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define NODEID        98    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW   //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ACK_TIME      30 // max # of ms to wait for an ack

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define SERIAL_BAUD   115200

int TRANSMITPERIOD = 10000; //transmit a packet to gateway so often (in ms)
char buff[35];
byte sendSize=0;
RFM69 radio;

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

long lastPeriod = 0;
long randNumber1 = 0;
long randNumber2 = 0;
void loop() {
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    Blink(LED,3);
    Serial.println();
  }

  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod=currPeriod;
    randNumber1 = random(99);
    randNumber2 = random(99);

    if (randNumber2 > 70)
      sprintf(buff, "F:%d.5", randNumber1);
    else
      sprintf(buff, "F:-%d.5", randNumber1);
    sendSize = strlen(buff);
    
    if (radio.sendWithRetry(GATEWAYID, buff, sendSize))
     Serial.print(" ok!");
    else Serial.print(" nothing...");

    Serial.println();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
