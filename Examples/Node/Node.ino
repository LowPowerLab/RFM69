#include <RFM69.h>
#include <SPI.h>

#define NODEID      25
#define NETWORKID   100
#define GATEWAYID   1
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_915MHZ)
#define KEY         "thisIsEncryptKey"
#define LED         9
#define SERIAL_BAUD 115200
#define ACK_TIME    10  // # of ms to wait for an ack

int TRANSMITPERIOD = 500; //transmit a packet to gateway so often (in ms)
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
byte sendSize = 0;
boolean requestACK = false;
RFM69 radio;

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  //radio.setHighPower(); //only for RFM69HW!
  radio.encrypt(KEY);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

long lastPeriod = -1;
void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input - 48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }
    
    if (input == 'd') //d=dump register values
      radio.readAllRegs();
    if (input == 'E') //E=enable encryption
      radio.encrypt(KEY);
    if (input == 'e') //e=disable encryption
      radio.encrypt(null);
  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print("]");

    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      Serial.print(" - ACK sent.");
    }
    Blink(LED,5);
    Serial.println();
  }
  
  int currPeriod = millis() / TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod=currPeriod;
    Serial.print("Sending[");
    Serial.print(sendSize);
    Serial.print("]: ");
    for(byte i = 0; i < sendSize; i++)
      Serial.print((char)payload[i]);

    if (radio.sendWithRetry(GATEWAYID, payload, sendSize))
     Serial.print(" ok!");
    else Serial.print(" nothing...");

//   //manual ACK handling
//    requestACK = ((sendSize % 3) == 0); //request ACK every 3rd xmission
//    radio.send(GATEWAYID, payload, sendSize, requestACK);
//    if (requestACK)
//    {
//      Serial.print(" - waiting for ACK...");
//      if (waitForAck(GATEWAYID)) Serial.print("ok!");
//      else Serial.print("nothing...");
//    }


    sendSize = (sendSize + 1) % 31;
    Serial.println();
    Blink(LED, 3);
  }
}

void Blink(byte pin, int delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delayMs);
  digitalWrite(pin, LOW);
}

//// wait a few milliseconds for proper ACK to me, return true if indeed received
//static bool waitForAck(byte theNodeID) {
//  long now = millis();
//  while (millis() - now <= ACK_TIME) {
//    if (radio.ACKReceived(theNodeID))
//      return true;
//  }
//  return false;
//}
