#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>

// Uncomment the appropriate frequency for your hardware
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define ENCRYPT_KEY "sampleEncryptKey"
#define ADDRESS 2
#define NETWORK_ID 100

RFM69 radio;

void setup() {
  Serial.begin(115200);
  Serial.println("Listen Mode Example");
  Serial.println("Press 'l' to enter listen mode");

  radio.initialize(FREQUENCY, ADDRESS, NETWORK_ID);
  radio.encrypt(ENCRYPT_KEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.ACKRequested()) {
      radio.sendACK();
    }
    Serial.println("Received a normal message");
    Serial.println((char*)radio.DATA);
    Serial.flush();
  }

  if (Serial.read() == 'l') {
    Serial.println("Entering low-power listen mode...");
    Serial.flush();

    radio.listenModeStart();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Woke up, check for a message
    Serial.println("Woke up!");
    uint8_t from = 0;
    long burstRemaining = 0;
    if (radio.DATALEN > 0) {
      Serial.println("Received a message in listen mode");
      Serial.println((char*)radio.DATA);
      Serial.flush();
      from = radio.SENDERID;
      burstRemaining = radio.LISTEN_BURST_REMAINING_MS;
    }

    // Radio goes back to standby, ready for normal operations
    radio.listenModeEnd();

    if (from) {
      while (burstRemaining > 0) {
        LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
        burstRemaining -= 60;
      }
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
      radio.send(from, "woke", 4);
    }
  }
}