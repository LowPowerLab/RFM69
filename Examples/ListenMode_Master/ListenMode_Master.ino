#include <SPI.h>
#include <RFM69.h>

// Uncomment the appropriate frequency for your hardware
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define ENCRYPT_KEY "sampleEncryptKey"
#define ADDRESS 1
#define NETWORK_ID 100
#define REMOTE_ADDRESS 2
#define BURST_REPLY_TIMEOUT_MS 250

RFM69 radio;

void setup() {
  Serial.begin(115200);
  Serial.println("Wake Example");
  Serial.println("Press 'b' to send a burst, 'm' for a normal message ");

  radio.initialize(FREQUENCY, ADDRESS, NETWORK_ID);
  radio.encrypt(ENCRYPT_KEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
}

char payload[] = "WAKE!";

void loop() {
  if (!Serial.available()) {
    return;
  }

  byte cmd = Serial.read();
  if (cmd == 'b') {
    Serial.print("Sending wakeup burst...");
    radio.listenModeSendBurst(REMOTE_ADDRESS, payload, sizeof(payload));

    bool replied = false;
    long start = millis();
    while (millis() - start < BURST_REPLY_TIMEOUT_MS) {
      if (radio.receiveDone()) {
        Serial.println("Success");
        replied = true;
        break;
      }
    }

    if (!replied) {
      Serial.println("Failed");
    }
  }

  if (cmd == 'm') {
    Serial.print("Sending normal message...");
    if (radio.sendWithRetry(REMOTE_ADDRESS, payload, sizeof(payload))) {
      Serial.println("Success");
    } else {
      Serial.println("Failed");
    }
  }
}