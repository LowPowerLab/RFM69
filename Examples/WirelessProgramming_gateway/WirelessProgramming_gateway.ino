/*
 * Copyright (c) 2013 by Felix Rusu <felix@lowpowerlab.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

// This sketch is an example of how wireless programming can be achieved with a Moteino
// that was loaded with a custom 1k Optiboot that is capable of loading a new sketch from
// an external SPI flash chip
// This is the GATEWAY node, it does not need a custom Optiboot nor any external FLASH memory chip
// (ONLY the target node will need those)
// The sketch includes logic to receive the new sketch from the serial port (from a host computer) and 
// transmit it wirelessly to the target node
// The handshake protocol that receives the sketch from the serial port 
// is handled by the SPIFLash/WirelessHEX69 library, which also relies on the RFM12B library
// These libraries and custom 1k Optiboot bootloader for the target node are at: http://github.com/lowpowerlab
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <WirelessHEX69.h>

#define MYID        1   // node ID used for this unit
#define TARGET_ID   55  // ID of node being wirelessly reprogrammed
#define NETWORKID   250
#define FREQUENCY   RF69_915MHZ // Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define LED         9
#define SERIAL_BAUD 115200
#define ACK_TIME    50  // # of ms to wait for an ack
#define TIMEOUT     3000
#define KEY         "thisIsEncryptKey"

RFM69 radio;
char c = 0;
char input[64]; //serial input buffer

void setup(){
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,MYID,NETWORKID);
  //radio.encrypt(KEY); //OPTIONAL
  Serial.print("Start wireless gateway...");
}

void loop(){
  byte inputLen = readSerialLine(input);
  
  if (inputLen == 4 && input[0]=='F' && input[1]=='L' && input[2]=='X' && input[3]=='?') {
    CheckForSerialHEX((byte*)input, inputLen, radio, TARGET_ID, TIMEOUT, ACK_TIME, true);
  }
  else if (inputLen>0) { //just echo back
    Serial.print("SERIAL IN > ");Serial.println(input);
  }
  
  if (radio.receiveDone())
  {
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    
    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    
    Serial.println();
  }
  Blink(LED,5); //heartbeat
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
