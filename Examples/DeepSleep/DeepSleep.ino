//***********************************************************************************************************
// Sample sketch that achieves the lowest power on a Moteino of ~6.5uA
// Everything is put to sleep including the MCU, the radio (if any) and the FlashMem chip
//**** SETTINGS *********************************************************************************************
#define WITH_RFM69              //comment this line out if you don't have a RFM69 on your Moteino
#define WITH_SPIFLASH           //comment this line out if you don't have the FLASH-MEM chip on your Moteino
//***********************************************************************************************************
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

#if defined(WITH_RFM69) || defined(WITH_SPIFLASH)
  #include <SPI.h>                //comes with Arduino IDE (www.arduino.cc)
  #if defined(WITH_RFM69)
    #include <RFM69.h>            //get it here: https://www.github.com/lowpowerlab/rfm69
    RFM69 radio;
  #endif
  #if defined(WITH_SPIFLASH)
    #include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
    SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
  #endif
#endif

void setup () {
#ifdef WITH_RFM69
  radio.initialize(RF69_915MHZ,2,100);
  radio.sleep();
#endif

#ifdef WITH_SPIFLASH
  if (flash.initialize())
    flash.sleep();
#endif

  for (uint8_t i=0; i<=A5; i++)
  {
#ifdef WITH_RFM69
    if (i == RF69_SPI_CS) continue;
#endif
#ifdef WITH_SPIFLASH
    if (i == SS_FLASHMEM) continue;
#endif
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  pinMode(0, OUTPUT);
  digitalWrite(0,HIGH);
  pinMode(1, OUTPUT);
  digitalWrite(1,HIGH);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(20);
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(20);
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(20);
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  Serial.begin(115200);
}

void longSleep(uint32_t sleepTime) {
  do {
    if (sleepTime > 8000)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      sleepTime-=8000;
    }
    else if (sleepTime > 4000)
    {
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
      sleepTime-=4000;
    }
    else if (sleepTime > 2000)
    {
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      sleepTime-=2000;
    }
    else if (sleepTime > 1000)
    {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      sleepTime-=1000;
    }
    else if (sleepTime > 512)
    {
      LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
      sleepTime-=512;
    }
    else if (sleepTime > 256)
    {
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      sleepTime-=256;
    }
    else if (sleepTime > 128)
    {
      LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
      sleepTime-=128;
    }
    else if (sleepTime > 64)
    {
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
      sleepTime-=64;
    }
    else if (sleepTime > 32)
    {
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
      sleepTime-=32;
    }
    else if (sleepTime > 16)
    {
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      sleepTime-=16;
    }
    else
    {
      sleepTime=0;
    }
  } while(sleepTime);
}

byte counter=0;
void loop () 
{
/*
  //optional blink to know radio/flash sleeping went OK
  digitalWrite(LED_BUILTIN, HIGH);
  //LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  Serial.print("LOOP");
  Serial.println(counter++);
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
*/
  //sleep MCU for 2seconds
  digitalWrite(LED_BUILTIN, LOW);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  //delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  delay(100);
  //sleep MCU for a custom # of millis
  //longSleep(3000);
}