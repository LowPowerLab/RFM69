#ifndef _RFM69_SPI_H
#define _RFM69_SPI_H

// Special SPI definition to support ATtiny microcontrollers
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__)

#include <util/atomic.h>

//USI ports and pins
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define SPI_DDR_PORT DDRA
#define USCK_DD_PIN DDA4
#define DO_DD_PIN DDA5
#define DI_DD_PIN DDA6
#elif defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define SPI_DDR_PORT DDRB
#define USCK_DD_PIN DDB2
#define DO_DD_PIN DDB1
#define DI_DD_PIN DDB0
#endif

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C


class ATtinySPI
{
  /*----------------------------------------------------------------------*
   * The ATtinySPI class was derived from tinySPI by Jack Christensen and *
   * is licensed under CC BY-SA 3.0. Changes were made to the original    *
   * work which can be found at https://github.com/JChristensen/tinySPI   *
   *                                                                      *
   * CC BY-SA:                                                            *
   * This work is licensed under the Creative Commons Attribution-        *
   * ShareAlike 3.0 Unported License. To view a copy of this license,     *
   * visit http://creativecommons.org/licenses/by-sa/3.0/                 *
   *----------------------------------------------------------------------*/
    public:
        static void begin(void)
        {
          USICR &= ~(_BV(USISIE) | _BV(USIOIE) | _BV(USIWM1));
          USICR |= _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);
          SPI_DDR_PORT |= _BV(USCK_DD_PIN);   //set the USCK pin as output
          SPI_DDR_PORT |= _BV(DO_DD_PIN);     //set the DO pin as output
          SPI_DDR_PORT &= ~_BV(DI_DD_PIN);    //set the DI pin as input
        }

        inline static uint8_t transfer(uint8_t data)
        {
          USIDR = data;
          USISR = _BV(USIOIF);                //clear counter and counter overflow interrupt flag
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //ensure a consistent clock period
              while ( !(USISR & _BV(USIOIF)) ) USICR |= _BV(USITC);
          }
          return USIDR;
        }

        static void end(void)
        {
          USICR &= ~(_BV(USIWM1) | _BV(USIWM0));
        }

        // The following functions aren't supported but exist here for backward
        // compatibility with the RFM69 library. They are not necessary on the 
        // ATtiny devices.
        inline static void setBitOrder(uint8_t)
        {
          __asm__("nop\n\t");
        }

        inline static void setDataMode(uint8_t)
        {
          __asm__("nop\n\t");
        }

        inline static void setClockDivider(uint8_t)
        {
          __asm__("nop\n\t");
        }
};

extern ATtinySPI SPI;

#else
  // For all other devices use the standard SPI library
  #include <SPI.h>

#endif // ATtiny device defines

#endif // _RFM69_SPI_H