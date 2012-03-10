/*
  The JTAG Whisperer: An library for JTAG.

  Original code for Arduino by Mike Tsao <http://github.com/sowbug>.
  Rewriten to C for avr-gcc by Vlastimil Slintak <slintak@uart.cz>

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#include "BitTwiddler.h"

// All Pins are on PORTB
#define     TMS  (_BV(0))  // Arduino 8
#define     TDI  (_BV(1))  // Arduino 9
#define     TDO  (_BV(2))  // Arduino 10
#define     TCK  (_BV(3))  // Arduino 11

#define BitTwiddler_set_tms()   BitTwiddler_set_port(TMS);
#define BitTwiddler_clr_tms()   BitTwiddler_clr_port(TMS);
#define BitTwiddler_set_tdi()   BitTwiddler_set_port(TDI);
#define BitTwiddler_clr_tdi()   BitTwiddler_clr_port(TDI);

// The current PORTB state. We write this only when we twiddle TCK.
uint8_t _portb;

// Knows how to set MCU-specific pins in a JTAG-relevant way.
void BitTwiddler_init() {
    _portb = 0;
    DDRB = TMS | TDI | TCK;
}

inline void BitTwiddler_pulse_clock() {
    BitTwiddler_clr_port(TCK);
    _delay_us(1);
    BitTwiddler_set_port(TCK);
}

bool BitTwiddler_pulse_clock_and_read_tdo() {
    BitTwiddler_clr_port(TCK);
    _delay_us(1);
    uint8_t pinb = PINB;
    BitTwiddler_set_port(TCK);
    return pinb & TDO;
}

void BitTwiddler_wait_time(unsigned long microsec) {
    while (microsec--) {
      BitTwiddler_pulse_clock();
      _delay_us(1);
    }
}

inline void BitTwiddler_write_portb_if_tck(uint8_t pin) {
    if (pin == TCK) {
      PORTB = _portb;
    }
}

inline void BitTwiddler_set_port(uint8_t pin) {
    _portb |= pin;
    BitTwiddler_write_portb_if_tck(pin);
}

inline void BitTwiddler_clr_port(uint8_t pin) {
    _portb &= ~pin;
    BitTwiddler_write_portb_if_tck(pin);
}
