/*
  The JTAG Whisperer: An library for JTAG.

  Original code for Arduino by Mike Tsao <http://github.com/sowbug>.
  Rewriten to C for avr-gcc by Vlastimil Slintak <slintak@uart.cz>

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#ifndef INCLUDE_JTAG_WHISPERER_BIT_TWIDDLER_H
#define INCLUDE_JTAG_WHISPERER_BIT_TWIDDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>

// All Pins are on PORTB
#define     TMS  (_BV(0))  // Arduino 8
#define     TDI  (_BV(1))  // Arduino 9
#define     TDO  (_BV(2))  // Arduino 10
#define     TCK  (_BV(3))  // Arduino 11

#define BitTwiddler_set_tms()   BitTwiddler_set_port(TMS);
#define BitTwiddler_clr_tms()   BitTwiddler_clr_port(TMS);
#define BitTwiddler_set_tdi()   BitTwiddler_set_port(TDI);
#define BitTwiddler_clr_tdi()   BitTwiddler_clr_port(TDI);

// Knows how to set MCU-specific pins in a JTAG-relevant way.
void BitTwiddler_init();
bool BitTwiddler_pulse_clock_and_read_tdo();
void BitTwiddler_wait_time(unsigned long microsec);

void BitTwiddler_pulse_clock();
void BitTwiddler_write_portb_if_tck(uint8_t pin);
void BitTwiddler_set_port(uint8_t pin);
void BitTwiddler_clr_port(uint8_t pin);

#endif  // INCLUDE_JTAG_WHISPERER_BIT_TWIDDLER_H
