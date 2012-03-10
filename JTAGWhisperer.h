/*
  The JTAG Whisperer: An library for JTAG.

  Original code for Arduino by Mike Tsao <http://github.com/sowbug>.
  Rewriten to C for avr-gcc by Vlastimil Slintak <slintak@uart.cz>

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#ifndef INCLUDE_JTAG_WHISPERER_JTAG_WHISPERER_H
#define INCLUDE_JTAG_WHISPERER_JTAG_WHISPERER_H

#include "BitTwiddler.h"
#include "SerialComm.h"
#include "Utilities.h"
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#define BLINK_PIN   5
#define BUFFER_SIZE 32

void JTAGWhisperer_init();
bool JTAGWhisperer_reached_xcomplete();
uint8_t JTAGWhisperer_read_next_instruction();
bool JTAGWhisperer_handle_instruction(uint8_t instruction);

#endif  // INCLUDE_JTAG_WHISPERER_JTAG_WHISPERER_H
