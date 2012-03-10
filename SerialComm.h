/*
  The JTAG Whisperer: An Arduino library for JTAG.

  By Mike Tsao <http://github.com/sowbug>.

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#ifndef INCLUDE_JTAG_WHISPERER_SERIAL_COMM_H
#define INCLUDE_JTAG_WHISPERER_SERIAL_COMM_H

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/delay.h>

#define UART_BAUD_RATE  115200
#include "lib/uart.h"

#ifndef DEBUG
    #define DEBUG 0
#endif

void SerialComm_init();
uint8_t SerialComm_GetNextByte();
void SerialComm_Important(const char* format, ...);
void SerialComm_Ready(const char* message);
void SerialComm_Quit(const char* message);
void SerialComm_Debug(const char* format, ...);
void SerialComm_DebugBytes(const char* s, const uint8_t* p, uint8_t n);

#endif  // INCLUDE_JTAG_WHISPERER_SERIAL_COMM_H
