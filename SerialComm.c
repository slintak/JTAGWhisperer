/*
  The JTAG Whisperer: An library for JTAG.

  Original code for Arduino by Mike Tsao <http://github.com/sowbug>.
  Rewriten to C for avr-gcc by Vlastimil Slintak <slintak@uart.cz>

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#include "SerialComm.h"

enum { BUFFER_SIZE = 128 };
uint8_t buffer_[BUFFER_SIZE];
uint8_t* read_ptr_;
uint8_t* write_ptr_;
uint32_t stream_sum_;
uint32_t stream_count_;

void SerialComm_init() {
    read_ptr_ = buffer_;
    write_ptr_ = buffer_;
    stream_sum_ = 0;
    stream_count_ = 0;

    // For ATMega328P we need specify double speed
    uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU));
    SerialComm_Ready("XSVF");
  }

void SerialComm_p(const char *fmt, ... ) {
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, 128, fmt, args);
    va_end(args);
    //Serial.print(tmp);
    uart_puts(tmp);
}

void SerialComm_print_bytes(const uint8_t* pb, uint8_t count) {
    while (count--) {
      SerialComm_p("%02x", *pb++);
    }
}

bool SerialComm_available() {
    return read_ptr_ != write_ptr_;
}

void SerialComm_load() {
    while (uart_available() > 0) {
      uint8_t c = (uint8_t)uart_getc();
      *write_ptr_++ = c;
      if (write_ptr_ == buffer_ + BUFFER_SIZE) {
        write_ptr_ -= BUFFER_SIZE;
      }
      if (!SerialComm_available()) {
        SerialComm_Important("Overran serial buffer");
      }
    }
}

void SerialComm_fill() {
    SerialComm_load();
    if (!SerialComm_available()) {
      SerialComm_Ready("SEND");
      //int delay_time = 5;
      do {
        SerialComm_load();
        _delay_ms(5);
        //delay_time = delay_time + delay_time;
      } while (!SerialComm_available());
    }
}

uint8_t SerialComm_get() {
    if (!SerialComm_available()) {
      SerialComm_fill();
    }
    uint8_t next_byte = *read_ptr_++;
    if (read_ptr_ == buffer_ + BUFFER_SIZE) {
      read_ptr_ -= BUFFER_SIZE;
    }
    return next_byte;
}

//  virtual ~SerialComm() {
//    Important("Checksum %lx/%lx.", stream_sum_, stream_count_);
//    Quit("Exiting!");
//  }

uint8_t SerialComm_GetNextByte() {
    uint8_t c = SerialComm_get();
    stream_sum_ += c;
    ++stream_count_;
    return c;
}

void SerialComm_Important(const char* format, ...) {
    va_list args;
    va_start(args, format);

    char tmp[128];
    vsnprintf(tmp, 128, format, args);

    va_end(args);

    uart_puts("! ");
    uart_puts(tmp);
    uart_putc('\n');
}

void SerialComm_Ready(const char* message) {
    //Serial.print("\nR");
    //Serial.println(message);
    uart_puts("\nR");
    uart_puts(message);
    uart_puts("\r\n");
}

void SerialComm_Quit(const char* message) {
    //Serial.print("\nQ ");
    //Serial.println(message);
    uart_puts("\nQ");
    uart_puts(message);
    uart_putc('\n');
}

#if DEBUG == 1
void SerialComm_Debug(const char* format, ...) {
    va_list args;
    va_start(args, format);

    char tmp[128];
    vsnprintf(tmp, 128, format, args);

    va_end(args);

    //Serial.print("D ");
    //Serial.println(tmp);
    uart_puts("D ");
    uart_puts(tmp);
}

void SerialComm_DebugBytes(const char* s, const uint8_t* p, uint8_t n)  {
    Serial.print("D ");
    Serial.print(s);
    print_bytes(p, n);
    Serial.println();
}
#else
void SerialComm_Debug(const char* format, ...) {}
void SerialComm_DebugBytes(const char* s, const uint8_t* p, uint8_t n)  {}
#endif
