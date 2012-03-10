/*
  The JTAG Whisperer: An library for JTAG.

  Original code for Arduino by Mike Tsao <http://github.com/sowbug>.
  Rewriten to C for avr-gcc by Vlastimil Slintak <slintak@uart.cz>

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.
*/

#include "JTAGWhisperer.h"

static bool is_pin_on;

uint8_t current_state_;
uint8_t sdrsize_bits_;
uint8_t sdrsize_bytes_;
uint8_t repeat_;
uint32_t runtest_;
bool reached_xcomplete_;

uint8_t instruction_buffer_[BUFFER_SIZE];
uint8_t* bp_;
uint8_t tdi_[BUFFER_SIZE];
uint8_t tdo_[BUFFER_SIZE];
uint8_t tdomask_[BUFFER_SIZE];
uint8_t tdo_expected_[BUFFER_SIZE];

enum {
    XCOMPLETE = 0,
    XTDOMASK,
    XSIR,
    XSDR,
    XRUNTEST,
    XRESERVED_5,
    XRESERVED_6,
    XREPEAT,
    XSDRSIZE,
    XSDRTDO,
    XSETSDRMASKS,
    XSDRINC,
    XSDRB,
    XSDRC,
    XSDRE,
    XSDRTDOB,
    XSDRTDOC,
    XSDRTDOE,
    XSTATE,
    XENDIR,
    XENDDR,
    XSIR2,
    XCOMMENT,
    XWAIT
};

enum {
    STATE_TLR,
    STATE_RTI,
    STATE_SELECT_DR_SCAN,
    STATE_CAPTURE_DR,
    STATE_SHIFT_DR,
    STATE_EXIT1_DR,
    STATE_PAUSE_DR,
    STATE_EXIT2_DR,
    STATE_UPDATE_DR,
    STATE_SELECT_IR_SCAN,
    STATE_CAPTURE_IR,
    STATE_SHIFT_IR,
    STATE_EXIT1_IR,
    STATE_PAUSE_IR,
    STATE_EXIT2_IR,
    STATE_UPDATE_IR,
};

// These tables and the code that uses them are taken from
// https://github.com/ben0109/XSVF-Player/.
static const uint8_t tms_transitions[] = {
  0x01, /* STATE_TLR    */
  0x21, /* STATE_RTI    */
  0x93, /* STATE_SELECT_DR_SCAN */
  0x54, /* STATE_CAPTURE_DR */
  0x54, /* STATE_SHIFT_DR */
  0x86, /* STATE_EXIT1_DR */
  0x76, /* STATE_PAUSE_DR */
  0x84, /* STATE_EXIT2_DR */
  0x21, /* STATE_UPDATE_DR  */
  0x0a, /* STATE_SELECT_IR_SCAN */
  0xcb, /* STATE_CAPTURE_IR */
  0xcb, /* STATE_SHIFT_IR */
  0xfd, /* STATE_EXIT1_IR */
  0xed, /* STATE_PAUSE_IR */
  0xfb, /* STATE_EXIT2_IR */
  0x21, /* STATE_UPDATE_IR  */
};

static const uint16_t tms_map[] = {
  0x0000, /* STATE_TLR    */
  0xfffd, /* STATE_RTI    */
  0xfe03, /* STATE_SELECT_DR_SCAN */
  0xffe7, /* STATE_CAPTURE_DR */
  0xffef, /* STATE_SHIFT_DR */
  0xff0f, /* STATE_EXIT1_DR */
  0xffbf, /* STATE_PAUSE_DR */
  0xff0f, /* STATE_EXIT2_DR */
  0xfefd, /* STATE_UPDATE_DR  */
  0x01ff, /* STATE_SELECT_IR_SCAN */
  0xf3ff, /* STATE_CAPTURE_IR */
  0xf7ff, /* STATE_SHIFT_IR */
  0x87ff, /* STATE_EXIT1_IR */
  0xdfff, /* STATE_PAUSE_IR */
  0x87ff, /* STATE_EXIT2_IR */
  0x7ffd, /* STATE_UPDATE_IR  */
};


bool JTAGWhisperer_handle_XCOMPLETE();
bool JTAGWhisperer_handle_XTDOMASK();
bool JTAGWhisperer_handle_XSIR();
bool JTAGWhisperer_handle_XSDR();
bool JTAGWhisperer_handle_XRUNTEST();
bool JTAGWhisperer_handle_XREPEAT();
bool JTAGWhisperer_handle_XSDRSIZE();
bool JTAGWhisperer_handle_XSDRTDO();
bool JTAGWhisperer_handle_XSTATE();
bool JTAGWhisperer_handle_XWAIT();

void JTAGWhisperer_init() {
    SerialComm_init();
    BitTwiddler_init();

    current_state_ = -1;
    sdrsize_bits_ = 0;
    sdrsize_bytes_ = 0;
    repeat_ = 32;
    runtest_ = 0;
    reached_xcomplete_ = false;
}

void JTAGWhisperer_get_next_bytes_from_stream(uint8_t count) {
    while (count--) {
      *bp_++ = SerialComm_GetNextByte();
    }
}

uint8_t JTAGWhisperer_get_next_byte() {
    return *bp_++;
}

uint16_t JTAGWhisperer_get_next_word() {
    return (uint16_t)JTAGWhisperer_get_next_byte() << 8 | JTAGWhisperer_get_next_byte();
}

uint32_t JTAGWhisperer_get_next_long() {
    return (uint32_t)JTAGWhisperer_get_next_byte() << 24 |
      (uint32_t)JTAGWhisperer_get_next_byte() << 16 |
      (uint32_t)JTAGWhisperer_get_next_byte() << 8 |
      JTAGWhisperer_get_next_byte();
}

void JTAGWhisperer_get_next_bytes(uint8_t* data, uint8_t count) {
    while (count--) {
      *data++ = JTAGWhisperer_get_next_byte();
    }
}

bool JTAGWhisperer_reached_xcomplete() { return reached_xcomplete_; }

uint8_t JTAGWhisperer_read_next_instruction() {
    bp_ = instruction_buffer_;
    uint8_t instruction = SerialComm_GetNextByte();
    switch (instruction) {
    case XCOMPLETE:
      break;
    case XTDOMASK:
      JTAGWhisperer_get_next_bytes_from_stream(sdrsize_bytes_);
      break;
    case XSIR: {
      uint8_t length = SerialComm_GetNextByte();
      *bp_++ = length;
      JTAGWhisperer_get_next_bytes_from_stream(BYTES(length));
      break;
    }
    case XSDR:
    case XSDRB:
    case XSDRC:
    case XSDRE:
      JTAGWhisperer_get_next_bytes_from_stream(sdrsize_bytes_);
      break;
    case XRUNTEST:
      JTAGWhisperer_get_next_bytes_from_stream(4);
      break;
    case XREPEAT:
      *bp_++ = SerialComm_GetNextByte();
      break;
    case XSDRSIZE:
      JTAGWhisperer_get_next_bytes_from_stream(4);
      break;
    case XSDRTDO:
    case XSDRTDOB:
    case XSDRTDOC:
    case XSDRTDOE:
    case XSETSDRMASKS:
      JTAGWhisperer_get_next_bytes_from_stream(sdrsize_bytes_ + sdrsize_bytes_);
      break;
    case XSTATE:
      *bp_++ = SerialComm_GetNextByte();
      break;
    case XWAIT:
      JTAGWhisperer_get_next_bytes_from_stream(6);
      break;
    case XSDRINC:
    default:
      SerialComm_Important("Unexpected instruction %d", instruction);
      break;
    }
    return instruction;
}

const char *JTAGWhisperer_instruction_name(uint8_t instruction) {
    switch (instruction) {
      NAME_FOR(XCOMPLETE);
      NAME_FOR(XTDOMASK);
      NAME_FOR(XSIR);
      NAME_FOR(XSDR);
      NAME_FOR(XRUNTEST);
      NAME_FOR(XREPEAT);
      NAME_FOR(XSDRSIZE);
      NAME_FOR(XSDRTDO);
      NAME_FOR(XSETSDRMASKS);
      NAME_FOR(XSDRINC);
      NAME_FOR(XSDRB);
      NAME_FOR(XSDRC);
      NAME_FOR(XSDRE);
      NAME_FOR(XSDRTDOB);
      NAME_FOR(XSDRTDOC);
      NAME_FOR(XSDRTDOE);
      NAME_FOR(XSTATE);
      NAME_FOR(XENDIR);
      NAME_FOR(XENDDR);
      NAME_FOR(XSIR2);
      NAME_FOR(XCOMMENT);
      NAME_FOR(XWAIT);
    default:
      return "XWTF";
    }
}

bool JTAGWhisperer_handle_instruction(uint8_t instruction) {
    bp_ = instruction_buffer_;

    SerialComm_Debug("Handling %s", JTAGWhisperer_instruction_name(instruction));
    switch (instruction) {
      HANDLE(XCOMPLETE);
      HANDLE(XTDOMASK);
      HANDLE(XSIR);
      HANDLE(XSDR);
      HANDLE(XRUNTEST);
      HANDLE(XREPEAT);
      HANDLE(XSDRSIZE);
      HANDLE(XSDRTDO);
      HANDLE(XSTATE);
      HANDLE(XWAIT);
    default:
      SerialComm_Debug("Unimplemented instruction: %d", instruction);
      return false;
    }
}

const char *JTAGWhisperer_state_name(uint8_t state) {
    switch (state) {
      NAME_FOR(STATE_TLR);
      NAME_FOR(STATE_RTI);
      NAME_FOR(STATE_SELECT_DR_SCAN);
      NAME_FOR(STATE_CAPTURE_DR);
      NAME_FOR(STATE_SHIFT_DR);
      NAME_FOR(STATE_EXIT1_DR);
      NAME_FOR(STATE_PAUSE_DR);
      NAME_FOR(STATE_EXIT2_DR);
      NAME_FOR(STATE_UPDATE_DR);
      NAME_FOR(STATE_SELECT_IR_SCAN);
      NAME_FOR(STATE_CAPTURE_IR);
      NAME_FOR(STATE_SHIFT_IR);
      NAME_FOR(STATE_EXIT1_IR);
      NAME_FOR(STATE_PAUSE_IR);
      NAME_FOR(STATE_EXIT2_IR);
      NAME_FOR(STATE_UPDATE_IR);
    default:
      return "STATE_WTF";
    }
}

bool JTAGWhisperer_is_tdo_as_expected() {
    SerialComm_DebugBytes("... received  ", tdo_, sdrsize_bytes_);
    for (int i = 0; i < sdrsize_bytes_; ++i) {
      uint8_t expected = tdo_expected_[i] & tdomask_[i];
      uint8_t actual = tdo_[i] & tdomask_[i];
      if (expected != actual) {
        SerialComm_Debug("... NO MATCH.");
        return false;
      }
    }
    SerialComm_Debug("... match!");
    return true;
}

void JTAGWhisperer_set_state(int state) {
    current_state_ = state;
}

void JTAGWhisperer_state_ack(bool tms) {
    if (tms) {
      JTAGWhisperer_set_state((tms_transitions[current_state_] >> 4) & 0xf);
    } else {
      JTAGWhisperer_set_state(tms_transitions[current_state_] & 0xf);
    }
}

void JTAGWhisperer_state_step(bool tms) {
    if (tms) {
      BitTwiddler_set_tms();
    } else {
      BitTwiddler_clr_tms();
    }
    BitTwiddler_pulse_clock();
    JTAGWhisperer_state_ack(tms);
}

void JTAGWhisperer_state_goto(int state) {
    if (state == STATE_TLR) {
      for (int i = 0; i < 5; ++i) {
        JTAGWhisperer_state_step(true);
      }
    } else {
      while (current_state_ != state) {
        JTAGWhisperer_state_step((tms_map[current_state_] >> state) & 1);
      }
    }
}


void JTAGWhisperer_shift_td(uint8_t *data, uint16_t data_length_bits, bool is_end) {
    int byte_count = BYTE_COUNT(data_length_bits);

    for (int i = 0; i < byte_count; ++i) {
      uint8_t byte_out = data[byte_count - 1 - i];
      uint8_t tdo_byte = 0;
      for (int j = 0; j < 8 && data_length_bits-- > 0; ++j) {
        if (data_length_bits == 0 && is_end) {
          BitTwiddler_set_tms();
          JTAGWhisperer_state_ack(1);
        }

        if (byte_out & 1) {
          BitTwiddler_set_tdi();
        } else {
          BitTwiddler_clr_tdi();
        }
        byte_out >>= 1;
        bool tdo = BitTwiddler_pulse_clock_and_read_tdo();
        tdo_byte |= tdo << j;
      }
      tdo_[byte_count - 1 - i] = tdo_byte;
    }
}

void JTAGWhisperer_wait_time(uint32_t microseconds) {
    SerialComm_Debug("Waiting %ld microseconds...", microseconds);
    while (microseconds--) {
      BitTwiddler_pulse_clock();
      _delay_us(1);
    }
}

bool JTAGWhisperer_sdr(bool should_begin, bool should_end, bool should_check) {
    int attempts_left = repeat_;
    bool matched = false;

    if (should_begin) {
      JTAGWhisperer_state_goto(STATE_SHIFT_DR);
    }

    while (!matched && attempts_left-- > 0) {
      JTAGWhisperer_shift_td(tdi_, sdrsize_bits_, should_end);
      if (should_check) {
        if (JTAGWhisperer_is_tdo_as_expected()) {
          matched = true;
        } else {
          JTAGWhisperer_state_goto(STATE_PAUSE_DR);
          JTAGWhisperer_state_goto(STATE_SHIFT_DR);
          JTAGWhisperer_state_goto(STATE_RTI);
          JTAGWhisperer_wait_time(runtest_);
          JTAGWhisperer_state_goto(STATE_SHIFT_DR);
        }
      }
    }
    if (should_check && !matched) {
      SerialComm_Important("SDR check failed.");
      return false;
    }
    if (should_end) {
      JTAGWhisperer_state_goto(STATE_RTI);
    }
    JTAGWhisperer_wait_time(runtest_);
    return true;
}

bool JTAGWhisperer_handle_XCOMPLETE() {
    SerialComm_Important("XCOMPLETE");
    reached_xcomplete_ = true;
    return false;
}

bool JTAGWhisperer_handle_XTDOMASK() {
    JTAGWhisperer_get_next_bytes(tdomask_, sdrsize_bytes_);
    SerialComm_DebugBytes("... tdomask now ", tdomask_, sdrsize_bytes_);
    return true;
}

bool JTAGWhisperer_handle_XSIR() {
    uint8_t bits = JTAGWhisperer_get_next_byte();
    JTAGWhisperer_get_next_bytes(tdi_, BYTES(bits));
    JTAGWhisperer_state_goto(STATE_SHIFT_IR);
    JTAGWhisperer_shift_td(tdi_, bits, true);
    JTAGWhisperer_state_goto(STATE_RTI);
    return true;
}

bool JTAGWhisperer_handle_XSDR() {
    JTAGWhisperer_get_next_bytes(tdi_, sdrsize_bytes_);
    SerialComm_DebugBytes("... sending ", tdi_, sdrsize_bytes_);
    return JTAGWhisperer_sdr(true, true, true);
}

bool JTAGWhisperer_handle_XRUNTEST() {
    runtest_ = JTAGWhisperer_get_next_long();
    SerialComm_Debug("... runtest now %ld", runtest_);
    return true;
}

bool JTAGWhisperer_handle_XREPEAT() {
    repeat_ = JTAGWhisperer_get_next_byte();
    SerialComm_Debug("... repeat now %d", repeat_);
    return true;
}

bool JTAGWhisperer_handle_XSDRSIZE() {
    sdrsize_bits_ = JTAGWhisperer_get_next_long();
    sdrsize_bytes_ = BYTES(sdrsize_bits_);
    SerialComm_Debug("... sdrsize now %d/%d", sdrsize_bits_, sdrsize_bytes_);
    return true;
}

bool JTAGWhisperer_handle_XSDRTDO() {
    JTAGWhisperer_get_next_bytes(tdi_, sdrsize_bytes_);
    JTAGWhisperer_get_next_bytes(tdo_expected_, sdrsize_bytes_);
    SerialComm_DebugBytes("... sending   ", tdi_, sdrsize_bytes_);
    SerialComm_DebugBytes("... expecting ", tdo_expected_, sdrsize_bytes_);
    return JTAGWhisperer_sdr(true, true, true);
}

bool JTAGWhisperer_handle_XSTATE() {
    JTAGWhisperer_state_goto(JTAGWhisperer_get_next_byte());
    return true;
}

bool JTAGWhisperer_handle_XWAIT() {
    JTAGWhisperer_state_goto(JTAGWhisperer_get_next_byte());
    uint8_t end_state = JTAGWhisperer_get_next_byte();
    uint32_t wait_time_usec = JTAGWhisperer_get_next_long();
    JTAGWhisperer_wait_time(wait_time_usec);
    JTAGWhisperer_state_goto(end_state);
    return true;
}

void JTAGWhisperer_blink() {
  if(is_pin_on) PORTB |= _BV(BLINK_PIN);
  else PORTB &= ~(_BV(BLINK_PIN));
  is_pin_on = !is_pin_on;
}

void JTAGWhisperer_setup() {

  DDRB |= _BV(BLINK_PIN); // Pin is output
  for (int i = 0; i < 10; ++i) {
    JTAGWhisperer_blink();
    _delay_ms(50);
  }

  sei();
}

void JTAGWhisperer_loop() {
  //SerialComm serial_comm;
  //BitTwiddler twiddler;
  JTAGWhisperer_init();
  uint32_t instruction_count = 0;

  while (true) {
    JTAGWhisperer_blink();
    uint8_t instruction = JTAGWhisperer_read_next_instruction();
    if (!JTAGWhisperer_handle_instruction(instruction)) {
      if (!JTAGWhisperer_reached_xcomplete()) {
        SerialComm_Important("Failure at instruction #%d", instruction_count);
      }
      break;
    }
    ++instruction_count;
  }
  SerialComm_Important("Processed %d instructions.", instruction_count);

  while (true);
}

int main(void) {
    JTAGWhisperer_setup();

    for(;;)
        JTAGWhisperer_loop();

    return 1;
}
