#include <avr/io.h>
#include <stdio.h>

#define BYTE_COUNT(bit_count) ((int)((bit_count + 7) >> 3))
#define SDR_BEGIN 0x01
#define SDR_END   0x02
#define SDR_CHECK 0x10
#define SDR_NOCHECK 0
#define SDR_CONTINUE 0
#define SDR_FULL 3

void p(const char *fmt, ... ) {
  char tmp[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, 128, fmt, args);
  va_end(args);
  Serial.print(tmp);
}

#define READY(fmt, ...)  { Serial.println("\n\n\n\nRXSVF"); }

#define DEBUG(fmt, ...)  { }

#define DEBUG_(fmt, ...)  { Serial.print("D ");  \
    p(fmt, ## __VA_ARGS__);                     \
    Serial.println();                           \
  }

void print_byte(uint8_t c) {
  p("%02x", c);
}

void print_bytes(uint8_t* p, uint8_t count) {
  p += count;
  while (count--) {
    print_byte(*--p);
  }
}

#define DEBUG_BYTES(s, p, n)  { Serial.print("D "); Serial.print(s);  \
    print_bytes(p, n);                                                \
    Serial.println();                                                 \
  }

#define QUIT(fmt, ...)  { Serial.print("Q ");   \
    p(fmt, ## __VA_ARGS__);                     \
    Serial.println();                           \
  }

// Knows how to set MCU-specific pins in a JTAG-relevant way.
class Twiddler {
 public:
 Twiddler() : portb_(0),
    tdo_(0) {
    DDRB = TMS | TDI | TCK;
  }

  void pulse_clock_and_read_tdo() {
    noInterrupts();
    tdo_ >>= 1;
    clr_port(TCK);
    set_port(TCK);
    tdo_ |= (PINB & TDO) ? 0x80 : 0;
    interrupts();
  }

  void wait_time(unsigned long microsec) {
    unsigned long until = micros() + microsec;
    while (microsec--) {
      pulse_clock_and_read_tdo();
    }
    while (micros() < until) {
      pulse_clock_and_read_tdo();
    }
  }

  void set_tms() {
    set_port(TMS);
  }

  void clr_tms() {
    clr_port(TMS);
  }

  void set_tdi() {
    set_port(TDI);
  }

  void clr_tdi() {
    clr_port(TDI);
  }

  uint8_t tdo() { return tdo_; }

 private:
  enum {
    TMS = _BV(PINB0),  // Arduino 8
    TDI = _BV(PINB1),  // Arduino 9
    TDO = _BV(PINB2),  // Arduino 10
    TCK = _BV(PINB3)   // Arduino 11
  };

  inline void write_portb_if_tck(uint8_t pin) {
    if (pin == TCK) {
      PORTB = portb_;
    }
  }

  void set_port(uint8_t pin) {
    portb_ |= pin;
    write_portb_if_tck(pin);
  }

  void clr_port(uint8_t pin) {
    portb_ &= ~pin;
    write_portb_if_tck(pin);
  }

  // The current PORTB state. We write this only when we twiddle TCK.
  uint8_t portb_;

  // Shift register. Every time the clock pulses we shift a bit into this byte.
  uint8_t tdo_;
};

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

#define BYTES(bits) ((int)((bits + 7) >> 3))
#define HANDLE(x) case x: return handle_##x()
#define NAME_FOR(x) case x: return #x;

void explode() {
  QUIT("EXPLODED");
  while (true) {
  }
}

void succeed() {
  QUIT("SUCCESS");
  while (true) {
  }
}

void fail() {
  QUIT("FAIL");
  while (true) {
  }
}

class TAP {
 public:
 TAP(Twiddler& twiddler)
   : twiddler_(twiddler),
    current_state_(-1),
    sdrsize_bits_(0),
    sdrsize_bytes_(0),
    repeat_(32),
    runtest_(0),
    reached_xcomplete_(false) {
  }

  void get_next_bytes_from_stream(uint8_t count) {
    while (count--) {
      *bp_++ = get_next_byte_from_stream();
    }
  }

  bool reached_xcomplete() { return reached_xcomplete_; }

  uint8_t read_next_instruction() {
    bp_ = instruction_buffer_;
    uint8_t instruction = get_next_byte_from_stream();
    switch (instruction) {
    case XCOMPLETE:
      break;
    case XTDOMASK:
      get_next_bytes_from_stream(sdrsize_bytes_);
      break;
    case XSIR: {
      uint8_t length = get_next_byte_from_stream();
      *bp_++ = length;
      get_next_bytes_from_stream(BYTES(length));
      break;
    }
    case XSDR:
    case XSDRB:
    case XSDRC:
    case XSDRE:
      get_next_bytes_from_stream(sdrsize_bytes_);
      break;
    case XRUNTEST:
      get_next_bytes_from_stream(4);
      break;
    case XREPEAT:
      *bp_++ = get_next_byte_from_stream();
      break;
    case XSDRSIZE:
      get_next_bytes_from_stream(4);
      break;
    case XSDRTDO:
    case XSDRTDOB:
    case XSDRTDOC:
    case XSDRTDOE:
    case XSETSDRMASKS:
      get_next_bytes_from_stream(sdrsize_bytes_ + sdrsize_bytes_);
      break;
    case XSDRINC:
      explode();
      break;
    case XSTATE:
      *bp_++ = get_next_byte_from_stream();
      break;
    default:
      explode();
      break;
    }
    return instruction;
  }

 private:
  Twiddler twiddler_;
  int current_state_;
  uint8_t sdrsize_bits_;
  uint8_t sdrsize_bytes_;
  uint8_t repeat_;
  uint8_t runtest_;
  bool reached_xcomplete_;

  enum {
    BUFFER_SIZE = 32
  };
  uint8_t instruction_buffer_[BUFFER_SIZE];
  uint8_t* bp_;
  uint8_t tdi_[BUFFER_SIZE];
  uint8_t tdo_[BUFFER_SIZE];
  uint8_t tdomask_[BUFFER_SIZE];
  uint8_t tdo_expected_[BUFFER_SIZE];

  uint8_t get_next_byte() {
    return *bp_++;
  }

  uint16_t get_next_word() {
    return (uint16_t)get_next_byte() << 8 | get_next_byte();
  }

  uint32_t get_next_long() {
    return (uint32_t)get_next_byte() << 24 |
      (uint32_t)get_next_byte() << 16 |
      (uint32_t)get_next_byte() << 8 |
      get_next_byte();
  }

  void get_next_bytes(uint8_t* data, uint8_t count) {
    data += count;
    while (count--) {
      *--data = get_next_byte();
    }
  }

  bool handle_XCOMPLETE() {
    reached_xcomplete_ = true;
    return false;
  }

  bool handle_XTDOMASK() {
    get_next_bytes(tdomask_, sdrsize_bytes_);
    DEBUG_BYTES("... tdomask now ", tdomask_, sdrsize_bytes_);
    return true;
  }

  bool handle_XSIR() {
    uint8_t bits = get_next_byte();
    get_next_bytes(tdi_, BYTES(bits));
    //DEBUG_BYTES("... sending ", tdi_, BYTES(bits));
    state_goto(STATE_SHIFT_IR);
    shift_td(tdi_, bits, true);
    state_goto(STATE_RTI);
    DEBUG("..................");
    return true;
  }

  bool handle_XSDR() {
    get_next_bytes(tdi_, sdrsize_bytes_);
    DEBUG_BYTES("... sending ", tdi_, sdrsize_bytes_);
    return sdr(true, false);
  }

  bool handle_XRUNTEST() {
    runtest_ = get_next_long();
    DEBUG("... runtest now %x", runtest_);
    return true;
  }

  bool handle_XREPEAT() {
    repeat_ = get_next_byte();
    DEBUG("... repeat now %d", repeat_);
    return true;
  }

  bool handle_XSDRSIZE() {
    sdrsize_bits_ = get_next_long();
    sdrsize_bytes_ = BYTES(sdrsize_bits_);
    DEBUG("... sdrsize now %d", sdrsize_bytes_);
    return true;
  }

  bool handle_XSDRTDO() {
    get_next_bytes(tdi_, sdrsize_bytes_);
    get_next_bytes(tdo_expected_, sdrsize_bytes_);
    DEBUG_BYTES("... sending   ", tdi_, sdrsize_bytes_);
    DEBUG_BYTES("... expecting ", tdo_expected_, sdrsize_bytes_);
    return sdr(true, false);
  }

  bool is_tdo_as_expected() {
    DEBUG_BYTES("... received  ", tdo_, sdrsize_bytes_);
    for (int i = 0; i < sdrsize_bytes_; ++i) {
      uint8_t expected = tdo_expected_[i] & tdomask_[i];
      uint8_t actual = tdo_[i] & tdomask_[i];
      if (expected != actual) {
        DEBUG("... NO MATCH.");
        return false;
      }
    }
    DEBUG("... match!");
    return true;
  }

  bool handle_XSTATE() {
    state_goto(get_next_byte());
    return true;
  }

 public:
  bool handle_instruction(uint8_t instruction) {
    bp_ = instruction_buffer_;

    DEBUG("Handling %s", instruction_name(instruction));
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
    default:
      DEBUG("Got unknown instruction: %d", instruction);
      return false;
    }
  }

  void shift_td(uint8_t *data, uint16_t data_length_bits, bool is_end) {
    int byte_count = BYTE_COUNT(data_length_bits);

    for (int i = 0; i < byte_count; ++i) {
      uint8_t byte_out = data[i];
      for (int j = 0; j < 8; j++) {
        if (data_length_bits == 1 && is_end) {
          twiddler_.set_tms();
          state_ack(1);
        }

        if (data_length_bits--) {
          if (byte_out & 1) {
            twiddler_.set_tdi();
          } else {
            twiddler_.clr_tdi();
          }
          byte_out >>= 1;
          twiddler_.pulse_clock_and_read_tdo();
        }
      }
      tdo_[i] = twiddler_.tdo();
    }
  }

  bool sdr(bool should_check, bool should_end) {
    int attempts_left = repeat_;
    bool matched = false;
    while (!matched && attempts_left-- > 0) {
      shift_td(tdi_, sdrsize_bits_, false);
      if (should_check) {
        if (is_tdo_as_expected()) {
          matched = true;
        } else {
          state_step(0); /* Pause-DR state */
          state_step(1); /* Exit2-DR state */
          state_step(0); /* Shift-DR state */
          state_step(1); /* Exit1-DR state */

          state_goto(STATE_RTI);
          wait_time();
          state_goto(STATE_SHIFT_DR);
        }
      }
    }
    if (attempts_left == 0) {
      return false;
    }
    if (should_end) {
      state_goto(STATE_RTI);
    }
    wait_time();
    return true;
  }

  void wait_time() {
    uint32_t microsec = runtest_;
    uint32_t until = micros() + microsec;
    while (microsec--) {
      twiddler_.pulse_clock_and_read_tdo();
    }
    while (micros() < until) {
      twiddler_.pulse_clock_and_read_tdo();
    }
  }

  void set_state(int state) {
    current_state_ = state;
  }

  void state_ack(bool tms) {
    if (tms) {
      set_state((tms_transitions[current_state_] >> 4) & 0xf);
    } else {
      set_state(tms_transitions[current_state_] & 0xf);
    }
  }

  void state_step(bool tms) {
    if (tms) {
      twiddler_.set_tms();
    } else {
      twiddler_.clr_tms();
    }
    twiddler_.pulse_clock_and_read_tdo();
    state_ack(tms);
  }

  void state_goto(int state) {
    if (state == STATE_TLR) {
      for (int i = 0; i < 5; ++i) {
        state_step(true);
      }
    } else {
      while (current_state_ != state) {
        state_step((tms_map[current_state_] >> state) & 1);
      }
    }
  }

 private:
  enum {
    XCOMPLETE = 0,
    XTDOMASK,
    XSIR,
    XSDR,
    XRUNTEST,
    XREPEAT = 7,
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
    XSTATE
  };
  const char *instruction_name(uint8_t instruction) {
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
    default:
      return "XWTF";
    }
  }

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

};

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.begin(57600);
  READY();
}

uint8_t get_next_byte_from_stream() {
  while (Serial.available() <= 0) {
  }
  return Serial.read();
}

void loop() {
  Twiddler twiddler;
  TAP tap(twiddler);

  while (true) {
    uint8_t instruction = tap.read_next_instruction();
    if (!tap.handle_instruction(instruction)) {
      if (!tap.reached_xcomplete()) {
        fail();
      }
      break;
    }
  }
  succeed();
}
